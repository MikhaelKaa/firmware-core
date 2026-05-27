/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include "adc.h"
#include "stm32f407xx.h"
#include <string.h>
#include <errno.h>

/* Driver version */
const char *dev_adc1_version = "stm32f407vgt6 adc1; PA0-CH0; DMA; 10000Hz; ver 0.0.0";

/* Ring buffer for ADC1 DMA data */
static uint16_t adc1_dma_buffer[ADC1_BUFFER_SIZE];

/* DMA transfer state */
static volatile uint8_t adc1_running = 0;
static volatile uint32_t adc1_status = 0;

/* Callbacks for DMA events */
static void (*adc1_half_callback)(void) = NULL;
static void (*adc1_full_callback)(void) = NULL;

/* Sampling frequency configuration */
static volatile uint32_t adc1_freq = ADC1_DEFAULT_FREQ;

/* Initialize ADC1 */
static int adc1_init(void) {
    uint32_t adc_pre;
    
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    /* Configure PA0 as analog input */
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;  /* No pull-up/pull-down */
    
    /* ADC clock divider via common register (max 36MHz) */
    /* ADCCLK = PCLK2 / (2^ADC_PRE) where ADC_PRE = 0,1,2,3 -> div 2,4,8,16 */
    /* Need ADCCLK <= 36MHz, so for 84MHz PCLK2: 84/4 = 21MHz OK */
    adc_pre = 1;  /* Divide by 4 */
    ADC->CCR &= ~(ADC_CCR_ADCPRE_Msk);
    ADC->CCR |= (adc_pre << ADC_CCR_ADCPRE_Pos);
    
    /* Disable ADC before config */
    ADC1->CR2 &= ~ADC_CR2_ADON;
    
    /* Reset registers to default */
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->JSQR = 0;
    
    /* Configure SMPR2 for channel 0 sampling time (480 cycles for accuracy) */
    /* SMPR2_SMP0[2:0] bits at position 0 */
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0_Msk;
    ADC1->SMPR2 |= (7UL << ADC_SMPR2_SMP0_Pos);  /* 480.5 sampling cycles */
    
    /* Configure ADC1 CR1:
     * - Scan mode (for multi-channel support)
     */
    ADC1->CR1 = ADC_CR1_SCAN;  /* Scan mode enabled */
    
    /* Configure ADC1 CR2:
     * - Right alignment
     * - External trigger TIM6 TRGO (EXTSEL = 0b1111 = 15)
     * - External trigger enable (rising edge)
     * - DMA continuous requests (DDS = 1 for single mode)
     * - DMA enable
     */
    ADC1->CR2 = ADC_CR2_ALIGN |                    /* Right alignment */
                (15 << ADC_CR2_EXTSEL_Pos) |       /* TIM6 TRGO trigger */
                ADC_CR2_EXTEN_0 | ADC_CR2_EXTEN_1 | /* Rising edge trigger */
                ADC_CR2_DDS |                       /* DMA continuous mode */
                ADC_CR2_DMA;                        /* DMA enable */
    
    /* Enable ADC1 (double write for power-up) */
    ADC1->CR2 |= ADC_CR2_ADON;
    (void)ADC1->DR;  /* Wake up */
    ADC1->CR2 |= ADC_CR2_ADON;
    
    /* Wait for ADC ready */
    {
        int i;
        for (i = 0; i < 1000000; i++) {
            if (ADC1->SR & ADC_SR_EOC) break;
        }
    }
    
    /* Configure TIM6 for ADC1 trigger */
    {
        uint32_t apb1_clk = 84000000U;  /* TIM6 clock = APB1 clock */
        uint32_t prescaler;
        uint32_t arr_val;
        
        /* Calculate timer prescaler and auto-reload value */
        prescaler = 0;
        arr_val = 0;
        
        for (prescaler = 0; prescaler < 0xFFFF; prescaler++) {
            arr_val = (apb1_clk / (apb1_clk / (prescaler + 1))) / adc1_freq - 1;
            if (arr_val < 0x10000) {
                break;
            }
        }
        
        if (arr_val >= 0x10000) {
            arr_val = 0xFFFF;
            prescaler = 0;
        }
        
        /* Configure TIM6 */
        TIM6->PSC = prescaler;
        TIM6->ARR = arr_val;
        
        /* Set TRGO source to update event */
        TIM6->CR2 = TIM_CR2_MMS_1 | TIM_CR2_MMS_0;  /* TRGO = TIM6 update */
        
        /* Enable TIM6 */
        TIM6->CR1 |= TIM_CR1_CEN;
    }
    
    /* Clear DMA buffer */
    memset(adc1_dma_buffer, 0, sizeof(adc1_dma_buffer));
    
    adc1_status = ADC1_STATUS_READY;
    
    return 0;
}

/* Deinitialize ADC1 */
static int adc1_deinit(void) {
    /* Stop ADC */
    ADC1->CR2 &= ~ADC_CR2_ADON;
    
    /* Disable DMA */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    
    /* Disable TIM6 */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    
    /* Disable interrupts */
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    
    adc1_running = 0;
    adc1_status = 0;
    
    return 0;
}

/* Read data from ADC1 (interface implementation) */
static int adc1_read(void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_to_read;
    size_t available;
    uint32_t current_ndtr;
    uint32_t samples_to_read;
    
    if (!adc1_running) {
        return -EAGAIN;
    }
    
    /* Get current DMA remaining count */
    current_ndtr = DMA2_Stream0->NDTR;
    
    /* Calculate available samples */
    available = ADC1_BUFFER_SIZE - current_ndtr;
    
    if (available == 0) {
        return 0;  /* No data available */
    }
    
    /* Calculate how many samples we can read */
    samples_to_read = available;
    bytes_to_read = samples_to_read * sizeof(uint16_t);
    
    if (bytes_to_read > count) {
        bytes_to_read = count;
        samples_to_read = count / sizeof(uint16_t);
    }
    
    /* Copy data from DMA buffer to user buffer */
    memcpy(buffer, adc1_dma_buffer, bytes_to_read);
    
    return (int)bytes_to_read;
}

/* Write to ADC1 (interface implementation) - not supported */
static int adc1_write(const void *buf, size_t count) {
    (void)buf;
    (void)count;
    /* ADC is read-only device */
    return -ENOTSUP;
}

/* Start ADC1 conversion (interface implementation) */
static int adc1_start(void) {
    if (adc1_running) {
        return 0;  /* Already running */
    }
    
    /* Configure DMA for ADC1 (circular mode) */
    /* DMA2 Stream0 for ADC1 */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;        /* Peripheral address */
    DMA2_Stream0->M0AR = (uint32_t)adc1_dma_buffer;  /* Memory address */
    DMA2_Stream0->NDTR = ADC1_BUFFER_SIZE;           /* Number of data items */
    
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) |  /* Channel 0 */
                       DMA_SxCR_MINC |               /* Memory increment */
                       DMA_SxCR_CIRC |               /* Circular mode */
                       DMA_SxCR_PL_0 | DMA_SxCR_PL_1 | /* Very high priority */
                       DMA_SxCR_MSIZE_0 |            /* Memory word size (16-bit) */
                       DMA_SxCR_PSIZE_0 |            /* Peripheral word size (16-bit) */
                       DMA_SxCR_HTIE |               /* Half transfer interrupt */
                       DMA_SxCR_TCIE;                 /* Transfer complete interrupt */
    
    DMA2_Stream0->FCR = 0;  /* No FIFO */
    
    /* Enable DMA request */
    ADC1->CR2 |= ADC_CR2_DMA;
    
    /* Enable DMA2 Stream0 interrupt */
    NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
    /* Enable ADC interrupt */
    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);
    
    /* Start conversion by enabling ADC */
    ADC1->CR2 |= ADC_CR2_ADON;
    
    adc1_running = 1;
    adc1_status |= ADC1_STATUS_RUNNING;
    
    return 0;
}

/* Stop ADC1 conversion (interface implementation) */
static int adc1_stop(void) {
    if (!adc1_running) {
        return 0;  /* Already stopped */
    }
    
    /* Disable ADC */
    ADC1->CR2 &= ~ADC_CR2_ADON;
    
    /* Disable DMA */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    
    /* Disable interrupts */
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    
    adc1_running = 0;
    adc1_status &= (uint32_t)~(uint32_t)ADC1_STATUS_RUNNING;
    
    return 0;
}

/* IO Control for ADC1 (interface implementation) */
static int adc1_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case INTERFACE_INIT:
            return adc1_init();
        
        case INTERFACE_DEINIT:
            return adc1_deinit();
        
        case INTERFACE_GET_INFO:
            if (arg != NULL) {
                *(const char **)arg = dev_adc1_version;
                return 0;
            }
            return -EINVAL;
        
        case ADC1_GET_AVAILABLE:
            if (arg != NULL) {
                uint32_t current_ndtr = DMA2_Stream0->NDTR;
                *(uint32_t *)arg = ADC1_BUFFER_SIZE - current_ndtr;
                return 0;
            }
            return -EINVAL;
        
        case ADC1_SET_FREQ:
            if (arg != NULL) {
                adc1_freq = *(uint32_t *)arg;
                if (adc1_running) {
                    /* Reconfigure timer while running */
                    uint32_t apb1_clk = 84000000U;
                    uint32_t prescaler;
                    uint32_t arr_val;
                    
                    for (prescaler = 0; prescaler < 0xFFFF; prescaler++) {
                        arr_val = (apb1_clk / (apb1_clk / (prescaler + 1))) / adc1_freq - 1;
                        if (arr_val < 0x10000) {
                            break;
                        }
                    }
                    
                    if (arr_val >= 0x10000) {
                        arr_val = 0xFFFF;
                        prescaler = 0;
                    }
                    
                    TIM6->PSC = prescaler;
                    TIM6->ARR = arr_val;
                }
                return 0;
            }
            return -EINVAL;
        
        case ADC1_SET_CB_HALF:
            {
                union {
                    void (*func)(void);
                    void *ptr;
                } cast;
                cast.ptr = arg;
                adc1_half_callback = cast.func;
                return 0;
            }
        
        case ADC1_SET_CB_FULL:
            {
                union {
                    void (*func)(void);
                    void *ptr;
                } cast;
                cast.ptr = arg;
                adc1_full_callback = cast.func;
                return 0;
            }
        
        case ADC1_GET_STATUS:
            if (arg != NULL) {
                *(uint32_t *)arg = adc1_status;
                return 0;
            }
            return -EINVAL;
        
        case ADC1_START:
            return adc1_start();
        
        case ADC1_STOP:
            return adc1_stop();
        
        default:
            return -ENOTSUP;
    }
}

/* ADC1 device instance */
static const drv_face_t dev_adc1 = {
    .read = adc1_read,
    .write = adc1_write,
    .ioctl = adc1_ioctl
};

/* ADC1 device instance accessor */
const drv_face_t* dev_adc1_get(void) {
    return (const drv_face_t *)&dev_adc1;
}

/* DMA2 Stream0 Interrupt Handler (ADC1) */
void DMA2_Stream0_IRQHandler(void) {
    /* Half Transfer Interrupt */
    if (DMA2->LISR & DMA_LISR_HTIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF0;  /* Clear flag */
        if (adc1_half_callback != NULL) {
            adc1_half_callback();
        }
    }
    
    /* Transfer Complete Interrupt */
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;  /* Clear flag */
        if (adc1_full_callback != NULL) {
            adc1_full_callback();
        }
    }
}

/* ADC Interrupt Handler */
void ADC_IRQHandler(void) {
    /* Handle ADC errors */
    if (ADC1->SR & (ADC_SR_OVR | ADC_SR_AWD)) {
        (void)ADC1->SR;  /* Clear flags by reading */
        (void)ADC1->DR;  /* Clear OVR */
    }
}