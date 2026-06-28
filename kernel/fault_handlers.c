/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2026 Michael Kaa */

#include <stdint.h>
#include <stdio.h>


typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} stack_frame_t;

static void print_fault_info(const char* fault_name, stack_frame_t* frame) {
    printf("\r\n\r\n*** %s ***\r\n", fault_name);
    printf("PC:  0x%08lX\r\n", frame->pc);
    printf("LR:  0x%08lX\r\n", frame->lr);
    printf("PSR: 0x%08lX\r\n", frame->psr);
    printf("R0:  0x%08lX\r\n", frame->r0);
    printf("R1:  0x%08lX\r\n", frame->r1);
    printf("R2:  0x%08lX\r\n", frame->r2);
    printf("R3:  0x%08lX\r\n", frame->r3);
    printf("R12: 0x%08lX\r\n", frame->r12);
    
    /// SCB registers
    uint32_t* scb = (uint32_t*)0xE000ED00;
    printf("CFSR: 0x%08lX\r\n", scb[0x28/4]);
    printf("HFSR: 0x%08lX\r\n", scb[0x2C/4]);
    printf("DFSR: 0x%08lX\r\n", scb[0x30/4]);
    printf("AFSR: 0x%08lX\r\n", scb[0x3C/4]);
    printf("BFAR: 0x%08lX\r\n", scb[0x38/4]);
    printf("MMFAR: 0x%08lX\r\n", scb[0x34/4]);
    
    printf("\r\nSystem halted.\r\n");
    printf_flush();
    while(1);
}

void HardFault_Handler(void) {
    __asm volatile (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b hardfault_handler_c\n"
    );
}

void hardfault_handler_c(stack_frame_t* frame) {
    print_fault_info("HARDFAULT", frame);
}

void MemManage_Handler(void) {
    __asm volatile (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b memmanage_handler_c\n"
    );
}

void memmanage_handler_c(stack_frame_t* frame) {
    print_fault_info("MEMMANAGE FAULT", frame);
}

void BusFault_Handler(void) {
    __asm volatile (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b busfault_handler_c\n"
    );
}

void busfault_handler_c(stack_frame_t* frame) {
    print_fault_info("BUSFAULT", frame);
}

void UsageFault_Handler(void) {
    __asm volatile (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b usagefault_handler_c\n"
    );
}

void usagefault_handler_c(stack_frame_t* frame) {
    print_fault_info("USAGEFAULT", frame);
}
