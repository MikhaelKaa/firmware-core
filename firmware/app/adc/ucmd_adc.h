#ifndef _UCMD_ADC_H_
#define _UCMD_ADC_H_

#include <stdint.h>

/* Callback for ADC commands */
typedef int (*ucmd_adc_cb)(int, char **);

/* Main ADC command handler */
int ucmd_adc(int argc, char *argv[]);

#endif /* _UCMD_ADC_H_ */