#ifndef __STM32L476G_DISCOVERY_ADC_DAC_H
#define __STM32L476G_DISCOVERY_ADC_DAC_H

#include "stm32l476xx.h"

//**********************************
// ADC
//**********************************
void ADC_Init(void);	// Sets up PA1 as input.
uint32_t ADC1_read (void);

//**********************************
// DAC
//**********************************

void DAC1_Init (void);	// Drive PA3 through an op-amp unity-gain follower.
void DAC2_Init (void);	// Drive PA5 directly from the DAC (with its buffer).

// Write 12-bit unsigned data to DAC 1, which drives PA3.
void DAC1_write (uint32_t data);

// Write 12-bit unsigned data to DAC 2, which drives pin PA5.
void DAC2_write (uint32_t data);

#endif /* __STM32L476G_DISCOVERY_ADC_DAC_H */
