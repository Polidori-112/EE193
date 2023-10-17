#ifndef __STM32L476G_DISCOVERY_LCD_H
#define __STM32L476G_DISCOVERY_LCD_H

#include <stdint.h>

// Initialization function, called once at the beginning of the program.
void LCD_Initialization(void);

// Expects a C-style string. Not all characters are legal; some non-displayable
// characters are silently converted to displayable ones.
void LCD_DisplayString(uint8_t* ptr);

#endif /* __STM32L476G_DISCOVERY_LCD_H */
