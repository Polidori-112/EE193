#include "stm32l476xx.h"

// The clock setup from the Zhu book.
// These two clock-setup routines seem independent.
void clock_setup_16MHz(void);		// 16 MHz, AHB and APH1/2 prescale=1x
void clock_setup_80MHz(void);		// 80 MHz, AHB and APH1/2 prescale=1x

void set_AHB_APB1_prescalers_x1(void);
void ms_delay(int ms);

// Setup timer 2, channel 2 as a divider so that we can drive it to a pin and
// thus figure out the main clock speed.
void setup_TIM2_ch2_as_divider (unsigned int presc, unsigned int reload);

// Set a GPIO pin as a simple digital output.
void GPIO_set_output (GPIO_TypeDef *gpio,unsigned int pin);

// Set a GPIO pin to one of its alternate-function mode.
void set_gpio_alt_func (GPIO_TypeDef *gpio,unsigned int pin,unsigned int func);

// Set a GPIO pin on or off.
void set_GPIO_pin (GPIO_TypeDef *gpio, int pin, int val);

// Initial setup of the LEDs.
void init_red_LED(void);
void init_grn_LED(void);

// Set an LED on or off.
void set_grn_LED (int val);
void set_red_LED (int val);

// Toggle an LED
void toggle_grn_LED (void);
void toggle_red_LED (void);
