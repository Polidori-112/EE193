#include "stm32l476xx.h"
#include "lib_ee115.h"

////////////////////////////////////////////////////////////////////
// Clock stuff
////////////////////////////////////////////////////////////////////

#include <stdint.h>	// to get uint32_t.
#ifdef LL_DEFINES_SYSTEMCORECLOCK
    extern uint32_t SystemCoreClock;
#else
    uint32_t SystemCoreClock = 4000000;
#endif

// Use the high-speed internal clock (16 MHz) as the system clock source SYSCLK.
void clock_setup_16MHz(void) {
    // Clock Control Register (CR), bit 8
    // Enable High Speed Internal Clock (HSI = 16 MHz). This just turns it on,
    // without changing any mux selects.
    RCC->CR |= ((uint32_t)RCC_CR_HSION);

    // Wait until HSI has settled down after turnon. We know this by checking
    // another bit in RCC->CR.
    while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 )
	;

    // Select line on the main SYSCLK mux, choosing between HSE, MSI, HSI16 and
    // a frequency-multiplied PLL to drive SYSCLK.
    // Note we don't flip the select line until *after* HSI has stabilized :-)
    // And the reset-default value is to use the MSI oscillator.
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));	// First clear the bits
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;		// Then set to 01.

    // Wait till the mux select has actually happened. My guess is there are a
    // few dead cycles to avoid runt clock pulses.
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 )
  	;

    // Update the software global variable to talk with other API funcs.
    SystemCoreClock = 16000000;
}

// Set SYSCLK=80MHz. Set the AHB, APB1 and APB2 prescalers to x1 so that HCLK,
// PCLK2 and PCLK2 are also 80MHz. Set the SAI1 clock = 11.294MHz.
// Don't configure the various other clocks.
void clock_setup_80MHz(void){
    uint32_t HSITrim;

    // To correctly read data from FLASH memory, the number of wait states
    // (LATENCY) must be correctly programmed according to the frequency of the
    // CPU clock (HCLK) and the supply voltage of the device.		
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
    // Clock Control Register (CR), bit 8
    // Enable High Speed Internal Clock (HSI = 16 MHz). This just turns it on,
    // without changing any mux selects.
    RCC->CR |= RCC_CR_HSION;

    // Wait until HSI has settled down after turnon. We know this by checking
    // another bit in RCC->CR.
    while((RCC->CR & RCC_CR_HSIRDY) == 0)
	;

    // Internal Clock Sources Calibration Register (ICSCR).
    // This 32-bit register has 16 bits each for the HSI and MSI clocks. For
    // each of those, the 16 bits is 8 bits of read-only manufacturing fuses,
    // and 8 bits of user-selected trim values. The manufacturing fuses are
    // pretty good, so we just pick a nice midrange user-trim here.
    HSITrim = 16; // user-programmable trim, added to ICSCR.HSICAL[7:0].
    RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;	// Zero out user trim.
    RCC->ICSCR |= HSITrim << 24;	// And set a midrange value.

    // Turn off the main PLL. It's off at reset anyway, but just be sure in case
    // this function has been called before, or whatever.
    RCC->CR    &= ~RCC_CR_PLLON; 

    // Wait for it to lock. Not sure why this is needed, since it's off!
    while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY)
	;

    // Now that the PLL is off, let's configure it.
    // First, select clock source to PLL. The reset value is no clock at all.
    // Now we switch it to HSI (i.e., 16 MHz).
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;	// 00=No clock, 01=MSI,
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;	// 10=HSI, 11 = HSE

    // Set the PLL dividers to get 80 MHz output. The final output is
    // 16 MHz * N / (M*R). We'll set N=20, M=2, R=2 to get 16 * 20/4 = 80 MHz.
    //	000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4,
    // Note that N must be in [8,86].
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8; // N=20.
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4;  // M=2.

    // Set R=2. This is the default, anyway, with bits[26:25]=00.
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;

    // The PLL is configured; now turn it on. Note that there are two bits to
    // poke, not just one.
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output 
    RCC->CR   |= RCC_CR_PLLON; 

    // And wait for it to lock.
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
	;

    // Select line on the main SYSCLK mux, choosing between HSE, MSI, HSI16 and
    // a frequency-multiplied PLL to drive SYSCLK. We haven't touched it so far,
    // and the reset-default value is to use the MSI oscillator. Now we set it
    // to use the PLL that we just configured/locked.
    RCC->CFGR &= ~RCC_CFGR_SW;	  // Turn off all bits, so that we can...
    RCC->CFGR |= RCC_CFGR_SW_PLL; // ...set to the PLL.

    // Wait until the mux has switched
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	;
	
    // Set the AHB, APB1 and APB2 clock prescalers all to x1 (which is their
    // reset default anyway).
    // AHB controls some peripherals itself, as well as driving APB1 (the
    // low-speed peripheral clock) and APB2 (high-speed peripheral clock) via
    // prescalers.
    RCC->CFGR &= ~RCC_CFGR_HPRE;  // HCLK = SYSCLK x 1
    RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 = HCLK x 1
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 = HCLK x 1

    // Turn off the SA1 PLL
    RCC->CR &= ~RCC_CR_PLLSAI1ON;
    // Then wait for it to actually be off.
    while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON )
	;

    // SA1 VCO freq = PLL-in-freq * N/M = 16 MHz * 24/2 = 192 MHz
    // We already set the PLL-in-freq=16Mh and M=2 above, for the main PLL.
    // Now set divider SA1N=24.
    RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;	// bits[14:8]=0
    RCC->PLLSAI1CFGR |= 24U<<8;				// bits[14:8]=0x18

    // SA1 divider P = 17
    // This sets the SAI1 PLL P output = 192 MHz / 17 = 11.294MHz
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;	// bit[17]=1 -> P=17.

    // Enable the SA1 PLL clock output.
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;	// bit[16]

    // SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
    // RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
    // RCC->PLLSAI1CFGR |= U<<21;
    // RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;

    // PLLSAI1 division factor for PLLADC1CLK (ADC clock)
    // 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
    // RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R; 
    // RCC->PLLSAI1CFGR |= U<<25;
    // RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;

    // Turn on the SA1 PLL
    RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
    // Then wait for it to actually be on.
    while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0)
	;

    // Swing the final mux to drive the SA1 clock. It can come from
    // the SAI1 PLL P output, the SAI2 PLL P output, the main-PLL P output
    // or an external clock. We choose the SAI1-PLL P output.
    RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

    // Final clock enable for the SA1 clock.
    RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;

    // Update the software global variable to talk with other API funcs.
    SystemCoreClock = 80000000;
}

// Set the AHB and APB1 clock prescalers to x1.
// AHB controls some peripherals itself, as well as driving APB1 (the low-speed
// peripheral clock) and APB2 (high-speed peripheral clock) via prescalers.
void set_AHB_APB1_prescalers_x1 (void) {
    // Clock Configuration Register (RCC_CFGR), for AHB and APB1 prescalers.
    // Bits 10:8 control the APB1 prescaler. 0->/1, 4,5,6,7->/2,/4,/8,/16.
    // Bits  7:4 control the AHB prescaler.. 0->/1, 8-15->/2,/4,...,/512.
    // RCC defaults to all 0 -> both prescalers being /1.
    // Here, we set them to 1x just to be sure (this code usually is moot).
    // Just clear both fields (which sets them both to 1x).
    RCC->CFGR &= ~RCC_CFGR_PPRE1;	// Set APB1 prescaler.
    RCC->CFGR &= ~RCC_CFGR_HPRE;	// Now set AHB prescaler.
}

////////////////////////////////////////////////////////////////////
// Timers
////////////////////////////////////////////////////////////////////

// Set up timer 2, channel 2 to drive its output (which happens to be GPIO B3).
// Note you must set up GPIO B3 separately.
void setup_TIM2_ch2_as_divider (unsigned int presc, unsigned int reload) {
    // Clock enable for timer 2, on the APB1 clock.
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Control register 1 (CR1). Set timer to count up (applies to all channels)
    TIM2->CR1 &= ~TIM_CR1_DIR;

    // Prescaler (PSC). Note we divide by val+1, not by val.
    TIM2->PSC = presc-1;			// Prescale by 1000

    // Auto-reload register. Again, the count sequence has val+1 numbers.
    TIM2->ARR = reload-1;			// Auto-reload register.

    // Value for compare/capture register for channel 2. It doesn't really
    // what it is for our application, as long as it's in [0,ARR]. So we set
    // it nice and low.
    TIM2->CCR2 = 2;				// The counter "match" value

    // Compare-capture mode register for channels 1/2 (CCMR1) (CCMR2 is similar,
    // for channels 3/4). For each channel #, the CC#S bit sets whether the
    // channel is an input or output channel, and the rest of the fields then
    // get different meanings accordingly. Since we're an output for channel 2
    // we must set CC2S=00; then set OC2M=0011 so that OCREF toggles.
    // There are a few other bits (enables for clear, preload and fast) that we
    // don't touch.
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;		// Clear channel-2 mode.
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1;

    // Capture/Compare Enable Register (CCER). This both enables the output
    // channel to drive an output pin, and sets the polarity of CC and CCN vs.
    // OCREF. It's one register for four output channels. If channel # is an
    // output channel, then CC#E=1, CC#NP=0, and CC#P sets the polarity (which
    // for our use is irrelevant).
    TIM2->CCER &= ~TIM_CCER_CC2NP;		// Force NPD=2
    TIM2->CCER |= TIM_CCER_CC2E;

    // Control Register 1. Various control bits, including enabling the timer.
    TIM2->CR1 |= TIM_CR1_CEN;			// Enable timer 2.
}

////////////////////////////////////////////////////////////////////
// GPIO stuff
////////////////////////////////////////////////////////////////////

static void gpio_enable_port (GPIO_TypeDef *gpio) {
    unsigned long field;
    if (gpio==GPIOA)      field=RCC_AHB2ENR_GPIOAEN;
    else if (gpio==GPIOB) field=RCC_AHB2ENR_GPIOBEN;
    else if (gpio==GPIOC) field=RCC_AHB2ENR_GPIOCEN;
    else if (gpio==GPIOD) field=RCC_AHB2ENR_GPIODEN;
    else if (gpio==GPIOE) field=RCC_AHB2ENR_GPIOEEN;
    else if (gpio==GPIOF) field=RCC_AHB2ENR_GPIOFEN;
    else if (gpio==GPIOG) field=RCC_AHB2ENR_GPIOGEN;
    else 		  field=RCC_AHB2ENR_GPIOHEN;
    RCC->AHB2ENR |= field;			// Turn on the GPIO clock
}

// Enable the clock to a GPIO port and set a given pin to be a
// general-purpose output pin.
void GPIO_set_output (GPIO_TypeDef *gpio,unsigned int pin) {
    gpio_enable_port (gpio);		// Send power to this GPIO port.

    // Each GPIO port has a MODER register. It has two bits/pin; each can be
    // one of 00: Input mode, 01: General purpose output mode
    //       10: Alternate function mode, 11: Analog mode (reset state)
    unsigned int b0 = 1<<(pin<<1);	// bit 0 of this pin's two-bit field
    gpio->MODER &= ~(b0<<1);		// Clear the field's bit 1
    gpio->MODER |= b0;			// Set   the field's bit 0
}

// Initial setup for the LEDs.
void init_red_LED(void) {
    GPIO_set_output (GPIOB, 2);		// Set up pin B2 as a digital output
}
void init_grn_LED(void) {
    GPIO_set_output (GPIOE, 8);		// Set up pin E8 as a digital output
}

void set_GPIO_pin (GPIO_TypeDef *gpio, int pin, int val) {
    uint32_t shft = 1<<pin;
    if (val)
	gpio->ODR |= shft;
    else
	gpio->ODR &= ~shft;
}

// GPIO port B, pin 2.
// Val is either 0 for off, nonzero for on
void set_red_LED (int val) {
    if (val)	GPIOB->ODR |= GPIO_ODR_ODR_2;
    else	GPIOB->ODR &= ~GPIO_ODR_ODR_2;
}

void toggle_red_LED (void) {
    GPIOB->ODR ^= GPIO_ODR_ODR_2;
}

// GPIO port E, pin 8.
// Val is either 0 for off, nonzero for on
void set_grn_LED (int val) {
    if (val)	GPIOE->ODR |= GPIO_ODR_ODR_8;
    else	GPIOE->ODR &= ~GPIO_ODR_ODR_8;
}

void toggle_grn_LED (void) {
    GPIOE->ODR ^= GPIO_ODR_ODR_8;
}

// Setup for the GPIO port B, pin 3 to be alternate-function #1.
// Params:
//	gpio: which port; one of GPIOA, GPIOB, ... GPIOH.
//	pin:  0-15, for which GPIO pin in the port.
//	func: which of the 15 alternate functions to use.
void set_gpio_alt_func (GPIO_TypeDef *gpio,unsigned int pin,unsigned int func){
    gpio_enable_port (gpio);

    // Mode Register (MODER). Two bits of mode for each of the 16 pins/port.
    // And 10 -> alternate function.
    gpio->MODER &= ~(3UL << (2*pin));		// Clear the appropriate field.
    gpio->MODER |= 2UL << (2*pin);		// And set to binary 10.

    // AFRL sets the alternate function for pins 0-7; AFRH for pins 8-15.
    // Each register is just eight four-bit fields (one for each pin).
    // The .h file calls the two registers AFR[0] and AFR[1], but then names
    // the bits with the H and L suffixes!
    int idx = (pin>=8);
    gpio->AFR[idx] &= ~(0xFUL << (4*pin));
    gpio->AFR[idx] |=  (func  << (4*pin));

    // Output Speed Register (OSPEEDR). Two bits for each of the 16 pins/port.
    // And 00 -> low speed.
    gpio->OSPEEDR &= ~(3UL<<(2*pin));		// GPIO output speed=slow
    // Pull Up/Pull Down Register (PUPDR). Two bits for each of the 16
    // pins/port. And 00 -> no pullup or pulldown.
    gpio->PUPDR &= ~(3UL <<(2*pin));		// No PUP or PDN

}

// This was calibrated against lab_dac.
void ms_delay(int ms)
{
   unsigned int nop_per_ms = (unsigned int) (SystemCoreClock / 9700);
   while (ms-- > 0) {
      volatile unsigned int x=nop_per_ms;
      while (x-- > 0)
         __asm("nop");
   }
}
