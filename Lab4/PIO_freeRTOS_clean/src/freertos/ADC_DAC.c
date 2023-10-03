//**********************************
// ADC
//**********************************

#include "ADC_DAC.h"
#include "stm32l476xx.h"
#include <stdint.h>
#include "lib_ee115.h"		// for delay_ms().

// Analog Inputs: 
//    PA1 (ADC12_IN6), PA2 (ADC12_IN7)
//    These pins are not used: PA0 (ADC12_IN5, PA3 (ADC12_IN8)

//**********************************************************************************
// STM32L4x6xx Errata sheet
// When the delay between two consecutive ADC conversions is higher than 1 ms the
// result of the second conversion might be incorrect. The same issue occurs when
// the delay between the calibration and the first conversion is higher than 1 ms.
// Workaround
// When the delay between two ADC conversions is higher than the above limit, perform
// two ADC consecutive conversions in single, scan or continuous mode: the first is
// a dummy conversion of any ADC channel. This conversion should not be taken into
// account by the application.

static void ADC_Pin_Init(void);
static void ADC_Common_Configuration(void);
static void ADC_Wakeup (void);


//*********************************************************************************
// Initialize ADC	
// The single master entry point for setting up our ADCs.
// We enable clocks, take the ADCs in/out of reset, then call ADC_Pin_Init(),
// ADC_Common_Configuration() and ADC_Wakeup() below.
// Then, set ADC #1 to 12 bits, right-aligned, single (as opposed to multiple
// scanned) conversions of channel #6 (which is hardwired to PA1), single-ended
// input, sample time of .3us, single-shot (not continuous) conversion from a
// software trigger, using ADC 1.
// A few weirdnesses:
// - ADC_Pin_Init() sets up both pins PA1 and PA2 (which is input channel #7),
//   but we're not set up to actually look at PA2.
// - ADC_Common_Configuration() sets up regular-simultaneous mode for ADC1/2,
//   but then nothing ever gets set up to use ADC2.
//*********************************************************************************
void ADC_Init(void){
	// Enable the clock of ADC
	// RCC_AHB2ENR is the AHB2 peripheral-clock-enable register. 
	// ADCEN enables the clock to all ADCs.
	RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN;
	// RCC_AHB2RSTR is the AHB2 peripheral-reset register.
	// Setting ADCRST=1 resets the ADC.
	RCC->AHB2RSTR	|= RCC_AHB2RSTR_ADCRST;
	(void)RCC->AHB2RSTR; // short delay
	RCC->AHB2RSTR	&= ~RCC_AHB2RSTR_ADCRST;	// Take out of reset.
	
	ADC_Pin_Init();
	ADC_Common_Configuration();
	ADC_Wakeup();
	
	// ADC configuration reg (one per ADC). Set the resolution and alignment
	// for ADC #1 to 12 bits, right-aligned.
	// Resolution, (00=12-bit, 01=10-bit, 10=8-bit, 11=6-bit)
	ADC1->CFGR &= ~ADC_CFGR_RES;
	// Data Alignment (0=Right alignment, 1=Left alignment)
	ADC1->CFGR &= ~ADC_CFGR_ALIGN;

	// ADC regular sequence register 1 (ADC_SQR1)
	// L[3:0] tells how many ADC conversions are done (in scan mode) in each
	// scan. The rest of the register (and several others) details the scan
	// sequence.
	// L[3:0]=0000 means just 1 conversion in the regular channel conversion
	// sequence
	ADC1->SQR1 &= ~ADC_SQR1_L;
	
	// Specify the channel number of the 1st conversion in regular sequence
	// In the same CSR, SQ1[4:0] is which input gets converted. It's bits[10:6].
	// We set it for ADC12_IN6, which is PA1.
	ADC1->SQR1 &= ~ADC_SQR1_SQ1;		// Clear bits[10:6]
	ADC1->SQR1 |=  ( 6U << 6 );           	// It's ADC12_IN6, which is PA1.

	// DIFSEL is 1 bit/channel; 1->differential mode, 0->single ended.
	// Again, we hit input #6.
	// Single-ended for ADC12_IN6 (pin PA1).
	ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_6;
	
	// ADC Sample Time
	// This sampling time must be enough for the input voltage source to charge the embedded
	// capacitor to the input voltage level.
	// ADC Sample-time Register 1 (SMPR1) and SMPR2 contain three-bit fields
	// SMP[18:0][2:0]. Each 3-bit field controls the sample time for the
	// corresponding ADC input channel.
	// We set channel #6 to be 24.5 clock cycles.
	// Software is allowed to write these bits only when ADSTART=0 and JADSTART=0
	//   000: 2.5 ADC clock cycles      001: 6.5 ADC clock cycles
	//   010: 12.5 ADC clock cycles     011: 24.5 ADC clock cycles
	//   100: 47.5 ADC clock cycles     101: 92.5 ADC clock cycles
	//   110: 247.5 ADC clock cycles    111: 640.5 ADC clock cycles	
	// NOTE: These bits must be written only when ADON=0. 
	ADC1->SMPR1  &= ~ADC_SMPR1_SMP6;      // ADC Sample Time
	ADC1->SMPR1  |= 3U << 18;             // 3: 24.5 ADC clock cycles @80MHz = 0.3 us

	// ADC configuration register; turn off continuous-conversion mode, so
	// we request ADC conversions one at a time.
	ADC1->CFGR &= ~ADC_CFGR_CONT;       // ADC Single/continuous conversion mode for regular conversion

	// Configuring the trigger polarity for regular external triggers
	// 00: Hardware Trigger detection disabled, software trig detection enabled
	// 01: Hardware Trigger with detection on the rising edge
	// 10: Hardware Trigger with detection on the falling edge
	// 11: Hardware Trigger with detection on both the rising and falling edges
	// Make sure we only start an ADC from software, rather than any HW trigger.
	ADC1->CFGR &= ~ADC_CFGR_EXTEN; 
	
	// Enable ADC1
	// ADC control register; enable ADC #1. It won't really be enabled until
	// the HW sets ADRDY.
	ADC1->CR |= ADC_CR_ADEN;  
	// ISR is the interrupt and status register. HW turns on its ADRDY bit
	// when the ADC is actually ready to be used.
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0); 
}


//****************************************************************************
// 	ADC 1/2 Interrupt Handler
//****************************************************************************
#if 0 // unused
static void ADC1_2_IRQHandler(void){
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);
	
	// ADC End of Conversion (EOC)
	if ((ADC1->ISR & ADC_ISR_EOC) == ADC_ISR_EOC) {
		// It is cleared by software writing 1 to it or by reading the corresponding ADCx_JDRy register
		ADC1->ISR |= ADC_ISR_EOC;
	}
	
	// ADC End of Injected Sequence of Conversions  (JEOS)
	if ((ADC1->ISR & ADC_ISR_EOS) == ADC_ISR_EOS) {
		// It is cleared by software writing 1 to it.
		ADC1->ISR |= ADC_ISR_EOS;		
	}
}
#endif


//****************************************************************************
// 	ADC Pin Initialization
// Enable GPIO pins PA1 and PA2 as analog inputs.
// Specifically: turn on the GPIO port-A clock; then set each pin to be analog
// input, with no pull-up or pull-down, and switch-connected to the A2Ds.
// These two pins are hard-wired to ADC12_IN6 and ADC12_IN7 respectively.
//****************************************************************************
static void ADC_Pin_Init(void){	
	// Enable the clock of GPIO Port A, since we'll use the
	// pins A1 and A2 as analog inputs.
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOAEN;
	
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	// Configure PA1 (ADC12_IN6), PA2 (ADC12_IN7) as Analog
	// The GPIO Mode Register has two bits/pin.
	GPIOA->MODER |=  3U<<(2*1) | 3U<<(2*2);  // Mode 11 = Analog
	
	// GPIO Pullup-pulldown register (PUPDR)
	//	No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	// Two bits/pin, and we're setting pins A1 and A2.
	GPIOA->PUPDR &= ~( 3U<<(2*1) | 3U<<(2*2)); // No pull-up, no pull-down
	
	// GPIO port analog switch control register (ASCR)
	// Enable A1 and A2 as analog input pins.
	// 0: Disconnect analog switch to the ADC input (reset state)
	// 1: Connect analog switch to the ADC input
	GPIOA->ASCR |= GPIO_ASCR_EN_1 | GPIO_ASCR_EN_2;
}


//********************************************************************************
// 	ADC Common Configuration
// The second function called by ADC_Init(). It sets a few global features
// (analog switch resistances, clock source, regular-simultaneous mode for
// ADC1/2.
//********************************************************************************
static void ADC_Common_Configuration(void){
	// System Configuration Controller (SYSCFG) is a set of CSRs controlling
	// numerous system-wide features. We're writing the SYSCFG Configuration
	// Register 1 (CFGR1). The reference manual recommends setting BoostEn=1
	// when the ADC is in low-Vdd_analog mode, allowing the analog input
	// switches to have a lower resistance.
	// I/O analog switches voltage booster
	// The I/O analog switches resistance increases when the VDDA voltage is too
	// low. This requires having the sampling time adapted accordingly (see
	// datasheet for electrical characteristics). This resistance can be
	// minimized at low VDDA by enabling an internal voltage booster with
	// BOOSTEN bit in the SYSCFG_CFGR1 register.
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN;
	
	// V_REFINT enable
	// ADC Common Control Register (CCR). One CSR controlling all three
	// ADCs. We're enabling the VrefInt channel; this lets the ADCs'
	// internal reference voltage also drive a dedicated ADC input channel
	// (so you can measure it if you want; it should by definition be the
	// maximum reading).
	ADC123_COMMON->CCR |= ADC_CCR_VREFEN;  
	
	// ADC Clock Source: System Clock, PLLSAI1, PLLSAI2
	// Maximum ADC Clock: 80 MHz
	
	// ADC prescaler to select the frequency of the clock to the ADC
	// Same register: don't divide down (i.e., prescale) the ADC clock.
	ADC123_COMMON->CCR &= ~ADC_CCR_PRESC;   // 0000: input ADC clock not divided
	
	// ADC clock mode
	//   00: CK_ADCx (x=123) (Asynchronous clock mode),
	//   01: HCLK/1 (Synchronous clock mode).
	//   10: HCLK/2 (Synchronous clock mode)
	//   11: HCLK/4 (Synchronous clock mode)	 
	// Same CSR: set CkMode[1:0] to 01, to just use HClk. According to the
	// reference manual, you can only do this if HClk has a 50% duty cycle
	// (i.e., no need to /2 to get 50%), and the AHB clock prescaler is set
	// to 1 (HPRE[3:0] = 0xxx in RCC_CFGR register).
	ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;  // HCLK = 80MHz
	ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;

	//////////////////////////////////////////////////////////////////
	// Independent Mode
	// Same CSR: set the master & slave ADCs to work simultaneously.
	// ADC1 is always the master & ADC2 the slave; they share the same input
	// channels. In regular-simultaneous mode, you just kick off the master
	// and the slave automatically converts at the same time.
	ADC123_COMMON->CCR &= ~ADC_CCR_DUAL;
	ADC123_COMMON->CCR |= 6U;  // 00110: Regular simultaneous mode only
}


//********************************************************************************
// ADC Wakeup
// By default, the ADC is in deep-power-down mode where its supply is internally
// switched off to reduce the leakage currents.
// This is the final function we call from ADC_Init()
//********************************************************************************
static void ADC_Wakeup (void) {
	int wait_time;
	
	// To start ADC operations, the following sequence should be applied
	// DEEPPWD = 0: ADC not in deep-power down
	// DEEPPWD = 1: ADC in deep-power-down (default reset state)
	if ((ADC1->CR & ADC_CR_DEEPPWD) == ADC_CR_DEEPPWD)
		ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode if still in that state
	
	// Enable the ADC internal voltage regulator
	// Before performing any operation such as launching a calibration or enabling the ADC, the ADC
	// voltage regulator must first be enabled and the software must wait for the regulator start-up time.
	ADC1->CR |= ADC_CR_ADVREGEN;	

	// Wait for ADC voltage regulator start-up time
	// The software must wait for the startup time of the ADC voltage regulator (T_ADCVREG_STUP) 
	// before launching a calibration or enabling the ADC.
	// T_ADCVREG_STUP = 20 us
	wait_time = 20 * (80000000 / 1000000);
	while(wait_time != 0) {
		wait_time--;
	}   
}

// Read data from ADC #1 using a spin-wait loop. The wait is short, since the HW
// converts one bit/cycle.
uint32_t ADC1_read (void) {
    // Kick off a conversion.
    ADC1->CR |= ADC_CR_ADSTART;
    // Wait until the hardware says it's done.
    while ( ( ADC123_COMMON->CSR & ADC_CSR_EOC_MST ) == 0 )
	;
    // And return the data value.
    return (ADC1->DR);
}

//**********************************
// DAC
//**********************************

//*************************************************************************
// DAC Calibration
//*************************************************************************
static void DAC_Calibration_Channel(uint32_t channel){
	
	uint32_t trimmingvalue, delta, offset;
	uint32_t DAC_CR_CEN_Flag, DAC_CCR_OTRIM_Flag, DAC_SR_CAL_Flag;
	
	if (channel == 1) {
		DAC_CR_CEN_Flag = DAC_CR_CEN1;
		DAC_CCR_OTRIM_Flag = DAC_CCR_OTRIM1;
		DAC_SR_CAL_Flag = DAC_SR_CAL_FLAG1;
		offset = 0;
	} else {
		DAC_CR_CEN_Flag = DAC_CR_CEN2;
		DAC_CCR_OTRIM_Flag  = DAC_CCR_OTRIM2;
		DAC_SR_CAL_Flag = DAC_SR_CAL_FLAG2;
		offset = 16;
	}
	
	if (channel == 1) {
		DAC->CR &= ~DAC_CR_EN1;  // Ensure DAC 1 is off
	} else {
		DAC->CR &= ~DAC_CR_EN2;  // Ensure DAC 2 is off
	}
	
	// Enable DAC Channel calibration 
	DAC->CR |=  DAC_CR_CEN_Flag;  
	
	/* Init trimming counter */    
  /* Medium value */
  trimmingvalue = 16; 
  delta = 8;
  while (delta != 0) {
		
    /* Set candidate trimming */
		// DAC calibration control register (DAC_CCR)
		DAC->CCR &= ~DAC_CCR_OTRIM_Flag;
		DAC->CCR |= ((trimmingvalue<<offset) & DAC_CCR_OTRIM_Flag);
  
    /* tOFFTRIMmax delay x ms as per datasheet (electrical characteristics */ 
    /* i.e. minimum time needed between two calibration steps */
    ms_delay(1);
  
		if ((DAC->SR & DAC_SR_CAL_Flag) == 0) 
			/* DAC_SR_CAL_FLAGx is HIGH, try higher trimming */
			trimmingvalue += delta;
		else
			trimmingvalue -= delta;
		   
		delta >>= 1;
	}
	
	/* Still need to check if right calibration is current value or one step below */
	/* Indeed the first value that causes the DAC_SR_CAL_FLAGx bit to change from 0 to 1  */
	/* Set candidate trimming */
	DAC->CCR &= ~DAC_CCR_OTRIM_Flag;
	DAC->CCR |= ((trimmingvalue<<offset) & DAC_CCR_OTRIM_Flag);
  
	/* tOFFTRIMmax delay x ms as per datasheet (electrical characteristics */ 
	/* i.e. minimum time needed between two calibration steps */
  ms_delay(1);
    
	if ((DAC->SR & DAC_SR_CAL_Flag) == 0) { 
		/* OPAMP_CSR_OUTCAL is actually one value more */
		trimmingvalue++;
		/* Set right trimming */
		DAC->CCR &= ~DAC_CCR_OTRIM_Flag;
		DAC->CCR |= ((trimmingvalue<<offset) & DAC_CCR_OTRIM_Flag);
	}
	
	DAC->CR &= ~DAC_CR_CEN_Flag; 
}

//*************************************************************************
// DAC Configuration
// Turn on the DAC clocks, calibrate DAC2, set DAC2 to drive PA5 via the DAC
// buffer, and use software triggering.
//*************************************************************************
static void DAC_Configuration(void){
    // APB1 Peripheral clock Enable Register 1 (APB1ENR1)
    // It looks like this one bit enables the clock for both DACs.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    DAC_Calibration_Channel(2);  // Calibrate DAC Channel 2

    // DAC mode control register (DAC_MCR). Each of the two DACs has three bits
    // of mode. We set 000, or DAC2 driving its external pin (PA5) via a buffer.
    // The buffer allows higher drive current.
    // This value of 000 also turns off sample-and-hold mode.
    DAC->MCR &= ~DAC_MCR_MODE2;

    // DAC channel2 trigger enable. Without this, DAC2 cannot trigger (I'm not
    // sure why you would ever not set it)
    DAC->CR |=  DAC_CR_TEN2;       // Trigger enable 

    // In the same register, a 3-bit field of which trigger is used. The choice
    // is various timers, an external pin, and (what we use) software trigger.
    DAC->CR |=  DAC_CR_TSEL2;     // Software Trigger

    // Same register again: enable DAC #2.
    DAC->CR |=  DAC_CR_EN2;       // Enable DAC Channel 2

    ms_delay(1);
}

//*************************************************************************
// DAC output pin Initialization
// Set GPIO pin PA5 to be an analog output, This is the output pin that DAC #2
// uses. (Note that DAC #1 uses PA4, which the 476 Discovery Board doesn't hook
// up).
//*************************************************************************
static void DAC_Pin_Configuration(void){
    // Enable the clock of GPIO Port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA5 (DAC1_OUT2) as Analog
    GPIOA->MODER |=   3U<<(2*5);  // 2 bits of mode per pin; 11b = Analog
    // GPIO port pup/pulldown register. It has 2 bits per pin, and we set 
    // 00=>No pull-up or pull-down (after all, it's an analog output).
    GPIOA->PUPDR &= ~(3U<<(2*5));

    // GPIO port analog switch control register (ASCR). One bit/pin.
    // 0: Disconnect analog switch to the ADC input (reset state)
    // 1: Connect analog switch to the ADC input
    // We're not using the ADC, so no need to connect the pin (as an input) to
    // the ADC.
    GPIOA->ASCR |= GPIO_ASCR_EN_5;
}

//*************************************************************************
// DAC Initialization
//*************************************************************************
void DAC1_Init(void){
    // Op-amp1 takes its + input from DAC 1, out 1 and drives PA3.
    // According to the schematic, PA3 does come to a pin; but it's JOY_UP,
    // which has an R52*C41 connection to ground via 100nF and 0ohms(!). We'll
    // nonetheless use it, because op-amp2 doesn't come to a pin at all.
    // Set GPIO pin PA3 (the op-amp-1 output) to be an analog output.

    // Enable the clock of GPIO Port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Configure PA2 as analog
    GPIOA->MODER |=   3U<<(2*3);  // 2 bits of mode per pin; 11b = Analog
    // GPIO port pup/pulldown register. It has 2 bits per pin, and we set 
    // 00=>No pull-up or pull-down (after all, it's an analog output).
    GPIOA->PUPDR &= ~(3U<<(2*3));

    // GPIO port analog switch control register (ASCR). One bit/pin.
    // 0: Disconnect analog switch to the ADC input (reset state)
    // 1: Connect analog switch to the ADC input
    // We're not using the ADC, so no need to connect the pin (as an input) to
    // the ADC.
    //GPIOA->ASCR |= GPIO_ASCR_EN_2;	// Seems backwards!

    // Now for the DAC.
    // APB1 Peripheral clock Enable Register 1 (APB1ENR1)
    // It looks like this one bit enables the clock for both DACs.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // DAC mode control register (DAC_MCR). Each of the two DACs has three bits
    // of mode. We set 011, or DAC driving internal peripherals but not an
    // external pin, and no buffer.
    DAC->MCR &= ~DAC_MCR_MODE1;
    DAC->MCR |=  DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0;

    // DAC channel1 trigger enable. Without this, DAC1 cannot trigger (I'm not
    // sure why you would ever not set it)
    DAC->CR |=  DAC_CR_TEN1;       // Trigger enable 

    // In the same register, a 3-bit field of which trigger is used. The choice
    // is various timers, an external pin, and (what we use) software trigger.
    DAC->CR |=  DAC_CR_TSEL1;     // Software Trigger (code 111)

    // Same register again: finally, enable DAC #1.
    DAC->CR |=  DAC_CR_EN1;       // Enable DAC Channel 2
    ms_delay(1);

    // Turn on clocks to the op-amp CSRs (the op-amp itself doesn't need this).
    RCC->APB1ENR1 |= RCC_APB1ENR1_OPAMPEN;

    // OPAMP1 control/status register (OPAMP1_CSR)
    // DAC drives op-amp1's Vin+ (VP_sel (bit #10)=1).
    OPAMP1->CSR |= OPAMP1_CSR_VPSEL;

    // OpaMode (bits 3:2)=11 (Set it to be a unity-gain voltage follower)
    OPAMP1->CSR |= OPAMP1_CSR_OPAMODE;

    // OpaEn (bit #0)=1: op-amp enabled (done last)
    OPAMP1->CSR |= OPAMP1_CSR_OPAEN;
}

void DAC2_Init(void){
    // Set GPIO pin A5 (the DAC-2 output) to be an analog output.
    DAC_Pin_Configuration();

    // Set up DAC2 to drive PA5 via a buffer, and use software triggering.
    DAC_Configuration();
}

// Write 12-bit unsigned data to DAC 1, which drives PA3.
void DAC1_write (uint32_t data) {
    // 12-bit right-aligned holding register.
    DAC->DHR12R1 = data;

    // DAC software trigger register (DAC_SWTRGR)
    // We already wrote data into the Holding Register. But it doesn't
    // affect the DAC until we now write DAC_SWTRGR.
    // It has two bits (one for each channel). We write the appropriate bit
    // to 1, and the HW clears the bit once the transfer has happened.
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

// Write 12-bit unsigned data to DAC 2, which drives pin PA5.
void DAC2_write (uint32_t data) {
    DAC->DHR12R2 = data;
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
}
