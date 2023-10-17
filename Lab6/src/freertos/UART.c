#include "UART.h"

static void UART2_GPIO_Init(void);
static void USART_Init (USART_TypeDef * USARTx);
static void USART_Delay(uint32_t us);

/* Flags:
ISR bit 11 = Rx timeout
ISR bit 4 = Idle line
ISR bit 3 = Overrun
ISR bit 1 = Framing error
 */

// UART Ports:
// ===================================================
// PA.0 = UART4_TX (AF8)   |  PA.1 = UART4_RX (AF8)      
// PB.6 = USART1_TX (AF7)  |  PB.7 = USART1_RX (AF7) 
// PD.5 = USART2_TX (AF7)  |  PD.6 = USART2_RX (AF7)

void UART2_Init(void) {
    // Enable the clock of USART 1 & 2
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;  // Enable USART 2 clock

    // Select SYSCLK as the USART1 clock source. The reset default is PCLK1;
    // we usually set both SYSCLK and PCLK1 to 80MHz anyway.
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
    RCC->CCIPR |=  RCC_CCIPR_USART2SEL_0;

    UART2_GPIO_Init();
    USART_Init(USART2);
}

// The 476 Disco wires D5 to the ST-Link's UART_TX pin, and D6 to UART_RX.
// In this function, we set up D5 and D6 appropriately.
// Both these need to be alternate function 7 (which are the codes for D5 to
// be USART2_TX and D6 to be USART2_RX). This presumably sets most on the GPIO
// pin's internals. However, we still set them to high-speed, pullup-only,
// push-pull drive.
static void UART2_GPIO_Init(void) {
    // Enable the clock for GPIO Port D
    RCC->AHB2ENR |=   RCC_AHB2ENR_GPIODEN;

    // ********************** USART 2 ***************************
    // PD5 = USART2_TX (AF7)
    // PD6 = USART2_RX (AF7)
    // Alternate function, High Speed, Push pull, Pull up
    // **********************************************************

    // Set both D5 and D6 to alternate-function. We, trickily, set the four-
    // bit field that covers both D5 and D6 (two bits each).
    GPIOD->MODER   &= ~(0xFU << (2*5));	// Clear bits
    GPIOD->MODER   |=   0xA << (2*5);      	// Set each to 10 (AF).

    // AFR[0] is the alternate-function register for pins 0-7. With the same
    // trick as above, set both D5 and D6 to b0111 (for AF7).
    GPIOD->AFR[0]  |=   0x77<< (4*5);       	

    // Set D5 and D6 to high-speed. This changes the output slew rate.
    GPIOD->OSPEEDR |=   0xF<<(2*5);

    // Both D5 and D6 are in pullup/down mode 01, which means pull-up only.
    GPIOD->PUPDR   &= ~(0xFU<<(2*5));
    GPIOD->PUPDR   |=   0x5<<(2*5);    				

    // Both D5 and D6 are push-pull (which is the reset default, anyway).
    GPIOD->OTYPER  &=  ~(0x3U<<5) ;       	
}


// Set for 8 data bits, 1 start & 1 stop bit, 16x oversampling, 9600 baud.
// And by default, we also get no parity, no hardware flow control (USART_CR3),
// asynch mode (USART_CR2).
// UART4 gets special treatment, since it's UART and not USART.
// UART2 gets special treatment (enabling DMA?).
static void USART_Init (USART_TypeDef * USARTx) {
    // Disable the USART.
    USARTx->CR1 &= ~USART_CR1_UE;  // Disable USART

    // The "M" field is two bits, M1 and M0. We're setting it to 00 (which
    // is the reset value anyway), to use 8-bit words and one start bit.
    USARTx->CR1 &= ~USART_CR1_M;

    // Configure oversampling mode: Oversampling by 16 (which is the
    // default). This means that our Rx runs at 16x the nominal baud rate.
    USARTx->CR1 &= ~USART_CR1_OVER8;

    // Configure stop bits to 1 stop bit (which is the default). Other
    // choices are .5, 1.5 and 2 stop bits.
    USARTx->CR2 &= ~USART_CR2_STOP;   

    // Set baudrate=9600. This is done by dividing down the APB1 clock.
    // So, 80MHz/9600 = 8333 = 0x208D.
    // (We're oversampling by 16; the calculation would be slightly
    // different if we were 8x mode).
    USARTx->BRR  = 0x208D;

    // Turn on transmitter and receiver enables. Note that the entire USART
    // is still disabled, though. Turning on the Rx enable kicks off the Rx
    // looking for a stop bit.
    USARTx->CR1  |= (USART_CR1_RE | USART_CR1_TE);
	
    // Special case UART4, which is a UART and not a USART.
    if (USARTx == UART4){	
	// Received Data Ready to be Read Interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;
	// Transmission Complete Interrupt
	USARTx->CR1 &= ~USART_CR1_TCIE;
	// Idle Line Detected Interrupt 
	USARTx->CR1 &= ~USART_CR1_IDLEIE;
	// Transmit Data Register Empty Interrupt 
	USARTx->CR1 &= ~USART_CR1_TXEIE;
	// Parity Error Interrupt 
	USARTx->CR1 &= ~USART_CR1_PEIE;
	// LIN Break Detection Interrupt Enable
	USARTx->CR1 &= ~USART_CR2_LBDIE;
	// Error Interrupt Enable (Frame error, noise error, overrun error) 
	USARTx->CR1 &= ~USART_CR3_EIE;
	//USARTx->CR3 &= ~USART_CR3_CTSIE;	// CTS Interrupt
    }

    // USART2 is special for some reason: enable DMA mode.
    if (USARTx == USART2){
	// Clear the transmit-complete flag in the interrupt-and-status reg.
	USARTx->ICR |= USART_ICR_TCCF;
	// Enable DMA mode for both transmit and receive.
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
    }

    // We originally turned off the USART -- now turn it back on.
    USARTx->CR1  |= USART_CR1_UE; // USART enable                 
	
    // Verify that the USART is ready to transmit...
    while ( (USARTx->ISR & USART_ISR_TEACK) == 0)
	;
    // ... and to receive.
    while ( (USARTx->ISR & USART_ISR_REACK) == 0)
	;
}


// Very basic function: just spin wait, checking the data-available bit until
// we have data. Then return one byte.
uint8_t USART_Read (USART_TypeDef * USARTx) {
    // The SR_RXNE (Read data register not empty) bit is set by hardware.
    // We spin wait until that bit is set
    while (!(USARTx->ISR & USART_ISR_RXNE))
	;

    // Reading USART_DR automatically clears the RXNE flag 
    return ((uint8_t)(USARTx->RDR & 0xFF));
}

// Very basic function: just give nBytes bytes to the UART, one byte at a time.
// Spin wait after each byte until the UART is ready for the next byte.
void USART_Write(USART_TypeDef *USARTx, const uint8_t *buffer) {
    // The main flag we use is Tx Empty (TXE). The HW sets it when the
    // transmit data register (TDR) is ready for more data. TXE is then
    // cleared when we write new data in (by a write to the USART_DR reg).
    // When the HW transfers the TDR into the shift register, it sets TXE=1.
    for (unsigned int i = 0; buffer[i] != '\0'; i++) {
	// wait until TXE (TX empty) bit is set
	while (!(USARTx->ISR & USART_ISR_TXE));
	// Writing USART_DR automatically clears the TXE flag 	
	USARTx->TDR = buffer[i] & 0xFF;
	// Wait 300us or so, to let the HW clear TXE.
	USART_Delay(300);
    }
    // wait until TC bit is set 
    while (!(USARTx->ISR & USART_ISR_TC));
    USARTx->ISR &= ~USART_ISR_TC;
}   
 

// Assume that each usec of delay is about 13 times around the NOP loop.
// That's probably about right at 80 MHz (maybe a bit too slow).
static void USART_Delay(uint32_t us) {
    uint32_t time = 100*us/7;    
    while(--time);   
}
