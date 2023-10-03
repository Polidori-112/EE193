/*
 * Connections:
 *	Reads an analog signal from pin PA1 using ADC1.
 *	Writes the ADC output to UART #2 (which drives USB to the host).
 *	Writes a canned ECG out to pin PA3 using DAC #1.
 *
 * Usage:
 *	At startup, types "Type the letter 'g' to go"
 *	When they do that, it turns on the green LED and starts sampling.
 *	Typical sample speed is 500 samples/second.
 *	While it is sampling, it's also writing the digital output to USB at
 *	9600 baud.
 *	When sampling is done, the LED changes from solid green to blink at 1Hz.
 *	When printing is done, the LED turns off.
 */

#define N_DATA_SAMPLES 5000	// Take this many samples.
#define SAMPLE_DELAY 2		// Sample every 2 ms (so, 500 samples/sec).

// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "stm32l4xx.h"
#include <stdbool.h>
#include "stm32l476xx.h"
#include "lib_ee115.h"
#include "ADC_DAC.h"
#include "UART.h"

static uint16_t g_data[N_DATA_SAMPLES];
static int g_n_samples_taken=0;

#define TICKS_PER_CANNED_ECG_PT 2	// Assume it was sampled at 500 Hz.
#define ECG_DATA_FILE "ecg_normal_board_calm1_redone.c_data"
static unsigned short int ECG_data[] = {
#include ECG_DATA_FILE
};

// Write a canned ECG out on DAC 1, which drives PA3.
void task_canned_ECG (void * pvParameters) {
    DAC1_Init();
    int n_datapoints = (sizeof ECG_data) / (sizeof (short int));

    int i=0;
    while (1) {
	if (++i == n_datapoints)
	    i = 0;

	unsigned int data = ECG_data[i];
	DAC1_write (data);
	vTaskDelay(TICKS_PER_CANNED_ECG_PT);
    }
}

static int LEDmode=0; 	// 0=off, 1=on, 2=blink
#define BLINK_GREEN_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_green(void *pvParameters) {
    for ( ;; ) {
	if (LEDmode==2)
	    toggle_grn_LED();
	else
	    set_grn_LED (LEDmode);
	vTaskDelay(BLINK_GREEN_DELAY);
    }
}

void task_ADC (void * pvParameters) {
    ADC_Init();	// Read from PA1
    LEDmode=1;	// Solid ON while reading samples.
    while (g_n_samples_taken < N_DATA_SAMPLES) {
	uint32_t sample = ADC1_read ();
	g_data[g_n_samples_taken++] = sample;
	vTaskDelay(SAMPLE_DELAY);
    }
    LEDmode=2;	// Blink when done reading samples.
    while (1)
	vTaskDelay(10);
}

#define MAX_DIGITS 6
static char *int_to_string (int val) {
    static char buf[MAX_DIGITS+1];

    int pos = MAX_DIGITS;	// rightmost position.
    buf [MAX_DIGITS] = '\0';

    while ((val>0) && (--pos >= 0)) {
	int digit = val % 10;
	val /= 10;
	buf[pos] = digit + '0';
    }
    if (pos==MAX_DIGITS)	// Special case: val=0 yields empty string
	buf[--pos]='0';
    return (&buf[pos]);
}

void task_UART_write (void * pvParameters) {
    int n_chars_printed = 0;
    while (1) {
	if (n_chars_printed < g_n_samples_taken) {
	    int sample = g_data[n_chars_printed];
	    USART_Write(USART2, (uint8_t *)int_to_string (sample));
	    USART_Write(USART2, (uint8_t *)"\n\r");
	    ++n_chars_printed;
	} else if (n_chars_printed==N_DATA_SAMPLES)
	    LEDmode=0;	// Turn LED off when all done.
    }
}

int main(void){
    clock_setup_80MHz();		// 80 MHz, AHB and APH1/2 prescale=1x

    UART2_Init();			// To dump our DAC output to the host.

    // Setup for the grn LED (GPIO port B, pin 3)
    init_grn_LED();

    // Wait for a character to be typed before starting.
    USART_Write(USART2, (uint8_t *)"Type the letter 'g' to go\r\n");
    while (USART_Read(USART2) != 'g')
	;

    // Create tasks.
    TaskHandle_t task_handle_green = NULL;
    BaseType_t status = xTaskCreate (
	    task_blink_green, "Blink Green LED",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+2, // priority
	    &task_handle_green);
    if (status != pdPASS)
	for ( ;; );

    TaskHandle_t task_handle_UART = NULL;
    status = xTaskCreate (
	    task_UART_write, "Write data to the UART",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+1, // priority
	    &task_handle_UART);
    if (status != pdPASS)
	for ( ;; );

    TaskHandle_t task_handle_ADC = NULL;
    status = xTaskCreate (
	    task_ADC, "Take ADC samples",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+3, // priority
	    &task_handle_ADC);
    if (status != pdPASS)
	for ( ;; );

    TaskHandle_t task_handle_canned_ECG = NULL;
    status = xTaskCreate (
	task_canned_ECG, "Task to drive a canned ECG out to PA3",
	100, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY+2, // priority
	&task_handle_canned_ECG);
    if (status != pdPASS )
	for ( ;; );

    vTaskStartScheduler();
}
