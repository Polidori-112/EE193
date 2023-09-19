// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "UART.h"
#include "lib_ee115.h"

bool do_blink_red=1, do_blink_grn=1;

#define BLINK_RED_DELAY ( 500 / portTICK_PERIOD_MS )
// Keep blinking as long as do_blink_red==true.
void task_blink_red( void * pvParameters )
{
    // Most tasks have an infinite loop in them.
    // for ( ;; ) {
	// if do_blink_red==true, then blink the red LED;
	// use vTaskDelay to pause for some time;
    // }
}

// Keep blinking as long as do_blink_grn==true.
#define BLINK_GRN_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_grn( void * pvParameters )
{
    for ( ;; ) {
	//... just like the red task, but now for the green LED;
    }
}

void task_uart (void *pvParameters) {
    const char prompt[] = "R=red, G=green, B=both, N=neither: ";
    char rxByte, buf[40];
    while (1) {
	USART_Write(USART2, (uint8_t *)prompt);
	rxByte = USART_Read (USART2);
	int red  = (rxByte == 'R' || rxByte == 'r');
	int grn  = (rxByte == 'G' || rxByte == 'g');
	int both = (rxByte == 'B' || rxByte == 'b');
	do_blink_red = red || both;
	do_blink_grn = grn || both;
	strcpy (buf, "Red=");   strcat (buf, (do_blink_red?"on":"off"));
	strcat (buf, ", grn=");	strcat (buf, (do_blink_grn?"on\n\r":"off\n\r"));
	USART_Write(USART2, (uint8_t *)buf);
    }
}

int main() {
    // The UART only works at 80MHz. Not really sure why...
    //clock_setup_16MHz();		// 16 MHz
    clock_setup_80MHz();		// 80 MHz
    UART2_Init();

    // Setup for the red LED (GPIO port B, pin 2) and green (port E, pin 8).
    init_red_LED();			// Set up the red LED.
    init_grn_LED();			// Set up the grn LED.

    // Create tasks.
    TaskHandle_t task_handle_red = NULL;
    BaseType_t task_red_OK = xTaskCreate (
	    task_blink_red,
	    "Blink Red LED",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+2, // priority
	    &task_handle_red);
    if (task_red_OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_grn = NULL;
    BaseType_t task_grn_OK = xTaskCreate (
	    task_blink_grn,
	    "Blink Green LED",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+2, // priority
	    &task_handle_grn);
    if (task_grn_OK != pdPASS) for ( ;; );

/*
    UART task is commented out at first...

    TaskHandle_t task_handle_uart = NULL;
    BaseType_t task_uart_OK = xTaskCreate (
	    task_uart,
	    "Decide which LEDs to blink",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+???, // what priority, vs. the LED tasks?
	    &task_handle_uart);
    if (task_uart_OK != pdPASS) for ( ;; );
*/

    vTaskStartScheduler();
}
