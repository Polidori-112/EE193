// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "stm32l4xx.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "UART.h"
#include "lib_ee115.h"

bool do_blink_red=1, do_blink_grn=1;

#define BLINK_RED_DELAY ( 500 / portTICK_PERIOD_MS )
// Keep blinking forever.
void task_blink_red( void * pvParameters )
{
    for ( ;; ) {
	if (do_blink_red) {
	    toggle_red_LED();
	    vTaskDelay( BLINK_RED_DELAY );
	}
    }
}

// Keep blinking forever.
#define BLINK_GREEN_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_green( void * pvParameters )
{
    for ( ;; ) {
	if (do_blink_grn) {
	    toggle_grn_LED();
	    vTaskDelay( BLINK_GREEN_DELAY );
	}
    }
}


void task_uart (void *pvParameters) {
    const char prompt[] = "R=red, G=green, B=both, N=neither\r\n";
    char rxByte, buf[40];
    while (1) {
	USART_Write(USART2, (const uint8_t *)prompt);
	rxByte = USART_Read(USART2);
	int red  = (rxByte == 'R' || rxByte == 'r');
	int grn  = (rxByte == 'G' || rxByte == 'g');
	int both = (rxByte == 'B' || rxByte == 'b');
	do_blink_red = red || both;
	do_blink_grn = grn || both;
	sprintf (buf, "Red %s, green %s\n\r\n\r", (red||both?"on":"off"),
				  (grn||both?"on":"off"));
	USART_Write(USART2, (uint8_t *)buf);
    }
}

int main() {
    system_clock_init();	// 80 MHz, AHB and APH1/2 prescale=1x
    UART2_Init();

    // Set up GPIO.
    init_red_LED();	// Red is B2
    init_grn_LED();	// Grn is E8

    // Create tasks.
    TaskHandle_t task_handle_red = NULL;
    BaseType_t task_create_red_return = xTaskCreate (
	    task_blink_red,
	    "Blink Red LED",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+2, // priority
	    &task_handle_red);
    if (task_create_red_return != pdPASS) for ( ;; );

    TaskHandle_t task_handle_green = NULL;
    BaseType_t task_create_green_return = xTaskCreate (
	    task_blink_green,
	    "Blink Green LED",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+2, // priority
	    &task_handle_green);
    if (task_create_green_return != pdPASS) for ( ;; );

    TaskHandle_t task_handle_uart = NULL;
    BaseType_t task_create_uart = xTaskCreate (
	    task_uart,
	    "Decide which LEDs to blink",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+1, // priority
	    &task_handle_uart);
    if ( task_create_uart != pdPASS) for ( ;; );

    vTaskStartScheduler();
}
