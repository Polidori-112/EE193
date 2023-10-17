#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "ADC_DAC.h"
#include "lib_ee115.h"

#include "stm32l4xx.h"
#include <stdint.h> // ???
#include <stdbool.h> // ???


float history0 = 0.0f;
int lerpfilter (float *history, int sample, float weight) {
    float x = sample / (float)(1 << 12);	// fixed -> float
    *history = *history + weight*(x - *history);// interpolate
    return *history * (1 << 12);		// float -> fixed
}

#define READ_WRITE_DELAY ( 1 / portTICK_PERIOD_MS ) // sample at 500 Hz
void task_read_adc_filter_write_dac( void * pvParameters ) {
    TickType_t lastwake = xTaskGetTickCount();

    for ( ;; ) {
	// Read ADC.
	uint32_t sample = ADC1_read();
	sample *= 2;
	sample = lerpfilter( &history0, sample, 0.1f );

	// Write to DAC.
	DAC2_write (sample);

	vTaskDelay( READ_WRITE_DELAY );
    }
}

#define BLINK_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink (void *pvParameters)
{
    for ( ;; ) {
	toggle_grn_LED();
	toggle_red_LED();
	vTaskDelay( BLINK_DELAY );
    }
}

int main() {
    //clock_setup_16MHz();
    clock_setup_80MHz();
    
    // Set up GPIO.
    GPIO_set_output (GPIOB, 2);	// Red is B2
    GPIO_set_output (GPIOE, 8);	// Grn is E8

    //ADC_Init();
    DAC2_Init();
    //set up ADC
    ADC_Init();

    // Create tasks.
    TaskHandle_t task_handle_LEDs = NULL;
    BaseType_t task_create_OK = xTaskCreate (
	task_blink, "Blink LEDs",
	100, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY+1, // priority
	&task_handle_LEDs);
    if (task_create_OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_read_write = NULL;
    BaseType_t task_create_read_write_OK = xTaskCreate (
	task_read_adc_filter_write_dac,	"Read ADC; filter; write to dac",
	256, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY, // priority
	&task_handle_read_write);
    if (task_create_read_write_OK != pdPASS) for ( ;; );

    vTaskStartScheduler();
}
