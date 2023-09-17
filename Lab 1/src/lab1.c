#include "stm32l476xx.h"
#include "lib_ee115.h"

//*********************************************************************
// Lab #1 -- Blinky Disco on bare metal. 
//*********************************************************************

int main(void){
    // The default clock is 4MHz, which is more than fast enough for LEDs.
    // clock_setup_16MHz();		// 16 MHz
    // clock_setup_80MHz();		// 80 MHz

    // Setup for the red LED (GPIO port B, pin 2)
    init_red_LED();			// Set up the red LED.
    set_red_LED (1);			// Set it to initially be on
    init_grn_LED();			// Set up the red LED.
    set_grn_LED (1);			// Set it to initially be on

    // Dead loop & program hangs here

    //challenge 1
    // while(1) {
    //     ms_delay (500);			// spin-wait loop for .5 sec
    //     toggle_red_LED();		// toggle the red LED
    //     toggle_grn_LED();		// toggle the red LED
    // }

    // challenge 2
    // toggle_red_LED();
    // for (int i = 0; i >= 0; i++) {
    //     ms_delay(1);
    //     if (i % 500 == 0) {
    //         toggle_red_LED();
    //         toggle_grn_LED();
    //     }
    // }

    // challenge 3
    for (int i = 0; i >= 0; i++) {
        ms_delay(1);
        if (i % 500 == 0) {
            toggle_red_LED();
        }
        if (i % 333 == 0) {
            toggle_grn_LED();
        }
    }
}
