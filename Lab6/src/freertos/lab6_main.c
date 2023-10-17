// The pin hookup:
//	- analog ECG input data comes in on PA1
//	- drive out a canned ECG signal, if desired, on PA3 (and then jumper
//	  PA3 -> PA1).
//	- drive out debug information on PA5

// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"

#include "stm32l4xx.h"

#include <stdint.h>
#include <stdbool.h>

#include "ADC_DAC.h"
#include "LCD.h"
#include "lib_ee115.h"

static bool dual_QRS = false;
static bool dual_QRS_last = false;

struct biquadcoeffs {
    float b0, b1, b2,	// numerator
	  a0, a1, a2;	// denominator
};

#define N_BIQUAD_SECS 2

static struct biquadcoeffs biquad_20Hz_lowpass [2] = {
	{8.59278969e-05f, 1.71855794e-04f, 8.59278969e-05f,
	 1.0f,		-1.77422345e+00f, 7.96197268e-01f},
	{1.0f,	2.0f,	1.0f,
	 1.0f,	-1.84565849e+00f,	9.11174670e-01f}};

struct biquadstate { float x_nm1, x_nm2, y_nm1, y_nm2; };
static struct biquadstate biquad_state[N_BIQUAD_SECS] = {0};

// Biquad filtering routine.
// - The input is assumed to be a 12-bit unsigned integer coming straight from
//   the ADC. We convert it immediately to a float xn in the range [0,1).
// - Compute yn = b0*xn + b1*x_nm1 + b2*x_nm2 - a1*y_nm1 - a2*y_nm2 (where 
// - Update x_nm1->x_nm2, xn->x_nm1, y_nm1->y_nm2, yn->y_nm1
// - Clamp yn before returning it (again, as a 12-bit integer). Note that the
//   filter state never gets clamped.
int biquad (const struct biquadcoeffs *coeffs, struct biquadstate *state,
	    int sample, int samplebits) {
    float xn = (float)sample / (float)( 1 << samplebits ); // current sample
    float yn = coeffs->b0*xn + coeffs->b1*state->x_nm1 + coeffs->b2*state->x_nm2
	     - coeffs->a1*state->y_nm1 - coeffs->a2*state->y_nm2; // output

    state->x_nm2 = state->x_nm1;
    state->x_nm1 = xn;
    state->y_nm2 = state->y_nm1;
    state->y_nm1 = yn;

    //!!!CHANGED!!! Removed the clipping.
    //if ( yn < 0.0f ) yn = 0.0f;
    //if ( yn > 0.95f ) yn = 0.95f;

    return (yn * (1 << samplebits));
}

// Given an input, just return (input-prev_input); then save the input as
// prev_input to facilitate the next call. It's used as a prelude to finding a
// local peak; i.e., its output always just goes into find_peak().
// Note we keep the previous-input value as a static variable; this only works
// because simple_diff() is only called once!
int simple_diff (int sample) {
    static int prev_sample;
    int diff = sample - prev_sample;
    prev_sample = sample;
    return (diff);
}

// Compute derivative using a 5-point algorithm.
// Given an input 'sample', keep track of its last 5 values, which they call
// xp2, xp1, x0, xm1 and xm2. Then when we get a new sample, compute & return
//	(-xm2 - 2*xm1 + 2*xp1 + xp2)/8
// and then shift to do xm2=xm1, xm1=x0, x0=xp1, xp1=xp2, xp2=sample
// So you can think of this as implementing a differentiator with a delay of
// two time units.
struct deriv_5pt_state {
    int xp2, xp1, x0, xm1, xm2;
};

struct deriv_5pt_state deriv_5pt_state = { 0 };
struct deriv_5pt_state deriv_5pt_state1 = { 0 };
struct deriv_5pt_state deriv_5pt_state2 = { 0 };

int deriv_5pt (int sample, struct deriv_5pt_state * state) {
    int r = -state->xm2 - 2 * state->xm1 + 2 * state->xp1 + state->xp2;
    r /= 8;

    state->xm2 = state->xm1;
    state->xm1 = state->x0;
    state->x0 = state->xp1;
    state->xp1 = state->xp2;
    state->xp2 = sample;

    return r;
}

// Just a running average of the last 100 samples, using a circular buffer.
#define WINDOW_SIZE 100 // samples
int window( int sample ) {
    static int window_cbuf [WINDOW_SIZE] = { 0 };
    static int window_ctail = 0;
    static long window_sum = 0;

    window_sum -= window_cbuf[ window_ctail ];
    window_cbuf[ window_ctail ] = sample;
    window_sum += sample;
    window_ctail = ( window_ctail + 1 ) % WINDOW_SIZE;

    return window_sum / WINDOW_SIZE;
}

struct threshold_state {
    int threshold; // running peak threshold
    int max, min;
    int decay; // amount that max and min thresholds decay each sample
};
struct threshold_state threshold_state_1 = { 0x7FF, 0x000, 0xFFF, 15 };
struct threshold_state threshold_state_2 = { 0x7FF, 0x000, 0x2FF, 4 };

// The moving-threshold algorithm.
// It always keeps a running min & running max. Each time we get a new sample
// (and hence call this function)...
//    - A negative sample is the exception; immediately return the current
//	threshold (which is (min+max)/2).
//    - The new sample goes into the running min & max.
//    -	The max decrements by a fixed delta, and the min increments by the
//	same fixed delta. Clamp the max to never go <0, and the min to never
//	go >0xFFF.
//    - Return (min + max)/2
// Does it really make sense to have max < min??? This algorithm allows that!
// And note that this algorithm is completely different than Pan Tompkins.
int threshold( struct threshold_state * state, int psample ) {
    if ( psample <= 0 ) return state->threshold; // no sample peak

    if ( psample > state->max ) state->max = psample;
    if ( psample < state->min ) state->min = psample;

    // Implement decay.
    state->max -= state->decay;
    if (state->max < 0x000) state->max = 0x000;
    state->min += state->decay;
    if (state->min > 0xFFF) state->min = 0xFFF;

    state->threshold = (state->min + state->max)/2;
    return (state->threshold);
}

struct compute_peak_state {
    struct deriv_5pt_state der5_state;
    int prev_deriv;
};

// Usually return0; but when the input signal hits a peak, then return the
// value of the signal (i.e., of the peak).
int compute_peak (int sample, struct compute_peak_state *state) {
    // First compute the derivative.
    int deriv = deriv_5pt (sample, &state->der5_state);

    // Peak==1 if the current sample fell and the previous one rose. I.e., we
    // just had a peak.
    bool peak = (state->prev_deriv>=0) && (deriv<0);
    state->prev_deriv = deriv; // update derivative history

    // We'll usually return 0, but data if we just had a peak.
    return (peak? sample : 0);
}

#define READ_WRITE_DELAY ( 2 / portTICK_PERIOD_MS ) // sample at 500 Hz
#define REFTIMER_THRESHOLD 100 // 200 ms at 500 Hz
// Schedule this task every 2ms.
void task_read_adc_filter_write_dac (void * pvParameters) {
    int sample_count=0;	// To ignore startup artifacts.
    int refractory_counter = 0;
    struct compute_peak_state peak_state_1, peak_state_2;

    for ( ;; ) {
	vTaskDelay (READ_WRITE_DELAY);

	// Read ADC, using a spin-wait loop.
	uint32_t sample = ADC1_read ();
	sample *= 2;

	// Run it through one or more cascaded biquads.
	int filtered = sample;
	for (int i=0; i<N_BIQUAD_SECS; ++i)
	    filtered = biquad(&biquad_20Hz_lowpass[i],
			      &biquad_state[i], filtered, 12);

	// Left-side analysis
	// Peak_1 is usually 0; but when the bandpass-filtered signal hits a
	// peak, then peak_1 is the bandpass-filtered signal.
	int peak_1   = compute_peak (filtered, &peak_state_1);
	int thresh_1 = threshold (&threshold_state_1, peak_1);

	// Right-side processing
	// Fancy 5-point derivative of the bandpass-filtered signal.
	int deriv_2 = deriv_5pt (filtered, &deriv_5pt_state);
	int deriv_sq_2 = deriv_2 * deriv_2;

	// Running_avg over a 200ms window.
	int avg_200ms_2 = window (deriv_sq_2) * 2;

	// Right-side analysis
	int peak_2 = compute_peak (avg_200ms_2, &peak_state_2);
	if (++sample_count < 250) continue;
	int thresh_2 = threshold (&threshold_state_2, peak_2);

	dual_QRS_last = dual_QRS;
	refractory_counter ++;
	dual_QRS = (filtered > thresh_1) && (avg_200ms_2 > thresh_2)
		&& (refractory_counter > REFTIMER_THRESHOLD);
	if (dual_QRS_last && !dual_QRS) refractory_counter = 0;

	int final = dual_QRS ? 0x7FF : 0x000;
	//int final = dual_QRS;

	// Write to DAC 2, which drives pin PA5.
	DAC2_write (final);
    }
}

#define BLINK_RED_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_red (void *pvParameters) {
    for ( ;; ) {
	toggle_red_LED();
	vTaskDelay( BLINK_RED_DELAY );
    }
}

// 250Hz beep.
#define BEEP_TONE_DELAY ( 4 / portTICK_PERIOD_MS )
void task_beep (void * pvParameters) {
    int val=0;
    for ( ;; ) {
	if (dual_QRS) {
	    set_GPIO_pin (GPIOB, 3, !val);
	    val = !val;
	}
	vTaskDelay( BEEP_TONE_DELAY );
    }
}

uint8_t *fixed_to_string (float f, int before, int after, uint8_t *buf) {
    for (int i=0; i<after; ++i)
	f *= 10;
    int fixed = f + .5;
    int has_dp = (after>0);	// do we have a decimal point?
    int pos = before + after + has_dp;	// rightmost position.
    buf [pos] = '\0';

    while (--pos >= 0) {
	if (has_dp && (pos==before)) {
	    buf[pos]='.';
	    --pos;
	}
	int digit = fixed % 10;
	fixed /= 10;
	buf[pos] = digit + '0';
	if ((digit==0) && (fixed==0) && (pos < before-1))	//If leading zero...
	    buf[pos] = ' ';
    }
    if (fixed > 0)	// Detect and signal overflow.
	buf[0]='!';
    return (buf);
}

void task_displaybpm(void *pvParameters) {
    static TickType_t last_new_beat=0;
    uint8_t buf[7];
    for ( ;; ) {
	if (!dual_QRS_last && dual_QRS) {	// for every *new* heartbeat...
	    TickType_t time = xTaskGetTickCount();
	    // Convert time in milisec to beats/minute.
	    float bpm = 60.0f * 1000.0f / (time - last_new_beat);
	    LCD_DisplayString (fixed_to_string (bpm, 3, 2, buf));
	    last_new_beat = time;
	}
	vTaskDelay (1);
    }
}

#define TICKS_PER_PT 2	// Typically 500 Hz sampling, so TICKS_PER_PT=2
#define ECG_DATA_FILE "ecg_normal_board_calm1_redone.c_data"
static unsigned short int ECG_data[] = {
#include ECG_DATA_FILE
};

// Write a canned ECG out on DAC 1, which drives PA3.
void task_canned_ECG (void * pvParameters) {
    int n_datapoints = (sizeof ECG_data) / (sizeof (short int));

    int i=0;
    while (1) {
	if (++i == n_datapoints)
	    i = 0;

	unsigned int data = ECG_data[i];
	DAC1_write (data);
	vTaskDelay(TICKS_PER_PT);
    }
}

int main()
{
    clock_setup_80MHz();

    init_red_LED();
    init_grn_LED();
    set_grn_LED (0);	// Turn it off.

    // Set up piezo GPIO.
    GPIO_set_output (GPIOB, 3);

    ADC_Init();	// Read from PA1
    DAC1_Init();	// Drives out to pin PA3, generates the input waveform
    DAC2_Init();	// Drives out to pin PA5, debugging any old signal.

    LCD_Initialization();
    uint8_t buf[7];
    LCD_DisplayString (fixed_to_string (62.3, 3, 2, buf));

    // Create tasks.
    TaskHandle_t task_handle_red = NULL;
    BaseType_t task_create_red_OK = xTaskCreate	(
	task_blink_red, "Blink Red LED",
	128, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY+2, // priority
	&task_handle_red);
    if (task_create_red_OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_read_write = NULL;
    BaseType_t task_create_read_write_OK = xTaskCreate (
	task_read_adc_filter_write_dac,
	"Read ADC; filter; write to dac",
	256, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY+1, // priority
	&task_handle_read_write);
    if (task_create_read_write_OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_beep = NULL;
    BaseType_t task_create_beep_OK = xTaskCreate (
	    task_beep, "Beep Piezo Buzzer",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY, // priority
	    &task_handle_beep);
    if ( task_create_beep_OK != pdPASS ) for ( ;; );

    TaskHandle_t task_handle_displaybpm = NULL;
    BaseType_t task_create_displaybpm_OK = xTaskCreate (
	    task_displaybpm, "Display BPM",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY, // priority
	    &task_handle_displaybpm);
    if (task_create_displaybpm_OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_canned_ECG = NULL;
    BaseType_t task_create_canned_ECG_OK = xTaskCreate (
	task_canned_ECG, "Task to drive a canned ECG out to PA3",
	100, // stack size in words
	NULL, // parameter passed into task, e.g. "(void *) 1"
	tskIDLE_PRIORITY+2, // priority
	&task_handle_canned_ECG);
    if ( task_create_canned_ECG_OK != pdPASS )
	for ( ;; );

    vTaskStartScheduler();
}
