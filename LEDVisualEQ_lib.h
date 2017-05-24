//  LEDVisualEQ.c

//	Board: 32L476GDISCOVERY
//	MCU: STM32L476VGT6

//	Header for LEDVisualEQ_lib.c

//  Christian Knight
//  5/2/17

#include "stm32l476g_discovery.h"
#include "ece486.h"
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define FS FS_48K		// sampling rate
#define NSAMP 20		// input block size
#define FFTSIZE 512		// size of FFT

// LED pins referenced 0-3
const short LED[] = {
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
};

const float offset = -0.99;	// DC offset to remove from processed blocks

// thresholds for turning LEDs on/off
const float thresh_in = -0.6;
const float thresh_lo = -0.6;
const float thresh_lo_mid = -0.65;
const float thresh_mid_hi = -0.8;
const float thresh_hi = -0.8;

// scaling factors for each filter
const float scale_in = 1;
const float scale_lo = 3;
const float scale_lo_mid = 3;
const float scale_mid_hi = 4;
const float scale_hi = 5;

typedef struct Channel{
  float scale, thresh, offset, *coefs, sections, mean, *pstate, *output;
  arm_biquad_cascade_df2T_instance_f32 filter;
} Channel;

extern FlagStatus KeyPressed;   // detect button presses

static volatile uint32_t ticks;

// low-band filter coefficients
// Fpass = 60Hz, Fstop = 150Hz, Astop = -60dB, Fs = 48ksps
static int sections_lo = 2;
static float coefs_lo[] = {
	1.258200e-01f, -2.513427e-01f, 1.258200e-01f, 1.994561e+00f, -9.945792e-01f, 
	7.923738e-03f, -1.584405e-02f, 7.923738e-03f, 1.997930e+00f, -9.979912e-01f
};

// low-mid band filter coefficients
// Fstop1 = 200Hz, Fpass1 = 350Hz, Fpass2 = 1kHz, Fstop2 = 1.25kHz, Astop = -60dB, Fs = 48ksps
static int sections_lo_mid = 6;
static float coefs_lo_mid[] = {
	6.824965e-01f, -1.342789e+00f, 6.824965e-01f, 1.968462e+00f, -9.824747e-01f, 
	6.824965e-01f, -1.364243e+00f, 6.824965e-01f, 1.989989e+00f, -9.925206e-01f, 
	4.732228e-01f, -8.784125e-01f, 4.732228e-01f, 1.964619e+00f, -9.731216e-01f, 
	4.732228e-01f, -9.463310e-01f, 4.732228e-01f, 1.977077e+00f, -9.812105e-01f, 
	9.792858e-02f, -1.934950e-01f, 9.792858e-02f, 1.977834e+00f, -9.948572e-01f, 
	9.792858e-02f, -1.957116e-01f, 9.792858e-02f, 1.996086e+00f, -9.981879e-01f
};

// mid-high band filter coefficients
// Fstop1 = 1.6kHz, Fpass1 = 2kHz, Fpass2 = 3kHz, Fstop2 = 3.4kHz, Astop = -60dB, Fs = 48ksps
static int sections_mid_hi = 6;
static float coefs_mid_hi[] = {
	6.821886e-01f, -1.226853e+00f, 6.821886e-01f, 1.838502e+00f, -9.778613e-01f, 
	6.821886e-01f, -1.329568e+00f, 6.821886e-01f, 1.910660e+00f, -9.837985e-01f, 
	4.808136e-01f, -7.363451e-01f, 4.808136e-01f, 1.848419e+00f, -9.628343e-01f, 
	4.808136e-01f, -9.517375e-01f, 4.808136e-01f, 1.879406e+00f, -9.672782e-01f, 
	9.794532e-02f, -1.786521e-01f, 9.794532e-02f, 1.842045e+00f, -9.936369e-01f, 
	9.794532e-02f, -1.901388e-01f, 9.794532e-02f, 1.927611e+00f, -9.956874e-01f
};

// high-band filter coefficients
// Fstop = 4.5kHz, Fpass = 5kHz, Astop = -60dB, Fs = 48ksps
static int sections_hi = 4;
static float coefs_hi[] = {
	1.089714e+00f, -1.856104e+00f, 1.089714e+00f, 1.478079e+00f, -9.268764e-01f, 
	1.989986e+00f, -3.616470e+00f, 1.989986e+00f, 1.155312e+00f, -7.533239e-01f, 
	8.291499e+00f, -1.633572e+01f, 8.291499e+00f, 1.626331e-01f, -2.251002e-01f, 
	1.179967e-02f, -1.957278e-02f, 1.179967e-02f, 1.573169e+00f, -9.835571e-01f
};

void LED_init(void);	// sets up 4 digital output pins on port E to turn LEDs on/off

void LED_write(uint16_t LED, GPIO_PinState state);

void channel_init(Channel C0, Channel C1, Channel C2, Channel C3);

void filt_init(Channel C0, Channel C1, Channel C2, Channel C3);

void do_filter(float *input, Channel C0, Channel C1, Channel C2, Channel C3, short nsamp);

void proc_blocks(Channel C0, Channel C1, Channel C2, Channel C3, short nsamp);

void flash(int time);