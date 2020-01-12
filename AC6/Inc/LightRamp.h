/****************************************************
************      	  LightRamp.h        ************
************		Christian Knight	  ***********
************      	     v0.0.1        **************
****************************************************/

#ifndef __LIGHTRAMP_H
#define __LIGHTRAMP_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>


/* Defines */
// LED control
#define LED1	TIM_CHANNEL_1
#define LED2	TIM_CHANNEL_2
#define LED3	TIM_CHANNEL_3
#define LED4	TIM_CHANNEL_4
#define NUM_LEDS 4
uint32_t LED[NUM_LEDS];
#define H_LED_TIM &htim2
#define H_ADC_TIM &htim3
#define H_ADC &hadc1

// Error handling
#define ERRORBUFLEN 100		// number of errors to record in a circular buffer
#define SAMPLE_OVERRUN 2		// ADC buffer filled before the user serviced the buffer
#define MEMORY_ALLOCATION_ERROR 3	// malloc() or calloc() returned NULL
#define DAC_CONFIG_ERROR 4
#define ADC_CONFIG_ERROR 5
#define SETBLOCKSIZE_ERROR 6		// setblocksize() must be called BEFORE initialize()
#define UART_CONFIG_ERROR 7
#define CLOCK_CONFIG_ERROR 8

// Sampling
#define DEFAULT_BLOCKSIZE 100	// default # samples per block of streamed ADC/DAC data

// Filtering
#define SECTIONS_LO 2
#define SECTIONS_LO_MID 6
#define SECTIONS_MID_HI 6
#define SECTIONS_HI 4
#define COEFS_PER_SECTION 5

/* Function declarations */
void filt_init (arm_biquad_cascade_df2T_instance_f32 * filt, int sects, float * coefs, float32_t * pstate);
void do_filter (arm_biquad_cascade_df2T_instance_f32 * filt, float32_t * input, float32_t * output, uint32_t len);
void do_scale  (float32_t * input, float32_t scale, float32_t * output, uint32_t len);
void do_abs    (float32_t * input, float32_t * output, uint32_t len);
void do_offset (float32_t * input, float32_t offset, float32_t * output, uint32_t len);
void do_mean   (float32_t * input, uint32_t len, float32_t * mean_val);

void initerror();
void flagerror(int);
void breathing(uint8_t);
void adjust_brightness(uint32_t, float32_t);
void pulse(uint32_t, float32_t, uint8_t);
void LightRamp_init(void);


/* Variable declarations */
// Sampling
extern uint32_t ADC_Block_Size;	 // Number of samples user accesses per data block
extern uint32_t ADC_Buffer_Size; // Total buffer size being filled by DMA for ADC/DAC
extern volatile int Lower_Ready; // Flag to indicate which half of ADC buffer may be processed
extern volatile uint32_t *ADC_Input_Buffer;
extern volatile uint32_t *DAC_Output_Buffer;
enum Processor_Task {
  STARTUP,               // User is not ready for data yet.
  PROCESS_BUFFER,	// User is working on a buffer of data
  WAIT_FOR_NEXT_BUFFER	// User is done working... waiting for the next buffer
};
extern enum Processor_Task volatile Sampler_Status;
int getblocksize(void);
void setblocksize(uint32_t);
void getblock(float * working);
void getblockstereo(float * chan1, float * chan2);
void putblock(float * working);
void putblockstereo(float * chan1, float * chan2);
static volatile uint32_t * inbuf;
static volatile uint32_t * outbuf;
enum Num_Channels_In {
  MONO_IN,		// Mono Input: Only configure ADC1, single DMA Transfer
  STEREO_IN		// Stereo Input: Configure ADC1 and ADC2, dual DMA Transfer
};
enum Num_Channels_Out {
  MONO_OUT,		// Mono Output: Only configure DAC1, single DMA Transfer
  STEREO_OUT		// Stereo Output: Configure DAC1 and DAC2, dual DMA Transfer
};
extern enum Num_Channels_Out Output_Configuration;
extern enum Num_Channels_In Input_Configuration;
__IO FlagStatus KeyPressed;
uint32_t nsamp;	// number of samples per block

// Error Handling
static void print_error(int index);
extern int errorbuf[];
extern int erroridx;

// Buffers to hold previous filter samples
float32_t pstate_lo[2*SECTIONS_LO], pstate_lo_mid[2*SECTIONS_LO_MID],
	pstate_mid_hi[2*SECTIONS_MID_HI], pstate_hi[2*SECTIONS_HI];

// Filtering
arm_biquad_cascade_df2T_instance_f32 filter_lo, filter_lo_mid, filter_mid_hi, filter_hi;
int sections_lo, sections_lo_mid, sections_mid_hi, sections_hi;
float coefs_lo[SECTIONS_LO * COEFS_PER_SECTION], coefs_lo_mid[SECTIONS_LO_MID * COEFS_PER_SECTION],
	coefs_mid_hi[SECTIONS_MID_HI * COEFS_PER_SECTION], coefs_hi[SECTIONS_HI * COEFS_PER_SECTION];
float32_t mean_lo, mean_lo_mid, mean_mid_hi, mean_hi;	// for storing average value

#endif /* __LIGHTRAMP_H */
