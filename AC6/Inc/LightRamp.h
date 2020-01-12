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

void initerror();
void flagerror(int);
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

#endif /* __LIGHTRAMP_H */
