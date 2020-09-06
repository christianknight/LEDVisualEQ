/**
 ******************************************************************************
 * @file    lightramp.h
 * @author  Christian Knight
 * @date    08/23/2020
 * @brief   Header file for controlling ADC audio sampling and LED driver
 * PWM control w/ STM32F446 MCU.
 ******************************************************************************
 **/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIGHTRAMP_H
#define __LIGHTRAMP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/*----------------------------------------------------------------------------*/
/* Defines -------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Hardware control */
#define LED1	     TIM_CHANNEL_1
#define LED2	     TIM_CHANNEL_2
#define LED3	     TIM_CHANNEL_3
#define LED4	     TIM_CHANNEL_4
#define NUM_LEDS     4
#define H_LED_TIM    &htim2
#define H_ADC_TIM    &htim3
#define H_ADC        &hadc1

/* Sampling */
#define DEFAULT_BLOCKSIZE    100	/* Default number of samples per block of streamed ADC/DAC data */
#define BLOCKSIZE    DEFAULT_BLOCKSIZE

/* Error handling */
#define ERRORBUFLEN                100    /* Number of errors to record in a circular buffer */
#define SAMPLE_OVERRUN             2      /* ADC buffer filled before the user serviced the buffer */
#define MEMORY_ALLOCATION_ERROR    3      /* malloc() or calloc() returned NULL */
#define DAC_CONFIG_ERROR           4
#define ADC_CONFIG_ERROR           5
#define SETBLOCKSIZE_ERROR         6      /* setblocksize() must be called BEFORE initialize() */
#define UART_CONFIG_ERROR          7
#define CLOCK_CONFIG_ERROR         8

/*----------------------------------------------------------------------------*/
/* Enum definitions ----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
enum num_channels_in {
    MONO_IN,     /* Mono Input: Only configure one ADC, single DMA Transfer */
    STEREO_IN    /* Stereo Input: Configure two ADCs, dual DMA Transfer */
};

enum num_channels_out {
    MONO_OUT,     /* Mono Output: Only configure one DAC, single DMA Transfer */
    STEREO_OUT    /* Stereo Output: Configure two DACs, dual DMA Transfer */
};

enum processor_task {
    STARTUP,                /* User is not ready for data yet */
    PROCESS_BUFFER,         /* User is working on a buffer of data */
    WAIT_FOR_NEXT_BUFFER    /* User is done working... waiting for the next buffer */
};

/*----------------------------------------------------------------------------*/
/* Extern variable declarations ----------------------------------------------*/
/*----------------------------------------------------------------------------*/
extern uint32_t LED[NUM_LEDS];

extern enum num_channels_in input_configuration;
extern enum num_channels_out output_configuration;
extern enum processor_task volatile sampler_status;

extern __IO uint8_t led_enable;

/* Sampling */
extern uint32_t nsamp;    /* Number of samples per block */
extern uint32_t adc_blocksize;     /* Number of samples user accesses per data block */
extern uint32_t adc_buffer_size;    /* Total buffer size being filled by DMA for ADC/DAC */

/* Error Handling */
extern int error_buffer[ERRORBUFLEN];
extern int error_idx;

/*----------------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void lightramp_init(void);
int  getblocksize(void);
void setblocksize(uint32_t);
void getblock(float * working);
void putblock(float * working);
void getblockstereo(float * chan1, float * chan2);
void putblockstereo(float * chan1, float * chan2);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef * hadc);
void initerror(void);
void flagerror(int errorcode);
uint32_t lightramp_calc_gamma(float gamma, uint32_t val_in);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __LIGHTRAMP_H */
