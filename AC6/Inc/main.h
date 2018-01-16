/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Audio_In_L_Pin GPIO_PIN_0
#define Audio_In_L_GPIO_Port GPIOA
#define Audio_In_R_Pin GPIO_PIN_1
#define Audio_In_R_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define LED1_SET()    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_RESET()  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED2_SET()    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_RESET()  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED3_SET()    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_RESET()  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED4_SET()    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED4_RESET()  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)

// extern FlagStatus KeyPressed;   // detect button presses

void flash(int time);

// for sampling
#define DEFAULT_BLOCKSIZE 100	//!< Default # samples per block of streamed ADC/DAC data
extern uint32_t ADC_Block_Size;	 //!< Number of samples user accesses per data block
extern uint32_t ADC_Buffer_Size; //!< Total buffer size being filled by DMA for ADC/DAC
extern volatile int Lower_Ready; //!< Flag to indicate which half of ADC buffer may be processed
extern volatile uint32_t *ADC_Input_Buffer;
extern volatile uint32_t *DAC_Output_Buffer;
enum Processor_Task {
  STARTUP,               //!< User is not ready for data yet.
  PROCESS_BUFFER,	//!< User is working on a buffer of data
  WAIT_FOR_NEXT_BUFFER	//!< User is done working... waiting for the next buffer
};
extern enum Processor_Task volatile Sampler_Status;
int getblocksize(void);
void setblocksize(uint32_t blksiz);
void getblock(float * working);
void getblockstereo(float * chan1, float * chan2);
void putblock(float * working);
void putblockstereo(float * chan1, float * chan2);
static volatile uint32_t * inbuf;
static volatile uint32_t * outbuf;
enum Num_Channels_In {
  MONO_IN,		//!< Mono Input: Only configure ADC1, single DMA Transfer
  STEREO_IN		//!< Stereo Input: Configure ADC1 and ADC2, dual DMA Transfer
};
enum Num_Channels_Out {
  MONO_OUT,		//!< Mono Output: Only configure DAC1, single DMA Transfer
  STEREO_OUT		//!< Stereo Output: Configure DAC1 and DAC2, dual DMA Transfer
};
extern enum Num_Channels_Out Output_Configuration;
extern enum Num_Channels_In Input_Configuration;
// void initialize(uint16_t timer_count_value, enum Num_Channels_In chanin, enum Num_Channels_Out chanout, enum Clock_Reference clkref);
// float getsamplingfrequency(void);

// error handling
#define ERRORBUFLEN 100		// number of errors to record in a circular buffer
#define SAMPLE_OVERRUN 2		// ADC buffer filled before the user serviced the buffer
#define MEMORY_ALLOCATION_ERROR 3	// malloc() or calloc() returned NULL
#define DAC_CONFIG_ERROR 4
#define ADC_CONFIG_ERROR 5
#define SETBLOCKSIZE_ERROR 6		// setblocksize() must be called BEFORE initialize()
#define UART_CONFIG_ERROR 7
#define CLOCK_CONFIG_ERROR 8
static void print_error(int index);
extern int errorbuf[];
extern int erroridx;
void initerror();
void flagerror(int errorcode);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
