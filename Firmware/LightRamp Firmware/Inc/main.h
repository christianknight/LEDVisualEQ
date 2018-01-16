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

extern FlagStatus KeyPressed;   // detect button presses

// low-band filter coefficients
const int sections_lo = 3;
const float coefs_lo[] = {
	2.603860e-01f, -5.206777e-01f, 2.603860e-01f, 1.997446e+00f, -9.974836e-01f,
	1.037341e-01f, -2.072098e-01f, 1.037341e-01f, 1.996066e+00f, -9.960752e-01f,
	3.691527e-03f, -7.382281e-03f, 3.691527e-03f, 1.999129e+00f, -9.991905e-01f
};

// low-mid band filter coefficients
const int sections_lo_mid = 6;
const float coefs_lo_mid[] = {
	6.828877e-01f, -1.352992e+00f, 6.828877e-01f, 1.978480e+00f, -9.863217e-01f,
	6.828877e-01f, -1.365400e+00f, 6.828877e-01f, 1.993106e+00f, -9.944085e-01f,
	4.712733e-01f, -9.022227e-01f, 4.712733e-01f, 1.974448e+00f, -9.791039e-01f,
	4.712733e-01f, -9.424909e-01f, 4.712733e-01f, 1.983500e+00f, -9.856781e-01f,
	9.793550e-02f, -1.945244e-01f, 9.793550e-02f, 1.986403e+00f, -9.959891e-01f,
	9.793550e-02f, -1.957976e-01f, 9.793550e-02f, 1.997583e+00f, -9.986558e-01f
};

// mid-high band filter coefficients
const int sections_mid_hi = 6;
const float coefs_mid_hi[] = {
	6.822613e-01f, -1.249698e+00f, 6.822613e-01f, 1.859188e+00f, -9.826378e-01f,
	6.822613e-01f, -1.328025e+00f, 6.822613e-01f, 1.914335e+00f, -9.866131e-01f,
	4.759652e-01f, -7.833861e-01f, 4.759652e-01f, 1.865677e+00f, -9.704703e-01f,
	4.759652e-01f, -9.403309e-01f, 4.759652e-01f, 1.889254e+00f, -9.734410e-01f,
	9.793013e-02f, -1.811671e-01f, 9.793013e-02f, 1.862648e+00f, -9.950364e-01f,
	9.793013e-02f, -1.899658e-01f, 9.793013e-02f, 1.928320e+00f, -9.964095e-01f
};

// high-band filter coefficients
const int sections_hi = 4;
const float coefs_hi[] = {
	1.107652e+00f, -1.971959e+00f, 1.107652e+00f, 1.597598e+00f, -9.353801e-01f,
	2.089015e+00f, -3.897720e+00f, 2.089015e+00f, 1.315506e+00f, -7.766538e-01f,
	9.515451e+00f, -1.882463e+01f, 9.515451e+00f, 3.547976e-01f, -2.410542e-01f,
	1.187221e-02f, -2.073522e-02f, 1.187221e-02f, 1.678874e+00f, -9.855717e-01f
};

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
#define FS_2K 40000 	//!< 2 ksamples/sec
#define FS_4K 20000 	//!< 4 ksamples/sec
#define FS_5K 16000 	//!< 5 ksamples/sec
#define FS_8K 10000 	//!< 8 ksamples/sec
#define FS_10K 8000 	//!< 10 ksamples/sec
#define FS_16K 5000 	//!< 16 ksamples/sec
#define FS_20K 4000 	//!< 20 ksamples/sec
#define FS_24K 3333 	//!< 24.0024 ksamples/sec
#define FS_25K 3200 	//!< 25 ksamples/sec
#define FS_32K 2500 	//!< 32 ksamples/sec
#define FS_40K 2000 	//!< 40 ksamples/sec
#define FS_48K 1667 	//!< 47.9904 ksamples/sec
#define FS_50K 1600 	//!< 50 ksamples/sec
#define FS_64K 1250 	//!< 64 ksamples/sec
#define FS_80K 1000 	//!< 80 ksamples/sec
#define FS_96K 833 	//!< 96.0384 ksamples/sec
#define FS_100K 800 	//!< 100 ksamples/sec
#define FS_125K 640 	//!< 125 ksamples/sec
#define FS_128K 625 	//!< 128 ksamples/sec
#define FS_160K 500 	//!< 160 ksamples/sec
#define FS_200K 400 	//!< 200 ksamples/sec
#define FS_250K 320 	//!< 250 ksamples/sec
#define FS_320K 250 	//!< 320 ksamples/sec
#define FS_400K 200 	//!< 400 ksamples/sec
#define FS_500K 160 	//!< 500 ksamples/sec
#define FS_625K 128 	//!< 625 ksamples/sec
#define FS_640K 125 	//!< 640 ksamples/sec
#define FS_800K 100 	//!< 800 ksamples/sec
#define FS_1000K 80 	//!< 1000 ksamples/sec

enum Num_Channels_In {
  MONO_IN,		//!< Mono Input: Only configure ADC1, single DMA Transfer
  STEREO_IN		//!< Stereo Input: Configure ADC1 and ADC2, dual DMA Transfer
};
enum Num_Channels_Out {
  MONO_OUT,		//!< Mono Output: Only configure DAC1, single DMA Transfer
  STEREO_OUT		//!< Stereo Output: Configure DAC1 and DAC2, dual DMA Transfer
};
enum Clock_Reference {
  MSI_INTERNAL_RC,	//!< Internal MSI RC Oscillator
  HSE_EXTERNAL_8MHz	//!< External 8MHz reference
};
extern enum Num_Channels_Out Output_Configuration;
extern enum Num_Channels_In Input_Configuration;
void initialize(uint16_t timer_count_value, enum Num_Channels_In chanin, enum Num_Channels_Out chanout, enum Clock_Reference clkref);
float getsamplingfrequency(void);

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
