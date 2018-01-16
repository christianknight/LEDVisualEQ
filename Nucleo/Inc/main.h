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

//#include "stm32l476g_discovery.h"
//#include "ece486.h"
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOC
#define AUDIO_IN_R_Pin GPIO_PIN_0
#define AUDIO_IN_R_GPIO_Port GPIOA
#define AUDIO_IN_L_Pin GPIO_PIN_1
#define AUDIO_IN_L_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* Defines */
#define LO_SET()    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define LO_RESET()  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define LO_MID_SET()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LO_MID_RESET()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define MID_HI_SET()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define MID_HI_RESET()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define HI_SET()    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define HI_RESET()  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)

#define DEFAULT_BLOCKSIZE 100	//!< Default # samples per block of streamed ADC/DAC data

#define ERROR_LED LED4		//!< Red LED, STM32L476G-Discovery
#define NORMAL_LED LED5		//!< Green LED, STM32L476G-Discovery

/* Variable declarations */
extern FlagStatus KeyPressed;   // detect button presses

// global data buffers which are being filled/emptied by the DMAs
volatile uint32_t *ADC_Input_Buffer=NULL;
volatile uint32_t *DAC_Output_Buffer=NULL;

// data block sizes for streamed ADC/DAC data
uint32_t ADC_Block_Size = DEFAULT_BLOCKSIZE;	//!< Number of samples user accesses per data block
uint32_t ADC_Buffer_Size = 2*DEFAULT_BLOCKSIZE; //!< Total buffer size being filled by DMA for ADC/DAC

// pointers to the half-buffer which should be worked upon
static volatile uint32_t * inbuf;
static volatile uint32_t * outbuf;

enum Processor_Task volatile Sampler_Status; // indicates whether user is finished working on buffer or not
volatile int Lower_Ready = 0;	// set by ISR to indicate which buffer is available

enum Processor_Task {
	STARTUP,               //!< User is not ready for data yet.
	PROCESS_BUFFER,	//!< User is working on a buffer of data
	WAIT_FOR_NEXT_BUFFER	//!< User is done working... waiting for the next buffer
};

uint32_t nsamp;	// number of samples per block
float32_t mean;	// for storing average value

// buffers for input block and filtered output blocks
float32_t *input, *output_lo, *output_lo_mid,
	*output_mid_hi, *output_hi;

// buffers to hold previous filter samples
float32_t pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid],
	pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];

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

/* Function prototypes */
void flash(int time);	// LED test flash routine
void flagerror(void);

int getblocksize(void);
void setblocksize(uint32_t blksiz);
void getblock(float * working);
void getblockstereo(float * chan1, float * chan2);
void putblock(float * working);
void putblockstereo(float * chan1, float * chan2);

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
