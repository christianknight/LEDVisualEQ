/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	nsamp = 20;		// number of samples per block
	setblocksize(nsamp);		// set number of samples per block
	nsamp = getblocksize();	// get back blocksize

	const float offset = -0.99;	// DC offset to add to filtered block
	// thresholds for turning LEDs on/off
	float thresh_lo = -0.6;
	float thresh_lo_mid = -0.65;
	float thresh_mid_hi = -0.8;
	float thresh_hi = -0.8;
	// scaling factors for each filter
	float scale_input = 1;
	float scale_lo = 3;
	float scale_lo_mid = 3;
	float scale_mid_hi = 4;
	float scale_hi = 5;
	// float increment = 1.3;

	// set up filter structures
	arm_biquad_cascade_df2T_instance_f32 filter_lo;
	arm_biquad_cascade_df2T_init_f32(&filter_lo,sections_lo,coefs_lo,pstate_lo);
	arm_biquad_cascade_df2T_instance_f32 filter_lo_mid;
	arm_biquad_cascade_df2T_init_f32(&filter_lo_mid,sections_lo_mid,coefs_lo_mid,pstate_lo_mid);
	arm_biquad_cascade_df2T_instance_f32 filter_mid_hi;
	arm_biquad_cascade_df2T_init_f32(&filter_mid_hi,sections_mid_hi,coefs_mid_hi,pstate_mid_hi);
	arm_biquad_cascade_df2T_instance_f32 filter_hi;
	arm_biquad_cascade_df2T_init_f32(&filter_hi,sections_hi,coefs_hi,pstate_hi);

	// allocate memory for buffers
	input = (float*)malloc(sizeof(float)*nsamp);
	output_lo = (float*)malloc(sizeof(float)*nsamp);
	output_lo_mid = (float*)malloc(sizeof(float)*nsamp);
	output_mid_hi = (float*)malloc(sizeof(float)*nsamp);
	output_hi = (float*)malloc(sizeof(float)*nsamp);

	if (input==NULL || output_lo==NULL || output_lo_mid==NULL || output_mid_hi==NULL || output_hi==NULL)	{
		flagerror(MEMORY_ALLOCATION_ERROR);
		while(1);
	}

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		getblock(input);	// grabs input samples

		if (scale_input > 1)	arm_scale_f32(input,scale_input,input,nsamp);

		// execute each filter
		arm_biquad_cascade_df2T_f32(&filter_lo,input,output_lo,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_lo_mid,input,output_lo_mid,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_mid_hi,input,output_mid_hi,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_hi,input,output_hi,nsamp);

		// scale each filtered block up
		arm_scale_f32(output_lo,scale_lo,output_lo,nsamp);
		arm_scale_f32(output_lo_mid,scale_lo_mid,output_lo_mid,nsamp);
		arm_scale_f32(output_mid_hi,scale_mid_hi,output_mid_hi,nsamp);
		arm_scale_f32(output_hi,scale_hi,output_hi,nsamp);

		// get absolute value of each filtered block
		arm_abs_f32(output_lo,output_lo,nsamp);
		arm_abs_f32(output_lo_mid,output_lo_mid,nsamp);
		arm_abs_f32(output_mid_hi,output_mid_hi,nsamp);
		arm_abs_f32(output_hi,output_hi,nsamp);

		// remove DC offset from filtered block
		arm_offset_f32(output_lo,offset,output_lo,nsamp);
		arm_offset_f32(output_lo_mid,offset,output_lo_mid,nsamp);
		arm_offset_f32(output_mid_hi,offset,output_mid_hi,nsamp);
		arm_offset_f32(output_hi,offset,output_hi,nsamp);

		// get mean of each filtered block and set/clear LEDs
		arm_mean_f32(output_lo,nsamp,&mean);
		if(mean > thresh_lo) LO_SET();
		else LO_RESET();
		arm_mean_f32(output_lo_mid,nsamp,&mean);
		if(mean > thresh_lo_mid) LO_MID_SET();
		else LO_MID_RESET();
		arm_mean_f32(output_mid_hi,nsamp,&mean);
		if(mean > thresh_mid_hi) MID_HI_SET();
		else MID_HI_RESET();
		arm_mean_f32(output_hi,nsamp,&mean);
		if(mean > thresh_hi) HI_SET();
		else HI_RESET();

		if (KeyPressed) {
			KeyPressed = RESET;
			flash_lo(50);
		}

	}

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

// simple function to return block size
int getblocksize()
{
	return ADC_Block_Size;
}

// sets the number of samples that the user should expect to process per block
void setblocksize( uint32_t blksiz )
{
	/*
	 * setblocksize() should only be called before calling initialize().
	 * If the ADC & DAC buffers have already been allocated, then initialize()
	 * must have already been called, and we're too late to change the buffer
	 * sizes.  Flag an error and return without changing anything.
	 */
	if (ADC_Input_Buffer != NULL) {
		flagerror(SETBLOCKSIZE_ERROR);
		return;
	}

	ADC_Block_Size = blksiz;
	ADC_Buffer_Size = 2*blksiz;
}

// get new data block from ADC
void getblock(float * working)
{
	uint32_t i;

	// Wait for the DMA to finish filling a block of data
	Sampler_Status = WAIT_FOR_NEXT_BUFFER;
	while (Sampler_Status == WAIT_FOR_NEXT_BUFFER) __WFI();

	// The DMA ISR sets the Lower_Ready flag to indicate whether we should
	// be processing the upper or lower half of the DMA transfer block.
	if (Lower_Ready) {
		inbuf = ADC_Input_Buffer;
		outbuf = DAC_Output_Buffer;
	} else {
		inbuf = &(ADC_Input_Buffer[ADC_Block_Size]);
		outbuf = &(DAC_Output_Buffer[ADC_Block_Size]);
	}

	// Now convert the valid ADC data into the caller's array of floats.
	// Samples are normalized to range from -1.0 to 1.0
	for (i=0; i< ADC_Block_Size; i++) {
		// 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing)
		working[i] = ((float)((int)inbuf[i]-32767))*3.0517578e-05f;
	}
}

// send output samples to the DAC
void putblock(float * working)
{
	uint32_t i;

	// the "outbuf" pointer is set by getblock() to indicate the
	// appropriate destination of any output samples.
	//
	// floating point values between -1 and +1 are mapped
	// into the range of the DAC
	for (i=0; i<ADC_Block_Size; i++) {
		outbuf[i] = ((int)((working[i]+1.0)*32768.0f)) & 0x0000ffff;
	}
}

void putblockstereo(float * chan1, float * chan2)
{
	uint32_t i;

	if (Output_Configuration == MONO_OUT) {
		putblock( chan1 );
		return;
	}

	// the "outbuf" pointer is set by getblock() to indicate the
	// appropriate destination of any output samples.
	//
	// floating point values between -1 and +1 are mapped
	// into the range of the DAC
	//
	// chan1 goes into the most-significant 16 bits (DAC1),
	// chan2 in the least significant (DAC1)
	for (i=0; i<ADC_Block_Size; i++) {
		outbuf[i] = ( ((int)((chan2[i]+1.0)*32768.0f)) & 0x0000ffff ) |
				((((int)((chan1[i]+1.0)*32768.0f)) & 0x0000ffff)<<16);
	}
}

void getblockstereo(float *chan1, float *chan2)
{
	uint32_t i;

	if (Input_Configuration == MONO_IN) {
		getblock( chan1 );
		return;
	}

	// Wait for the DMA to finish filling a block of data
	Sampler_Status = WAIT_FOR_NEXT_BUFFER;
	while (Sampler_Status == WAIT_FOR_NEXT_BUFFER) __WFI();

	// The DMA ISR sets the Lower_Ready flag to indicate whether we should
	// be processing the upper or lower half of the DMA transfer block.
	if (Lower_Ready) {
		inbuf = ADC_Input_Buffer;
		outbuf = DAC_Output_Buffer;
	} else {
		inbuf = &(ADC_Input_Buffer[ADC_Block_Size]);
		outbuf = &(DAC_Output_Buffer[ADC_Block_Size]);
	}

	// Now convert the valid ADC data into the caller's arrays of floats.
	// Samples are normalized to range from -1.0 to 1.0
	// Channel 1 is in the least significant 16 bits of the DMA transfer data,
	// Channel 2 is in the most significant 16 bits.
	for (i = 0; i< ADC_Block_Size; i++) {
		// 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing)
		chan1[i]=((float)( (int)(inbuf[i]&0x0000ffff)-32767))*3.0517578e-05f;
		chan2[i]=((float)( (int)((inbuf[i]&0xffff0000)>>16)-32767))*3.0517578e-05f;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// The ADC has filled the input buffer, and is beginning to over-write
	// the beginning of the array.  The user should get to work processing the
	// end of the array.
	Lower_Ready = 0;

	if (Sampler_Status == STARTUP) {
		// No need to do anything... user is still initializing
	}
	else if (Sampler_Status == WAIT_FOR_NEXT_BUFFER) {
		Sampler_Status = PROCESS_BUFFER;	// Turn the supervisor loose on the
		// next buffer
	} else {
		flagerror(SAMPLE_OVERRUN);	// If the supervisor was not waiting for the next
		// buffer, flag the error to let him/her know that
		// they're missing blocks of data.
	}

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	// The ADC has half-filled the input buffer, and is beginning to over-write
	// the second-half of the array.  The user should get to work processing the
	// beginning of the array.
	Lower_Ready = 1;

	if (Sampler_Status == STARTUP) {
		// No need to do anything... user is still initializing
	}
	else if (Sampler_Status == WAIT_FOR_NEXT_BUFFER) {
		Sampler_Status = PROCESS_BUFFER;	// Turn the supervisor loose on the
		// next buffer
	} else {
		flagerror(SAMPLE_OVERRUN);	// If the supervisor was not waiting for the next
		// buffer, flag the error to let him/her know that
		// they're missing blocks of data.
	}
}

/* LED test flash routine */
void flash(int time)	{
	while (1)	{
		if (KeyPressed) {
			KeyPressed = RESET;
			time = time / 2;
		}
		LO_SET(), LO_MID_SET(), MID_HI_SET(), HI_SET();
		HAL_Delay(time);
		LO_RESET(), LO_MID_RESET(), MID_HI_RESET(), HI_RESET();
		HAL_Delay(time);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
