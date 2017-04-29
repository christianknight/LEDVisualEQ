#include "stm32l476g_discovery.h"

#include "ece486.h"
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define LO_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define LO_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define MID_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define MID_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define HI_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define HI_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)

int main(void)	{

	GPIO_InitTypeDef  GPIO_LO;
	__GPIOE_CLK_ENABLE();                    // Clock on for Port C
	GPIO_LO.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_LO.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_LO.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_LO.Pin = GPIO_PIN_13;              // Set up PE12
	HAL_GPIO_Init(GPIOE, &GPIO_LO);

	GPIO_InitTypeDef  GPIO_MID;
	__GPIOE_CLK_ENABLE();                    // Clock on for Port E
	GPIO_MID.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_MID.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_MID.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_MID.Pin = GPIO_PIN_14;              // Set up PE11
	HAL_GPIO_Init(GPIOE, &GPIO_MID);

	GPIO_InitTypeDef  GPIO_HI;
	__GPIOE_CLK_ENABLE();                    // Clock on for Port E
	GPIO_HI.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_HI.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_HI.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_HI.Pin = GPIO_PIN_15;              // Set up PE10
	HAL_GPIO_Init(GPIOE, &GPIO_HI);

	uint32_t nsamp;
	float32_t *input, *output_lo, *output_mid, *output_hi;

	static int sections_lo = 3;
	static float coefs_lo[] = {
		4.693751e-01f, -9.387229e-01f, 4.693751e-01f, 1.998685e+00f, -9.986981e-01f, 
		2.213580e-01f, -4.426350e-01f, 2.213580e-01f, 1.997611e+00f, -9.976150e-01f, 
		9.606199e-03f, -1.921205e-02f, 9.606199e-03f, 1.999620e+00f, -9.996393e-01f
	};

	static int sections_mid = 6;
	static float coefs_mid[] = {
		6.828028e-01f, -1.354460e+00f, 6.828028e-01f, 1.978259e+00f, -9.831846e-01f, 
		6.828028e-01f, -1.365562e+00f, 6.828028e-01f, 1.996288e+00f, -9.965002e-01f, 
		4.715760e-01f, -9.005567e-01f, 4.715760e-01f, 1.972977e+00f, -9.750623e-01f, 
		4.715760e-01f, -9.431467e-01f, 4.715760e-01f, 1.987298e+00f, -9.877945e-01f, 
		9.793356e-02f, -1.947929e-01f, 9.793356e-02f, 1.988477e+00f, -9.951062e-01f, 
		9.793356e-02f, -1.958577e-01f, 9.793356e-02f, 1.999084e+00f, -9.992421e-01f
	};

	static int sections_hi = 3;
	static float coefs_hi[] = {
		2.050874e+00f, -4.090430e+00f, 2.050874e+00f, 1.920568e+00f, -9.446642e-01f, 
		1.897494e+01f, -3.793318e+01f, 1.897494e+01f, 1.607320e+00f, -6.827026e-01f, 
		1.810025e-02f, -3.604153e-02f, 1.810025e-02f, 1.972934e+00f, -9.894796e-01f
	};

	float32_t pstate_lo[2*sections_lo], pstate_mid[2*sections_mid], pstate_hi[2*sections_hi];

	initialize(FS_200K, MONO_IN, STEREO_OUT);
	nsamp = getblocksize();

	float32_t mean;

	arm_biquad_cascade_df2T_instance_f32 filter_lo;
	arm_biquad_cascade_df2T_init_f32(&filter_lo,sections_lo,coefs_lo,pstate_lo);
	arm_biquad_cascade_df2T_instance_f32 filter_mid;
	arm_biquad_cascade_df2T_init_f32(&filter_mid,sections_mid,coefs_mid,pstate_mid);
	arm_biquad_cascade_df2T_instance_f32 filter_hi;
	arm_biquad_cascade_df2T_init_f32(&filter_hi,sections_hi,coefs_hi,pstate_hi);

	input = (float*)malloc(sizeof(float)*nsamp);
	output_lo = (float*)malloc(sizeof(float)*nsamp);
	output_mid = (float*)malloc(sizeof(float)*nsamp);
	output_hi = (float*)malloc(sizeof(float)*nsamp);
	if (input==NULL || output_lo==NULL || output_mid==NULL || output_hi==NULL) {
		flagerror(MEMORY_ALLOCATION_ERROR);
		while(1);
	}
	
	while(1)	{
		getblock(input);
		DIGITAL_IO_SET();
		arm_biquad_cascade_df2T_f32(&filter_lo,input,output_lo,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_mid,input,output_mid,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_hi,input,output_hi,nsamp);
		arm_scale_f32(output_lo,4,output_lo,nsamp);
		arm_scale_f32(output_mid,4,output_mid,nsamp);
		arm_scale_f32(output_hi,4,output_hi,nsamp);
		arm_abs_f32(output_lo,output_lo,nsamp);
		arm_abs_f32(output_mid,output_mid,nsamp);
		arm_abs_f32(output_hi,output_hi,nsamp);
		arm_offset_f32(output_lo,-0.99,output_lo,nsamp);
		arm_offset_f32(output_mid,-0.99,output_mid,nsamp);
		arm_offset_f32(output_hi,-0.99,output_hi,nsamp);
		arm_mean_f32(output_lo,nsamp,&mean);
		if(mean > -0.33) LO_SET();
		else LO_RESET();
		arm_mean_f32(output_mid,nsamp,&mean);
		if(mean > -0.71) MID_SET();
		else MID_RESET();
		arm_mean_f32(output_hi,nsamp,&mean);
		if(mean > -0.87) HI_SET();
		else HI_RESET();
		DIGITAL_IO_RESET();
	}
}
