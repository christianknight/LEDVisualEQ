#include "stm32l476g_discovery.h"

#include "ece486.h"
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define LO_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET)
#define LO_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET)
#define LO_MID_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define LO_MID_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define MID_HI_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define MID_HI_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define HI_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define HI_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)

int main(void)	{

	GPIO_InitTypeDef  GPIO_LO;
	__GPIOE_CLK_ENABLE();                  // Clock on for Port E
	GPIO_LO.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_LO.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_LO.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_LO.Pin = GPIO_PIN_12;             // Set up PE12
	HAL_GPIO_Init(GPIOE, &GPIO_LO);

	GPIO_InitTypeDef  GPIO_LO_MID;
	__GPIOE_CLK_ENABLE();                   // Clock on for Port E
	GPIO_LO_MID.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_LO_MID.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_LO_MID.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_LO_MID.Pin = GPIO_PIN_13;             // Set up PE13
	HAL_GPIO_Init(GPIOE, &GPIO_LO_MID);

	GPIO_InitTypeDef  GPIO_MID_HI;
	__GPIOE_CLK_ENABLE();                  // Clock on for Port E
	GPIO_MID_HI.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_MID_HI.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_MID_HI.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_MID_HI.Pin = GPIO_PIN_14;             // Set up PE14
	HAL_GPIO_Init(GPIOE, &GPIO_MID_HI);

	GPIO_InitTypeDef  GPIO_HI;
	__GPIOE_CLK_ENABLE();                  // Clock on for Port E
	GPIO_HI.Mode = GPIO_MODE_OUTPUT_PP;    // Push/Pull digital output
	GPIO_HI.Pull = GPIO_NOPULL;            // No pullup or pulldown resistor
	GPIO_HI.Speed = GPIO_SPEED_HIGH;       // LOW, MEDIUM, FAST, or HIGH
	GPIO_HI.Pin = GPIO_PIN_15;             // Set up PE15
	HAL_GPIO_Init(GPIOE, &GPIO_HI);

	uint32_t nsamp;
	float32_t *input, *output_lo, *output_lo_mid, *output_mid_hi, *output_hi;

	static int sections_lo = 3;
	static float coefs_lo[] = {
		2.605727e-01f, -5.211322e-01f, 2.605727e-01f, 1.999050e+00f, -9.990556e-01f, 
		1.038061e-01f, -2.075758e-01f, 1.038061e-01f, 1.998525e+00f, -9.985264e-01f, 
		3.692344e-03f, -7.384579e-03f, 3.692344e-03f, 1.999688e+00f, -9.996964e-01f
	};

	static int sections_lo_mid = 6;
	static float coefs_lo_mid[] = {
		6.842074e-01f, -1.366609e+00f, 6.842074e-01f, 1.993738e+00f, -9.948458e-01f, 
		6.842074e-01f, -1.368362e+00f, 6.842074e-01f, 1.997717e+00f, -9.979003e-01f, 
		4.698467e-01f, -9.339450e-01f, 4.698467e-01f, 1.991454e+00f, -9.921131e-01f, 
		4.698467e-01f, -9.396855e-01f, 4.698467e-01f, 1.994300e+00f, -9.946072e-01f, 
		9.797716e-02f, -1.957646e-01f, 9.797716e-02f, 1.997142e+00f, -9.984926e-01f, 
		9.797716e-02f, -1.959440e-01f, 9.797716e-02f, 1.999345e+00f, -9.994959e-01f
	};

	static int sections_mid_hi = 6;
	static float coefs_mid_hi[] = {
		6.833335e-01f, -1.346943e+00f, 6.833335e-01f, 1.971670e+00f, -9.915961e-01f, 
		6.833335e-01f, -1.361737e+00f, 6.833335e-01f, 1.983532e+00f, -9.939210e-01f, 
		4.702698e-01f, -9.073043e-01f, 4.702698e-01f, 1.969488e+00f, -9.858850e-01f, 
		4.702698e-01f, -9.391671e-01f, 4.702698e-01f, 1.975086e+00f, -9.876385e-01f, 
		9.794774e-02f, -1.934366e-01f, 9.794774e-02f, 1.975994e+00f, -9.975914e-01f, 
		9.794774e-02f, -1.950825e-01f, 9.794774e-02f, 1.988755e+00f, -9.983886e-01f
	};

	static int sections_hi = 4;
	static float coefs_hi[] = {
		1.332423e+00f, -2.652495e+00f, 1.332423e+00f, 1.952443e+00f, -9.754978e-01f, 
		3.165774e+00f, -6.315970e+00f, 3.165774e+00f, 1.876846e+00f, -9.162081e-01f, 
		2.270895e+01f, -4.540216e+01f, 2.270895e+01f, 1.454138e+00f, -5.868100e-01f, 
		6.579374e-03f, -1.308295e-02f, 6.579374e-03f, 1.975097e+00f, -9.941714e-01f
	};

	float32_t pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid], pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];

	setblocksize(50);
	initialize(FS_128K, MONO_IN, STEREO_OUT);
	nsamp = getblocksize();

	float32_t mean;

	arm_biquad_cascade_df2T_instance_f32 filter_lo;
	arm_biquad_cascade_df2T_init_f32(&filter_lo,sections_lo,coefs_lo,pstate_lo);
	arm_biquad_cascade_df2T_instance_f32 filter_lo_mid;
	arm_biquad_cascade_df2T_init_f32(&filter_lo_mid,sections_lo_mid,coefs_lo_mid,pstate_lo_mid);
	arm_biquad_cascade_df2T_instance_f32 filter_mid_hi;
	arm_biquad_cascade_df2T_init_f32(&filter_mid_hi,sections_mid_hi,coefs_mid_hi,pstate_mid_hi);
	arm_biquad_cascade_df2T_instance_f32 filter_hi;
	arm_biquad_cascade_df2T_init_f32(&filter_hi,sections_hi,coefs_hi,pstate_hi);

	input = (float*)malloc(sizeof(float)*nsamp);
	output_lo = (float*)malloc(sizeof(float)*nsamp);
	output_lo_mid = (float*)malloc(sizeof(float)*nsamp);
	output_mid_hi = (float*)malloc(sizeof(float)*nsamp);
	output_hi = (float*)malloc(sizeof(float)*nsamp);
	if (input==NULL || output_lo==NULL || output_lo_mid==NULL || output_mid_hi==NULL || output_hi==NULL) {
		flagerror(MEMORY_ALLOCATION_ERROR);
		while(1);
	}
	
	while(1)	{
		getblock(input);

		DIGITAL_IO_SET();

		arm_biquad_cascade_df2T_f32(&filter_lo,input,output_lo,nsamp);
		// arm_biquad_cascade_df2T_f32(&filter_lo_mid,input,output_lo_mid,nsamp);
		// arm_biquad_cascade_df2T_f32(&filter_mid_hi,input,output_mid_hi,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_hi,input,output_hi,nsamp);

		arm_scale_f32(output_lo,2,output_lo,nsamp);
		// arm_scale_f32(output_lo_mid,2,output_lo_mid,nsamp);
		// arm_scale_f32(output_mid_hi,3,output_mid_hi,nsamp);
		arm_scale_f32(output_hi,4,output_hi,nsamp);

		arm_abs_f32(output_lo,output_lo,nsamp);
		// arm_abs_f32(output_lo_mid,output_lo_mid,nsamp);
		// arm_abs_f32(output_mid_hi,output_mid_hi,nsamp);
		arm_abs_f32(output_hi,output_hi,nsamp);

		arm_offset_f32(output_lo,-0.99,output_lo,nsamp);
		// arm_offset_f32(output_lo_mid,-0.99,output_lo_mid,nsamp);
		// arm_offset_f32(output_mid_hi,-0.99,output_mid_hi,nsamp);
		arm_offset_f32(output_hi,-0.99,output_hi,nsamp);

		arm_mean_f32(output_lo,nsamp,&mean);
		if(mean > 0.6) LO_SET();
		else LO_RESET();
		// arm_mean_f32(output_lo_mid,nsamp,&mean);
		// if(mean > 0.1) LO_MID_SET();
		// else LO_MID_RESET();
		// arm_mean_f32(output_mid_hi,nsamp,&mean);
		// if(mean > -0.4) MID_HI_SET();
		// else MID_HI_RESET();
		arm_mean_f32(output_hi,nsamp,&mean);
		if(mean > 0.05) HI_SET();
		else HI_RESET();

		DIGITAL_IO_RESET();
	}
}
