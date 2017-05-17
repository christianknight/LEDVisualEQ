//  LEDVisualEQ.c

//	Board: 32L476GDISCOVERY
//	MCU: STM32L476VGT6

//	Uses 4 biquad filters to separate audio content in music sampled on PA1
//	and set/clear 4 LEDs on PE12, PE13, PE14, and PE15 based on music intensity.

//  Christian Knight, Nikko Noble
//  5/2/17

#include "LEDVisualEQ_init.c"

int main(void)	{

	uint32_t nsamp = 20;						// number of samples per block
	setblocksize(nsamp);						// set number of samples per block
	initialize(FS_48K, MONO_IN, STEREO_OUT);	// sets up ADC peripherals and processor clocks
	gpio_init();								// sets up 4 GPIO pins on port E to turn LEDs on/off

	float32_t *input, *output_lo, *output_lo_mid,
		*output_mid_hi, *output_hi;	// buffers for input block and filtered output blocks

	float32_t pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid],
		pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];	// buffers to hold previous filter samples

	float32_t mean;	// for storing average value
	static float offset = -0.99;	// DC offset to add to filtered block
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
	if (input==NULL || output_lo==NULL || output_lo_mid==NULL || output_mid_hi==NULL || output_hi==NULL) {
		flagerror(MEMORY_ALLOCATION_ERROR);
		while(1);
	}
	
	while(1)	{
		getblock(input);	// grabs 'blocksize' number of input samples

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

		// if (KeyPressed) {
 	// 		KeyPressed = RESET;
 	// 		scale_input *= increment;
 	// 	}

		if (KeyPressed) {
 			KeyPressed = RESET;
 			flash_lo();
 		}
	}
}
