#include "LEDVisualEQ_init.c"

int main(void)	{

	uint32_t nsamp;
	float32_t *input, *output_lo, *output_lo_mid, *output_mid_hi, *output_hi;
	float32_t pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid], pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];

	gpio_init();
	setblocksize(20);
	initialize(FS_48K, MONO_IN, STEREO_OUT);
	nsamp = getblocksize();

	float32_t mean;
	static float offset = -0.99;
	float thresh_lo = -0.4;
	float thresh_lo_mid = -0.65;
	float thresh_mid_hi = -0.8;
	float thresh_hi = -0.7;
	float scale_input = 1;
	float scale_lo = 2;
	float scale_lo_mid = 2;
	float scale_mid_hi = 3;
	float scale_hi = 4;
	float increment = 2;

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

		if (scale_input > 1)	arm_scale_f32(input,scale_input,input,nsamp);

		arm_biquad_cascade_df2T_f32(&filter_lo,input,output_lo,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_lo_mid,input,output_lo_mid,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_mid_hi,input,output_mid_hi,nsamp);
		arm_biquad_cascade_df2T_f32(&filter_hi,input,output_hi,nsamp);

		arm_scale_f32(output_lo,scale_lo,output_lo,nsamp);
		arm_scale_f32(output_lo_mid,scale_lo_mid,output_lo_mid,nsamp);
		arm_scale_f32(output_mid_hi,scale_mid_hi,output_mid_hi,nsamp);
		arm_scale_f32(output_hi,scale_hi,output_hi,nsamp);

		arm_abs_f32(output_lo,output_lo,nsamp);
		arm_abs_f32(output_lo_mid,output_lo_mid,nsamp);
		arm_abs_f32(output_mid_hi,output_mid_hi,nsamp);
		arm_abs_f32(output_hi,output_hi,nsamp);

		arm_offset_f32(output_lo,offset,output_lo,nsamp);
		arm_offset_f32(output_lo_mid,offset,output_lo_mid,nsamp);
		arm_offset_f32(output_mid_hi,offset,output_mid_hi,nsamp);
		arm_offset_f32(output_hi,offset,output_hi,nsamp);

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
			scale_input *= increment;
		}

		DIGITAL_IO_RESET();
	}
}
