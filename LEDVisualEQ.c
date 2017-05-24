//  LEDVisualEQ.c

//	Board: 32L476GDISCOVERY
//	MCU: STM32L476VGT6

//	Uses 4 biquad filters to separate audio content in music sampled on PA1
//	and set/clear 4 LEDs on PE12, PE13, PE14, and PE15 based on music intensity.

//  Christian Knight
//  5/2/17

#include "LEDVisualEQ_lib.c"

int main(void)	{

	setblocksize(NSAMP);					// set number of samples per block to process
	initialize(FS, MONO_IN, STEREO_OUT);	// sets up ADC peripherals and processor clocks
	LED_init();								// sets up 4 digital output pins on port E to control LEDs
	short nsamp = getblocksize();
	short mode = 0;
	short i, j;

	// buffers for input block and filtered output blocks
	float input[nsamp], output_lo[nsamp], output_lo_mid[nsamp],
		output_mid_hi[nsamp], output_hi[nsamp];

	// buffers to hold previous filter samples
	float pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid],
		pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];

	float fftinput[2*FFTSIZE], fftoutput[FFTSIZE];	// FFT buffers
	unsigned int ifftFlag = 0;    					// flag for ifft
	float mean;		// for storing average value of processed blocks

	const float increment = 1.2;

	// arm_rfft_fast_instance_f32 S;
	// arm_rfft_fast_init_f32(&S, FFTSIZE);

	char lcd_str[8];

	Channel *C0, *C1, *C2, *C3;

	C0 = malloc(sizeof(Channel));

	// C0.scale = scale_lo;
	// C1.scale = scale_lo_mid;
	// C2.scale = scale_mid_hi;
	// C3.scale = scale_hi;

	// C0.offset = offset;
	// C1.offset = offset;
	// C2.offset = offset;
	// C3.offset = offset;

	// C0.thresh = thresh_lo;
	// C1.thresh = thresh_lo_mid;
	// C2.thresh = thresh_mid_hi;
	// C3.thresh = thresh_hi;

	// C0.coefs = coefs_lo;
	// C1.coefs = coefs_lo_mid;
	// C2.coefs = coefs_mid_hi;
	// C3.coefs = coefs_hi;

	// C0.sections = sections_lo;
	// C1.sections = sections_lo_mid;
	// C2.sections = sections_mid_hi;
	// C3.sections = sections_hi;

	// C0.output = output_lo;
	// C1.output = output_lo_mid;
	// C2.output = output_mid_hi;
	// C3.output = output_hi;

	// C0.pstate = pstate_lo;
	// C1.pstate = pstate_lo_mid;
	// C2.pstate = pstate_mid_hi;
	// C3.pstate = pstate_hi;

	// arm_biquad_cascade_df2T_instance_f32 filter;
	// C0.filter = filter;
	// C1.filter = filter;
	// C2.filter = filter;
	// C3.filter = filter;

	// C0.mean = mean;
	// C1.mean = mean;
	// C2.mean = mean;
	// C3.mean = mean;

	channel_init(C0, C1, C2, C3);

	Channel chan[] = {C0, C1, C2, C3};

	filt_init(C0, C1, C2, C3);
	
	while(1)	{
		getblock(input);	// grabs 'nsamp' number of input samples

		DIGITAL_IO_SET();

		if (scale_in > 1)	arm_scale_f32(input,scale_in,input,nsamp);

		// // execute each filter
		// do_filter(input, C0, C1, C2, C3, nsamp);
		// proc_blocks(C0, C1, C2, C3, nsamp);

		arm_biquad_cascade_df2T_f32(&C0.filter,input,output_lo,nsamp);
		arm_biquad_cascade_df2T_f32(&C1.filter,input,output_lo_mid,nsamp);
		arm_biquad_cascade_df2T_f32(&C2.filter,input,output_mid_hi,nsamp);
		arm_biquad_cascade_df2T_f32(&C3.filter,input,output_hi,nsamp);

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
		if (mode == 0) {
			arm_mean_f32(output_lo,nsamp,&mean);
			if(mean > thresh_lo) LED_write(LED[0], GPIO_PIN_SET);
			else LED_write(LED[0], GPIO_PIN_RESET);
			arm_mean_f32(output_lo_mid,nsamp,&mean);
			if(mean > thresh_lo_mid) LED_write(LED[1], GPIO_PIN_SET);
			else LED_write(LED[1], GPIO_PIN_RESET);
			arm_mean_f32(output_mid_hi,nsamp,&mean);
			if(mean > thresh_mid_hi) LED_write(LED[2], GPIO_PIN_SET);
			else LED_write(LED[2], GPIO_PIN_RESET);
			arm_mean_f32(output_hi,nsamp,&mean);
			if(mean > thresh_hi) LED_write(LED[3], GPIO_PIN_SET);
			else LED_write(LED[3], GPIO_PIN_RESET);
		}
		// if (mode == 0) {
		// 	arm_mean_f32(C0.output,nsamp,&C0.mean);
		// 	if(C0.mean > C0.thresh) LED_write(LED[0], GPIO_PIN_SET);
		// 	else LED_write(LED[0], GPIO_PIN_RESET);
		// 	arm_mean_f32(C1.output,nsamp,&C1.mean);
		// 	if(C1.mean > C1.thresh) LED_write(LED[1], GPIO_PIN_SET);
		// 	else LED_write(LED[1], GPIO_PIN_RESET);
		// 	arm_mean_f32(C2.output,nsamp,&C2.mean);
		// 	if(C2.mean > C2.thresh) LED_write(LED[2], GPIO_PIN_SET);
		// 	else LED_write(LED[2], GPIO_PIN_RESET);
		// 	arm_mean_f32(C3.output,nsamp,&C3.mean);
		// 	if(C3.mean > C3.thresh) LED_write(LED[3], GPIO_PIN_SET);
		// 	else LED_write(LED[3], GPIO_PIN_RESET);
		// }

		else if (mode == 1) {
			arm_mean_f32(output_lo,nsamp,&mean);
			if(mean > thresh_lo) {
				for (i = 0; i < 4; i++)
				LED_write(LED[i], GPIO_PIN_SET);
			}
			else {
				for (i = 0; i < 4; i++)
				LED_write(LED[i], GPIO_PIN_RESET);
			}
		}

 		// arm_scale_f32(input,scale_in,input,nsamp);
 		// arm_abs_f32(input,input,nsamp);
 		// arm_offset_f32(input,offset,input,nsamp);
 		// arm_mean_f32(input,nsamp,&mean);

 		// if (KeyPressed) {
 		// 	KeyPressed = RESET;
 		// 	scale_in *= increment;
 		// }

 		// if (KeyPressed) {
 		// 	KeyPressed = RESET;
 		// 	mode++;
 		// }

 		putblock(input);

 		DIGITAL_IO_RESET();

 		// if (KeyPressed) {
 		// 	KeyPressed = RESET;
 		// 	LED_write(LED[i],GPIO_PIN_SET);
 		// 	if (i > 0) LED_write(LED[i-1],GPIO_PIN_RESET);
 		// 	if (i < 3) i++;
 		// 	else i = 0;
 		// 	//HAL_Delay(1000);
 		// }

		// if(mean > thresh_in)	{
		// 	LED_write(LED[i],ON);
		// 	if (i > 0) LED_write(LED[i-1],OFF);
		// 	if (i < 3) i++;
		// 	else i = 0;
		// }
		// else LED_write(LED[i],OFF);

		// for(i = 0; i < FFTSIZE/nsamp; i++){
		// 	getblock(input);
		// 	for (j = 0; j < nsamp; j++){
		// 		fftinput[j+2*nsamp*i] = input[j];
		// 	}
		// }

		// arm_rfft_fast_f32(&arm_rfft_sR_f32_len1024, input, ifftFlag);
	}
}
