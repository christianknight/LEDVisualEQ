//  LEDVisualEQ_lib.c

//	Board: 32L476GDISCOVERY
//	MCU: STM32L476VGT6

//	Library file for LEDVisualEQ.c

//  Christian Knight
//  5/2/17

#include "LEDVisualEQ_lib.h"

void LED_init(void)	{
	GPIO_InitTypeDef  GPIO_LO;
	__GPIOE_CLK_ENABLE();					// Clock on for Port E
	GPIO_LO.Mode = GPIO_MODE_OUTPUT_PP;		// Push/Pull digital output
	GPIO_LO.Pull = GPIO_NOPULL;				// No pullup or pulldown resistor
	GPIO_LO.Speed = GPIO_SPEED_HIGH;		// LOW, MEDIUM, FAST, or HIGH
	GPIO_LO.Pin = GPIO_PIN_12;				// Set up PE12
	HAL_GPIO_Init(GPIOE, &GPIO_LO);

	GPIO_InitTypeDef  GPIO_LO_MID;
	__GPIOE_CLK_ENABLE();						// Clock on for Port E
	GPIO_LO_MID.Mode = GPIO_MODE_OUTPUT_PP;		// Push/Pull digital output
	GPIO_LO_MID.Pull = GPIO_NOPULL;				// No pullup or pulldown resistor
	GPIO_LO_MID.Speed = GPIO_SPEED_HIGH;		// LOW, MEDIUM, FAST, or HIGH
	GPIO_LO_MID.Pin = GPIO_PIN_13;				// Set up PE13
	HAL_GPIO_Init(GPIOE, &GPIO_LO_MID);

	GPIO_InitTypeDef  GPIO_MID_HI;
	__GPIOE_CLK_ENABLE();						// Clock on for Port E
	GPIO_MID_HI.Mode = GPIO_MODE_OUTPUT_PP;		// Push/Pull digital output
	GPIO_MID_HI.Pull = GPIO_NOPULL;				// No pullup or pulldown resistor
	GPIO_MID_HI.Speed = GPIO_SPEED_HIGH;		// LOW, MEDIUM, FAST, or HIGH
	GPIO_MID_HI.Pin = GPIO_PIN_14;				// Set up PE14
	HAL_GPIO_Init(GPIOE, &GPIO_MID_HI);

	GPIO_InitTypeDef  GPIO_HI;
	__GPIOE_CLK_ENABLE();					// Clock on for Port E
	GPIO_HI.Mode = GPIO_MODE_OUTPUT_PP;		// Push/Pull digital output
	GPIO_HI.Pull = GPIO_NOPULL;				// No pullup or pulldown resistor
	GPIO_HI.Speed = GPIO_SPEED_HIGH;		// LOW, MEDIUM, FAST, or HIGH
	GPIO_HI.Pin = GPIO_PIN_15;				// Set up PE15
	HAL_GPIO_Init(GPIOE, &GPIO_HI);
}

void LED_write(uint16_t LED, GPIO_PinState state)	{
	HAL_GPIO_WritePin(GPIOE, LED, state);
}

void channel_init(Channel C0, Channel C1, Channel C2, Channel C3)	{
	C0.scale = scale_lo;
	C1.scale = scale_lo_mid;
	C2.scale = scale_mid_hi;
	C3.scale = scale_hi;

	C0.offset = offset;
	C1.offset = offset;
	C2.offset = offset;
	C3.offset = offset;

	C0.thresh = thresh_lo;
	C1.thresh = thresh_lo_mid;
	C2.thresh = thresh_mid_hi;
	C3.thresh = thresh_hi;

	C0.coefs = coefs_lo;
	C1.coefs = coefs_lo_mid;
	C2.coefs = coefs_mid_hi;
	C3.coefs = coefs_hi;

	C0.sections = sections_lo;
	C1.sections = sections_lo_mid;
	C2.sections = sections_mid_hi;
	C3.sections = sections_hi;

	C0.output = (float *)malloc(sizeof(float)*NSAMP);
	C1.output = (float *)malloc(sizeof(float)*NSAMP);
	C2.output = (float *)malloc(sizeof(float)*NSAMP);
	C3.output = (float *)malloc(sizeof(float)*NSAMP);

	C0.pstate = (float *)malloc(sizeof(float)*2*sections_lo);
	C1.pstate = (float *)malloc(sizeof(float)*2*sections_lo_mid);
	C2.pstate = (float *)malloc(sizeof(float)*2*sections_mid_hi);
	C3.pstate = (float *)malloc(sizeof(float)*2*sections_hi);

	arm_biquad_cascade_df2T_instance_f32 filter;
	C0.filter = filter;
	C1.filter = filter;
	C2.filter = filter;
	C3.filter = filter;

	C0.mean = (float)malloc(sizeof(float));
	C1.mean = (float)malloc(sizeof(float));
	C2.mean = (float)malloc(sizeof(float));
	C3.mean = (float)malloc(sizeof(float));
}

void filt_init(Channel C0, Channel C1, Channel C2, Channel C3)	{
	// set up filter structures
	arm_biquad_cascade_df2T_init_f32(&C0.filter,C0.sections,C0.coefs,C0.pstate);
	arm_biquad_cascade_df2T_init_f32(&C1.filter,C1.sections,C1.coefs,C1.pstate);
	arm_biquad_cascade_df2T_init_f32(&C2.filter,C2.sections,C2.coefs,C2.pstate);
	arm_biquad_cascade_df2T_init_f32(&C3.filter,C3.sections,C3.coefs,C3.pstate);
}

void do_filter(float *input, Channel C0, Channel C1, Channel C2, Channel C3, short nsamp)	{
	arm_biquad_cascade_df2T_f32(&C0.filter,input,C0.output,nsamp);
	arm_biquad_cascade_df2T_f32(&C1.filter,input,C1.output,nsamp);
	arm_biquad_cascade_df2T_f32(&C2.filter,input,C2.output,nsamp);
	arm_biquad_cascade_df2T_f32(&C3.filter,input,C3.output,nsamp);
}

void proc_blocks(Channel C0, Channel C1, Channel C2, Channel C3, short nsamp)	{
	// scale each filtered block up
	arm_scale_f32(C0.output,C0.scale,C0.output,nsamp);
	arm_scale_f32(C1.output,C1.scale,C1.output,nsamp);
	arm_scale_f32(C2.output,C2.scale,C2.output,nsamp);
	arm_scale_f32(C3.output,C3.scale,C3.output,nsamp);

	// get absolute value of each filtered block
	arm_abs_f32(C0.output,C0.output,nsamp);
	arm_abs_f32(C1.output,C1.output,nsamp);
	arm_abs_f32(C2.output,C2.output,nsamp);
	arm_abs_f32(C3.output,C3.output,nsamp);

	// remove DC offset from filtered block
	arm_offset_f32(C0.output,C0.offset,C0.output,nsamp);
	arm_offset_f32(C1.output,C1.offset,C1.output,nsamp);
	arm_offset_f32(C2.output,C2.offset,C2.output,nsamp);
	arm_offset_f32(C3.output,C3.offset,C3.output,nsamp);
}

void flash(int time)	{
	while (1)	{
		int i;
		if (KeyPressed) {
 			KeyPressed = RESET;
 			time = time / 2;
 		}
 		for (i = 0; i < 4; i++)
 			LED_write(LED[i], GPIO_PIN_SET);
 		HAL_Delay(time);
 		for (i = 0; i < 4; i++)
 			LED_write(LED[i], GPIO_PIN_SET);
 		HAL_Delay(time);
 	}
}
