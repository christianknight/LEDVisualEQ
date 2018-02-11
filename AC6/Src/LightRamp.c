/****************************************************
************      	  LightRamp.c        ************
************		Christian Knight	  ***********
************      	     v0.0.1        **************
****************************************************/

#include "LightRamp.h"

/* Filters */
arm_biquad_cascade_df2T_instance_f32 filter_lo;
arm_biquad_cascade_df2T_instance_f32 filter_lo_mid;
arm_biquad_cascade_df2T_instance_f32 filter_mid_hi;
arm_biquad_cascade_df2T_instance_f32 filter_hi;

// low band
const int sections_lo = 3;
const float coefs_lo[] = {
	2.603860e-01f, -5.206777e-01f, 2.603860e-01f, 1.997446e+00f, -9.974836e-01f,
	1.037341e-01f, -2.072098e-01f, 1.037341e-01f, 1.996066e+00f, -9.960752e-01f,
	3.691527e-03f, -7.382281e-03f, 3.691527e-03f, 1.999129e+00f, -9.991905e-01f
};

// low-mid band
const int sections_lo_mid = 6;
const float coefs_lo_mid[] ={
	6.828877e-01f, -1.352992e+00f, 6.828877e-01f, 1.978480e+00f, -9.863217e-01f,
	6.828877e-01f, -1.365400e+00f, 6.828877e-01f, 1.993106e+00f, -9.944085e-01f,
	4.712733e-01f, -9.022227e-01f, 4.712733e-01f, 1.974448e+00f, -9.791039e-01f,
	4.712733e-01f, -9.424909e-01f, 4.712733e-01f, 1.983500e+00f, -9.856781e-01f,
	9.793550e-02f, -1.945244e-01f, 9.793550e-02f, 1.986403e+00f, -9.959891e-01f,
	9.793550e-02f, -1.957976e-01f, 9.793550e-02f, 1.997583e+00f, -9.986558e-01f
};

// mid-high band
const int sections_mid_hi = 6;
const float coefs_mid_hi[] = {
	6.822613e-01f, -1.249698e+00f, 6.822613e-01f, 1.859188e+00f, -9.826378e-01f,
	6.822613e-01f, -1.328025e+00f, 6.822613e-01f, 1.914335e+00f, -9.866131e-01f,
	4.759652e-01f, -7.833861e-01f, 4.759652e-01f, 1.865677e+00f, -9.704703e-01f,
	4.759652e-01f, -9.403309e-01f, 4.759652e-01f, 1.889254e+00f, -9.734410e-01f,
	9.793013e-02f, -1.811671e-01f, 9.793013e-02f, 1.862648e+00f, -9.950364e-01f,
	9.793013e-02f, -1.899658e-01f, 9.793013e-02f, 1.928320e+00f, -9.964095e-01f
};

// high band
const int sections_hi = 4;
const float coefs_hi[] = {
	1.107652e+00f, -1.971959e+00f, 1.107652e+00f, 1.597598e+00f, -9.353801e-01f,
	2.089015e+00f, -3.897720e+00f, 2.089015e+00f, 1.315506e+00f, -7.766538e-01f,
	9.515451e+00f, -1.882463e+01f, 9.515451e+00f, 3.547976e-01f, -2.410542e-01f,
	1.187221e-02f, -2.073522e-02f, 1.187221e-02f, 1.678874e+00f, -9.855717e-01f
};

/* Variables */
extern TIM_HandleTypeDef htim2;
uint32_t nsamp = 20;	// number of samples per block
extern enum Num_Channels_Out Output_Configuration;
extern enum Num_Channels_In Input_Configuration;
int errorbuf[ERRORBUFLEN];
int first_error = 0;
int erroridx = 0;
volatile uint32_t *ADC_Input_Buffer = NULL;
volatile uint32_t *DAC_Output_Buffer = NULL;
uint32_t ADC_Block_Size = DEFAULT_BLOCKSIZE;	//!< Number of samples user accesses per data block
uint32_t ADC_Buffer_Size = 2*DEFAULT_BLOCKSIZE; //!< Total buffer size being filled by DMA for ADC/DAC
enum Processor_Task volatile Sampler_Status;
volatile int Lower_Ready = 0;      // Set by the ISR to indicate which
extern __IO FlagStatus KeyPressed;

float32_t mean;	// for storing average value
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

// set up filter structures
void filt_init(void)	{
	// buffers to hold previous filter samples
	float32_t pstate_lo[2*sections_lo], pstate_lo_mid[2*sections_lo_mid],
		pstate_mid_hi[2*sections_mid_hi], pstate_hi[2*sections_hi];
	arm_biquad_cascade_df2T_init_f32(&filter_lo,sections_lo,coefs_lo,pstate_lo);
	arm_biquad_cascade_df2T_init_f32(&filter_lo_mid,sections_lo_mid,coefs_lo_mid,pstate_lo_mid);
	arm_biquad_cascade_df2T_init_f32(&filter_mid_hi,sections_mid_hi,coefs_mid_hi,pstate_mid_hi);
	arm_biquad_cascade_df2T_init_f32(&filter_hi,sections_hi,coefs_hi,pstate_hi);
}

// execute each filter
void do_filter(float32_t *input)	{
	arm_biquad_cascade_df2T_f32(&filter_lo,input,output_lo,nsamp);
	arm_biquad_cascade_df2T_f32(&filter_lo_mid,input,output_lo_mid,nsamp);
	arm_biquad_cascade_df2T_f32(&filter_mid_hi,input,output_mid_hi,nsamp);
	arm_biquad_cascade_df2T_f32(&filter_hi,input,output_hi,nsamp);
}

// scale each filtered block up
void do_scale(void)	{
	arm_scale_f32(output_lo,scale_lo,output_lo,nsamp);
	arm_scale_f32(output_lo_mid,scale_lo_mid,output_lo_mid,nsamp);
	arm_scale_f32(output_mid_hi,scale_mid_hi,output_mid_hi,nsamp);
	arm_scale_f32(output_hi,scale_hi,output_hi,nsamp);
}

// get absolute value of each filtered block
void do_abs(void)	{
	arm_abs_f32(output_lo,output_lo,nsamp);
	arm_abs_f32(output_lo_mid,output_lo_mid,nsamp);
	arm_abs_f32(output_mid_hi,output_mid_hi,nsamp);
	arm_abs_f32(output_hi,output_hi,nsamp);
}

// remove DC offset from filtered block
void do_offset(void)	{
	arm_offset_f32(output_lo,offset,output_lo,nsamp);
	arm_offset_f32(output_lo_mid,offset,output_lo_mid,nsamp);
	arm_offset_f32(output_mid_hi,offset,output_mid_hi,nsamp);
	arm_offset_f32(output_hi,offset,output_hi,nsamp);
}

// get mean of each filtered block
void do_mean(void)	{
	arm_mean_f32(output_lo,nsamp,&mean);
	arm_mean_f32(output_lo_mid,nsamp,&mean);
	arm_mean_f32(output_mid_hi,nsamp,&mean);
	arm_mean_f32(output_hi,nsamp,&mean);
}

// set/clear LEDs based on mean of filtered blocks
void do_LEDs(void)	{
	if(mean > thresh_lo) LED1_SET();
	else LED1_RESET();
	if(mean > thresh_lo_mid) LED2_SET();
	else LED2_RESET();
	if(mean > thresh_mid_hi) LED3_SET();
	else LED3_RESET();
	if(mean > thresh_hi) LED4_SET();
	else LED4_RESET();
}

void flash(int delay)	{
	while (1)
	{
 		LED1_SET(), LED2_SET(), LED3_SET(), LED4_SET();
 		HAL_Delay(delay);
 		LED1_RESET(), LED2_RESET(), LED3_RESET(), LED4_RESET();
 		HAL_Delay(delay);
 	}
}

// for sampling
int getblocksize()
{
  return ADC_Block_Size;
}

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
  for (i=0; i< ADC_Block_Size; i++) {
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

// error handling
void initerror()
{
  int i;

  for (i=0; i<ERRORBUFLEN; i++) errorbuf[i] = 0;

  // turn off our error indicating LED
  // BSP_LED_Off(ERROR_LED);
}

void flagerror(int errorcode)
{
  char err_str[8];

  // Display the first error on the LCD if available
  if (first_error == 0) {
    first_error = errorcode;
    // sprintf(err_str, "ERR %2d", errorcode);
    // BSP_LCD_GLASS_DisplayString((uint8_t*)err_str);
    // print_error(errorcode);
  }

  //turn on our error indicating LED
  // BSP_LED_On(ERROR_LED);


  // Store an array of the most recent errors for examination by a debugger.
  errorbuf[erroridx] = errorcode;
  erroridx++;
  if (erroridx == ERRORBUFLEN) erroridx = 0;
}

static void print_error(int index){

    char* error;

    if(index == 2)		error = "SAMPLE_OVERRUN";
    else if(index == 3)		error = "MEMORY_ALLOCATION_ERROR";
    else if(index == 4)		error =	"DAC_CONFIG_ERROR";
    else if(index == 5)		error =	"ADC_CONFIG_ERROR";
    else if(index == 6)		error =	"SETBLOCKSIZE_ERROR";
    else if(index == 7)		error = "INVALID_MIC_SAMPLE_RATE";
    else if(index == 8)		error = "CLOCK_CONFIG_ERROR";
    else if(index == 13)	error = "DEBUG_ERROR";
    else error = "UNKNOWN";

    // printf("\n*** ERROR %d ***: %s\n",index, error);
}

void breathing(uint8_t delay)	{
	int i;
	for (i = 0; i < 100; i++){
		adjust_brightness(TIM_CHANNEL_1, i);
		adjust_brightness(TIM_CHANNEL_2, i);
		adjust_brightness(TIM_CHANNEL_3, i);
		adjust_brightness(TIM_CHANNEL_4, i);
		HAL_Delay(delay);
	}
	for (i = 100; i > 0; i--){
		adjust_brightness(TIM_CHANNEL_1, i);
		adjust_brightness(TIM_CHANNEL_2, i);
		adjust_brightness(TIM_CHANNEL_3, i);
		adjust_brightness(TIM_CHANNEL_4, i);
		HAL_Delay(delay);
	}
}

void adjust_brightness(uint16_t channel, uint8_t val)	{
	const int maxBrightness = 1000;
	__HAL_TIM_SET_COMPARE(&htim2, channel, val * maxBrightness / 100);
}

/* GPIO pins 10-15 external interrupt handler */
void EXTI15_10_IRQHandler(void)
{
	for (uint16_t i = 0; i < 0xFFFF; i++); // software button de-bounce
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		KeyPressed = SET;

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

//void initialize(uint16_t timer_count_value, enum Num_Channels_In chanin, enum Num_Channels_Out chanout, enum Clock_Reference clkref){}
