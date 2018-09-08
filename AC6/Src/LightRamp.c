/****************************************************
************      	  LightRamp.c        ************
************		Christian Knight	  ***********
************      	     v0.0.1        **************
****************************************************/

#include "LightRamp.h"

uint32_t LED[NUM_LEDS] = {LED1, LED2, LED3, LED4};

// low band
int sections_lo = SECTIONS_LO;
float coefs_lo[] = {
	1.258200e-01f, -2.513427e-01f, 1.258200e-01f, 1.994561e+00f, -9.945792e-01f,
	7.923738e-03f, -1.584405e-02f, 7.923738e-03f, 1.997930e+00f, -9.979912e-01f
};

// low-mid band
int sections_lo_mid = SECTIONS_LO_MID;
float coefs_lo_mid[] = {
	6.824965e-01f, -1.342789e+00f, 6.824965e-01f, 1.968462e+00f, -9.824747e-01f,
	6.824965e-01f, -1.364243e+00f, 6.824965e-01f, 1.989989e+00f, -9.925206e-01f,
	4.732228e-01f, -8.784125e-01f, 4.732228e-01f, 1.964619e+00f, -9.731216e-01f,
	4.732228e-01f, -9.463310e-01f, 4.732228e-01f, 1.977077e+00f, -9.812105e-01f,
	9.792858e-02f, -1.934950e-01f, 9.792858e-02f, 1.977834e+00f, -9.948572e-01f,
	9.792858e-02f, -1.957116e-01f, 9.792858e-02f, 1.996086e+00f, -9.981879e-01f
};

// mid-high band
int sections_mid_hi = SECTIONS_MID_HI;
float coefs_mid_hi[] = {
	6.821886e-01f, -1.226853e+00f, 6.821886e-01f, 1.838502e+00f, -9.778613e-01f,
	6.821886e-01f, -1.329568e+00f, 6.821886e-01f, 1.910660e+00f, -9.837985e-01f,
	4.808136e-01f, -7.363451e-01f, 4.808136e-01f, 1.848419e+00f, -9.628343e-01f,
	4.808136e-01f, -9.517375e-01f, 4.808136e-01f, 1.879406e+00f, -9.672782e-01f,
	9.794532e-02f, -1.786521e-01f, 9.794532e-02f, 1.842045e+00f, -9.936369e-01f,
	9.794532e-02f, -1.901388e-01f, 9.794532e-02f, 1.927611e+00f, -9.956874e-01f
};

// high band
int sections_hi = SECTIONS_HI;
float coefs_hi[] = {
	1.089714e+00f, -1.856104e+00f, 1.089714e+00f, 1.478079e+00f, -9.268764e-01f,
	1.989986e+00f, -3.616470e+00f, 1.989986e+00f, 1.155312e+00f, -7.533239e-01f,
	8.291499e+00f, -1.633572e+01f, 8.291499e+00f, 1.626331e-01f, -2.251002e-01f,
	1.179967e-02f, -1.957278e-02f, 1.179967e-02f, 1.573169e+00f, -9.835571e-01f
};

/* Variables */
extern TIM_HandleTypeDef htim2, htim3;
extern ADC_HandleTypeDef hadc1;
extern enum Num_Channels_Out Output_Configuration;
extern enum Num_Channels_In Input_Configuration;
int errorbuf[ERRORBUFLEN];
int first_error = 0;
int erroridx = 0;
volatile uint32_t *ADC_Input_Buffer = NULL;
volatile uint32_t *DAC_Output_Buffer = NULL;
uint32_t ADC_Block_Size = DEFAULT_BLOCKSIZE;	// number of samples user accesses per data block
uint32_t ADC_Buffer_Size = 2*DEFAULT_BLOCKSIZE; // total buffer size being filled by DMA for ADC/DAC
enum Processor_Task volatile Sampler_Status;
volatile int Lower_Ready = 0;      // set by the ISR to indicate which

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

void LightRamp_init(void)	{
	nsamp = BLOCKSIZE;	// number of samples per block
	setblocksize(nsamp);	// set number of samples per block

	filt_init();	// set up filter structures

	// allocate memory buffers for input block and filtered output blocks
	input = (float*)malloc(sizeof(float)*nsamp);
	output_lo = (float*)malloc(sizeof(float)*nsamp);
	output_lo_mid = (float*)malloc(sizeof(float)*nsamp);
	output_mid_hi = (float*)malloc(sizeof(float)*nsamp);
	output_hi = (float*)malloc(sizeof(float)*nsamp);

	// start PWM on all channels
	for (int i = 0; i < NUM_LEDS; i++)
		HAL_TIM_PWM_Start(H_LED_TIM, LED[i]);

	// set initial brightness to 0 for all channels
	for (int i = 0; i < NUM_LEDS; i++)
		adjust_brightness(LED[i], 0);

	KeyPressed = RESET;	// reset button push flag
	while (KeyPressed == RESET); // wait for user button push
	KeyPressed = RESET;	// reset button push flag

	HAL_TIM_OC_Start(H_ADC_TIM, TIM_CHANNEL_1);

	ADC_Input_Buffer = (uint32_t *)malloc(sizeof(uint32_t)*ADC_Buffer_Size);
	DAC_Output_Buffer = (uint32_t *)malloc(sizeof(uint32_t)*ADC_Buffer_Size);

	HAL_ADC_Start_DMA(H_ADC, (uint32_t *)ADC_Input_Buffer, ADC_Buffer_Size);
}

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

void getblock(float *working)
{
  uint32_t i;

  // wait for the DMA to finish filling a block of data
  Sampler_Status = WAIT_FOR_NEXT_BUFFER;
  while (Sampler_Status == WAIT_FOR_NEXT_BUFFER) __WFI();

  /* The DMA ISR sets the Lower_Ready flag to indicate whether we should
   * be processing the upper or lower half of the DMA transfer block.
   */
  if (Lower_Ready) {
	  inbuf = ADC_Input_Buffer;
	  outbuf = DAC_Output_Buffer;
  }
  else {
	  inbuf = &(ADC_Input_Buffer[ADC_Block_Size]);
	  outbuf = &(DAC_Output_Buffer[ADC_Block_Size]);
  }

  /* Now convert the valid ADC data into the caller's array of floats.
   * Samples are normalized to range from -1.0 to 1.0.
   */
  for (i = 0; i < ADC_Block_Size; i++)
	  // 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing)
	  working[i] = ((float)((int)inbuf[i]-32767))*3.0517578e-05f;
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
	int i, j;
	static int len = 100;
	for (i = 0; i < len; i++){
		for (j = 0; j < NUM_LEDS; j++)
			adjust_brightness(LED[j], i);
		HAL_Delay(delay);
	}
	for (i = len; i > 0; i--){
		for (j = 0; j < NUM_LEDS; j++)
			adjust_brightness(LED[j], i);
		HAL_Delay(delay);
	}
}

void adjust_brightness(uint32_t channel, float32_t val)	{
	__HAL_TIM_SET_COMPARE(H_LED_TIM, channel, (uint16_t)(val * 0xFFFF));
}

void pulse(uint32_t channel, float32_t val, uint8_t time)	{
	adjust_brightness(channel, val);
	HAL_Delay(time);
	adjust_brightness(channel, 0);
}

//void initialize(uint16_t timer_count_value, enum Num_Channels_In chanin, enum Num_Channels_Out chanout, enum Clock_Reference clkref){}
