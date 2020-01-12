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

float32_t *input,
          *output_lo,
          *output_lo_mid,
          *output_mid_hi,
          *output_hi;

const float offset = -0.99;	// DC offset to add to filtered block

/* Scalar multipliers for each sample in a filtered block */
float scale_lo     = 12;
float scale_lo_mid = 6;
float scale_mid_hi = 18;
float scale_hi     = 34;

void LightRamp_init(void)	{
	nsamp = BLOCKSIZE;	// number of samples per block
	setblocksize(nsamp);	// set number of samples per block

    /* Set up biquad IIR filter structures */
    filt_init(&filter_lo,     sections_lo,     coefs_lo,     pstate_lo);
    filt_init(&filter_lo_mid, sections_lo_mid, coefs_lo_mid, pstate_lo_mid);
    filt_init(&filter_mid_hi, sections_mid_hi, coefs_mid_hi, pstate_mid_hi);
    filt_init(&filter_hi,     sections_hi,     coefs_hi,     pstate_hi);

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

/* Biquad IIR filtering routines */
/* Initialize a biquad IIR filtering structure */
void
filt_init(arm_biquad_cascade_df2T_instance_f32 * filt, int sects, float * coefs, float32_t * pstate) {
    arm_biquad_cascade_df2T_init_f32(filt, sects, coefs, pstate);
}

/* Execute biquad IIR filter on samples in input buffer */
void
do_filter(arm_biquad_cascade_df2T_instance_f32 * filt, float32_t * input, float32_t * output, uint32_t len) {
    arm_biquad_cascade_df2T_f32(filt, input, output, len);
}

/* Scale samples from input buffer by scaling factor */
void
do_scale(float32_t * input, float32_t scale, float32_t * output, uint32_t len) {
    arm_scale_f32(input, scale, output, len);
}

/* Transform samples from input buffer into absolute values */
void
do_abs(float32_t * input, float32_t * output, uint32_t len) {
    arm_abs_f32(input, output, len);
}

/* Shift all samples in input buffer using given offset */
void
do_offset(float32_t * input, float32_t offset, float32_t * output, uint32_t len) {
    arm_offset_f32(input, offset, output, len);
}

/* Get mean values of samples in input buffer */
void
do_mean(float32_t * input, uint32_t len, float32_t * mean_val) {
    arm_mean_f32(input, len, mean_val);
}

// for sampling
int getblocksize()
{
  return ADC_Block_Size;
}

void setblocksize(uint32_t blksiz)
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
    getblock(chan1);
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

/* Set compare-capture value to given channel of TIM2 to adjust individual LED PWM dimming level */
void
adjust_brightness(uint32_t channel, float32_t val) {
    __HAL_TIM_SET_COMPARE(H_LED_TIM, channel, (uint16_t)(val * 0xFFFF));
}

void pulse(uint32_t channel, float32_t val, uint8_t time)	{
	adjust_brightness(channel, val);
	HAL_Delay(time);
	adjust_brightness(channel, 0);
}
