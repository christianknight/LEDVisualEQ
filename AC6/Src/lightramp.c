/**
 ******************************************************************************
 * @file    lightramp.c
 * @author  Christian Knight
 * @date    08/23/2020
 * @brief   Source file for controlling ADC audio sampling and LED driver
 * PWM control w/ STM32F446 MCU.
 ******************************************************************************
 **/

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include "lightramp.h"

/*----------------------------------------------------------------------------*/
/* Extern variable declarations ----------------------------------------------*/
/*----------------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2, htim3;
extern ADC_HandleTypeDef hadc1;

/*----------------------------------------------------------------------------*/
/* Static variable definitions -----------------------------------------------*/
/*----------------------------------------------------------------------------*/
static volatile uint32_t * inbuf  = NULL,
                         * outbuf = NULL,
                         * adc_input_buffer = NULL,
                         * dac_output_buffer = NULL;

/*----------------------------------------------------------------------------*/
/* Global variable definitions -----------------------------------------------*/
/*----------------------------------------------------------------------------*/
uint32_t LED[NUM_LEDS] = {LED1, LED2, LED3, LED4};

enum num_channels_in input_configuration = MONO_IN;
enum num_channels_out output_configuration = MONO_OUT;
enum processor_task volatile sampler_status = STARTUP;

__IO uint8_t led_enable = 1;

/* Sampling */
uint32_t nsamp = BLOCKSIZE;                          /* Number of samples per block */
uint32_t adc_blocksize = DEFAULT_BLOCKSIZE;          /* Number of samples user accesses per data block */
uint32_t adc_buffer_size = 2 * DEFAULT_BLOCKSIZE;    /* Total buffer size being filled by DMA for ADC/DAC */
volatile int lower_ready = 0;                        /* Set by the ISR to indicate which half of the ADC buffer is available for processing */

/* Error Handling */
int error_buffer[ERRORBUFLEN] = {0};
int error_idx = 0;

/*----------------------------------------------------------------------------*/
/* Static function definitions -----------------------------------------------*/
/*----------------------------------------------------------------------------*/
static char * print_error(int index);

/*----------------------------------------------------------------------------*/
/* Function definitions ------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void
lightramp_init(void) {
    setblocksize(nsamp);    /* Set number of samples per block */

    /* Start PWM on all channels */
    for (int i = 0; i < NUM_LEDS; i++) {
        HAL_TIM_PWM_Start(H_LED_TIM, LED[i]);
    }

    HAL_TIM_OC_Start(H_ADC_TIM, TIM_CHANNEL_1);

    adc_input_buffer  = (uint32_t *)malloc(sizeof(uint32_t) * adc_buffer_size);
    dac_output_buffer = (uint32_t *)malloc(sizeof(uint32_t) * adc_buffer_size);

    HAL_ADC_Start_DMA(H_ADC, (uint32_t *)adc_input_buffer, adc_buffer_size);
}

/* For sampling */
int
getblocksize(void) {
    return adc_blocksize;
}

void
setblocksize(uint32_t blksiz) {
    /* setblocksize() should only be called before calling initialize().
     * If the ADC & DAC buffers have already been allocated, then initialize()
     * must have already been called, and we're too late to change the buffer
     * sizes.  Flag an error and return without changing anything. */
    if (adc_input_buffer != NULL) {
        flagerror(SETBLOCKSIZE_ERROR);
        return;
    }

    adc_blocksize  = blksiz;
    adc_buffer_size = 2 * blksiz;
}

void
getblock(float * working) {
    uint32_t i;

    /* Wait for the DMA to finish filling a block of data */
    sampler_status = WAIT_FOR_NEXT_BUFFER;
    while (sampler_status == WAIT_FOR_NEXT_BUFFER) {
        __WFI();
    }

    /* The DMA ISR sets the lower_ready flag to indicate whether we should
     * be processing the upper or lower half of the DMA transfer block. */
    if (lower_ready) {
        inbuf = adc_input_buffer;
        outbuf = dac_output_buffer;
    }
    else {
        inbuf = &(adc_input_buffer[adc_blocksize]);
        outbuf = &(dac_output_buffer[adc_blocksize]);
    }

    /* Now convert the valid ADC data into the caller's array of floats.
     * Samples are normalized to range from -1.0 to 1.0. */
    for (i = 0; i < adc_blocksize; i++) {
        /* 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing) */
	    working[i] = ((float)((int)inbuf[i] - 32767)) * 3.0517578e-05f;
    }
}

void
putblock(float * working) {
    uint32_t i;

    /* The "outbuf" pointer is set by getblock() to indicate the
     * appropriate destination of any output samples.
     * floating point values between -1 and +1 are mapped
     * into the range of the DAC */
    for (i = 0; i < adc_blocksize; i++) {
        outbuf[i] = ((int)((working[i] + 1.0) * 32768.0f)) & 0x0000FFFF;
    }
}

void
getblockstereo(float * chan1, float * chan2) {
    uint32_t i;

    if (input_configuration == MONO_IN) {
        getblock(chan1);
        return;
    }

    /* Wait for the DMA to finish filling a block of data */
    sampler_status = WAIT_FOR_NEXT_BUFFER;
    while (sampler_status == WAIT_FOR_NEXT_BUFFER) {
    	__WFI();
    }

    /* The DMA ISR sets the lower_ready flag to indicate whether we should
     * be processing the upper or lower half of the DMA transfer block. */
    if (lower_ready) {
        inbuf = adc_input_buffer;
        outbuf = dac_output_buffer;
    }
    else {
        inbuf = &(adc_input_buffer[adc_blocksize]);
        outbuf = &(dac_output_buffer[adc_blocksize]);
    }

    /* Now convert the valid ADC data into the caller's arrays of floats.
     * Samples are normalized to range from -1.0 to 1.0
     * Channel 1 is in the least significant 16 bits of the DMA transfer data,
     * Channel 2 is in the most significant 16 bits. */
    for (i = 0; i < adc_blocksize; i++) {
        /* 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing) */
        chan1[i] = ((float)((int)(inbuf[i] & 0x0000FFFF) - 32767)) * 3.0517578e-05f;
        chan2[i] = ((float)((int)((inbuf[i] & 0xFFFF0000) >> 16) - 32767)) * 3.0517578e-05f;
    }
}

void
putblockstereo(float * chan1, float * chan2) {
    uint32_t i;

    if (output_configuration == MONO_OUT) {
        putblock(chan1);
        return;
    }

    /* the "outbuf" pointer is set by getblock() to indicate the
     * appropriate destination of any output samples.
     * Floating point values between -1 and +1 are mapped
     * into the range of the DAC.
     * chan1 goes into the most-significant 16 bits (DAC1),
     * chan2 in the least significant (DAC1) */
    for (i = 0; i < adc_blocksize; i++) {
        outbuf[i] = (((int)((chan2[i] + 1.0) * 32768.0f)) & 0x0000FFFF) |
                    ((((int)((chan1[i] + 1.0) * 32768.0f)) & 0x0000FFFF) << 16);
    }
}

void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc) {
    /* The ADC has filled the input buffer, and is beginning to over-write
     * the beginning of the array.  The user should get to work processing the
     * end of the array. */
    lower_ready = 0;

    if (sampler_status == STARTUP) {
        /* No need to do anything... user is still initializing */
    }
    else if (sampler_status == WAIT_FOR_NEXT_BUFFER) {
        sampler_status = PROCESS_BUFFER;    /* Turn the supervisor loose on the next buffer */
    }
    else {
        flagerror(SAMPLE_OVERRUN);
        /* If the supervisor was not waiting for the next
         * buffer, flag the error to let them know that
         * they're missing blocks of data. */
    }
}

void
HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef * hadc) {
    /* The ADC has half-filled the input buffer, and is beginning to over-write
     * the second-half of the array.  The user should get to work processing the
     * beginning of the array. */
    lower_ready = 1;

    if (sampler_status == STARTUP) {
        /* No need to do anything... user is still initializing */
    }
    else if (sampler_status == WAIT_FOR_NEXT_BUFFER) {
        sampler_status = PROCESS_BUFFER;    /* Turn the supervisor loose on the next buffer */
    }
    else {
        flagerror(SAMPLE_OVERRUN);
        /* If the supervisor was not waiting for the next
         * buffer, flag the error to let them know that
         * they're missing blocks of data. */
    }
}

/* Error handling */
void
initerror(void) {
    int i;

    for (i = 0; i < ERRORBUFLEN; i++) {
        error_buffer[i] = 0;
    }
}

void
flagerror(int errorcode) {
    static int first_error = 0;
    char err_str[8];

    /* Display the first error on the LCD if available */
    if (first_error == 0) {
        first_error = errorcode;
    }

    /* Store an array of the most recent errors for examination by a debugger. */
    error_buffer[error_idx] = errorcode;
    error_idx++;
    if (error_idx == ERRORBUFLEN) {
        error_idx = 0;
    }
}

uint32_t
lightramp_calc_gamma(float gamma, float brightness, uint32_t val_in) {
    uint32_t retval = (uint32_t)(pow((double)val_in/(double)0xFFFFFFFF, gamma) * brightness * 0xFFFFFFFF + 0.5);

    return retval;
}

/*----------------------------------------------------------------------------*/
/* Static function definitions -----------------------------------------------*/
/*----------------------------------------------------------------------------*/
static char *
print_error(int index) {
    char * error;

    if (index == 2)	        error = "SAMPLE_OVERRUN";
    else if (index == 3)    error = "MEMORY_ALLOCATION_ERROR";
    else if (index == 4)    error =	"DAC_CONFIG_ERROR";
    else if (index == 5)    error =	"ADC_CONFIG_ERROR";
    else if (index == 6)    error =	"SETBLOCKSIZE_ERROR";
    else if (index == 7)    error = "INVALID_MIC_SAMPLE_RATE";
    else if (index == 8)    error = "CLOCK_CONFIG_ERROR";
    else if (index == 13)   error = "DEBUG_ERROR";
    else error = "UNKNOWN";

    return error;
}
