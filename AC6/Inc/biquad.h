/**
 ******************************************************************************
 * @file    biquad.h
 * @author  Christian Knight
 * @date    1/12/2020
 * @brief   Header file for filtering operations with biquad IIR structures on
 * STM32F4 MCUs.
 ******************************************************************************
 **/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BIQUAD_H
#define __BIQUAD_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx.h"
#include "arm_math.h"

/*----------------------------------------------------------------------------*/
/* Defines -------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Number of biquad filter sections for each frequency band */
#define BQ_SECTIONS_LO          2
#define BQ_SECTIONS_LO_MID      6
#define BQ_SECTIONS_MID_HI      6
#define BQ_SECTIONS_HI          4

/* Each biquad filter section needs 5 filter coefficients */
#define BQ_COEFS_PER_SECTION    5

/*----------------------------------------------------------------------------*/
/* Global Variables ----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Filter structures for the ARM processor to process each frequency band */
arm_biquad_cascade_df2T_instance_f32 bq_filter_lo,
                                     bq_filter_lo_mid,
                                     bq_filter_mid_hi,
                                     bq_filter_hi;

/* For holding biquad filter states */
float32_t bq_pstate_lo     [2 * BQ_SECTIONS_LO],
          bq_pstate_lo_mid [2 * BQ_SECTIONS_LO_MID],
          bq_pstate_mid_hi [2 * BQ_SECTIONS_MID_HI],
          bq_pstate_hi     [2 * BQ_SECTIONS_HI];

/* Biquad filter coefficients for each frequency band */
extern float bq_coefs_lo     [BQ_SECTIONS_LO     * BQ_COEFS_PER_SECTION],
             bq_coefs_lo_mid [BQ_SECTIONS_LO_MID * BQ_COEFS_PER_SECTION],
             bq_coefs_mid_hi [BQ_SECTIONS_MID_HI * BQ_COEFS_PER_SECTION],
             bq_coefs_hi     [BQ_SECTIONS_HI     * BQ_COEFS_PER_SECTION];

/*----------------------------------------------------------------------------*/
/* Function Prototypes -------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void bq_filt_init_all (void);
void bq_filt_init     (arm_biquad_cascade_df2T_instance_f32 * filt, int sects, float * coefs, float32_t * pstate);
void bq_do_filter     (arm_biquad_cascade_df2T_instance_f32 * filt, float32_t * input, float32_t * output, uint32_t len);

void bq_do_scale  (float32_t * input, float32_t scale, float32_t * output, uint32_t len);
void bq_do_abs    (float32_t * input, float32_t * output, uint32_t len);
void bq_do_offset (float32_t * input, float32_t offset, float32_t * output, uint32_t len);
void bq_do_mean   (float32_t * input, uint32_t len, float32_t * mean_val);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BIQUAD_H */
