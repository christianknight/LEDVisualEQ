/**
 ******************************************************************************
 * @file    biquad.c
 * @author  Christian Knight
 * @date    1/12/2020
 * @brief   Source file for filtering operations with biquad IIR structures on
 * STM32F4 MCUs.
 ******************************************************************************
 **/

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include "biquad.h"

/*----------------------------------------------------------------------------*/
/* Global Variables ----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Low band biquad filter coefficents */
float bq_coefs_lo[BQ_SECTIONS_LO * BQ_COEFS_PER_SECTION] = {
    1.258200e-01f, -2.513427e-01f, 1.258200e-01f, 1.994561e+00f, -9.945792e-01f,
    7.923738e-03f, -1.584405e-02f, 7.923738e-03f, 1.997930e+00f, -9.979912e-01f
};

/* Low-mid band biquad filter coefficents */
float bq_coefs_lo_mid[BQ_SECTIONS_LO_MID * BQ_COEFS_PER_SECTION] = {
    6.824965e-01f, -1.342789e+00f, 6.824965e-01f, 1.968462e+00f, -9.824747e-01f,
    6.824965e-01f, -1.364243e+00f, 6.824965e-01f, 1.989989e+00f, -9.925206e-01f,
    4.732228e-01f, -8.784125e-01f, 4.732228e-01f, 1.964619e+00f, -9.731216e-01f,
    4.732228e-01f, -9.463310e-01f, 4.732228e-01f, 1.977077e+00f, -9.812105e-01f,
    9.792858e-02f, -1.934950e-01f, 9.792858e-02f, 1.977834e+00f, -9.948572e-01f,
    9.792858e-02f, -1.957116e-01f, 9.792858e-02f, 1.996086e+00f, -9.981879e-01f
};

/* Mid-high band biquad filter coefficents */
float bq_coefs_mid_hi[BQ_SECTIONS_MID_HI * BQ_COEFS_PER_SECTION] = {
    6.821886e-01f, -1.226853e+00f, 6.821886e-01f, 1.838502e+00f, -9.778613e-01f,
    6.821886e-01f, -1.329568e+00f, 6.821886e-01f, 1.910660e+00f, -9.837985e-01f,
    4.808136e-01f, -7.363451e-01f, 4.808136e-01f, 1.848419e+00f, -9.628343e-01f,
    4.808136e-01f, -9.517375e-01f, 4.808136e-01f, 1.879406e+00f, -9.672782e-01f,
    9.794532e-02f, -1.786521e-01f, 9.794532e-02f, 1.842045e+00f, -9.936369e-01f,
    9.794532e-02f, -1.901388e-01f, 9.794532e-02f, 1.927611e+00f, -9.956874e-01f
};

/* High band biquad filter coefficents */
float bq_coefs_hi[BQ_SECTIONS_HI * BQ_COEFS_PER_SECTION] = {
    1.089714e+00f, -1.856104e+00f, 1.089714e+00f, 1.478079e+00f, -9.268764e-01f,
    1.989986e+00f, -3.616470e+00f, 1.989986e+00f, 1.155312e+00f, -7.533239e-01f,
    8.291499e+00f, -1.633572e+01f, 8.291499e+00f, 1.626331e-01f, -2.251002e-01f,
    1.179967e-02f, -1.957278e-02f, 1.179967e-02f, 1.573169e+00f, -9.835571e-01f
};

/*----------------------------------------------------------------------------*/
/* Function Definitions ------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void
bq_filt_init_all(void) {
    bq_filt_init(&bq_filter_lo,     BQ_SECTIONS_LO,     bq_coefs_lo,     bq_pstate_lo);
    bq_filt_init(&bq_filter_lo_mid, BQ_SECTIONS_LO_MID, bq_coefs_lo_mid, bq_pstate_lo_mid);
    bq_filt_init(&bq_filter_mid_hi, BQ_SECTIONS_MID_HI, bq_coefs_mid_hi, bq_pstate_mid_hi);
    bq_filt_init(&bq_filter_hi,     BQ_SECTIONS_HI,     bq_coefs_hi,     bq_pstate_hi);
}

/* Initialize a biquad IIR filtering structure */
void
bq_filt_init(arm_biquad_cascade_df2T_instance_f32 * filt, int sects, float * coefs, float32_t * pstate) {
    arm_biquad_cascade_df2T_init_f32(filt, sects, coefs, pstate);
}

/* Execute biquad IIR filter on samples in input buffer */
void
bq_do_filter(arm_biquad_cascade_df2T_instance_f32 * filt, float32_t * input, float32_t * output, uint32_t len) {
    arm_biquad_cascade_df2T_f32(filt, input, output, len);
}

/* Scale samples from input buffer by scaling factor */
void
bq_do_scale(float32_t * input, float32_t scale, float32_t * output, uint32_t len) {
    arm_scale_f32(input, scale, output, len);
}

/* Transform samples from input buffer into absolute values */
void
bq_do_abs(float32_t * input, float32_t * output, uint32_t len) {
    arm_abs_f32(input, output, len);
}

/* Shift all samples in input buffer using given offset */
void
bq_do_offset(float32_t * input, float32_t offset, float32_t * output, uint32_t len) {
    arm_offset_f32(input, offset, output, len);
}

/* Get mean values of samples in input buffer */
void
bq_do_mean(float32_t * input, uint32_t len, float32_t * mean_val) {
    arm_mean_f32(input, len, mean_val);
}
