/**
 ******************************************************************************
 * @file    running_mean.h
 * @author  Christian Knight
 * @date    08/23/2020
 * @brief   Header file for running mean calculation.
 ******************************************************************************
 **/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RUNNING_MEAN_H
#define __RUNNING_MEAN_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include <stdint.h>

/*----------------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
typedef struct
running_mean_struct {
    int m;             /* Number of samples used in mean-square value calculation */
    float one_by_m;    /* Set to 1/'m' (Used to avoid division later). */
    float * hist;      /* History array used to hold 'm' number of previous input samples */
    int index;         /* Index of the oldest sample in the circular history array. Between calls to 'calc_running_mean()',
                        * 'index' gives the oldest sample in 'hist' (which will be replaced by the next incoming sample) */
    int b;             /* Blocksize for subsequent calls to 'calc_running_mean()' */
    float sum;	       /* Sum of all values in the 'hist' array */
} running_mean_t;

/*----------------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
running_mean_t * running_mean_init(int n, int blocksize);
void             running_mean_calc(running_mean_t * running_mean, float * in, float * out);
void             running_mean_free(running_mean_t * running_mean);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __RUNNING_MEAN_H */
