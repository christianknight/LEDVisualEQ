/**
 ******************************************************************************
 * @file    running_mean.c
 * @author  Christian Knight
 * @date    08/23/2020
 * @brief   Source file for running mean calculation.
 ******************************************************************************
 **/

/*----------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#include <stdlib.h>
#include "running_mean.h"

/*----------------------------------------------------------------------------*/
/* Function definitions ------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Initialize a running mean calculation structure */
running_mean_t *
running_mean_init(int n, int blocksize) {
    running_mean_t * running_mean;
  
    /* Allocate space for the filter structure */
    running_mean = (running_mean_t *)malloc(sizeof(running_mean_t));

    running_mean->m = n;
    running_mean->one_by_m = 1.0f / (float)n;    /* Avoid division later on by doing 1/'n' here */
    running_mean->b = blocksize;
  
    /* The 'hist' buffer will save the input samples from one call to the next.
     * It will be updated with every new input sample so that it contains
     * the values of each of the most recent 'm' samples. */
    running_mean->hist = (float *)malloc(n * sizeof(float));
    for (int i = 0; i < n; i++) {
        running_mean->hist[i] = 0.0f;
    }
  
    /* The 'hist' buffer is a circular buffer in order to shift the array
     * as each new sample arrives.  The value of 'result->index'
     * provides the index into the array to access the oldest stored sample.
     * As each input sample arrives, it replaces the oldest sample, and
     * "index" is incremented.  When the end of the array is reached, 'index' is
     * reset to the beginning of the array. */
    running_mean->index = 0;
  
    /* 'sum' provides the sum of all the the values in the 'hist' buffer,
     * and is updated as the old samples in the buffer are replaced by new
     * samples. Start the sum at '0.0f'. */
    running_mean->sum = 0.0f;
  
    return(running_mean);
}

/* Execute the running mean calculation */
void
running_mean_calc(running_mean_t * running_mean, float * in, float * out) {
    for (int i = 0; i < running_mean->b; i++) {
        /* The oldest sample is removed from the 'hist' buffer, and subtracted
         * from 'sum'. The new sample 'in[i]' replaces it. */
        running_mean->sum -= running_mean->hist[running_mean->index];
        running_mean->hist[running_mean->index] = in[i];
        running_mean->sum += running_mean->hist[running_mean->index];

        out[i] = (running_mean->sum) * (running_mean->one_by_m);    /* Current mean-square value estimate */

        /* Increment 'index' into the 'hist' buffer circularly to keep track
         * of where the oldest sample is. If it has hit 'm', reset it to zero. */
        if (++(running_mean->index) == running_mean->m) {
            running_mean->index = 0;
        }
    }
}

/* Free the whole running_mean_t struct from memory */
void
running_mean_free(running_mean_t * running_mean) {
    free(running_mean->hist);
    free(running_mean);
}
