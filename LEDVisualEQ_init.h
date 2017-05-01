#include "stm32l476g_discovery.h"
#include "ece486.h"
#include "arm_math.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define LO_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET)
#define LO_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET)
#define LO_MID_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define LO_MID_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define MID_HI_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define MID_HI_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define HI_SET()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define HI_RESET()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)

extern FlagStatus KeyPressed;   // Use to detect button presses

static int sections_lo = 3;
static float coefs_lo[] = {
	2.603860e-01f, -5.206777e-01f, 2.603860e-01f, 1.997446e+00f, -9.974836e-01f, 
	1.037341e-01f, -2.072098e-01f, 1.037341e-01f, 1.996066e+00f, -9.960752e-01f, 
	3.691527e-03f, -7.382281e-03f, 3.691527e-03f, 1.999129e+00f, -9.991905e-01f
};

static int sections_lo_mid = 6;
static float coefs_lo_mid[] = {
	6.828877e-01f, -1.352992e+00f, 6.828877e-01f, 1.978480e+00f, -9.863217e-01f, 
	6.828877e-01f, -1.365400e+00f, 6.828877e-01f, 1.993106e+00f, -9.944085e-01f, 
	4.712733e-01f, -9.022227e-01f, 4.712733e-01f, 1.974448e+00f, -9.791039e-01f, 
	4.712733e-01f, -9.424909e-01f, 4.712733e-01f, 1.983500e+00f, -9.856781e-01f, 
	9.793550e-02f, -1.945244e-01f, 9.793550e-02f, 1.986403e+00f, -9.959891e-01f, 
	9.793550e-02f, -1.957976e-01f, 9.793550e-02f, 1.997583e+00f, -9.986558e-01f
};

static int sections_mid_hi = 6;
static float coefs_mid_hi[] = {
	6.822613e-01f, -1.249698e+00f, 6.822613e-01f, 1.859188e+00f, -9.826378e-01f, 
	6.822613e-01f, -1.328025e+00f, 6.822613e-01f, 1.914335e+00f, -9.866131e-01f, 
	4.759652e-01f, -7.833861e-01f, 4.759652e-01f, 1.865677e+00f, -9.704703e-01f, 
	4.759652e-01f, -9.403309e-01f, 4.759652e-01f, 1.889254e+00f, -9.734410e-01f, 
	9.793013e-02f, -1.811671e-01f, 9.793013e-02f, 1.862648e+00f, -9.950364e-01f, 
	9.793013e-02f, -1.899658e-01f, 9.793013e-02f, 1.928320e+00f, -9.964095e-01f
};

static int sections_hi = 4;
static float coefs_hi[] = {
	1.107652e+00f, -1.971959e+00f, 1.107652e+00f, 1.597598e+00f, -9.353801e-01f, 
	2.089015e+00f, -3.897720e+00f, 2.089015e+00f, 1.315506e+00f, -7.766538e-01f, 
	9.515451e+00f, -1.882463e+01f, 9.515451e+00f, 3.547976e-01f, -2.410542e-01f, 
	1.187221e-02f, -2.073522e-02f, 1.187221e-02f, 1.678874e+00f, -9.855717e-01f
};

void gpio_init(void);