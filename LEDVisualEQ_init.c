#include "LEDVisualEQ_init.h"

void gpio_init(void)	{
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