/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
* This file should not be modified. It was automatically generated by 
* ModusToolbox 1.0.0
* 
********************************************************************************
* Copyright (c) 2017-2018 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include "cycfg_pins.h"

const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_0_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_ANALOG,
	.hsiom = ioss_0_port_0_pin_0_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_1_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_ANALOG,
	.hsiom = ioss_0_port_0_pin_1_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t PIN_BUTTON_config = 
{
	.outVal = 0,
	.driveMode = CY_GPIO_DM_PULLDOWN,
	.hsiom = PIN_BUTTON_HSIOM,
	.intEdge = CY_GPIO_INTR_RISING,
	.intMask = 1UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_10_pin_0_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_HIGHZ,
	.hsiom = ioss_0_port_10_pin_0_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_10_pin_1_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = ioss_0_port_10_pin_1_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t LED_BLUE_config = 
{
	.outVal = 0,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = LED_BLUE_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t LED_RED_config = 
{
	.outVal = 0,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = LED_RED_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};


void init_cycfg_pins(void)
{
	Cy_GPIO_Pin_Init(ioss_0_port_0_pin_0_PORT, ioss_0_port_0_pin_0_PIN, &ioss_0_port_0_pin_0_config);

	Cy_GPIO_Pin_Init(ioss_0_port_0_pin_1_PORT, ioss_0_port_0_pin_1_PIN, &ioss_0_port_0_pin_1_config);

	Cy_GPIO_Pin_Init(PIN_BUTTON_PORT, PIN_BUTTON_PIN, &PIN_BUTTON_config);

	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_0_PORT, ioss_0_port_10_pin_0_PIN, &ioss_0_port_10_pin_0_config);

	Cy_GPIO_Pin_Init(ioss_0_port_10_pin_1_PORT, ioss_0_port_10_pin_1_PIN, &ioss_0_port_10_pin_1_config);

	Cy_GPIO_Pin_Init(LED_BLUE_PORT, LED_BLUE_PIN, &LED_BLUE_config);

	Cy_GPIO_Pin_Init(LED_RED_PORT, LED_RED_PIN, &LED_RED_config);
}