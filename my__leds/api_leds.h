/***************************************************************************************************
* @file   api_leds.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __API_LEDS_H__
#define __API_LEDS_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "driver_leds.h"
#include "rtos_utility.h"
#include "stm32f4xx_hal.h"
#include "ee24.h"
#include "global_configs.h"
/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool LEDS_api_init(void);
bool LEDS_blink_pattern1(leds_periods_en period);
bool LEDS_blink_pattern2(leds_periods_en period);
bool LEDS_blink_pattern3(leds_periods_en period);
void LEDS_check_queue (void);
#endif
