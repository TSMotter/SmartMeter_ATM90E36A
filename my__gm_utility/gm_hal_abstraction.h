/***************************************************************************************************
* @file   gm_hal_abstraction.h
* @brief  Offers a new level of abstraction to ST's HAL
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __GM_HAL_ABSTRACTION_H__
#define __GM_HAL_ABSTRACTION_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include "stm32f4xx_hal.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/

typedef struct
{
  void    		*Port;
  uint16_t    Pin;
} gpio_hal_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
void      assign_port_pin    (void *port, uint16_t pin, gpio_hal_st *dest);
void      drive_gpio         (gpio_hal_st *gpio, uint8_t val);
uint8_t   read_gpio          (gpio_hal_st *gpio);
void      toggle_gpio        (gpio_hal_st *gpio);
#endif
