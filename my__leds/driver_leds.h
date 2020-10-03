/***************************************************************************************************
* @file   driver_leds.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __DRIVER_LEDS_H__
#define __DRIVER_LEDS_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "gm_hal_abstraction.h"
#include "gm_timer.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/
typedef enum
{
  FAST_LED = 200,
  MED_LED = 500,
  SLOW_LED = 800,
}leds_periods_en;

typedef struct
{
  gpio_hal_st gpio;
  soft_tim_st *tmr;
}led_hal_st;


typedef struct
{
  led_hal_st green;
  led_hal_st orange;
  led_hal_st red;
  led_hal_st blue;
}leds_hw_st;

typedef struct 
{
  leds_hw_st  hw;

  void  (*all_leds_on)(void);
  void  (*all_leds_off)(void);
  void  (*all_leds_stop)(void);
}leds_drv_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
void    LEDS_hw_init       (void);
bool    LEDS_drv_init      (leds_drv_st *handle);
void    LEDS_call_back     (gpio_hal_st *led);
void    LEDS_all_on        (void);
void    LEDS_all_off       (void);
void    LEDS_stop_all      (void);

#endif
