/***************************************************************************************************
* @file   api_leds.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "driver_leds.h"
#include "stm32f407xx.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

/***************************************************************************************************
* Externals
***************************************************************************************************/

/***************************************************************************************************
* Vars
***************************************************************************************************/
static leds_hw_st KitLedsHw = {0};

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_hw_init(void)
{
  assign_port_pin(GPIOD, GPIO_PIN_12, &KitLedsHw.green.gpio);
  assign_port_pin(GPIOD, GPIO_PIN_13, &KitLedsHw.orange.gpio);
  assign_port_pin(GPIOD, GPIO_PIN_14, &KitLedsHw.red.gpio);
  assign_port_pin(GPIOD, GPIO_PIN_15, &KitLedsHw.blue.gpio);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LEDS_drv_init(leds_drv_st *handle)
{
  SofTim_InitializeModule();

  KitLedsHw.green.tmr = SofTim_AllocateTimer(SLOW_LED, true, &LEDS_call_back, &KitLedsHw.green.gpio, 0);
  KitLedsHw.orange.tmr = SofTim_AllocateTimer(SLOW_LED, true, &LEDS_call_back, &KitLedsHw.orange.gpio, 0);
  KitLedsHw.red.tmr = SofTim_AllocateTimer(SLOW_LED, true, &LEDS_call_back, &KitLedsHw.red.gpio, 0);
  KitLedsHw.blue.tmr = SofTim_AllocateTimer(SLOW_LED, true, &LEDS_call_back, &KitLedsHw.blue.gpio, 0);

  if((KitLedsHw.green.tmr == NULL)||(KitLedsHw.orange.tmr == NULL)||(KitLedsHw.red.tmr == NULL)||(KitLedsHw.blue.tmr == NULL))
  {
    return false;
  }

  handle->hw = KitLedsHw;
  handle->all_leds_on = LEDS_all_on;
  handle->all_leds_off = LEDS_all_off;  
  handle->all_leds_stop = LEDS_stop_all;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_call_back(gpio_hal_st *led)
{
  toggle_gpio(led);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_all_on(void)
{
  drive_gpio(&KitLedsHw.green.gpio, 1);
  drive_gpio(&KitLedsHw.orange.gpio, 1);
  drive_gpio(&KitLedsHw.red.gpio, 1);
  drive_gpio(&KitLedsHw.blue.gpio, 1);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_all_off(void)
{
  drive_gpio(&KitLedsHw.green.gpio, 0);
  drive_gpio(&KitLedsHw.orange.gpio, 0);
  drive_gpio(&KitLedsHw.red.gpio, 0);
  drive_gpio(&KitLedsHw.blue.gpio, 0);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_stop_all(void)
{
  SofTim_StopTimer(KitLedsHw.green.tmr);
  SofTim_StopTimer(KitLedsHw.orange.tmr);
  SofTim_StopTimer(KitLedsHw.red.tmr);
  SofTim_StopTimer(KitLedsHw.blue.tmr);
}