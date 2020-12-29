/***************************************************************************************************
* @file   ee24Config.h
* @brief
* @author
* @date   09/2020
***************************************************************************************************/

#ifndef	_EE24CONFIG_H
#define	_EE24CONFIG_H

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
/***************************************************************************************************
* Defines
***************************************************************************************************/
#define		_EEPROM_SIZE_KBIT							32
#define		_EEPROM_I2C   								hi2c2			
#define		_EEPROM_USE_FREERTOS          1
#define		_EEPROM_ADDRESS               0xA0
#define		_EEPROM_USE_WP_PIN            0
#define   WRITE_SIZE_TEST               32

#if (_EEPROM_USE_WP_PIN==1)
#define		_EEPROM_WP_GPIO								EE_WP_GPIO_Port
#define		_EEPROM_WP_PIN								EE_WP_Pin
#endif

#endif

