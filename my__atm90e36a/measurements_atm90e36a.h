/***************************************************************************************************
* @file   measurements_atm90e36a.h
* @brief  Arquivo com funções de aquisição de medidas do atm90e36a
* @author Giuliano Motter
* @date   03/2021
***************************************************************************************************/

#ifndef __MEASUREMENTS_ATM90E36A_H__
#define __MEASUREMENTS_ATM90E36A_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_atm90e36a.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool assina_medidas  (uint16_t id);
bool realiza_medidas (void);

#endif