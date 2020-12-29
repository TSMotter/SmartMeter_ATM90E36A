/***************************************************************************************************
* @file   ee24.h
* @brief  
* @author 
* @date   09/2020
***************************************************************************************************/

#ifndef	_EE24_H
#define	_EE24_H

/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    2.2.1
  
  
  Reversion History:
  
  (2.2.1)
  Fix erase chip bug.
  
  (2.2.0)
  Add erase chip function.
  
  (2.1.0)
  Fix write bytes.
  
  (2.0.0)
  Rewrite again.

*/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ee24Config.h"

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool    ee24_isConnected  (void);
bool    ee24_write        (uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);	
bool    ee24_read	        (uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool    ee24_eraseChip    (void);

bool    EE24_TestChip     (void);

#endif
