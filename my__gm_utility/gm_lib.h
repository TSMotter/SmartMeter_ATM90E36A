/***************************************************************************************************
* @file 	gm_lib.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/
#ifndef __GM_LIB_H__
#define __GM_LIB_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdbool.h>
#include "cmsis_os.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/
// These two are only valid for (0 - 99) values
// Transforms (0d89 or 0x59) into (0x89)
#define BinToBCD(bin) ((((bin) / 10) << 4) + ((bin) % 10))
// Transforms (0x89 or 0d137) into (0d89)
#define HexToDec(hex) ((((hex & 0xF0) >> 4) * 10) | (hex & 0x0F))

#define gAssert(func)  	 	 		          \
    		{                  		          \
    		  int _err = func; 		          \
    		  if (!_err) { return false; }  \
    		}	

#define zAssert(func)  	 	 			\
    		{                  			\
    		  int _err = func; 			\
    		  if (!_err) { break; } \
    		}	

       
/***************************************************************************************************
* Types
***************************************************************************************************/


/***************************************************************************************************
* Prototypes
***************************************************************************************************/

uint8_t GM_Byte_Coder (uint8_t stripped_nib);

void  GM_U8_TO_2ASCIIS    (uint8_t hex, char *ascii);
void  GM_U16_TO_4ASCIIS   (uint16_t hex, char *ascii);
void  GM_U32_TO_8ASCIIS   (uint32_t hex, char *ascii);
void  GM_U64_TO_16ASCIIS  (uint64_t hex, char *ascii);

/*
void          GM_SET_PWM              (TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);
void          GM_RTC_PRINT            (uint8_t vect_in[7], uint8_t *vect_out);
*/

#endif
