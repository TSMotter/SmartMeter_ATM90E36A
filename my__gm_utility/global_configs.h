/***************************************************************************************************
* @file   global_configs.h
* @brief  This file contains all #define configurations used all across the firmware
* @author Giuliano Motter
* @date   01/2021
***************************************************************************************************/

#ifndef __GLOBAL_CONFIGS_H__
#define __GLOBAL_CONFIGS_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/

/***************************************************************************************************
* Defines
***************************************************************************************************/
// ATM90
#define SIMULA_DADOS_ENERGIA 				0
#define SIMULA_ASSINA_DADOS					0

#define USE_SAG_REAL

// LORA
//#define LORA_TEST_TX
//#define LORA_TEST_RX


/* 
 Define através de qual porta de comunicação os dados de saida de energia
 devem ser enviados

 -> Comment or uncomment to selec LORA or UART port respectivelly
*/
//#define USE_LORA_PORT

#ifndef USE_LORA_PORT
  #define USE_UART_PORT
#endif


/***************************************************************************************************
* Types
***************************************************************************************************/

/***************************************************************************************************
* Prototypes
***************************************************************************************************/


#endif
