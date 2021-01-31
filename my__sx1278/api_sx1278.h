/***************************************************************************************************
* @file   api_sx1278.h
* @brief  
* @author Giuliano Motter
* @date   11/2020
***************************************************************************************************/

#ifndef __API_SX1278_H__
#define __API_SX1278_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "rtos_utility.h"
#include "driver_sx1278.h"
#include "protocolo.h"
/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/
typedef enum
{
  SxState_Suspended = 1,
}sx1278_states_en;

typedef struct
{
  sx1278_states_en Current;
  sx1278_states_en Previous;
}sx1278_states_st;

typedef enum
{
  ListeningMode,
}sx1278_op_mode_en;

typedef struct
{
  uint8_t   data[GM_Max_Command_Len];
  uint8_t   len;
  int       rssi;  
}sx1278_packet_st;

typedef struct
{
  sx1278_drv_st Drv;

	void(*tx_done_callback)(void);

  sx1278_packet_st    Pacote;

	QueueHandle_t			  Queue;
	sx1278_states_st	  State;
	uint16_t	      	  Retry;  
}SX1278_app_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool SX_api_init(void);
bool SX_config(uint64_t frequency, uint8_t power, uint8_t LoRa_SF, 
                uint8_t LoRa_BW, uint8_t LoRa_CR, uint8_t LoRa_CRC_sum);
bool SX_EntryTxMode(uint8_t len);
bool SX_EntryRxMode(uint8_t len);
bool SX_TxPacket(uint8_t *data, uint16_t len,	uint32_t timeout);
bool SX_RxPacket(uint8_t *data, uint16_t *len, uint8_t LoRa_SF);

#endif
