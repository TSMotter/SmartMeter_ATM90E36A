/***************************************************************************************************
* @file   rtos_utility.h
* @brief  Common infrastructure to access RTOS resources
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __RTOS_UTILITY_H__
#define __RTOS_UTILITY_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gm_lib.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/
//------------------------------------------------------------------------
// LISTA DE COMANDOS UART -> ATM
//------------------------------------------------------------------------
// Comando 1
#define Cmd_ConfigMode 1
// Comando 2
#define Cmd_CalibrationMode 2
//------ sub commands ------
#define subCmd_AdjStart  0
#define subCmd_OffSet_Va 1
#define subCmd_Offset_Ia 2
#define subCmd_Offset_Vb 3
#define subCmd_Offset_Ib 4
#define subCmd_Offset_Vc 5
#define subCmd_Offset_Ic 6
#define subCmd_Gain_all_phases_V 7
#define subCmd_Gain_all_phases_I 8
#define subCmd_Read_WriteCS3 9
// Comando 3
#define Cmd_OperationMode 3
// Comando 4
#define Cmd_SuspendedMode 4
// Comando 5
#define Cmd_ReadSpecificRegister 5
//------ sub commands ------
// eg: reg 0x6A --> subCmd = 106
// Comando 6
#define Cmd_Reset 6
//------ sub commands ------
#define subCmd_SoftwareReset 1
#define subCmd_HardwareReset 2

//------------------------------------------------------------------------
// LISTA DE COMANDOS ATM -> UART
//------------------------------------------------------------------------
// Comando 1 (Comando generico para printar n bytes)
#define Cmd_PrintThis 1
// Comando 2
#define Cmd_CheckSumERROR 2
//------ sub commands ------
#define subCmd_CS0 0x40 // mascaras para os bits do reg. SysStatus0, rodado.
#define subCmd_CS1 0x10 // (assumindo que so possa ter 1 CS errado por vez)
#define subCmd_CS2 0x04
#define subCmd_CS3 0x01

//------------------------------------------------------------------------
// LISTA DE COMANDOS ATM -> LEDS
//------------------------------------------------------------------------
#define Cmd_BlinkPattern1	1
#define Cmd_BlinkPattern2	2
#define Cmd_BlinkPattern3 3
//#define Cmd_Blink	2
//#define Cmd_Blink 3
//#define Cmd_Blink 4
//#define Cmd_Blink	5


/***************************************************************************************************
* Types
***************************************************************************************************/

// Indice para acessar as filas, alocadas no vetor "QueueHandleVECTOR[]"
typedef enum
{
  QueueIDX_ATM,
  QueueIDX_UART,
  QueueIDX_LEDS,
  QueueIDX_ENERGY,

  QueueIDX_NUMBER_OF_QUEUES,
} RTOS_QueueIdx_en;

// Enumerate que definira qual a origem e destino de um evento de fila
typedef enum
{
  EvntFromATMtoUART,
  EvntFromATMtoLEDS,
  EvntFromATMtoENERGY,

  EvntFromUARTtoATM,
  EvntFromUARTtoPCF,
  EvntFromUARTtoLEDS,
} RTOS_Events_en;

// Estrutura basica de comunicacao entre Tasks
typedef struct
{
  RTOS_Events_en  enEvent;    // Evento
  uint8_t         byCmd;      // Commando
  uint8_t         bySubCmd;   // Sub-Commando
  uint8_t         *pbyData;   // Ponteiro para os dados
  uint16_t        wDataLen;   // Quantidade de dados
} GenericQueueData_st;


// Estrutura basica de comunicacao especifica para dados de ENERGIA
typedef struct
{
  RTOS_Events_en  enEvent;  // Evento
  uint16_t        *pwData;  // Ponteiro para os dados
  uint16_t        wDataLen; // Quantidade de dados
} EnergyQueueData_st;

/***************************************************************************************************
* Macros
***************************************************************************************************/
#define SIZE_OF_QUEUE_ITEM 		      sizeof(GenericQueueData_st)
#define SIZE_OF_ENERGY_QUEUE_ITEM 	sizeof(EnergyQueueData_st)
#define RTOS_DELAY_MS(t)		        (t / portTICK_RATE_MS)

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
QueueHandle_t *RTOS_Get_Queue_Idx (RTOS_QueueIdx_en QueueIDX);
bool RTOS_Send_Data_To_Specific_Queue(xQueueHandle* posQueHandle, GenericQueueData_st* pstEvt, portTickType xTicksToWait);
bool RTOS_Send_Data_To_Energy_Queue(xQueueHandle* posQueHandle, EnergyQueueData_st* pstEvt, portTickType xTicksToWait);

#endif