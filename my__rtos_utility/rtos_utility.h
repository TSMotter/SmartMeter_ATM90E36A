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
// Comandos
enum
{
  Cmd_ConfigMode = 1,
  Cmd_CalibrationMode, 
  Cmd_OperationMode,
  Cmd_SuspendedMode,
  Cmd_ReadSpecificRegister,
  Cmd_WriteSpecificRegister,
  Cmd_Reset,
  Cmd_SignMeasurements,
};

// Sub comandos para Cmd_CalibrationMode
enum
{
  subCmd_AdjStart = 1,
  subCmd_OffSet_Va,
  subCmd_Offset_Ia,
  subCmd_Offset_Vb,
  subCmd_Offset_Ib,
  subCmd_Offset_Vc,
  subCmd_Offset_Ic,
  subCmd_Gain_all_phases_V,
  subCmd_Gain_all_phases_I,
  subCmd_Read_WriteCS3,
};

// Sub comandos para Cmd_Reset
enum
{
  subCmd_SoftwareReset = 1,
  subCmd_HardwareReset,  
};

//------------------------------------------------------------------------
// LISTA DE COMANDOS UART -> LEDS
//------------------------------------------------------------------------
// Comandos
enum
{
  Cmd_TestEEPROM = 1,
};

//------------------------------------------------------------------------
// LISTA DE COMANDOS ATM -> UART
//------------------------------------------------------------------------
// Comandos
enum
{
  Cmd_PrintThis = 1,
};

// Sub comandos
enum
{
  subCmd_print_start_msg = 0,
  subCmd_print_this,
  subCmd_print_warn,
  subCmd_print_irq0,
  subCmd_print_irq1,
  subCmd_print_line_feed,
  subCmd_print_teste_eeprom_ok,
  subCmd_print_teste_eeprom_error,
};

//------------------------------------------------------------------------
// LISTA DE COMANDOS ATM -> LEDS
//------------------------------------------------------------------------
// Comandos
enum
{
  Cmd_BlinkPattern1 = 1,
  Cmd_BlinkPattern2,
  Cmd_BlinkPattern3,
};

// Sub comandos 
enum
{
  subCmd_BlinkSpeed_Slow = 800,
  subCmd_BlinkSpeed_Medium = 500,
  subCmd_BlinkSpeed_Fast = 200,
};

//------------------------------------------------------------------------
// LISTA DE COMANDOS ATM -> LORA
//------------------------------------------------------------------------
// Comandos
enum
{
  Cmd_SendEnergyData = 1,
};

// Sub comandos 


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
  QueueIDX_LORA,

  QueueIDX_NUMBER_OF_QUEUES,
} RTOS_QueueIdx_en;

// Enumerate que definira qual a origem e destino de um evento de fila
typedef enum
{
  EvntFromATMtoUART,
  EvntFromATMtoLEDS,
  EvntFromATMtoLORA,

  EvntFromUARTtoATM,
  EvntFromUARTtoPCF,
  EvntFromUARTtoLEDS,

  EvntFromLEDStoUART,
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
  uint32_t        Data; 
} EnergyQueueData_st;

/***************************************************************************************************
* Macros
***************************************************************************************************/
#define SIZE_OF_QUEUE_ITEM 		      sizeof(GenericQueueData_st)
#define SIZE_OF_ENERGY_QUEUE_ITEM 	sizeof(EnergyQueueData_st)
#define RTOS_DELAY_MS(t)		        (t / portTICK_RATE_MS)

//------------------------------------------------------------------------
// Delays padrao das tasks
//------------------------------------------------------------------------
#define ATM_RTOS_DEFAULT_DELAYS			RTOS_DELAY_MS(20)
#define SX_RTOS_DEFAULT_DELAYS			RTOS_DELAY_MS(50)
#define LEDS_RTOS_DEFAULT_DELAYS    RTOS_DELAY_MS(20)
#define UART_RTOS_DEFAULT_DELAYS    0

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
QueueHandle_t *RTOS_Get_Queue_Idx (RTOS_QueueIdx_en QueueIDX);
bool RTOS_Send_Data_To_Specific_Queue(xQueueHandle* posQueHandle, GenericQueueData_st* pstEvt, portTickType xTicksToWait);
bool RTOS_Send_Data_To_Energy_Queue(uint32_t* data, portTickType xTicksToWait);

#endif