/***************************************************************************************************
* @file   api_uart.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/
#ifndef __API_UART__
#define __API_UART__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "gm_lib.h"
#include "ring_buffer.h"
#include "rtos_utility.h"
#include "stm32f4xx_hal.h"
#include "global_configs.h"
#include "protocolo.h"
/***************************************************************************************************
* Defines
***************************************************************************************************/
#define RB_Size 128 	// (in bytes)

/* 
#define ATM_PHASE_A 	1
#define ATM_PHASE_B 	2
#define ATM_PHASE_C 	3
#define UNIDADE_V     "Vrms*"
#define UNIDADE_A     "Arms*"
#define UNIDADE_kW    "kW*"
#define UNIDADE_kVar  "kVar*"
#define UNIDADE_kVA   "kVA*"
 */
/***************************************************************************************************
* Types
***************************************************************************************************/

typedef struct
{
  RINGBUFF_T          *Rb;
  QueueHandle_t		    Queue;
  QueueHandle_t       EnergyQueue;
  SemaphoreHandle_t   SemEOF;
  UART_HandleTypeDef  *huart;
}UART_app_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool UART_api_init  (void);

void UART_Parser_Comando_Burro  (uint8_t *pbyRxBuff);
void UART_Recebe_Comando        (void);
bool UART_Envia_Eventos         (void);
void UART_check_queue           (void);
void UART_check_EnergyQueue     (void);

/* 
uint8_t Convert_To_Print_Timestamp  (uint32_t dwDadoIn, char *pbyFullOutputVect);
uint8_t Convert_To_Print_TypeA      (uint16_t wDadoIn, char *pbyFullOutputVect, char *Unidade, uint8_t byPhase);
uint8_t Convert_To_Print_TypeB      (uint16_t wDadoIn, uint8_t Indentificador, char *pbyFullOutputVect, char *Unidade, uint8_t byPhase);
uint8_t Convert_To_Print_TypeC      (uint16_t wDadoIn, char *pbyFullOutputVect, uint8_t byPhase);
void    Convert_To_Print_CrLf       (char *pbyFullOutputVect);

#define GM_CONVERT_TO_PRINT_Va(DadoIn, OutputVect) Convert_To_Print_TypeA(DadoIn, OutputVect, UNIDADE_V, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_Vb(DadoIn, OutputVect) Convert_To_Print_TypeA(DadoIn, OutputVect, UNIDADE_V, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_Vc(DadoIn, OutputVect) Convert_To_Print_TypeA(DadoIn, OutputVect, UNIDADE_V, ATM_PHASE_C)

#define GM_CONVERT_TO_PRINT_Ia(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'I', OutputVect, UNIDADE_A, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_Ib(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'I', OutputVect, UNIDADE_A, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_Ic(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'I', OutputVect, UNIDADE_A, ATM_PHASE_C)

#define GM_CONVERT_TO_PRINT_Pa(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'P', OutputVect, UNIDADE_kW, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_Pb(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'P', OutputVect, UNIDADE_kW, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_Pc(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'P', OutputVect, UNIDADE_kW, ATM_PHASE_C)

#define GM_CONVERT_TO_PRINT_Qa(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'Q', OutputVect, UNIDADE_kVar, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_Qb(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'Q', OutputVect, UNIDADE_kVar, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_Qc(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'Q', OutputVect, UNIDADE_kVar, ATM_PHASE_C)

#define GM_CONVERT_TO_PRINT_Sa(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'S', OutputVect, UNIDADE_kVA, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_Sb(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'S', OutputVect, UNIDADE_kVA, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_Sc(DadoIn, OutputVect) Convert_To_Print_TypeB(DadoIn, (uint8_t)'S', OutputVect, UNIDADE_kVA, ATM_PHASE_C)

#define GM_CONVERT_TO_PRINT_PFa(DadoIn, OutputVect) Convert_To_Print_TypeC(DadoIn, OutputVect, ATM_PHASE_A)
#define GM_CONVERT_TO_PRINT_PFb(DadoIn, OutputVect) Convert_To_Print_TypeC(DadoIn, OutputVect, ATM_PHASE_B)
#define GM_CONVERT_TO_PRINT_PFc(DadoIn, OutputVect) Convert_To_Print_TypeC(DadoIn, OutputVect, ATM_PHASE_C)
 */
#endif
