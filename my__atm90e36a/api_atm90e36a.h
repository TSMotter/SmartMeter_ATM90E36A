/***************************************************************************************************
* @file   api_atm90e36a.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __API_ATM90E36A_H__
#define __API_ATM90E36A_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "rtos_utility.h"
#include "driver_atm90e36a.h"
#include "global_configs.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/
#define MAX_RETRIES 								10
#define NUM_AMOSTRAS_MEDIA_CALIB		20									

/***************************************************************************************************
* Types
***************************************************************************************************/
// State Machine states
typedef enum
{

	// Uses 0 as an invalid value
	

	// States for Suspended Op. Mode:
	AtmState_Suspended = 1,
	AtmState_Stall,
	AtmState_HardReset,
	AtmState_ReadingReg,
	AtmState_WritingReg,

	// States for Configuration Op. Mode:
	AtmState_SoftReset,
	AtmState_ConfigFuncEn,
	AtmState_ConfigZx,
	AtmState_ConfigThresholds,
	AtmState_ConfigCS0,

	// States for Calibration Op. Mode:
	AtmState_CalibAdjStartReg,
	AtmState_CalibUoffsetAReg,
	AtmState_CalibIoffsetAReg,
	AtmState_CalibUoffsetBReg,
	AtmState_CalibIoffsetBReg,
	AtmState_CalibUoffsetCReg,
	AtmState_CalibIoffsetCReg,
	AtmState_CalibGain_all_phases_V,
	AtmState_CalibGainIa,
	AtmState_CalibGainIb,
	AtmState_CalibGainIc,
	AtmState_CalibGainVa_Inc,
	AtmState_CalibGainVa_Dec,
	AtmState_CalibGainVb_Inc,
	AtmState_CalibGainVb_Dec,
	AtmState_CalibGainVc_Inc,
	AtmState_CalibGainVc_Dec,	
	AtmState_ReadWrite_CS3,	
				 
	// States for Operation Op. Mode:
	AtmState_SubscribeMeasures,
	AtmState_Acquiring,

} atm_states_en;

typedef struct
{
  atm_states_en Current;
  atm_states_en Previous;
}atm_states_st;

typedef enum
{
	SuspendedMode,
	ConfigMode,
	CalibMode,
	OperationMode,
} atm_op_mode_en;

typedef struct 
{
	atm_drv_st	    Drv;

	QueueHandle_t			Queue;
	atm_states_st	  	State;
  atm_op_mode_en  	Mode;
	uint16_t	      	Retry;
}ATM90_app_st;

typedef struct
{
	bool 	active;
	bool	(*read_func)(uint16_t *val);
}aquisicao_de_medidas_st;


/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool 	ATM_api_init							(void);

void 	ATM_api_periodic_checks		(void);

void 	ATM_machine_suspended_mode	(void);
void 	ATM_machine_config_mode			(void);
void 	ATM_machine_calib_mode			(void);
void 	ATM_machine_operation_mode	(void);

#endif
