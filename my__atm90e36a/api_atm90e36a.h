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

/***************************************************************************************************
* Defines
***************************************************************************************************/
#define MAX_RETRIES 10
#define ATM_RTOS_DEFAULT_DELAYS	RTOS_DELAY_MS(20)
#define SIMULA_DADOS_ENERGIA 0
#define MAX_MEDIDAS_ASSINADAS 10

#define zAssert(func)  	 	 		\
    		{                  		\
    		  int _err = func; 		\
    		  if (!_err) { break; } \
    		}												

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
	AtmState_ReadingReg,

	// States for Configuration Op. Mode:
	AtmState_SoftReset,
	AtmState_ConfigFuncEn,
	AtmState_ConfigZx,
	AtmState_ConfigThresholds,
	AtmState_ConfigCS0,

	// States for Calibration Op. Mode:
	AtmState_CalibInit,
				 
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

	uint16_t			MeasuresBitMap;
	QueueHandle_t			Queue;
	atm_states_st	  	State;
  atm_op_mode_en  	Mode;
	uint16_t	      	Retry;
}ATM90_app_st;

// Must always start from 1
typedef enum
{
	voltage_rms_a = 1,
	current_rms_a,
	active_power_a,
	reactive_power_a,
	aparent_power_a,
	active_fundamental_power_a,
	active_harmonic_power_a,
	voltage_thd_a,
	current_thd_a,

	voltage_rms_b,
	current_rms_b,
	active_power_b,
	reactive_power_b,
	aparent_power_b,
	active_fundamental_power_b,
	active_harmonic_power_b,
	voltage_thd_b,
	current_thd_b,

	voltage_rms_c,
	current_rms_c,
	active_power_c,
	reactive_power_c,
	aparent_power_c,
	active_fundamental_power_c,
	active_harmonic_power_c,
	voltage_thd_c,
	current_thd_c,

	frequency,

	num_of_total_available_measures,
}measurements_id_en;

typedef struct
{
	measurements_id_en 	id;
	bool				(*read_func)(uint16_t *val);
}assinatura_de_medidas_st;


/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool 	ATM_api_init							(void);
void 	ATM_api_change_state			(atm_states_en next_state);
void 	ATM_api_check_retry				(void);
void 	ATM_api_check_hw_pins			(void);
void 	ATM_api_check_queue				(void);
void 	ATM_api_periodic_checks		(void);

void 	ATM_machine_suspended_mode	(void);
void 	ATM_machine_config_mode			(void);
void 	ATM_machine_calib_mode			(void);
void 	ATM_machine_operation_mode	(void);

#endif
