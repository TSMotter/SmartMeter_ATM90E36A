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

enum
{
	voltage_rms_a = 0,
	current_rms_a,
	active_power_a,
	reactive_power_a,
	aparent_power_a,
	active_fundamental_power_a,
	active_harmonic_power_a,
	voltage_thdn_a,
	current_thdn_a,
	voltage_2_order_harmonic_ratio_a,
	voltage_3_order_harmonic_ratio_a,
	voltage_4_order_harmonic_ratio_a,
	voltage_5_order_harmonic_ratio_a,
	voltage_thd_a, // phase A, Voltage, Total Harmonic Distortion Ratio
	current_2_order_harmonic_ratio_a,
	current_3_order_harmonic_ratio_a,
	current_4_order_harmonic_ratio_a,
	current_5_order_harmonic_ratio_a,	
	current_thd_a, // phase A, Current, Total Harmonic Distortion Ratio

	voltage_rms_b,
	current_rms_b,
	active_power_b,
	reactive_power_b,
	aparent_power_b,
	active_fundamental_power_b,
	active_harmonic_power_b,
	voltage_thdn_b,
	current_thdn_b,

	voltage_rms_c,
	current_rms_c,
	active_power_c,
	reactive_power_c,
	aparent_power_c,
	active_fundamental_power_c,
	active_harmonic_power_c,
	voltage_thdn_c,
	current_thdn_c,

	frequency,

	numero_total_medidas,
};

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool assina_medidas  (uint16_t id);
bool realiza_medidas (void);

#endif
