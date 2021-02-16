/***************************************************************************************************
* @file   driver_atm90e36a.h
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __DRIVER_ATM90E36A_H__
#define __DRIVER_ATM90E36A_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>

#include "consts_atm90e36a.h"
#include "gm_hal_abstraction.h"
#include "rtos_utility.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/
#define STD_TIMEOUT_SPI 50 // (mili seconds)

/***************************************************************************************************
* Types
***************************************************************************************************/
typedef enum
{
	ATM_RC_OK,
	ATM_RC_ERROR,
	ATM_RC_TIMEOUT,

	ATM_RC_NONE,
}atm_result_code_en;

enum
{
	Va_,
	Vb_,
	Vc_,
	Ia_,
	Ib_,
	Ic_,
	Total_Params,	
};
// Presents all the pins and SPI definitions
typedef struct
{
	gpio_hal_st	 Reset;

	gpio_hal_st	 ZX0;
	gpio_hal_st	 ZX1;
	gpio_hal_st	 ZX2;

  gpio_hal_st	 CF1;
	gpio_hal_st	 CF2;
	gpio_hal_st	 CF3;
	gpio_hal_st	 CF4;

	gpio_hal_st	 WarnOut;

	gpio_hal_st	 IRQ0;
	gpio_hal_st	 IRQ1;

	gpio_hal_st	 PM0;
	gpio_hal_st	 PM1;

	gpio_hal_st	 CS;
	void 	 			*SPI;
} atm_hw_st;

typedef struct
{
	uint16_t Gain;
	uint16_t Offset;
} atm_parameter_st;

typedef struct
{
	// Hardware interaction function pointers
	atm_result_code_en (*write_reg)(uint8_t addr, uint16_t data);
	atm_result_code_en (*read_reg)(uint8_t addr, uint16_t *data);
	atm_result_code_en (*soft_reset)(void);
	void (*hard_reset)(void);
	bool (*monitor_irq0)(void);
	bool (*monitor_irq1)(void);
	bool (*monitor_warn)(void);

	atm_parameter_st Params[Total_Params];
} atm_drv_st;


/***************************************************************************************************
* Prototypes
***************************************************************************************************/
void ATM_hw_init	(void);
void ATM_drv_init	(atm_drv_st *drv);
void ATM_drv_default_gains(atm_drv_st *drv);
atm_result_code_en ATM_wait_SPI_available(int delay);

atm_result_code_en ATM_write_reg	(uint8_t addr, uint16_t data);
atm_result_code_en ATM_write_reg2	(uint8_t addr, uint16_t data);
atm_result_code_en ATM_read_reg		(uint8_t addr, uint16_t *data);
atm_result_code_en ATM_read_reg2	(uint8_t addr, uint16_t *data);
atm_result_code_en ATM_soft_reset	(void);
void ATM_hard_reset			(void);
bool ATM_IRQ0_is_set		(void);
bool ATM_IRQ1_is_set		(void);
bool ATM_WarnOut_is_set	(void);

#endif