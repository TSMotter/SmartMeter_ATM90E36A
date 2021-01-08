/***************************************************************************************************
* @file   driver_sx1278.h
* @brief  
* @author Giuliano Motter
* @date   11/2020
***************************************************************************************************/

#ifndef __DRIVER_SX1278_H__
#define __DRIVER_SX1278_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "gm_hal_abstraction.h"
#include "consts_sx1278.h"
/***************************************************************************************************
* Defines
***************************************************************************************************/

/***************************************************************************************************
* Types
***************************************************************************************************/
typedef enum
{
  header_is_implicit = 0,
  header_is_explicit,
}header_mode_en;
typedef struct 
{
	gpio_hal_st   Reset;
	gpio_hal_st   Dio0;
	gpio_hal_st   CS;
	void          *SPI;
} sx1278_hw_st;

typedef struct
{
  sx1278_hw_st hw;

  uint32_t        frequencia;
  header_mode_en  header;

  bool (*write_data)        (uint8_t addr, uint8_t data);
  bool (*write_burst_data)  (uint8_t addr, uint8_t *data, uint8_t len);
  bool (*read_data)         (uint8_t addr, uint8_t *data);
  bool (*read_burst_data)   (uint8_t addr, uint8_t *data, uint8_t len);
  bool (*change_mode)       (uint8_t mode);
  void (*hard_reset)        (void);
  bool (*monitor_dio0)      (void);

}sx1278_drv_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
void SX1278_hw_init(void);
void SX1278_drv_init(sx1278_drv_st *drv);
bool SX1278_read_data(uint8_t addr, uint8_t *data);
bool SX1278_read_burst_data(uint8_t addr, uint8_t *data, uint8_t len);
bool SX1278_write_data(uint8_t addr, uint8_t data);
bool SX1278_write_burst_data(uint8_t addr, uint8_t *data, uint8_t len);
bool SX1278_change_mode(uint8_t mode);
bool SX1278_monitor_dio0(void);
void SX1278_hard_reset(void);

#endif
