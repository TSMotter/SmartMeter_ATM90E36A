/***************************************************************************************************
* @file   driver_atm90e36a.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <string.h>

#include "consts_atm90e36a.h"
#include "driver_atm90e36a.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static atm_result_code_en map_return_code   (HAL_StatusTypeDef rc);

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern SPI_HandleTypeDef hspi1;

/***************************************************************************************************
* Vars
***************************************************************************************************/
static atm_hw_st AtmHw = {0};

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static atm_result_code_en map_return_code(HAL_StatusTypeDef rc)
{
  if(rc == HAL_OK)
  {
    return ATM_RC_OK;
  }
  else if (rc == HAL_ERROR)
  {
    return ATM_RC_ERROR;
  }
  else if (rc == HAL_BUSY)
  {
    return ATM_RC_ERROR;
  }
  else if (rc == HAL_TIMEOUT)
  {
    return ATM_RC_TIMEOUT;
  }
  return ATM_RC_NONE;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_hw_init(void)
{
  assign_port_pin(GPIOA, GPIO_PIN_4, &AtmHw.Reset);

  assign_port_pin(NULL, 0, &AtmHw.ZX1);
  assign_port_pin(NULL, 0, &AtmHw.ZX1);
  assign_port_pin(NULL, 0, &AtmHw.ZX2);

  assign_port_pin(NULL, 0, &AtmHw.CF1);
  assign_port_pin(NULL, 0, &AtmHw.CF2);
  assign_port_pin(NULL, 0, &AtmHw.CF3);
  assign_port_pin(NULL, 0, &AtmHw.CF4);
  
  assign_port_pin(GPIOB, GPIO_PIN_0, &AtmHw.WarnOut);

  assign_port_pin(GPIOC, GPIO_PIN_4, &AtmHw.IRQ0);
  assign_port_pin(GPIOC, GPIO_PIN_5, &AtmHw.IRQ1);

  assign_port_pin(NULL, 0, &AtmHw.PM0);
  assign_port_pin(NULL, 0, &AtmHw.PM1);  

  assign_port_pin(GPIOC, GPIO_PIN_2, &AtmHw.CS);
  AtmHw.SPI = &hspi1;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_drv_init(atm_drv_st *drv)
{
  drv->write_reg = ATM_write_reg;
  drv->read_reg = ATM_read_reg;
  drv->soft_reset = ATM_soft_reset;
  drv->hard_reset = ATM_hard_reset;
  drv->monitor_irq0 = ATM_IRQ0_is_set;
  drv->monitor_irq1 = ATM_IRQ1_is_set;
  drv->monitor_warn = ATM_WarnOut_is_set;  

  ATM_drv_default_gains(drv);
}

/***************************************************************************************************
* @brief Reseta valores de ganho representados no driver para o default
***************************************************************************************************/
void ATM_drv_default_gains(atm_drv_st *drv)
{
  // Load default values
  drv->Params[Va_].Gain = 0xce40;
  drv->Params[Va_].Offset = 0;
  drv->Params[Vb_].Gain = 0xce40;
  drv->Params[Vb_].Offset = 0;
  drv->Params[Vc_].Gain = 0xce40;
  drv->Params[Vc_].Offset = 0;
  drv->Params[Ia_].Gain = 0x7530;
  drv->Params[Ia_].Offset = 0;
  drv->Params[Ib_].Gain = 0x7530;
  drv->Params[Ib_].Offset = 0;
  drv->Params[Ic_].Gain = 0x7530;
  drv->Params[Ic_].Offset = 0;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_write_reg(uint8_t addr, uint16_t data)
{
  HAL_StatusTypeDef rc = 0;
  uint16_t FullTxData[2] = {0};

  FullTxData[0] = addr;
  FullTxData[1] = data;

  if(HAL_SPI_GetState(AtmHw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&AtmHw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(AtmHw.SPI, (uint8_t *)FullTxData, 2);
    drive_gpio(&AtmHw.CS, GPIO_PIN_SET);
  }
  else
  {
    rc = HAL_BUSY;
  }

  return map_return_code(rc);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_write_reg2(uint8_t addr, uint16_t data)
{
  HAL_StatusTypeDef rc = 0;
  uint16_t FullTxData[2] = {0};

  FullTxData[0] = addr;
  FullTxData[1] = data;

  if(HAL_SPI_GetState(AtmHw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&AtmHw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit(AtmHw.SPI, (uint8_t *)FullTxData, 2, STD_TIMEOUT_SPI);
    drive_gpio(&AtmHw.CS, GPIO_PIN_SET);
  }
  else
  {
    rc = HAL_BUSY;
  }

  return map_return_code(rc);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_read_reg(uint8_t addr, uint16_t *data)
{
  HAL_StatusTypeDef rc = 0;
  uint16_t FullTxData = 0x8000;
  FullTxData |= addr;

  if(HAL_SPI_GetState(AtmHw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&AtmHw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(AtmHw.SPI, (uint8_t *)&FullTxData, 1);
    if(rc == HAL_OK)
    {
      rc = HAL_SPI_Receive_IT(AtmHw.SPI, (uint8_t *)data, 1);
    }
    drive_gpio(&AtmHw.CS, GPIO_PIN_SET);
  }
  else
  {
    rc = HAL_BUSY;
  }

  return map_return_code(rc);  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_read_reg2(uint8_t addr, uint16_t *data)
{
  HAL_StatusTypeDef rc = 0;
  uint16_t FullTxData = 0x8000;
  FullTxData |= addr;

  if(HAL_SPI_GetState(AtmHw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&AtmHw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit(AtmHw.SPI, (uint8_t *)&FullTxData, 1, STD_TIMEOUT_SPI);
    if(rc == HAL_OK)
    {
      rc = HAL_SPI_Receive(AtmHw.SPI, (uint8_t *)data, 1, STD_TIMEOUT_SPI);
    }
    drive_gpio(&AtmHw.CS, GPIO_PIN_SET);
  }
  else
  {
    rc = HAL_BUSY;
  }

  return map_return_code(rc);  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_soft_reset(void)
{
  return ATM_write_reg(ATM_REG_SoftReset_Add, 0x789A);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_hard_reset(void)
{
  drive_gpio(&AtmHw.Reset, GPIO_PIN_RESET);
  vTaskDelay(ATM_RTOS_DEFAULT_DELAYS);
  drive_gpio(&AtmHw.Reset, GPIO_PIN_SET);
  vTaskDelay(ATM_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_IRQ0_is_set(void)
{
  return read_gpio(&AtmHw.IRQ0);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_IRQ1_is_set(void)
{
  return read_gpio(&AtmHw.IRQ1);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_WarnOut_is_set(void)
{
  return read_gpio(&AtmHw.WarnOut);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
atm_result_code_en ATM_wait_SPI_available(int delay)
{
  while (delay >= 0)
  {
    if(HAL_SPI_GetState(AtmHw.SPI) == HAL_SPI_STATE_READY)
    {
      return ATM_RC_OK;
    }

    delay -= 10;
    vTaskDelay(RTOS_DELAY_MS(10));
  }
  
  return ATM_RC_TIMEOUT;
}
