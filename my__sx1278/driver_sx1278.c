/***************************************************************************************************
* @file   driver_sx1278.c
* @brief  
* @author Giuliano Motter
* @date   11/2020
***************************************************************************************************/
/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "driver_sx1278.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern SPI_HandleTypeDef hspi3;
/***************************************************************************************************
* Vars
***************************************************************************************************/
sx1278_hw_st Sx1278Hw;


/***************************************************************************************************
* @brief 
***************************************************************************************************/
void SX1278_hw_init(void)
{
  assign_port_pin(GPIOD, GPIO_PIN_1, &Sx1278Hw.Reset);
  assign_port_pin(GPIOD, GPIO_PIN_2, &Sx1278Hw.Dio0);

  assign_port_pin(GPIOD, GPIO_PIN_0, &Sx1278Hw.CS);
  Sx1278Hw.SPI = &hspi3;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void SX1278_drv_init(sx1278_drv_st *drv)
{
  drv->hw = Sx1278Hw;
  drv->read_data = SX1278_read_data;
  drv->read_burst_data = SX1278_read_burst_data;
  drv->write_data = SX1278_write_data;
  drv->write_burst_data = SX1278_write_burst_data;
  drv->monitor_dio0 = SX1278_monitor_dio0;
  drv->hard_reset = SX1278_hard_reset;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX1278_read_data(uint8_t addr, uint8_t *data)
{
  HAL_StatusTypeDef rc = 0;

  if(HAL_SPI_GetState(Sx1278Hw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(Sx1278Hw.SPI, &addr, 1);
    if(rc == HAL_OK)
    {
      rc = HAL_SPI_Receive_IT(Sx1278Hw.SPI, data, 1);
    }
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_SET);
    if(rc == HAL_OK)
    {
      return true;
    }    
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX1278_read_burst_data(uint8_t addr, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef rc = 0;

  if(HAL_SPI_GetState(Sx1278Hw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(Sx1278Hw.SPI, &addr, 1);
    if(rc == HAL_OK)
    {
      for (uint8_t i = 0; i < len; i++) 
      {
        rc = HAL_SPI_Receive_IT(Sx1278Hw.SPI, (data + i), 1);
      }
    }
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_SET);
    if(rc == HAL_OK)
    {
      return true;
    }    
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX1278_write_data(uint8_t addr, uint8_t data)
{
  HAL_StatusTypeDef rc = 0;
  uint8_t FullTxData[2] = {0x80, data};
  FullTxData[0] |= addr;


  if(HAL_SPI_GetState(Sx1278Hw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(Sx1278Hw.SPI, FullTxData, 2);
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_SET);
    if(rc == HAL_OK)
    {
      return true;
    }
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX1278_write_burst_data(uint8_t addr, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef rc = 0;
  uint8_t FullAddr = 0x80;
  FullAddr |= addr;

	if (len <= 1) 
  {
		return false;
	} 

  if(HAL_SPI_GetState(Sx1278Hw.SPI) == HAL_SPI_STATE_READY)
  {
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_RESET);
    rc = HAL_SPI_Transmit_IT(Sx1278Hw.SPI, &FullAddr, 1);
    for (uint8_t i = 0; i < len; i++) 
    {
			HAL_SPI_Transmit_IT(Sx1278Hw.SPI, (data + i), 1);
		}
    drive_gpio(&Sx1278Hw.CS, GPIO_PIN_SET);
    if(rc == HAL_OK)
    {
      return true;
    }
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX1278_monitor_dio0(void)
{
  return read_gpio(&Sx1278Hw.Dio0);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void SX1278_hard_reset(void)
{
  drive_gpio(&Sx1278Hw.Reset, GPIO_PIN_RESET);
  HAL_Delay(200);
  drive_gpio(&Sx1278Hw.Reset, GPIO_PIN_SET);  
}
