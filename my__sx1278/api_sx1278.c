/***************************************************************************************************
* @file   api_sx1278.C
* @brief  
* @author Giuliano Motter
* @date   11/2020
***************************************************************************************************/
/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_sx1278.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

/***************************************************************************************************
* Externals
***************************************************************************************************/

/***************************************************************************************************
* Vars
***************************************************************************************************/
SX1278_app_st SX1278 = {0};

static QueueHandle_t *Queue_SX_HANDLE = NULL;

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_api_init(void)
{
  // Cria fila

  SX1278_hw_init();

  SX1278_drv_init(&SX1278.Drv);

  SX1278.Queue = *Queue_SX_HANDLE;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_config(uint64_t frequency, uint8_t power, uint8_t LoRa_SF, 
                uint8_t LoRa_BW, uint8_t LoRa_CR, uint8_t LoRa_CRC_sum)
{
  SX1278.Drv.hard_reset();

  // Enter sleep mode so that I can change module op mode
  gAssert(SX1278.Drv.change_mode(MODE_SLEEP));
  vTaskDelay(5*SX_RTOS_DEFAULT_DELAYS);
  gAssert(SX1278.Drv.change_mode(MODE_LORA_MODE));

	uint64_t freq = ((uint64_t) frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
  gAssert(SX1278.Drv.write_burst_data(LR_RegFrMsb, freq_reg, 3));
 

  // Value 0x34 is reserved for LoRaWAN networks
  //gAssert(SX1278.Drv.write_data(RegSyncWord, 0x34));

  gAssert(SX1278.Drv.write_data(LR_RegPaConfig, SX1278_Power[power]));

  // Desliga OCP (Over current protection) para PA
  gAssert(SX1278.Drv.write_data(LR_RegOcp, 0x0B));

  // Coloca ganho maximo e Boost On (150% LNA current)
  gAssert(SX1278.Drv.write_data(LR_RegLna, 0x23));

	if (SX1278_SpreadFactor[LoRa_SF] == 6) 
  {	
		uint8_t tmp = 0;
    tmp = ((SX1278_LoRaBandwidth[LoRa_BW] << 4) + (SX1278_CodingRate[LoRa_CR] << 1) + 0x01);
    gAssert(SX1278.Drv.write_data(LR_RegModemConfig1, tmp));
    tmp = ((SX1278_SpreadFactor[LoRa_SF] << 4) + (SX1278_CRC_Sum[LoRa_CRC_sum] << 2) + 0x03);
    gAssert(SX1278.Drv.write_data(LR_RegModemConfig2, tmp));
    uint8_t aux = 0;
    gAssert(SX1278.Drv.read_data(0x31, &aux));
    aux &= 0xf8;
		aux |= 0x05;
    gAssert(SX1278.Drv.write_data(0x31, aux));
    gAssert(SX1278.Drv.write_data(0x37, 0x0c));
  }
  else
  {
    uint8_t tmp = 0;
    tmp = ((SX1278_LoRaBandwidth[LoRa_BW] << 4) + (SX1278_CodingRate[LoRa_CR] << 1) + 0x00);
    gAssert(SX1278.Drv.write_data(LR_RegModemConfig1, tmp));
    tmp = ((SX1278_SpreadFactor[LoRa_SF] << 4) + (SX1278_CRC_Sum[LoRa_CRC_sum] << 2) + 0x00);
    gAssert(SX1278.Drv.write_data(LR_RegModemConfig2, tmp));
  }
  

  // Habilita AGC (automatic gain control)
  gAssert(SX1278.Drv.write_data(LR_RegModemConfig3, 0x04));
  //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
  //gAssert(SX1278.Drv.write_data(LR_RegSymbTimeoutLsb, 0x08));
  //RegDioMapping2 DIO5=00, DIO4=01
  gAssert(SX1278.Drv.write_data(REG_LR_DIOMAPPING2, 0x01));

  gAssert(SX1278.Drv.change_mode(MODE_STDBY));
  
  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_EntryTxMode(uint8_t len)
{
  uint8_t aux = 0;

  gAssert(SX1278.Drv.write_data(REG_LR_PADAC, 0x87));
  gAssert(SX1278.Drv.write_data(LR_RegHopPeriod, 0x00));
  gAssert(SX1278.Drv.write_data(REG_LR_DIOMAPPING1, 0x41));
  gAssert(SX1278.Drv.write_data(LR_RegIrqFlags, 0xff));
  gAssert(SX1278.Drv.write_data(LR_RegIrqFlagsMask, 0xf7));
  gAssert(SX1278.Drv.write_data(LR_RegPayloadLength, len));
  gAssert(SX1278.Drv.read_data(LR_RegFifoTxBaseAddr, &aux));
  gAssert(SX1278.Drv.write_data(LR_RegFifoAddrPtr, aux));

  uint8_t readVal = 0;
  for(uint8_t i = 0; i < 10; i++)
  {
    gAssert(SX1278.Drv.read_data(LR_RegPayloadLength, &readVal));
    if(readVal == len)
    {
      return true; // Sucesso
    }
    
    vTaskDelay(SX_RTOS_DEFAULT_DELAYS);
  }

  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_EntryRxMode(uint8_t len)
{
  uint8_t aux = 0;

  gAssert(SX1278.Drv.write_data(REG_LR_PADAC, 0x84));
  gAssert(SX1278.Drv.write_data(LR_RegHopPeriod, 0xff));
  gAssert(SX1278.Drv.write_data(REG_LR_DIOMAPPING1, 0x01));
  gAssert(SX1278.Drv.write_data(LR_RegIrqFlags, 0x3f));
  gAssert(SX1278.Drv.write_data(LR_RegIrqFlagsMask, 0xf7));
  gAssert(SX1278.Drv.write_data(LR_RegPayloadLength, len));
  gAssert(SX1278.Drv.read_data(LR_RegFifoRxBaseAddr, &aux));
  gAssert(SX1278.Drv.write_data(LR_RegFifoAddrPtr, aux));

  gAssert(SX1278.Drv.change_mode(MODE_LORA_MODE | MODE_RX_CONTINUOUS));

  uint8_t readVal = 0;
  for(uint8_t i = 0; i < 10; i++)
  {
    gAssert(SX1278.Drv.read_data(LR_RegModemStat, &readVal));
    if((readVal & 0x04) == 0x04)
    {
      return true; // Sucesso
    }
    vTaskDelay(SX_RTOS_DEFAULT_DELAYS);
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_TxPacket(uint8_t *data, uint16_t len,	uint32_t timeout)
{
  gAssert(SX1278.Drv.write_burst_data(0x00, data, len));

  // Tx mode
  gAssert(SX1278.Drv.change_mode(MODE_LORA_MODE | MODE_TX));

  while(true)
  {
    if(SX1278.Drv.monitor_dio0())
    {
      gAssert(SX1278.Drv.read_data(LR_RegIrqFlags, NULL));
      gAssert(SX1278.Drv.write_data(LR_RegIrqFlags, 0xff));

      gAssert(SX1278.Drv.change_mode(MODE_STDBY));

      return true;
    }

    if(--timeout == 0)
    {
      return false;
    }
    vTaskDelay(SX_RTOS_DEFAULT_DELAYS);
  }

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool SX_RxPacket(uint8_t *data, uint16_t *len, uint8_t LoRa_SF)
{
  uint8_t local_data = 0;

  if(SX1278.Drv.monitor_dio0())
  {
    gAssert(SX1278.Drv.read_data(LR_RegFifoRxCurrentaddr, &local_data));
    gAssert(SX1278.Drv.write_data(LR_RegFifoAddrPtr, local_data))

    if(LoRa_SF == SX1278_LORA_SF_6)
    {
      //*len = fixo
    }
    else
    {
      gAssert(SX1278.Drv.read_data(LR_RegRxNbBytes, (uint8_t*)len));
    }

    gAssert(SX1278.Drv.read_burst_data(0x00, data, *len));
    gAssert(SX1278.Drv.write_data(LR_RegIrqFlags, 0xff));
    return true;
  }
  return false;
}
