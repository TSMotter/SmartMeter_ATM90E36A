/***************************************************************************************************
* @file   api_atm90e36a.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_atm90e36a.h"
#include <string.h>

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static bool ATM_config_reg_MMode0(void);
static bool ATM_config_reg_MMode1(void);
static bool ATM_config_reg_CS(uint16_t reg);

static bool walk1_machine(bool method, atm_states_en next_state);
static bool walk2_machine(atm_result_code_en method, atm_states_en next_state);
static bool walk3_machine(uint16_t addr, uint16_t value, atm_states_en next_state);
/***************************************************************************************************
* Externals
***************************************************************************************************/
extern SemaphoreHandle_t SyncMeasureSem;

/***************************************************************************************************
* Vars
***************************************************************************************************/
ATM90_app_st ATM = {0};
static QueueHandle_t *Queue_ATM_HANDLE = NULL;

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_api_init(void)
{
  ATM_hw_init();

  ATM_drv_init(&ATM.Drv);

	// Cria fila ATM
	Queue_ATM_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_ATM);
	*Queue_ATM_HANDLE = xQueueCreate(32, SIZE_OF_QUEUE_ITEM);
	vQueueAddToRegistry(*Queue_ATM_HANDLE, "Fila_ATM");
  
  if(Queue_ATM_HANDLE == NULL)
  {
    return false;
  }

  ATM.State.Current = AtmState_Suspended;
  ATM.Mode = SuspendedMode;
  ATM.Queue = *Queue_ATM_HANDLE;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_api_change_state(atm_states_en next_state)
{
  ATM.State.Previous = ATM.State.Current;
  ATM.State.Current = next_state;
  ATM.Retry = 0;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_api_check_retry(void)
{
  if (ATM.Retry >= MAX_RETRIES)
  {
    ATM.State.Current = AtmState_Suspended;
    ATM.Mode = SuspendedMode;
    ATM.Retry = 0;
  }  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_api_check_hw_pins(void)
{
  bool flag = false;

  if(ATM.Drv.monitor_warn())
  {
    flag = true;
    uint16_t data = 0;
    ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data);
    // Send event to UART to print error msg
    ATM_send_event_to_uart(Cmd_PrintThis, 1, NULL, 0);
  }
  else if(ATM.Drv.monitor_irq0())
  {
    flag = true;
    uint16_t data = 0;
    ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data);
    #warning Print data as well
    // Send Event to UART
    ATM_send_event_to_uart(Cmd_PrintThis, 2, NULL, 0);
  }
  else if(ATM.Drv.monitor_irq1())
  {
    flag = true;
    uint16_t data = 0;
    ATM.Drv.read_reg(ATM_REG_SysStatus1_Add, &data);

    // Send Event to UART
    ATM_send_event_to_uart(Cmd_PrintThis, 3, NULL, 0);
  }  
  if(flag)
  {
    ATM.Mode = SuspendedMode;
    ATM_api_change_state(AtmState_Suspended);
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_api_check_queue(void)
{
  GenericQueueData_st NewEvent;  

  if (xQueueReceive(ATM.Queue, &NewEvent, ATM_RTOS_DEFAULT_DELAYS) != pdPASS)
  {
	  return;
  }

  switch(NewEvent.enEvent)
  {
	  case EvntFromUARTtoATM:
		  switch (NewEvent.byCmd)
		  {
        //----------------------------------------------
        case Cmd_ConfigMode:
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = ConfigMode;
        break;
        //----------------------------------------------
        case Cmd_CalibrationMode:
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = CalibMode;
        break;
        //----------------------------------------------
        case Cmd_SuspendedMode:
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = SuspendedMode;
        break;
        //----------------------------------------------
        case Cmd_OperationMode:
          ATM.MeasuresBitMap = NewEvent.wDataLen;
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = OperationMode;
        break;
        //----------------------------------------------
        default:
        break;    
      }
    default:
    break;
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_api_periodic_checks(void)
{
  if(ATM.Mode != SuspendedMode)
  {
    ATM_api_check_hw_pins();
    ATM_api_check_retry();
  }

  ATM_api_check_queue();
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool ATM_config_reg_MMode0(void)
{
  uint16_t LocalReadVal = 0;
  
  uint16_t data =  0;

	data |= ((ATM_REG_MMode0_Msk_Freq60Hz |  ATM_REG_MMode0_Msk_didtEn | ATM_REG_MMode0_Msk_001LSB |
            ATM_REG_MMode0_Msk_CF2varh  | ATM_REG_MMode0_Msk_ABSEnQ  | ATM_REG_MMode0_Msk_ABSEnP |
			      ATM_REG_MMode0_Msk_EnPA     | ATM_REG_MMode0_Msk_EnPB    | ATM_REG_MMode0_Msk_EnPC));
  

  if(ATM.Drv.write_reg(ATM_REG_MMode0_Add, data) == ATM_RC_OK)
  {
    if(ATM.Drv.read_reg(ATM_REG_MMode0_Add, &LocalReadVal) == ATM_RC_OK)
    {
      if(LocalReadVal == data)
      {
        return true;
      }
    }
  }

  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool ATM_config_reg_MMode1(void)
{
  uint16_t LocalReadVal = 0;
  
  uint16_t data = ( (ATM_REG_MMode1_Pos_DPGA_GAIN << ATM_REG_MMode1_Val_Gain1) | 
                    (ATM_REG_MMode1_Pos_PGA_GAIN_V3 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_V2 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_V2 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_V1 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_I4 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_I3 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_I2 << ATM_REG_MMode1_Val_1x)  |
                    (ATM_REG_MMode1_Pos_PGA_GAIN_I1 << ATM_REG_MMode1_Val_1x));
  

  if(ATM.Drv.write_reg(ATM_REG_MMode1_Add, data) == ATM_RC_OK)
  {
    if(ATM.Drv.read_reg(ATM_REG_MMode1_Add, &LocalReadVal) == ATM_RC_OK)
    {
      if(LocalReadVal == data)
      {
        return true;
      }
    }
  }

  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool ATM_config_reg_CS(uint16_t reg)
{
  uint16_t LocalReadVal = 0;

  if(ATM.Drv.read_reg(reg, &LocalReadVal) == ATM_RC_OK)
  {
    if(ATM.Drv.write_reg(reg, LocalReadVal) == ATM_RC_OK)
    {
      return true;  
    }
  }

  ATM.Retry++;
  return false;  
}











/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool walk1_machine(bool method, atm_states_en next_state)
{
  if(method)
  {
    if(next_state)
    {
      ATM_api_change_state(next_state);
    }
    return true;
  }
  else
  {
    ATM.Retry++;
    return false;
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool walk2_machine(atm_result_code_en method, atm_states_en next_state)
{
  if(method == ATM_RC_OK)
  {
    if(next_state)
    {
      ATM_api_change_state(next_state);
    }
    return true;
  }
  else
  {
    ATM.Retry++;
    return false;
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool walk3_machine(uint16_t addr, uint16_t value, atm_states_en next_state)
{
  uint16_t LocalReadVal = 0;

  if(ATM.Drv.write_reg(addr, value) == ATM_RC_OK)
  {
    if(ATM.Drv.read_reg(addr, &LocalReadVal) == ATM_RC_OK)
    {
      if(LocalReadVal == value)
      {
        if(next_state)
        {
          ATM_api_change_state(next_state);
        }
        return true;
      }
    }
  }
  ATM.Retry++;
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_machine_suspended_mode(void)
{
  switch(ATM.State.Current)
  {
    //----------------------------------------------
    case AtmState_Suspended:
      walk1_machine(ATM_send_event_to_leds(Cmd_BlinkPattern1, 800), AtmState_Stall);
    break;
    //----------------------------------------------
    case AtmState_Stall:
      // does nothing
      osDelay(10);
    break;
    //----------------------------------------------
    default:
    break;
  }  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_machine_config_mode(void)
{
  switch(ATM.State.Current)
  {
    //----------------------------------------------
    case AtmState_Suspended:
      ATM_api_change_state(AtmState_SoftReset);
    break;    
    //----------------------------------------------
    case AtmState_SoftReset:
      walk2_machine(ATM.Drv.soft_reset(), AtmState_ConfigFuncEn);
    break;
    //----------------------------------------------
    case AtmState_ConfigFuncEn:
      // 1.1) FuncEn0
      // Default

      // 1.2) FuncEn1
      // Default

      ATM_api_change_state(AtmState_ConfigZx);
    break;
    //----------------------------------------------
    case AtmState_ConfigZx:
      // 2.1) ZxConfig
      // Default
      
      ATM_api_change_state(AtmState_ConfigThresholds);      
    break;
    //----------------------------------------------
    case AtmState_ConfigThresholds:
    {
      // 3.1) Voltage sag Th
      zAssert(walk3_machine(ATM_REG_SagTh_Add, 0x1124, 0));

      // 3.2) Voltage phase loss Th
      zAssert(walk3_machine(ATM_REG_PhaseLossTh_Add, 0x023A, 0));

      // 3.3) In calculated Th
      zAssert(walk3_machine(ATM_REG_INWarnTh0_Add, 0x8CA0, 0));

      // 3.4) In sampled Th
      zAssert(walk3_machine(ATM_REG_INWarnTh1_Add, 0x8CA0, 0));

      // 3.5) Voltage THD+N Th
      zAssert(walk3_machine(ATM_REG_THDNUTh_Add, 0x03E8, 0));

      // 3.6) Current THD+N Th
      zAssert(walk3_machine(ATM_REG_THDNITh_Add, 0x03E8, AtmState_ConfigCS0));
    }
    break;    
    //----------------------------------------------
    case AtmState_ConfigCS0:
    {
      // 4.1) Set ConfigStart to initial value
      zAssert(walk3_machine(ATM_REG_ConfigStart_Add, CS_REG_Value_Calibration, 0));

      // 4.2) PLConst High
      // Default

      // 4.3) PLConst Low
      // Default

      // 4.4) MMode0
      zAssert(walk1_machine(ATM_config_reg_MMode0(), 0));

      // 4.5) MMode1
      zAssert(walk1_machine(ATM_config_reg_MMode1(), 0));

      // 4.6) P Start Th
      zAssert(walk3_machine(ATM_REG_PStartTh_Add, 0x0bb8, 0));

      // 4.7) Q Start Th
      zAssert(walk3_machine(ATM_REG_QStartTh_Add, 0x0bb8, 0));

      // 4.8) S Start Th
      zAssert(walk3_machine(ATM_REG_SStartTh_Add, 0x0bb8, 0));

      // 4.9) P Start Phase Th
      zAssert(walk3_machine(ATM_REG_PPhaseTh_Add, 0x00c8, 0));

      // 4.10) Q Start Phase  Th
      zAssert(walk3_machine(ATM_REG_QPhaseTh_Add, 0x00c8, 0));

      // 4.11) S Start Phase  Th
      zAssert(walk3_machine(ATM_REG_SPhaseTh_Add, 0x00c8, 0));

      // 4.12) Configura CS0 register
      zAssert(walk1_machine(ATM_config_reg_CS(ATM_REG_CS0_Add), 0));

      // 4.13) Set ConfigStart to final value
      zAssert(walk3_machine(ATM_REG_ConfigStart_Add, CS_REG_Value_Operation, AtmState_Suspended));

      ATM.Mode = SuspendedMode;
    }
    break;
    //----------------------------------------------
    default:
    break;
  }  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_machine_calib_mode(void)
{
  switch(ATM.State.Current)
  {
    //----------------------------------------------
    case AtmState_Suspended:
      walk1_machine(ATM_send_event_to_leds(Cmd_BlinkPattern3, 500), AtmState_CalibInit);
    break;
    //----------------------------------------------
    case AtmState_CalibInit:

    break;
    //----------------------------------------------
    default:
    break;
  }  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void ATM_machine_operation_mode(void)
{
  switch(ATM.State.Current)
  {
    //----------------------------------------------
    case AtmState_Suspended:
      walk1_machine(ATM_send_event_to_leds(Cmd_BlinkPattern3, 200), AtmState_SubscribeMeasures);
    break;

    //----------------------------------------------
    case AtmState_SubscribeMeasures:
    // Starta timer
    // inscreve variaveis no processo de leitura 
    break;
    //----------------------------------------------
    case AtmState_Acquiring:
    {
      if(xSemaphoreTake(SyncMeasureSem, ATM_RTOS_DEFAULT_DELAYS) == pdPASS)
      {
      #if SIMULA_DADOS_ENERGIA == 0

      // Loopa bitmap e realiza leituras, 

      #elif SIMULA_DADOS_ENERGIA == 1

      //

      #endif

      // Envia evento com dados

      }
    }
    break;
    //----------------------------------------------
    default:
    break;
  }  
}















/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_send_event_to_leds(uint8_t Command, uint16_t wMillisec)
{
  xQueueHandle *posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_LEDS);

  GenericQueueData_st stEvent;
  stEvent.enEvent = EvntFromATMtoLEDS;
  stEvent.byCmd = Command;
  stEvent.bySubCmd = 0;
  stEvent.pbyData = NULL;
  stEvent.wDataLen = wMillisec;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, ATM_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_send_event_to_uart(uint8_t Command, uint8_t SubCommand, uint8_t *pData, uint16_t DataLen)
{
  xQueueHandle *posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_UART);

  GenericQueueData_st stEvent;
  stEvent.enEvent = EvntFromATMtoUART;
  stEvent.byCmd = Command;
  stEvent.bySubCmd = SubCommand;
  memcpy(stEvent.pbyData, pData, DataLen);
  stEvent.wDataLen = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, ATM_RTOS_DEFAULT_DELAYS);
}
