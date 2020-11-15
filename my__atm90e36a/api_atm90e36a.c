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
#include <stdlib.h>

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static bool config_reg_MMode0 (void);
static bool config_reg_MMode1 (void);
static bool config_reg_CS     (uint16_t reg);

static bool walk1_machine (bool method, atm_states_en next_state);
static bool walk2_machine (atm_result_code_en method, atm_states_en next_state);
static bool walk3_machine (uint16_t addr, uint16_t value, atm_states_en next_state);

static bool send_event_to_leds  (uint8_t Command, uint16_t wMillisec);
static bool send_event_to_uart  (uint8_t Command, uint8_t SubCommand, uint16_t DataLen);

static bool assina_medidas  (uint8_t *data, uint16_t len);
static bool realiza_medidas (void);

static bool Acquire_Va      (uint16_t *read_val); 	
static bool Acquire_Vb      (uint16_t *read_val); 	
static bool Acquire_Vc      (uint16_t *read_val); 	
static bool Acquire_Ia      (uint16_t *read_val); 	
static bool Acquire_Ib      (uint16_t *read_val); 	
static bool Acquire_Ic      (uint16_t *read_val); 	
static bool Acquire_Pa      (uint16_t *read_val); 	
static bool Acquire_Pb      (uint16_t *read_val); 	
static bool Acquire_Pc      (uint16_t *read_val); 	
static bool Acquire_Qa      (uint16_t *read_val); 	
static bool Acquire_Qb      (uint16_t *read_val); 	
static bool Acquire_Qc      (uint16_t *read_val); 	
static bool Acquire_Sa      (uint16_t *read_val); 	
static bool Acquire_Sb      (uint16_t *read_val); 	
static bool Acquire_Sc      (uint16_t *read_val);  	 
static bool Acquire_Pa_fund (uint16_t *read_val);
static bool Acquire_Pb_fund (uint16_t *read_val);
static bool Acquire_Pc_fund (uint16_t *read_val);
static bool Acquire_Pa_harm (uint16_t *read_val);
static bool Acquire_Pb_harm (uint16_t *read_val);
static bool Acquire_Pc_harm (uint16_t *read_val);
/***************************************************************************************************
* Externals
***************************************************************************************************/
extern SemaphoreHandle_t SyncMeasureSem;
extern TIM_HandleTypeDef htim3;
/***************************************************************************************************
* Vars
***************************************************************************************************/
ATM90_app_st ATM = {0};
static QueueHandle_t *Queue_ATM_HANDLE = NULL;

static assinatura_de_medidas_st vetor_medidas_assinadas[MAX_MEDIDAS_ASSINADAS] = {0};

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool ATM_api_init(void)
{
	// Cria fila ATM
	Queue_ATM_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_ATM);
	*Queue_ATM_HANDLE = xQueueCreate(32, SIZE_OF_QUEUE_ITEM);
  if(Queue_ATM_HANDLE == NULL)
  {
    return false;
  }

  ATM_hw_init();

  ATM_drv_init(&ATM.Drv);

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
  uint16_t data = 0;

  if(ATM.Drv.monitor_warn())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, 1, data);
    }
  }
  else if(ATM.Drv.monitor_irq0())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, 2, data);
    }
  }
  else if(ATM.Drv.monitor_irq1())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus1_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, 3, data);
    }
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
          ATM.MeasuresBitMap = NewEvent.bySubCmd;
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = OperationMode;
        break;
        //----------------------------------------------
        case Cmd_ReadSpecificRegister:
          ATM.MeasuresBitMap = NewEvent.bySubCmd;
          ATM.State.Current = AtmState_ReadingReg;
          ATM.Mode = SuspendedMode;
        break;
        //----------------------------------------------
        case Cmd_SignMeasurements:
          if(assina_medidas(NewEvent.pbyData, NewEvent.wDataLen))
          {
            ATM.State.Current = AtmState_SubscribeMeasures;
            ATM.Mode = OperationMode;
          }
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
static bool send_event_to_leds(uint8_t Command, uint16_t wMillisec)
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
static bool send_event_to_uart(uint8_t Command, uint8_t SubCommand, uint16_t DataLen)
{
  xQueueHandle *posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_UART);
  GenericQueueData_st stEvent;

  stEvent.enEvent = EvntFromATMtoUART;
  stEvent.byCmd = Command;
  stEvent.bySubCmd = SubCommand;
  stEvent.pbyData = NULL;
  stEvent.wDataLen = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, ATM_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool config_reg_MMode0(void)
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
static bool config_reg_MMode1(void)
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
static bool config_reg_CS(uint16_t reg)
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
      walk1_machine(send_event_to_leds(Cmd_BlinkPattern1, 800), AtmState_Stall);
    break;
    //----------------------------------------------
    case AtmState_Stall:
      // does nothing
      osDelay(10);
    break;
    //----------------------------------------------
    case AtmState_ReadingReg:
    {
      uint16_t LocalReadVal = 0;

      if(ATM.Drv.read_reg((uint8_t)ATM.MeasuresBitMap, &LocalReadVal) == ATM_RC_OK)
      {
        walk1_machine(send_event_to_uart(Cmd_PrintThis, 0, LocalReadVal), AtmState_Stall);
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
      zAssert(walk1_machine(config_reg_MMode0(), 0));

      // 4.5) MMode1
      zAssert(walk1_machine(config_reg_MMode1(), 0));

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
      zAssert(walk1_machine(config_reg_CS(ATM_REG_CS0_Add), 0));

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
      walk1_machine(send_event_to_leds(Cmd_BlinkPattern3, 500), AtmState_CalibInit);
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
      walk1_machine(send_event_to_leds(Cmd_BlinkPattern3, 200), AtmState_SubscribeMeasures);
    break;

    //----------------------------------------------
    case AtmState_SubscribeMeasures:
    // inscreve variaveis no processo de leitura
    assina_medidas(NULL, 0);

    // Starta timer 
    HAL_TIM_Base_Start_IT(&htim3);

    // Muda de estado
    ATM_api_change_state(AtmState_Acquiring);

    break;
    //----------------------------------------------
    case AtmState_Acquiring:
    {
      if(xSemaphoreTake(SyncMeasureSem, ATM_RTOS_DEFAULT_DELAYS) == pdPASS)
      {
        realiza_medidas();
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
static bool assina_medidas(uint8_t *data, uint16_t len)
{

  vetor_medidas_assinadas[0].id = voltage_rms_a;
  vetor_medidas_assinadas[0].read_func = Acquire_Va;
  vetor_medidas_assinadas[1].id = voltage_rms_b;
  vetor_medidas_assinadas[1].read_func = Acquire_Vb;
  vetor_medidas_assinadas[2].id = voltage_rms_c;
  vetor_medidas_assinadas[2].read_func = Acquire_Vc;
  vetor_medidas_assinadas[3].id = current_rms_a;      
  vetor_medidas_assinadas[3].read_func = Acquire_Ia;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool realiza_medidas(void)
{

  #if SIMULA_DADOS_ENERGIA == 0

    uint32_t dado_de_envio = 0;
    uint16_t temp_read_val = 0, rc = 0;
    
    // Loopa bitmap e realiza leituras, 
    for(uint8_t k = 0; k < MAX_MEDIDAS_ASSINADAS; k++)
    {
      if(vetor_medidas_assinadas[k].id)
      {
        vetor_medidas_assinadas[k].read_func(&temp_read_val);
        uint16_t temp = vetor_medidas_assinadas[k].id;
        dado_de_envio = (uint32_t)(temp << 16);
        dado_de_envio |= (uint32_t)(temp_read_val);

        // Envia evento para task UART
        rc += RTOS_Send_Data_To_Energy_Queue(&dado_de_envio, ATM_RTOS_DEFAULT_DELAYS);
      }
    }

    if(rc >= MAX_MEDIDAS_ASSINADAS)
    {
      return true;
    }
    return false;

  #elif SIMULA_DADOS_ENERGIA == 1

  #endif
}




#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_line_voltage(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_LineVoltageRms_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }  
  return false;
}

static bool Acquire_Va(uint16_t *read_val)
{
  return atm_acquire_line_voltage(PHASE_A, read_val);
}
static bool Acquire_Vb(uint16_t *read_val)
{
  return atm_acquire_line_voltage(PHASE_B, read_val);
}
static bool Acquire_Vc(uint16_t *read_val)
{
  return atm_acquire_line_voltage(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_line_current(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_LineCurrentRms_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Ia(uint16_t *read_val)
{
  return atm_acquire_line_current(PHASE_A, read_val);
}
static bool Acquire_Ib(uint16_t *read_val)
{
  return atm_acquire_line_current(PHASE_B, read_val);
}
static bool Acquire_Ic(uint16_t *read_val)
{
  return atm_acquire_line_current(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_active_power(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_ActivePower_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Pa(uint16_t *read_val)
{
  return atm_acquire_active_power(PHASE_A, read_val);
}
static bool Acquire_Pb(uint16_t *read_val)
{
  return atm_acquire_active_power(PHASE_B, read_val);
}
static bool Acquire_Pc(uint16_t *read_val)
{
  return atm_acquire_active_power(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_reactive_power(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_ReactivePower_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Qa(uint16_t *read_val)
{
  return atm_acquire_reactive_power(PHASE_A, read_val);
}
static bool Acquire_Qb(uint16_t *read_val)
{
  return atm_acquire_reactive_power(PHASE_B, read_val);
}
static bool Acquire_Qc(uint16_t *read_val)
{
  return atm_acquire_reactive_power(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_aparent_power(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_AparantPower_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Sa(uint16_t *read_val)
{
  return atm_acquire_aparent_power(PHASE_A, read_val);
}
static bool Acquire_Sb(uint16_t *read_val)
{
  return atm_acquire_aparent_power(PHASE_B, read_val);
}
static bool Acquire_Sc(uint16_t *read_val)
{
  return atm_acquire_aparent_power(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_power_factor(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_PowerFactor_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_active_fundamental_power(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_ActiveFundamentalPower_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Pa_fund(uint16_t *read_val)
{
  return atm_acquire_active_fundamental_power(PHASE_A, read_val);
}
static bool Acquire_Pb_fund(uint16_t *read_val)
{
  return atm_acquire_active_fundamental_power(PHASE_B, read_val);
}
static bool Acquire_Pc_fund(uint16_t *read_val)
{
  return atm_acquire_active_fundamental_power(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_active_harmonic_power(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_ActiveHarmonicPower_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Pa_harm(uint16_t *read_val)
{
  return atm_acquire_active_harmonic_power(PHASE_A, read_val);
}
static bool Acquire_Pb_harm(uint16_t *read_val)
{
  return atm_acquire_active_harmonic_power(PHASE_B, read_val);
}
static bool Acquire_Pc_harm(uint16_t *read_val)
{
  return atm_acquire_active_harmonic_power(PHASE_C, read_val);
}
#pragma GCC diagnostic pop
