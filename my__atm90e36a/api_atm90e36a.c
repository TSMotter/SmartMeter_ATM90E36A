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
#include <math.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static void   ATM_api_check_queue       (void);
static void 	ATM_api_check_retry				(void);
static void 	ATM_api_check_hw_pins			(void);
static void   ATM_api_change_state      (atm_states_en next_state);

static bool config_reg_VoltageTh(uint16_t reg, uint16_t target);
static bool config_reg_MMode0   (void);
static bool config_reg_MMode1   (void);
static bool config_reg_CS       (uint16_t reg);

static bool walk1 (bool method, atm_states_en next_state);
static bool walk2 (atm_result_code_en method, atm_states_en next_state);
static bool walk3 (uint16_t addr, uint16_t value, atm_states_en next_state);

static bool send_event_to_leds  (uint8_t Command, uint16_t wMillisec);
static bool send_event_to_uart  (uint8_t Command, uint8_t SubCommand, uint16_t DataLen);
static bool send_event_to_lora  (uint8_t Command, uint8_t SubCommand, uint16_t DataLen);

static bool calib_gain_reg  (uint16_t reg[3], uint8_t idx);
static bool calib_offset_reg(uint16_t reg[3], uint8_t idx);
static uint16_t inc_dec_gain(uint16_t reg[3], uint8_t idx, bool inc_dec);
static uint16_t faz_calculos_processo_calib_Uoffset(uint32_t valor_medio);

static bool assina_medidas  (uint16_t id);
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
static bool Acquire_Va_thd  (uint16_t *read_val);
static bool Acquire_Vb_thd  (uint16_t *read_val);
static bool Acquire_Vc_thd  (uint16_t *read_val);
static bool Acquire_Ia_thd  (uint16_t *read_val);
static bool Acquire_Ib_thd  (uint16_t *read_val);
static bool Acquire_Ic_thd  (uint16_t *read_val);
static bool Acquire_Freq    (uint16_t *read_val);
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

// Reg address and value to individually read/write through UART commands
static uint16_t RegisterAddress = 0;
static uint16_t RegisterVal = 0;    

static uint16_t vetor_regs_calib_offset[6][3] = { 
  {ATM_REG_UrmsA_Add, ATM_REG_UrmsALSB_Add, ATM_REG_UoffsetA_Add}, 
  {ATM_REG_UrmsB_Add, ATM_REG_UrmsBLSB_Add, ATM_REG_UoffsetB_Add}, 
  {ATM_REG_UrmsC_Add, ATM_REG_UrmsCLSB_Add, ATM_REG_UoffsetC_Add}, 
  {ATM_REG_IrmsA_Add, ATM_REG_IrmsALSB_Add, ATM_REG_IoffsetA_Add}, 
  {ATM_REG_IrmsB_Add, ATM_REG_IrmsBLSB_Add, ATM_REG_IoffsetB_Add}, 
  {ATM_REG_IrmsC_Add, ATM_REG_IrmsCLSB_Add, ATM_REG_IoffsetC_Add}
  };

static uint16_t gains_previous[6] = {15048, 15080, 15060, 4098, 3922, 3976};
static uint16_t vetor_regs_calib_gain[6][3] = { 
  {ATM_REG_UrmsA_Add, ATM_REG_UgainA_Add, 12550},
  {ATM_REG_UrmsB_Add, ATM_REG_UgainB_Add, 12660}, 
  {ATM_REG_UrmsC_Add, ATM_REG_UgainC_Add, 12620},
  {ATM_REG_IrmsA_Add, ATM_REG_IgainA_Add, 930}, 
  {ATM_REG_IrmsB_Add, ATM_REG_IgainB_Add, 920}, 
  {ATM_REG_IrmsC_Add, ATM_REG_IgainC_Add, 910} 
  };


static aquisicao_de_medidas_st vetor_de_aquisicao[num_of_total_available_measures] = {
  // Phase A:
  {.active = false, .read_func = Acquire_Va},
  {.active = false, .read_func = Acquire_Ia},
  {.active = false, .read_func = Acquire_Pa},
  {.active = false, .read_func = Acquire_Qa},
  {.active = false, .read_func = Acquire_Sa},
  {.active = false, .read_func = Acquire_Pa_fund},
  {.active = false, .read_func = Acquire_Pa_harm},
  {.active = false, .read_func = Acquire_Va_thd},
  {.active = false, .read_func = Acquire_Ia_thd},

  // Phase B:
  {.active = false, .read_func = Acquire_Vb},
  {.active = false, .read_func = Acquire_Ib},
  {.active = false, .read_func = Acquire_Pb},
  {.active = false, .read_func = Acquire_Qb},
  {.active = false, .read_func = Acquire_Sb},
  {.active = false, .read_func = Acquire_Pb_fund},
  {.active = false, .read_func = Acquire_Pb_harm},
  {.active = false, .read_func = Acquire_Vb_thd},
  {.active = false, .read_func = Acquire_Ib_thd},

  // Phase C:
  {.active = false, .read_func = Acquire_Vc},
  {.active = false, .read_func = Acquire_Ic},
  {.active = false, .read_func = Acquire_Pc},
  {.active = false, .read_func = Acquire_Qc},
  {.active = false, .read_func = Acquire_Sc},
  {.active = false, .read_func = Acquire_Pc_fund},
  {.active = false, .read_func = Acquire_Pc_harm},
  {.active = false, .read_func = Acquire_Vc_thd},
  {.active = false, .read_func = Acquire_Ic_thd},    

  // Others
  {.active = false, .read_func = Acquire_Freq},
};

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
static void ATM_api_change_state(atm_states_en next_state)
{
  ATM.State.Previous = ATM.State.Current;
  ATM.State.Current = next_state;
  ATM.Retry = 0;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void ATM_api_check_retry(void)
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
static void ATM_api_check_hw_pins(void)
{
  bool flag = false;
  uint16_t data = 0;

  if(ATM.Drv.monitor_warn())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, subCmd_print_warn, data);
    }
  }
  else if(ATM.Drv.monitor_irq0())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus0_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, subCmd_print_irq0, data);
    }
  }
  else if(ATM.Drv.monitor_irq1())
  {
    flag = true;
    if(ATM.Drv.read_reg(ATM_REG_SysStatus1_Add, &data) == ATM_RC_OK)
    {
      send_event_to_uart(Cmd_PrintThis, subCmd_print_irq1, data);
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
static void ATM_api_check_queue(void)
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
        {
          ATM.Mode = CalibMode;
          if (NewEvent.bySubCmd == subCmd_AdjStart)
          {
          	ATM.State.Previous = AtmState_CalibAdjStartReg;
          	ATM.State.Current = AtmState_CalibAdjStartReg;
          }
          else if (NewEvent.bySubCmd == subCmd_OffSet_Va)
          	ATM.State.Current = AtmState_CalibUoffsetAReg;
          else if (NewEvent.bySubCmd == subCmd_Offset_Ia)
          	ATM.State.Current = AtmState_CalibIoffsetAReg;
          else if (NewEvent.bySubCmd == subCmd_Offset_Vb)
          	ATM.State.Current = AtmState_CalibUoffsetBReg;
          else if (NewEvent.bySubCmd == subCmd_Offset_Ib)
          	ATM.State.Current = AtmState_CalibIoffsetBReg;
          else if (NewEvent.bySubCmd == subCmd_Offset_Vc)
          	ATM.State.Current = AtmState_CalibUoffsetCReg;
          else if (NewEvent.bySubCmd == subCmd_Offset_Ic)
          	ATM.State.Current = AtmState_CalibIoffsetCReg;
          else if (NewEvent.bySubCmd == subCmd_Gain_all_phases_V)
          	ATM.State.Current = AtmState_CalibGain_all_phases_V;
          else if (NewEvent.bySubCmd == subCmd_Gain_Ia)
          	ATM.State.Current = AtmState_CalibGainIa;
          else if (NewEvent.bySubCmd == subCmd_Gain_Ib)
          	ATM.State.Current = AtmState_CalibGainIb;
          else if (NewEvent.bySubCmd == subCmd_Gain_Ic)
          	ATM.State.Current = AtmState_CalibGainIc;
          else if (NewEvent.bySubCmd == subCmd_Gain_Va_Inc)
          	ATM.State.Current = AtmState_CalibGainVa_Inc;
          else if (NewEvent.bySubCmd == subCmd_Gain_Va_Dec)
          	ATM.State.Current = AtmState_CalibGainVa_Dec;            
          else if (NewEvent.bySubCmd == subCmd_Gain_Vb_Inc)
          	ATM.State.Current = AtmState_CalibGainVb_Inc;
          else if (NewEvent.bySubCmd == subCmd_Gain_Vb_Dec)
          	ATM.State.Current = AtmState_CalibGainVb_Dec;            
          else if (NewEvent.bySubCmd == subCmd_Gain_Vc_Inc)
          	ATM.State.Current = AtmState_CalibGainVc_Inc;
          else if (NewEvent.bySubCmd == subCmd_Gain_Vc_Dec)
          	ATM.State.Current = AtmState_CalibGainVc_Dec;  
          else if (NewEvent.bySubCmd == subCmd_Gain_Va_SubDefault)
          {
            uint16_t NewVal = (NewEvent.pbyData[0] << 8) | (NewEvent.pbyData[1]);
            vetor_regs_calib_gain[Va_][2] = NewVal;
          }
          else if (NewEvent.bySubCmd == subCmd_Gain_Vb_SubDefault)
          {
            uint16_t NewVal = (NewEvent.pbyData[0] << 8) | (NewEvent.pbyData[1]);
            vetor_regs_calib_gain[Vb_][2] = NewVal;
          }
          else if (NewEvent.bySubCmd == subCmd_Gain_Vc_SubDefault)
          {
            uint16_t NewVal = (NewEvent.pbyData[0] << 8) | (NewEvent.pbyData[1]);
            vetor_regs_calib_gain[Vc_][2] = NewVal;
          }
          else if (NewEvent.bySubCmd == subCmd_Read_WriteCS3)
          	ATM.State.Current = AtmState_ReadWrite_CS3;         
        }
        break;
        //----------------------------------------------
        case Cmd_SuspendedMode:
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = SuspendedMode;
        break;
        //----------------------------------------------
        case Cmd_OperationMode:
          ATM.State.Current = AtmState_Suspended;
          ATM.Mode = OperationMode;
        break;
        //----------------------------------------------
        case Cmd_ReadSpecificRegister:
          RegisterAddress = NewEvent.bySubCmd;
          ATM.State.Current = AtmState_ReadingReg;
          ATM.Mode = SuspendedMode;
        break;
        //----------------------------------------------
        case Cmd_WriteSpecificRegister:
          RegisterAddress = NewEvent.bySubCmd;
          RegisterVal = (NewEvent.pbyData[0] << 8) | (NewEvent.pbyData[1]);
          ATM.State.Current = AtmState_WritingReg;
          ATM.Mode = SuspendedMode;
        break;        
        //----------------------------------------------
        case Cmd_Reset:
        {
          if(NewEvent.bySubCmd == subCmd_SoftwareReset)
          {
            ATM.State.Current = AtmState_SoftReset;
            ATM.Mode = SuspendedMode;
          }
          else if (NewEvent.bySubCmd == subCmd_HardwareReset)
          {
            ATM.State.Current = AtmState_HardReset;
            ATM.Mode = SuspendedMode;            
          }
        } 
        break;
        //----------------------------------------------
        case Cmd_SignMeasurements:
          assina_medidas(NewEvent.bySubCmd);
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
static bool send_event_to_lora(uint8_t Command, uint8_t SubCommand, uint16_t DataLen)
{
  xQueueHandle *posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_LORA);
  GenericQueueData_st stEvent;

  stEvent.enEvent = EvntFromATMtoLORA;
  stEvent.byCmd = Command;
  stEvent.bySubCmd = SubCommand;
  stEvent.pbyData = NULL;
  stEvent.wDataLen = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, ATM_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool config_reg_VoltageTh(uint16_t reg, uint16_t target)
{
  uint16_t TxVal = 0;

  #ifdef USE_SAG_REAL
  uint16_t biggest_gain = 0, temp1 = 0, temp2 = 0;

  // Se eu ja tenho os valores, nao preciso ler os registradores
  if((gains_previous[Va_]) && (gains_previous[Vb_]) && (gains_previous[Vc_]))
  {
    biggest_gain = gains_previous[Va_];
    temp1 = gains_previous[Vb_];
    temp2 = gains_previous[Vc_];
  }
  else
  {
    ATM.Drv.read_reg(vetor_regs_calib_gain[Va_][1], &biggest_gain);
    ATM.Drv.read_reg(vetor_regs_calib_gain[Vb_][1], &temp1);
    ATM.Drv.read_reg(vetor_regs_calib_gain[Vc_][1], &temp2);
  }
  
  biggest_gain = temp1 > biggest_gain ? temp1 : biggest_gain;
  biggest_gain = temp2 > biggest_gain ? temp2 : biggest_gain;
  TxVal = (target * 100 * sqrt(2))/(2 * biggest_gain/32768);

  #else

  if(reg == ATM_REG_SagTh_Add)
  {
    TxVal = 0x1124;
  }
  else if(reg == ATM_REG_PhaseLossTh_Add)
  {
    TxVal = 0x023A;
  }

  #endif

  if(ATM.Drv.write_reg(reg, TxVal) == ATM_RC_OK)
  {
    return true;
  }
  return false;

}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool config_reg_MMode0(void)
{
  uint16_t LocalReadVal = 0;
  
  uint16_t data =  0;

  /* 
	data |= ((ATM_REG_MMode0_Msk_Freq60Hz |  ATM_REG_MMode0_Msk_didtEn | ATM_REG_MMode0_Msk_001LSB |
            ATM_REG_MMode0_Msk_CF2varh  | ATM_REG_MMode0_Msk_ABSEnQ  | ATM_REG_MMode0_Msk_ABSEnP |
			      ATM_REG_MMode0_Msk_EnPA     | ATM_REG_MMode0_Msk_EnPB    | ATM_REG_MMode0_Msk_EnPC));
  */  

  data = 0x1087;
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
  return false;  
}




/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool walk1(bool method, atm_states_en next_state)
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
static bool walk2(atm_result_code_en method, atm_states_en next_state)
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
static bool walk3(uint16_t addr, uint16_t value, atm_states_en next_state)
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
      walk1(send_event_to_leds(Cmd_BlinkPattern1, subCmd_BlinkSpeed_Slow), 0);
      walk1(send_event_to_uart(Cmd_PrintThis, subCmd_print_start_msg, 0), AtmState_Stall);
    break;
    //----------------------------------------------
    case AtmState_Stall:
      // does nothing
      vTaskDelay(ATM_RTOS_DEFAULT_DELAYS);
    break;
    //----------------------------------------------
    case AtmState_SoftReset:
      walk2(ATM.Drv.soft_reset(), AtmState_Suspended);
      ATM_drv_default_gains(&ATM.Drv);
    break;
    //----------------------------------------------
    case AtmState_HardReset:
      ATM.Drv.hard_reset();
      ATM_api_change_state(AtmState_Suspended);
      ATM_drv_default_gains(&ATM.Drv);
    break;
    //----------------------------------------------
    case AtmState_ReadingReg:
    {
      uint16_t LocalReadVal = 0;

      if(ATM.Drv.read_reg((uint8_t)RegisterAddress, &LocalReadVal) == ATM_RC_OK)
      {
        walk1(send_event_to_uart(Cmd_PrintThis, subCmd_print_this, LocalReadVal), AtmState_Stall);
      }
      RegisterAddress = 0;
    }
    break; 
    //----------------------------------------------
    case AtmState_WritingReg:
    {
      walk3(RegisterAddress, RegisterVal, AtmState_Stall);

      RegisterAddress = 0;
      RegisterVal = 0;
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
      walk2(ATM.Drv.soft_reset(), AtmState_ConfigFuncEn);
      ATM_drv_default_gains(&ATM.Drv);
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
      // 3.1) Voltage sag Th (100Vrms)
      zAssert(walk1(config_reg_VoltageTh(ATM_REG_SagTh_Add, 100), 0));

      // 3.2) Voltage phase loss Th (13Vrms)
      zAssert(walk1(config_reg_VoltageTh(ATM_REG_PhaseLossTh_Add, 13), 0));

      // 3.3) In calculated Th
      zAssert(walk3(ATM_REG_INWarnTh0_Add, 0x8CA0, 0));

      // 3.4) In sampled Th
      zAssert(walk3(ATM_REG_INWarnTh1_Add, 0x8CA0, 0));

      // 3.5) Voltage THD+N Th
      zAssert(walk3(ATM_REG_THDNUTh_Add, 0x03E8, 0));

      // 3.6) Current THD+N Th
      zAssert(walk3(ATM_REG_THDNITh_Add, 0x03E8, AtmState_ConfigCS0));
    }
    break;    
    //----------------------------------------------
    case AtmState_ConfigCS0:
    {
      // 4.1) Set ConfigStart to initial value
      zAssert(walk3(ATM_REG_ConfigStart_Add, CS_REG_Value_Calibration, 0));

      // 4.2) PLConst High
      // Default

      // 4.3) PLConst Low
      // Default

      // 4.4) MMode0
      zAssert(walk1(config_reg_MMode0(), 0));

      // 4.5) MMode1
      // Default...
      //zAssert(walk1(config_reg_MMode1(), 0));

      // 4.6) P Start Th
      zAssert(walk3(ATM_REG_PStartTh_Add, 0x1D4C, 0));

      // 4.7) Q Start Th
      zAssert(walk3(ATM_REG_QStartTh_Add, 0x1D4C, 0));

      // 4.8) S Start Th
      zAssert(walk3(ATM_REG_SStartTh_Add, 0x1D4C, 0));

      // 4.9) P Start Phase Th
      zAssert(walk3(ATM_REG_PPhaseTh_Add, 0x02EE, 0));

      // 4.10) Q Start Phase  Th
      zAssert(walk3(ATM_REG_QPhaseTh_Add, 0x02EE, 0));

      // 4.11) S Start Phase  Th
      zAssert(walk3(ATM_REG_SPhaseTh_Add, 0x02EE, 0));

      // 4.12) Configura CS0 register
      zAssert(walk1(config_reg_CS(ATM_REG_CS0_Add), 0));

      // 4.13) Set ConfigStart to final value
      zAssert(walk3(ATM_REG_ConfigStart_Add, CS_REG_Value_Operation, AtmState_Suspended));

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
      walk1(send_event_to_leds(Cmd_BlinkPattern3, subCmd_BlinkSpeed_Medium), AtmState_CalibAdjStartReg);
    break;
    //----------------------------------------------
    case AtmState_Stall:
      // does nothing
      vTaskDelay(ATM_RTOS_DEFAULT_DELAYS);
    break;
    //----------------------------------------------
    case AtmState_CalibAdjStartReg:
      if(ATM.State.Previous == AtmState_CalibAdjStartReg)
      {
        zAssert(walk3(ATM_REG_AdjStart_Add, CS_REG_Value_Calibration, AtmState_Stall));
        ATM_drv_default_gains(&ATM.Drv);
      }
      else if (ATM.State.Previous == AtmState_ReadWrite_CS3)
      {
        zAssert(walk3(ATM_REG_AdjStart_Add, CS_REG_Value_Operation, AtmState_Suspended));
        ATM.Mode = SuspendedMode;
      }
    break;
    //----------------------------------------------
    case AtmState_CalibUoffsetAReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Va_], Va_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibUoffsetBReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Vb_], Vb_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibUoffsetCReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Vc_], Vc_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibIoffsetAReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Ia_], Ia_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibIoffsetBReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Ib_], Ib_), AtmState_Stall));
    break; 
    //----------------------------------------------
    case AtmState_CalibIoffsetCReg:
      zAssert(walk1(calib_offset_reg(vetor_regs_calib_offset[Ic_], Ic_), AtmState_Stall));
    break; 
    //----------------------------------------------
    case AtmState_CalibGain_all_phases_V:
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Va_], Va_), 0));
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Vb_], Vb_), 0));
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Vc_], Vc_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainIa:
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Ia_], Ia_), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainIb:
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Ib_], Ib_), AtmState_Stall));
    break;  
    //----------------------------------------------
    case AtmState_CalibGainIc:
      zAssert(walk1(calib_gain_reg(vetor_regs_calib_gain[Ic_], Ic_), AtmState_Stall));
    break; 
    //----------------------------------------------
    case AtmState_CalibGainVa_Inc:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Va_], Va_, true), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainVa_Dec:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Va_], Va_, false), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainVb_Inc:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Vb_], Vb_, true), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainVb_Dec:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Vb_], Vb_, false), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainVc_Inc:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Vc_], Vc_, true), AtmState_Stall));
    break;
    //----------------------------------------------
    case AtmState_CalibGainVc_Dec:
      zAssert(walk1(inc_dec_gain(vetor_regs_calib_gain[Vc_], Vc_, false), AtmState_Stall));
    break;    
    //----------------------------------------------
    case AtmState_ReadWrite_CS3:
      // Reconfigura Thresholds
      zAssert(walk1(config_reg_VoltageTh(ATM_REG_SagTh_Add, 100), 0));
      zAssert(walk1(config_reg_VoltageTh(ATM_REG_PhaseLossTh_Add, 13), 0));

      zAssert(walk1(config_reg_CS(ATM_REG_CS3_Add), AtmState_CalibAdjStartReg));
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
      zAssert(walk1(config_reg_CS(ATM_REG_CS1_Add), 0));
      zAssert(walk3(ATM_REG_CalStart_Add, CS_REG_Value_Operation, 0));
      zAssert(walk1(config_reg_CS(ATM_REG_CS2_Add), 0));
      zAssert(walk3(ATM_REG_HarmStart_Add, CS_REG_Value_Operation, 0));
      
      walk1(send_event_to_leds(Cmd_BlinkPattern3, subCmd_BlinkSpeed_Fast), AtmState_SubscribeMeasures);
    break;

    //----------------------------------------------
    case AtmState_SubscribeMeasures:

    // Starta timer 
    HAL_TIM_Base_Start_IT(&htim3);

    // Muda de estado
    ATM_api_change_state(AtmState_Acquiring);

    break;
    //----------------------------------------------
    case AtmState_Acquiring:
      if(xSemaphoreTake(SyncMeasureSem, ATM_RTOS_DEFAULT_DELAYS) == pdPASS)
      {
        if(realiza_medidas())
        {
          
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
static bool calib_gain_reg(uint16_t reg[3], uint8_t idx)
{
  uint16_t k = 0, RxPreviousVal = 0, TxVal = 0, min = 0, max = 0, cntr = 0;
  uint32_t valor_rms_acumulado = 0;

  // Se eu ja tenho valor previo de calibracao, so escrevo ele e vou embora
  if(gains_previous[idx] != 0)
  {
    if(walk3(reg[1], gains_previous[idx], 0))
    {
      ATM.Drv.Params[idx].Gain = gains_previous[idx];
      return true;
    }    
  }

  // Read reg rms value
  // Realiza uma serie de leituras
  for(k = 0; k < NUM_AMOSTRAS_MEDIA_CALIB; k++)
  {
    uint16_t RxV = 0;

    vTaskDelay(ATM_RTOS_DEFAULT_DELAYS);
    if(ATM.Drv.read_reg(reg[0], &RxV) == ATM_RC_OK)
    {
      cntr++;
      if(k == 0)
      {
        min = RxV;
        max = RxV;
      }
      else if(RxV < min)
      {
        min = RxV;
      }
      else if(RxV > max)
      {
        max = RxV;
      }
      valor_rms_acumulado += RxV;
      continue;
    }
    return false;
  }

  //valor_rms_acumulado = valor_rms_acumulado - min - max;
  uint16_t media_V = valor_rms_acumulado/(NUM_AMOSTRAS_MEDIA_CALIB);// - 2);

  // Le valor atual do registrador de ganho
  ATM.Drv.read_reg(reg[1], &RxPreviousVal);

  uint32_t temp = 0;

  temp = reg[2] * RxPreviousVal;
  TxVal = (uint16_t)(temp/media_V);
  
  if(TxVal != RxPreviousVal)
  {
    if(walk3(reg[1], TxVal, 0))
    {
      ATM.Drv.Params[idx].Gain = TxVal;
      return true;
    }
    return false;
  }

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool calib_offset_reg(uint16_t reg[3], uint8_t idx)
{
  uint16_t k = 0;
  uint32_t valor_rms_acumulado = 0, min = 0, max = 0;

  // Realiza uma serie de leituras
  for(k = 0; k < NUM_AMOSTRAS_MEDIA_CALIB; k++)
  {
    uint16_t RxVal = 0;
    uint32_t valor_rms_instantaneo = 0;

    vTaskDelay(ATM_RTOS_DEFAULT_DELAYS); 
    // Read reg rms value
    if(ATM.Drv.read_reg(reg[0], &RxVal) == ATM_RC_OK)
    {
      valor_rms_instantaneo = (uint32_t)(RxVal << 16) & 0xffff0000;

      // Read reg rms LSB value
      if(ATM.Drv.read_reg(reg[1], &RxVal) == ATM_RC_OK)
      {
        valor_rms_instantaneo |= RxVal;
        if(k == 0)
        {
          min = valor_rms_instantaneo;
          max = valor_rms_instantaneo;
        }
        else if(valor_rms_instantaneo < min)
        {
          min = valor_rms_instantaneo;
        }
        else if(valor_rms_instantaneo > max)
        {
          max = valor_rms_instantaneo;
        }
        valor_rms_acumulado += valor_rms_instantaneo;
        continue;
      }
    }
    return false;
  }

  // Calcula a media
  //valor_rms_acumulado = valor_rms_acumulado - min - max;
  valor_rms_acumulado = valor_rms_acumulado/(NUM_AMOSTRAS_MEDIA_CALIB );//- 2);
  uint16_t TxVal = faz_calculos_processo_calib_Uoffset(valor_rms_acumulado);

  // Realiza a escrita no registrador de calibracao
  if(walk3(reg[2], TxVal, 0))
  {
    ATM.Drv.Params[idx].Offset = TxVal;
    return true; // Sucesso !
  }

  return false;
}

/***************************************************************************************************
* @brief Realiza o processo explicado na sessao 4.2.5 do app. note
***************************************************************************************************/
static uint16_t faz_calculos_processo_calib_Uoffset(uint32_t valor_medio)
{
  uint32_t valor_complemento2 = 0;
  uint16_t rc = 0;

  // Descarta 7 bits LSB
  valor_medio = (valor_medio >> 7);

  // Mascara para garantir que 7 bits MSB são realmente zero
  valor_medio &= 0x01FFFFFF;

  // Realiza o complemento de 2
  valor_complemento2 = (~valor_medio) + 1;

  // Retorna 16 bits LSB
  rc = valor_complemento2;

  return rc;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static uint16_t inc_dec_gain(uint16_t reg[3], uint8_t idx, bool inc_dec)
{
  uint16_t TxVal = 0;

  // Inc
  if(inc_dec)
  {
    TxVal = ATM.Drv.Params[idx].Gain + 1;
  }
  // Dec
  else
  {
    TxVal = ATM.Drv.Params[idx].Gain - 1;
  }
  
  if(ATM.Drv.write_reg(reg[1], TxVal) == ATM_RC_OK)
  {
    ATM.Drv.Params[idx].Gain = TxVal;
    return true;
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool assina_medidas(uint16_t id)
{
  #if SIMULA_ASSINA_DADOS == 0

    vetor_de_aquisicao[id].active = !vetor_de_aquisicao[id].active;

  #elif SIMULA_ASSINA_DADOS == 1
  
    vetor_de_aquisicao[voltage_rms_a].active = true;
    vetor_de_aquisicao[current_rms_a].active = true;
    vetor_de_aquisicao[voltage_rms_b].active = true;
    vetor_de_aquisicao[voltage_rms_c].active = true;  
  
  #endif

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool realiza_medidas(void)
{

  #if SIMULA_DADOS_ENERGIA == 0

    uint32_t dado_de_envio = 0, dado_de_finalizacao = 0xffffffff;
    uint16_t temp_read_val = 0, rc = 0;
    
    // Varre vetor e realiza leituras, 
    for(uint16_t indice = 0; indice < num_of_total_available_measures; indice++)
    {
      if(vetor_de_aquisicao[indice].active)
      {
        vetor_de_aquisicao[indice].read_func(&temp_read_val);

        dado_de_envio = (uint32_t) (indice << 16);
        dado_de_envio |= (uint32_t)(temp_read_val);

        // Envia evento para task UART
        rc += RTOS_Send_Data_To_Energy_Queue(&dado_de_envio, ATM_RTOS_DEFAULT_DELAYS);
      }
    }

    #ifdef USE_UART_PORT
      return RTOS_Send_Data_To_Energy_Queue(&dado_de_finalizacao, ATM_RTOS_DEFAULT_DELAYS);
    #elif defined USE_LORA_PORT
      return send_event_to_lora(Cmd_SendEnergyData, 0, 0);
    #endif

  #elif SIMULA_DADOS_ENERGIA == 1

  #endif
}

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

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_voltage_thd(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_VoltageTHD_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Va_thd(uint16_t *read_val)
{
  return atm_acquire_voltage_thd(PHASE_A, read_val);
}
static bool Acquire_Vb_thd(uint16_t *read_val)
{
  return atm_acquire_voltage_thd(PHASE_B, read_val);
}
static bool Acquire_Vc_thd(uint16_t *read_val)
{
  return atm_acquire_voltage_thd(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool atm_acquire_current_thd(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_CurrentTHD_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
   return false;  
}

static bool Acquire_Ia_thd(uint16_t *read_val)
{
  return atm_acquire_current_thd(PHASE_A, read_val);
}
static bool Acquire_Ib_thd(uint16_t *read_val)
{
  return atm_acquire_current_thd(PHASE_B, read_val);
}
static bool Acquire_Ic_thd(uint16_t *read_val)
{
  return atm_acquire_current_thd(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool Acquire_Freq(uint16_t *read_val)
{
  return ATM.Drv.read_reg(ATM_REG_Freq_Add, read_val);
}

#pragma GCC diagnostic pop
