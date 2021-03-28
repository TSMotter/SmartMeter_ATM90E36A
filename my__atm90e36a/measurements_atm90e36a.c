/***************************************************************************************************
* @file   measurements_atm90e36a.c
* @brief  Arquivo com funções de aquisição de medidas do atm90e36a
* @author Giuliano Motter
* @date   03/2021
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "measurements_atm90e36a.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

static bool Acquire_Va       (uint16_t *read_val); 	
static bool Acquire_Vb       (uint16_t *read_val); 	
static bool Acquire_Vc       (uint16_t *read_val); 	
static bool Acquire_Ia       (uint16_t *read_val); 	
static bool Acquire_Ib       (uint16_t *read_val); 	
static bool Acquire_Ic       (uint16_t *read_val); 	
static bool Acquire_Pa       (uint16_t *read_val); 	
static bool Acquire_Pb       (uint16_t *read_val); 	
static bool Acquire_Pc       (uint16_t *read_val); 	
static bool Acquire_Qa       (uint16_t *read_val); 	
static bool Acquire_Qb       (uint16_t *read_val); 	
static bool Acquire_Qc       (uint16_t *read_val); 	
static bool Acquire_Sa       (uint16_t *read_val); 	
static bool Acquire_Sb       (uint16_t *read_val); 	
static bool Acquire_Sc       (uint16_t *read_val);  	 
static bool Acquire_Pa_fund  (uint16_t *read_val);
static bool Acquire_Pb_fund  (uint16_t *read_val);
static bool Acquire_Pc_fund  (uint16_t *read_val);
static bool Acquire_Pa_harm  (uint16_t *read_val);
static bool Acquire_Pb_harm  (uint16_t *read_val);
static bool Acquire_Pc_harm  (uint16_t *read_val);
static bool Acquire_Va_thdn  (uint16_t *read_val);
static bool Acquire_Vb_thdn  (uint16_t *read_val);
static bool Acquire_Vc_thdn  (uint16_t *read_val);
static bool Acquire_Ia_thdn  (uint16_t *read_val);
static bool Acquire_Ib_thdn  (uint16_t *read_val);
static bool Acquire_Ic_thdn  (uint16_t *read_val);
static bool Acquire_2_order_Ia_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_2_order_Ib_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_2_order_Ic_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Ia_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Ib_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Ic_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Ia_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Ib_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Ic_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Ia_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Ib_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Ic_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_Total_Ia_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_Total_Ib_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_Total_Ic_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_2_order_Va_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_2_order_Vb_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_2_order_Vc_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Va_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Vb_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_3_order_Vc_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Va_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Vb_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_4_order_Vc_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Va_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Vb_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_5_order_Vc_Harm_Ratio   (uint16_t *read_val);
static bool Acquire_Total_Va_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_Total_Vb_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_Total_Vc_Harm_Ratio     (uint16_t *read_val);
static bool Acquire_Freq    (uint16_t *read_val);

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern ATM90_app_st ATM;

/***************************************************************************************************
* Vars
***************************************************************************************************/
static aquisicao_de_medidas_st vetor_de_aquisicao[numero_total_medidas] = {
  // Phase A:
  {.active = false, .read_func = Acquire_Va},
  {.active = false, .read_func = Acquire_Ia},
  {.active = false, .read_func = Acquire_Pa},
  {.active = false, .read_func = Acquire_Qa},
  {.active = false, .read_func = Acquire_Sa},
  {.active = false, .read_func = Acquire_Pa_fund},
  {.active = false, .read_func = Acquire_Pa_harm},
  {.active = false, .read_func = Acquire_Va_thdn},
  {.active = false, .read_func = Acquire_Ia_thdn},
  {.active = false, .read_func = Acquire_2_order_Ia_Harm_Ratio},
  {.active = false, .read_func = Acquire_3_order_Ia_Harm_Ratio},
  {.active = false, .read_func = Acquire_4_order_Ia_Harm_Ratio},
  {.active = false, .read_func = Acquire_5_order_Ia_Harm_Ratio},
  {.active = false, .read_func = Acquire_Total_Ia_Harm_Ratio},
  {.active = false, .read_func = Acquire_2_order_Va_Harm_Ratio},
  {.active = false, .read_func = Acquire_3_order_Va_Harm_Ratio},
  {.active = false, .read_func = Acquire_4_order_Va_Harm_Ratio},
  {.active = false, .read_func = Acquire_5_order_Va_Harm_Ratio},
  {.active = false, .read_func = Acquire_Total_Va_Harm_Ratio},

  // Phase B:
  {.active = false, .read_func = Acquire_Vb},
  {.active = false, .read_func = Acquire_Ib},
  {.active = false, .read_func = Acquire_Pb},
  {.active = false, .read_func = Acquire_Qb},
  {.active = false, .read_func = Acquire_Sb},
  {.active = false, .read_func = Acquire_Pb_fund},
  {.active = false, .read_func = Acquire_Pb_harm},
  {.active = false, .read_func = Acquire_Vb_thdn},
  {.active = false, .read_func = Acquire_Ib_thdn},

  // Phase C:
  {.active = false, .read_func = Acquire_Vc},
  {.active = false, .read_func = Acquire_Ic},
  {.active = false, .read_func = Acquire_Pc},
  {.active = false, .read_func = Acquire_Qc},
  {.active = false, .read_func = Acquire_Sc},
  {.active = false, .read_func = Acquire_Pc_fund},
  {.active = false, .read_func = Acquire_Pc_harm},
  {.active = false, .read_func = Acquire_Vc_thdn},
  {.active = false, .read_func = Acquire_Ic_thdn},    

  // Others
  {.active = false, .read_func = Acquire_Freq},
};


/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool assina_medidas(uint16_t id)
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
bool realiza_medidas(void)
{

  #if SIMULA_DADOS_ENERGIA == 0

    uint32_t dado_de_envio = 0, dado_de_finalizacao = 0xffffffff;
    uint16_t temp_read_val = 0, rc = 0;
    
    // Varre vetor e realiza leituras, 
    for(uint16_t indice = 0; indice < numero_total_medidas; indice++)
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
static bool atm_acquire_line_voltage(phase_en phase, uint16_t *val)
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
static bool atm_acquire_line_current(phase_en phase, uint16_t *val)
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
static bool atm_acquire_active_power(phase_en phase, uint16_t *val)
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
static bool atm_acquire_reactive_power(phase_en phase, uint16_t *val)
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
static bool atm_acquire_aparent_power(phase_en phase, uint16_t *val)
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
static bool atm_acquire_power_factor(phase_en phase, uint16_t *val)
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
static bool atm_acquire_active_fundamental_power(phase_en phase, uint16_t *val)
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
static bool atm_acquire_active_harmonic_power(phase_en phase, uint16_t *val)
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
static bool atm_acquire_voltage_thdn(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_VoltageTHDN_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_Va_thdn(uint16_t *read_val)
{
  return atm_acquire_voltage_thdn(PHASE_A, read_val);
}
static bool Acquire_Vb_thdn(uint16_t *read_val)
{
  return atm_acquire_voltage_thdn(PHASE_B, read_val);
}
static bool Acquire_Vc_thdn(uint16_t *read_val)
{
  return atm_acquire_voltage_thdn(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_current_thdn(phase_en phase, uint16_t *val)
{
  if(ATM.Drv.read_reg((ATM_REG_CurrentTHDN_Offset + phase), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_Ia_thdn(uint16_t *read_val)
{
  return atm_acquire_current_thdn(PHASE_A, read_val);
}
static bool Acquire_Ib_thdn(uint16_t *read_val)
{
  return atm_acquire_current_thdn(PHASE_B, read_val);
}
static bool Acquire_Ic_thdn(uint16_t *read_val)
{
  return atm_acquire_current_thdn(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_2_order_current_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_2_Order_HarmRatio_Current_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_2_order_Ia_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_current_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_2_order_Ib_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_current_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_2_order_Ic_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_current_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_3_order_current_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_3_Order_HarmRatio_Current_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_3_order_Ia_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_current_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_3_order_Ib_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_current_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_3_order_Ic_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_current_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_4_order_current_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_4_Order_HarmRatio_Current_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_4_order_Ia_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_current_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_4_order_Ib_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_current_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_4_order_Ic_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_current_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_5_order_current_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_5_Order_HarmRatio_Current_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_5_order_Ia_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_current_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_5_order_Ib_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_current_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_5_order_Ic_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_current_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_total_current_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_Total_HarmRatio_Current_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_Total_Ia_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_current_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_Total_Ib_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_current_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_Total_Ic_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_current_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_2_order_voltage_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_2_Order_HarmRatio_Voltage_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_2_order_Va_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_voltage_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_2_order_Vb_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_voltage_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_2_order_Vc_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_2_order_voltage_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_3_order_voltage_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_3_Order_HarmRatio_Voltage_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_3_order_Va_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_voltage_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_3_order_Vb_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_voltage_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_3_order_Vc_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_3_order_voltage_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_4_order_voltage_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_4_Order_HarmRatio_Voltage_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_4_order_Va_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_voltage_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_4_order_Vb_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_voltage_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_4_order_Vc_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_4_order_voltage_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_5_order_voltage_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_5_Order_HarmRatio_Voltage_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_5_order_Va_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_voltage_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_5_order_Vb_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_voltage_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_5_order_Vc_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_5_order_voltage_harmonic_ratio(PHASE_C, read_val);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool atm_acquire_total_voltage_harmonic_ratio(phase_en phase, uint16_t *val)
{
  uint8_t offset = ((phase - 1) * 20);

  /*
    phase // offset
    A (1) //   0
    B (2) //   20
    C (3) //   40
  */

  if(ATM.Drv.read_reg((ATM_REG_Total_HarmRatio_Voltage_Offset + offset), val) == ATM_RC_OK)
  {
    return true;
  }
  return false;  
}

static bool Acquire_Total_Va_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_voltage_harmonic_ratio(PHASE_A, read_val);
}
static bool Acquire_Total_Vb_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_voltage_harmonic_ratio(PHASE_B, read_val);
}
static bool Acquire_Total_Vc_Harm_Ratio(uint16_t *read_val)
{
  return atm_acquire_total_voltage_harmonic_ratio(PHASE_C, read_val);
}
/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool Acquire_Freq(uint16_t *read_val)
{
  return ATM.Drv.read_reg(ATM_REG_Freq_Add, read_val);
}


#pragma GCC diagnostic pop