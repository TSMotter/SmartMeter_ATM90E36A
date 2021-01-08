/***************************************************************************************************
* @file   api_leds.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_leds.h"
#include "stm32f407xx.h"
#include "cmsis_os.h"
/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static bool send_event_to_uart(uint8_t Command, uint8_t SubCommand, uint16_t DataLen);

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern TIM_HandleTypeDef htim2;

/***************************************************************************************************
* Vars
***************************************************************************************************/
leds_drv_st LEDS = {0};

// RTOS related variables
QueueHandle_t *Queue_LEDS_HANDLE = NULL;

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LEDS_api_init(void)
{
  // Cria fila LEDS
  Queue_LEDS_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_LEDS);
  *Queue_LEDS_HANDLE = xQueueCreate(32, SIZE_OF_QUEUE_ITEM);
  if(Queue_LEDS_HANDLE == NULL)
  {
    return false;
  }

  LEDS_hw_init();

  if(!LEDS_drv_init(&LEDS))
  {
    return false;
  }
  

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LEDS_check_queue (void)
{
  GenericQueueData_st NewEvent;

  // Verifica se ha eventos para tratar
  if (xQueueReceive (*Queue_LEDS_HANDLE, &NewEvent, LEDS_RTOS_DEFAULT_DELAYS) != pdPASS)
  {
    return;
  }
  
  /* Verifica qual evento foi recebido */
  switch (NewEvent.enEvent)
  {
    case EvntFromATMtoLEDS:
      switch (NewEvent.byCmd)
	    {
        //----------------------------------------------
        case Cmd_BlinkPattern1:
          LEDS_blink_pattern1(NewEvent.wDataLen);
        break;
        //----------------------------------------------
        case Cmd_BlinkPattern2:
          LEDS_blink_pattern2(NewEvent.wDataLen);
        break;
        //----------------------------------------------
        case Cmd_BlinkPattern3:
          LEDS_blink_pattern3(NewEvent.wDataLen);
        break;                
        //----------------------------------------------
        default:
        break;
      }
    break;

    case EvntFromUARTtoLEDS:
      switch (NewEvent.byCmd)
	    {
        //----------------------------------------------
        case Cmd_TestEEPROM:
          if(EE24_TestChip())
          {
            send_event_to_uart(Cmd_PrintThis, subCmd_print_teste_eeprom_ok, 0);
          }
          else
          {
            send_event_to_uart(Cmd_PrintThis, subCmd_print_teste_eeprom_error, 0);
          }
        break;
        //----------------------------------------------
        default:
        break;        
      }    
    break;

    default:
    break;
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool send_event_to_uart(uint8_t Command, uint8_t SubCommand, uint16_t DataLen)
{
  xQueueHandle *posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_UART);
  GenericQueueData_st stEvent;

  stEvent.enEvent = EvntFromLEDStoUART;
  stEvent.byCmd = Command;
  stEvent.bySubCmd = SubCommand;
  stEvent.pbyData = NULL;
  stEvent.wDataLen = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, LEDS_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LEDS_blink_pattern1(leds_periods_en period)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  
  LEDS.all_leds_off();
  
  SofTim_StartTimer2(LEDS.hw.green.tmr, period);
  osDelay(period/3);
  SofTim_StartTimer2(LEDS.hw.orange.tmr, period);
  osDelay(period/3);
  SofTim_StartTimer2(LEDS.hw.red.tmr, period);
  osDelay(period/3);    
  SofTim_StartTimer2(LEDS.hw.blue.tmr, period);

  HAL_TIM_Base_Start_IT(&htim2);

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LEDS_blink_pattern2(leds_periods_en period)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  
  LEDS.all_leds_off();
  
  SofTim_StartTimer2(LEDS.hw.green.tmr, period);
  SofTim_StartTimer2(LEDS.hw.red.tmr, period);
  osDelay(period);
  SofTim_StartTimer2(LEDS.hw.orange.tmr, period);    
  SofTim_StartTimer2(LEDS.hw.blue.tmr, period);
  
  HAL_TIM_Base_Start_IT(&htim2);

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LEDS_blink_pattern3(leds_periods_en period)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  
  LEDS.all_leds_off();
  
  SofTim_StartTimer2(LEDS.hw.green.tmr, period);
  SofTim_StartTimer2(LEDS.hw.red.tmr, period);
  SofTim_StartTimer2(LEDS.hw.orange.tmr, period);    
  SofTim_StartTimer2(LEDS.hw.blue.tmr, period);
  
  HAL_TIM_Base_Start_IT(&htim2);

  return true;
}