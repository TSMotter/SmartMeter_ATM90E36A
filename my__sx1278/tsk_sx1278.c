/***************************************************************************************************
* @file   tsk_sx1278.c
* @brief  
* @author Giuliano Motter
* @date   11/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_sx1278.h"
#include "api_lora.h"
#include "cmsis_os.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern SX1278_app_st LORA;
extern osThreadId task_atm90e36aHandle;

/***************************************************************************************************
* Vars
***************************************************************************************************/

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void start_sx1278_task(void const * argument)
{
  if( (!LORA_api_init()) || (!LORA_config(434000000)) )
  {
    while(1);
  }

  xTaskNotify(task_atm90e36aHandle, 0, eIncrement);

  #if defined LORA_TEST_RX
    uint8_t packet[20] = {0}, len = 0;
    int rssi = 0;
  #elif defined LORA_TEST_TX
    uint8_t payload[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  #endif


  for(;;)
  {
    #if defined LORA_TEST_RX

      LORA_Receive(0, packet, &len, &rssi);

    #elif defined LORA_TEST_TX

	    LORA_Transmit(payload, sizeof(payload), 1000);
      payload[0] = payload[0] == 0x40 ? 0x30 : payload[0]+1;
	    vTaskDelay(RTOS_DELAY_MS(20000));

    #else

      LORA_Receive(0, &LORA.Pacote);

      if(LORA.Pacote.len)
      {
        // Processa e encaminha eventos para demais tasks
        LORA_Process_n_SendEvents(&LORA.Pacote);
      }

      LORA_api_periodic_checks();

    #endif
  }

}
