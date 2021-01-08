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

  uint8_t lora_rx_packet[LORA_RX_BUFFER_SIZE] = {0}, len = 0;
  int rssi = 0;

  for(;;)
  {
    #if defined LORA_TEST_RX
    LORA_Receive(0, packet, &len, &rssi);
    #elif defined   LORA_TEST_TX
	  LORA_Transmit(payload, sizeof(payload), 1000);
	  vTaskDelay(RTOS_DELAY_MS(2000));
    #endif

    LORA_Receive(0, lora_rx_packet, &len, &rssi);
    
    if(len)
    {
      // Processa
      LORA_Process(lora_rx_packet, len);

      // Limpa
      memset(lora_rx_packet, 0, LORA_RX_BUFFER_SIZE);
      len = 0;
      rssi = 0;
    }

    LORA_api_periodic_checks();
  }

}
