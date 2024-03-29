/***************************************************************************************************
* @file   tsk_uart.c
* @brief  Task that describes and controls the behaviour of the UART terminal
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_uart.h"
#include "cmsis_os.h"
//#include "stm32f4xx_hal.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern UART_app_st  UART;
extern osThreadId   task_atm90e36aHandle;

/***************************************************************************************************
* Vars
***************************************************************************************************/
static uint8_t     DummyByte = 0;

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void start_uart_task(void const * argument)
{

  if(!UART_api_init())
  {
    while(1);
  }
  
  xTaskNotify(task_atm90e36aHandle, 0, eIncrement);

  for(;;)
  {
    HAL_UART_Receive_IT(UART.huart, &DummyByte, 1);

    if (xSemaphoreTake(UART.SemEOF, UART_RTOS_DEFAULT_DELAYS) == pdPASS)
    {
    	UART_Recebe_Comando();
    	UART_Envia_Eventos();
    }

    UART_check_queue();
    
    #ifdef USE_UART_PORT
    UART_check_EnergyQueue();
    #endif
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t ReceivedByte = *(huart->pRxBuffPtr - 1);

  RingBuffer_Insert(UART.Rb, &ReceivedByte);

  if(ReceivedByte == GM_EOF)
  {
    xSemaphoreGiveFromISR(UART.SemEOF, &xHigherPriorityTaskWoken);
    return;
  }
  HAL_UART_Receive_IT(UART.huart, &DummyByte, 1);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}
