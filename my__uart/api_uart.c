/***************************************************************************************************
* @file   api_uart.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "api_uart.h"
#include "rtos_utility.h"
/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static void Insert_Byte_Buffer_Linear_Manager (uint8_t byByte);
static void Insert_Byte_Buffer_Linear         (uint8_t byByte);
static void Buffer_Clear                      (void);
static void Command_Parse                     (char *pbyBuffer);
static bool send_event_to_atm                 (RTOS_Events_en Target, uint8_t Command, uint8_t SubCommand, uint16_t DataLen, uint8_t *Data);
static bool send_event_to_leds                (RTOS_Events_en Target, uint8_t Command, uint8_t SubCommand, uint16_t DataLen);

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern UART_HandleTypeDef huart3;

/***************************************************************************************************
* Vars
***************************************************************************************************/
UART_app_st  UART = {0};

// Command reception vars
static uint8_t                RxBufferLinear[RB_Size] = {0};
static uint8_t                byIndiceMsg = 0;
static bool                   bSalvandoMensagem = false;
static receive_command_st     UARTStructure = {0};

static char print_buffer_uart_queue[40] = {0}, reg_value_in_char_uart_queue[5] = {0};


static SemaphoreHandle_t 	xSemaphoreEOF;
static QueueHandle_t 		  *Queue_UART_HANDLE = NULL;
static RINGBUFF_T         rbUart;
static uint8_t            byRingBuffer[RB_Size] = {0};

QueueHandle_t      *Queue_ENERGY_HANDLE = NULL;

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool UART_api_init(void)
{
  // Initialize UART queue
  Queue_UART_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_UART);
  *Queue_UART_HANDLE = xQueueCreate(32, SIZE_OF_QUEUE_ITEM);
  if(Queue_UART_HANDLE == NULL)
  {
    return false;
  }  

  // Initialize Energy queue
  Queue_ENERGY_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_ENERGY);
  *Queue_ENERGY_HANDLE = xQueueCreate(128, sizeof(uint32_t));
  if(Queue_ENERGY_HANDLE == NULL)
  {
    return false;
  } 

  // Initialize ring buffer
  RingBuffer_Init(&rbUart, byRingBuffer, sizeof(uint8_t), RB_Size);
  RingBuffer_Flush(&rbUart);

  // Initialize Semaphore
  xSemaphoreEOF = xSemaphoreCreateBinary();

  UART.Rb = &rbUart;
  UART.Queue = *Queue_UART_HANDLE;
  UART.EnergyQueue = *Queue_ENERGY_HANDLE;
  UART.SemEOF = xSemaphoreEOF;
  UART.huart = &huart3;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void Insert_Byte_Buffer_Linear_Manager(uint8_t byByte)
{
  // Inicio de uma nova mensagem
  if (byByte == GM_SOF)
  {
    Buffer_Clear();
    bSalvandoMensagem = true;
    Insert_Byte_Buffer_Linear(byByte);
  }

  // Final de uma mensagem
  else if (byByte == GM_EOF && bSalvandoMensagem)
  {
    bSalvandoMensagem = false;
    Insert_Byte_Buffer_Linear(byByte);

    // Processa
    Command_Parse((char*)RxBufferLinear);
  }

  // Meio de recepcao de uma mensagem
  else if (bSalvandoMensagem)
  {
     Insert_Byte_Buffer_Linear(byByte);
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void Insert_Byte_Buffer_Linear(uint8_t byByte)
{
  // Não deixa o buffer estourar
  if (byIndiceMsg >= RB_Size)
  {
    Buffer_Clear();
    return;
  }

  RxBufferLinear[byIndiceMsg] = byByte;
  byIndiceMsg++;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void Buffer_Clear(void)
{
  memset(RxBufferLinear, 0, RB_Size);
  bSalvandoMensagem = false;
  byIndiceMsg = 0;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void Command_Parse(char *pbyBuffer)
{
  rc_prot_en rc = PROT_Parser(&UARTStructure, pbyBuffer);

  if(rc == Prot_Erro_CRC)
  {
    HAL_UART_Transmit_IT(UART.huart, (uint8_t*)MSG_ERRO_CRC, strlen(MSG_ERRO_CRC));
  }
  else if (rc == Prot_OK)
  {
    HAL_UART_Transmit_IT(UART.huart, (uint8_t*)MSG_ACK, strlen(MSG_ACK));
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool send_event_to_atm(RTOS_Events_en Target, uint8_t Command, uint8_t SubCommand, uint16_t DataLen, uint8_t *Data)
{
  xQueueHandle* posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_ATM);

  GenericQueueData_st stEvent;
  stEvent.enEvent   = Target;
  stEvent.byCmd     = Command;
  stEvent.bySubCmd  = SubCommand;
  stEvent.pbyData   = Data;
  stEvent.wDataLen  = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, UART_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool send_event_to_leds(RTOS_Events_en Target, uint8_t Command, uint8_t SubCommand, uint16_t DataLen)
{
  xQueueHandle* posQueEvtHandle = RTOS_Get_Queue_Idx(QueueIDX_LEDS);

  GenericQueueData_st stEvent;
  stEvent.enEvent   = Target;
  stEvent.byCmd     = Command;
  stEvent.bySubCmd  = SubCommand;
  stEvent.pbyData   = 0;
  stEvent.wDataLen  = DataLen;

  return RTOS_Send_Data_To_Specific_Queue(posQueEvtHandle, &stEvent, UART_RTOS_DEFAULT_DELAYS);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void UART_Parser_Comando_Burro(uint8_t *pbyRxBuff)
{
  UARTStructure.ID      = (pbyRxBuff[1] - 0x30);
  UARTStructure.SubID     = (pbyRxBuff[3] - 0x30);
  UARTStructure.Data[0]  = (pbyRxBuff[13] - 0x30);
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void UART_Recebe_Comando()
{
  uint8_t RxByte = 0;

  while(RingBuffer_Pop(UART.Rb, &RxByte))
  {
    Insert_Byte_Buffer_Linear_Manager(RxByte);
  }
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool UART_Envia_Eventos(void)
{
  if (UARTStructure.ID == 1)
  {
    return send_event_to_atm(EvntFromUARTtoATM, UARTStructure.SubID, UARTStructure.Comando, UARTStructure.DataLen, UARTStructure.Data);
  }
  else if (UARTStructure.ID == 2)
  {
    return send_event_to_leds(EvntFromUARTtoLEDS, UARTStructure.SubID, UARTStructure.Comando, UARTStructure.DataLen);;
  }
  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void UART_check_queue(void)
{
  GenericQueueData_st NewEvent;

  // Verifica se ha eventos para tratar
  if (xQueueReceive(UART.Queue, &NewEvent, UART_RTOS_DEFAULT_DELAYS) != pdPASS)
  {
    return;
  }

  /* Verifica qual evento foi recebido */
  switch (NewEvent.enEvent)
  {
    //----------------------------------------------------------
    case EvntFromATMtoUART:
      switch (NewEvent.byCmd)
      {
        //----------------------------------------------------------
        case Cmd_PrintThis:
          osDelay(10);
          if(NewEvent.bySubCmd == subCmd_print_start_msg)
          {
            HAL_UART_Transmit(UART.huart, (uint8_t*)MSG_PRINT_STARTED, strlen(MSG_PRINT_STARTED), 1000);
            break;
          }
          else if(NewEvent.bySubCmd == subCmd_print_line_feed)
          {
            HAL_UART_Transmit(UART.huart, (uint8_t*)MSG_LINE_FEED, strlen(MSG_LINE_FEED), 1000);
            break;            
          }

          GM_U16_TO_4ASCIIS(NewEvent.wDataLen, reg_value_in_char_uart_queue);
          if(NewEvent.bySubCmd == subCmd_print_this)
          {
            sprintf(print_buffer_uart_queue, MSG_PRINT_THIS, reg_value_in_char_uart_queue);
          }
          else if(NewEvent.bySubCmd == subCmd_print_warn)
          {
            sprintf(print_buffer_uart_queue, MSG_ERRO_WARN, reg_value_in_char_uart_queue);
          }
          else if(NewEvent.bySubCmd == subCmd_print_irq0)
          {
            sprintf(print_buffer_uart_queue, MSG_ERRO_IRQ0, reg_value_in_char_uart_queue);            
          }
          else if(NewEvent.bySubCmd == subCmd_print_irq1)
          { 
            sprintf(print_buffer_uart_queue, MSG_ERRO_IRQ1, reg_value_in_char_uart_queue);            
          }
          HAL_UART_Transmit(UART.huart, (uint8_t*)print_buffer_uart_queue, strlen(print_buffer_uart_queue), 1000);
          memset(print_buffer_uart_queue, 0, 40);
        break;

        //----------------------------------------------------------
        default:
        break;
      } // switch byCmd
    break;
    
    //----------------------------------------------------------
    case EvntFromLEDStoUART:
      switch (NewEvent.byCmd)
      {
        //----------------------------------------------------------
        case Cmd_PrintThis:
          osDelay(10);
          if(NewEvent.bySubCmd ==   subCmd_print_teste_eeprom_ok)
          {
            HAL_UART_Transmit(UART.huart, (uint8_t*)MSG_TESTE_EEPROM_OK, strlen(MSG_TESTE_EEPROM_OK), 1000);
            break;
          }
          else if(NewEvent.bySubCmd ==   subCmd_print_teste_eeprom_error)
          {
            HAL_UART_Transmit(UART.huart, (uint8_t*)MSG_TESTE_EEPROM_ERR, strlen(MSG_TESTE_EEPROM_ERR), 1000);
            break;
          }
        break;
        //----------------------------------------------------------
        default:
        break;
      }    
    break;

    //----------------------------------------------------------
    default:
    break;
  } // switch enEvent
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void UART_check_EnergyQueue(void)
{
  uint32_t hex_data;
  // Verifica se ha eventos para tratar
  if (xQueueReceive(UART.EnergyQueue, &hex_data, UART_RTOS_DEFAULT_DELAYS) != pdPASS)
  {
    return;
  }
  
  char print_buffer_energy_queue[10] = {0};
  uint16_t size = 0; 

  if(hex_data == 0xffffffff)
  {
    print_buffer_energy_queue[0] = '\r';
    print_buffer_energy_queue[1] = '\0';
    size = 1;
  }
  else
  {
    GM_U32_TO_8ASCIIS(hex_data, print_buffer_energy_queue);
    strcat(print_buffer_energy_queue, ",");
    size = 9;
  }

  HAL_UART_Transmit(UART.huart, (uint8_t*)print_buffer_energy_queue, size, 1000);
}

/*
 * Exemplo:
 V = 116,53V
 I = 4,852A
 P = 565W = 00,565kW
 Q = 100Var = 00,100kVar
 S = 574VA = 00,574kVA
 PF = 0,9846

 Data type A: XXX.XX
 Data type B: XX.XXX
 Data type C: X.XXX
*/

/* 
uint8_t Convert_To_Print_Timestamp(uint32_t dwDadoIn, char *pbyFullOutputVect)
{
  // Local variables
  uint8_t aux = 0;
  uint8_t pbyOutBuff[16] = {0};

  pbyOutBuff[aux] = 0x2a; // '*'

  pbyOutBuff[++aux] = (((dwDadoIn/1000000000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/100000000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/10000000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/1000000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/100000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/10000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/1000)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/100)%10)+0x30);
  pbyOutBuff[++aux] = (((dwDadoIn/10)%10)+0x30);
  pbyOutBuff[++aux] = ((dwDadoIn%10)+0x30);

  pbyOutBuff[++aux] = 0x2a; // "*"

  // Append strings
  strcat(pbyFullOutputVect, (char*)pbyOutBuff);

  return 1;
}


uint8_t Convert_To_Print_TypeA(uint16_t wDadoIn, char *pbyFullOutputVect, char *Unidade, uint8_t byPhase)
{
  // Error condition
  if(wDadoIn == 0)
    return 0;

  // Local variables
  uint8_t aux = 0;
  uint8_t pbyOutBuff[20] = {0};

  pbyOutBuff[aux] = 0x2a; // "*"
  pbyOutBuff[++aux] = 0x56; // "V"

  if(byPhase == ATM_PHASE_A)
    pbyOutBuff[++aux] = 0x61;
  else if (byPhase == ATM_PHASE_B)
    pbyOutBuff[++aux] = 0x62;
  else if (byPhase == ATM_PHASE_C)
    pbyOutBuff[++aux] = 0x63;

  pbyOutBuff[++aux] = 0x3d; // "="

  pbyOutBuff[++aux] = ((wDadoIn/10000)+0x30);       // centena
  pbyOutBuff[++aux] = (((wDadoIn/1000)%10)+0x30);   // dezena
  pbyOutBuff[++aux] = (((wDadoIn/100)%10)+0x30);   // unidade
  pbyOutBuff[++aux] = 0x2c;                         // virgula
  pbyOutBuff[++aux] = (((wDadoIn/10)%10)+0x30);   // decimo
  pbyOutBuff[++aux] = ((wDadoIn%10)+0x30);       // centesimo

  memcpy(&pbyOutBuff[++aux], Unidade, strlen(Unidade)); // unidade

  // Append strings
  strcat(pbyFullOutputVect, (char*)pbyOutBuff);

  return 1;
}

uint8_t Convert_To_Print_TypeB(uint16_t wDadoIn, uint8_t Indentificador, char *pbyFullOutputVect, char *Unidade, uint8_t byPhase)
{
  // Error condition
  if(wDadoIn == 0)
    return 0;

  // Local variables
  uint8_t aux = 0;
  uint8_t pbyOutBuff[20] = {0};

  pbyOutBuff[aux] = 0x2a; // "*"
  pbyOutBuff[++aux] = Indentificador; // "I", "P", "S", etc

  if(byPhase == ATM_PHASE_A)
    pbyOutBuff[++aux] = 0x61;
  else if (byPhase == ATM_PHASE_B)
    pbyOutBuff[++aux] = 0x62;
  else if (byPhase == ATM_PHASE_C)
    pbyOutBuff[++aux] = 0x63;           // P = 565W = 00,565kW  0x0235  // 0x12f4 -- *Ia=4,852Arms*

  pbyOutBuff[++aux] = 0x3d; // "="

  pbyOutBuff[++aux] = ((wDadoIn/10000)+0x30);     // dezena
  pbyOutBuff[++aux] = (((wDadoIn/1000)%10)+0x30); // unidade
  pbyOutBuff[++aux] = 0x2c;                       // virgula
  pbyOutBuff[++aux] = (((wDadoIn/100)%10)+0x30);  // decimo
  pbyOutBuff[++aux] = (((wDadoIn/10)%10)+0x30);   // centesimo
  pbyOutBuff[++aux] = ((wDadoIn%10)+0x30);	  // milesimo

  memcpy(&pbyOutBuff[++aux], Unidade, strlen(Unidade)); // unidade

  // Append strings
  strcat(pbyFullOutputVect, (char*)pbyOutBuff);

  return 1;
}

uint8_t Convert_To_Print_TypeC(uint16_t wDadoIn, char *pbyFullOutputVect, uint8_t byPhase)
{
  // Error condition
  if(wDadoIn == 0)
    return 0;

  // Local variables
  uint8_t aux = 0;
  uint8_t pbyOutBuff[20] = {0};

  pbyOutBuff[aux] = 0x2a; // "*"
  pbyOutBuff[++aux] = 0X59; // "Y" para facilitar

  if(byPhase == ATM_PHASE_A)
    pbyOutBuff[++aux] = 0x61;
  else if (byPhase == ATM_PHASE_B)
    pbyOutBuff[++aux] = 0x62;
  else if (byPhase == ATM_PHASE_C)
    pbyOutBuff[++aux] = 0x63;           // 0x2676 = 0,9846

  pbyOutBuff[++aux] = 0x3d; // "="

  pbyOutBuff[++aux] = ((wDadoIn/10000)+0x30);	 // unidade
  pbyOutBuff[++aux] = 0x2c;                      // virgula
  pbyOutBuff[++aux] = (((wDadoIn/1000)%10)+0x30);// decimo
  pbyOutBuff[++aux] = (((wDadoIn/100)%10)+0x30); // centesimo
  pbyOutBuff[++aux] = (((wDadoIn/10)%10)+0x30);  // milesimo
  pbyOutBuff[++aux] = ((wDadoIn%10)+0x30);	 // decimo de milesimo

  pbyOutBuff[++aux] = 0x2a; // "*" (nao tem unidade)

  // Append strings
  strcat(pbyFullOutputVect, (char*)pbyOutBuff);

  return 1;
}

void Convert_To_Print_CrLf(char *pbyFullOutputVect)
{
  // Local Variables
  char CRLF[2] = {0x0D, 0x0A};

  // Append strings
  strcat(pbyFullOutputVect, CRLF);
}

*/
