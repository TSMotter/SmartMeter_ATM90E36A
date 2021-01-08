/***************************************************************************************************
* @file   api_lora.c
* @brief  Adaptação do arquivo de referencia de Arduino
* @author Giuliano Motter
* @date   01/2021
* @note   Copyright (c) Sandeep Mistry.
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "api_lora.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static void lora_api_check_queue  (void);
static bool lora_send_energy_data(void);

static bool   lora_sleep                        (void);
static bool   lora_standby                      (void);
static bool   lora_set_frequency                (uint32_t freq);
static bool   lora_set_tx_power                 (uint8_t level, int outputPin);
static bool   lora_set_over_current_protection  (uint8_t mA);
static bool   lora_is_transmitting              (void);
static bool   lora_enter_implicit_header_mode   (void);
static bool   lora_enter_explicit_header_mode   (void);
static bool   lora_get_rssi                     (int *rssi);
static float  lora_get_packet_snr               (void);

static bool     lora_tx_begin_packet  (uint8_t implicitHeader);
static size_t   lora_tx_write         (uint8_t *buffer, size_t len);
static bool     lora_tx_end_packet    (bool async, uint32_t timeout);

static bool lora_rx_single_begin_packet (uint8_t size, uint8_t *packetLen);
static bool lora_rx_single_read         (uint8_t len, uint8_t *packet);

/***************************************************************************************************
* Externals
***************************************************************************************************/
extern QueueHandle_t      *Queue_ENERGY_HANDLE;
/***************************************************************************************************
* Vars
***************************************************************************************************/
SX1278_app_st LORA = {0};
static QueueHandle_t *Queue_LORA_HANDLE = NULL;
static char lora_buffer_energy_queue[50] = {0};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LORA_api_init(void)
{
  // Cria fila
	Queue_LORA_HANDLE = RTOS_Get_Queue_Idx(QueueIDX_LORA);
	*Queue_LORA_HANDLE = xQueueCreate(32, SIZE_OF_QUEUE_ITEM);
  if(Queue_LORA_HANDLE == NULL)
  {
    return false;
  }

  SX1278_hw_init();

  SX1278_drv_init(&LORA.Drv);

  LORA.State.Current = SxState_Suspended;
  LORA.Mode = ListeningMode;
  LORA.Queue = *Queue_LORA_HANDLE;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LORA_config(uint32_t frequency)
{
  // Perform hardware reset
  LORA.Drv.hard_reset();

  // Check version
  uint8_t version = 0;
  gAssert(LORA.Drv.read_data(REG_VERSION, &version));
  if (version != 0x12) 
  {
    return false;
  }

  gAssert(lora_sleep());

  gAssert(lora_set_frequency(frequency));

  // set base addresses [default]
  gAssert(LORA.Drv.write_data(REG_FIFO_TX_BASE_ADDR, BASE_TX_POINTER));
  gAssert(LORA.Drv.write_data(REG_FIFO_RX_BASE_ADDR, BASE_RX_POINTER));

  // set LNA boost
  uint8_t aux = 0;
  gAssert(LORA.Drv.read_data(REG_LNA, &aux));
  gAssert(LORA.Drv.write_data(REG_LNA, aux | 0x03));

  // set auto AGC (Automatic gain control)
  gAssert(LORA.Drv.write_data(REG_MODEM_CONFIG_3, 0x04));

  // set output power to 17 dBm [default]
  gAssert(lora_set_tx_power(17, PA_OUTPUT_PA_BOOST_PIN));

  gAssert(lora_standby());

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LORA_Transmit(uint8_t *buffer, size_t len, uint32_t timeout)
{
  gAssert(lora_tx_begin_packet(0));
  gAssert(lora_tx_write(buffer, len));
  gAssert(lora_tx_end_packet(false, timeout));
  
  return true;
}

/***************************************************************************************************
* @brief 
* @param impl_hdr_size_to_read: Normally will be '0', meaning that the module is operating in Explicit
* header mode. If Implicit header mode is wanted, user must inform how many bytes want to recieve here
***************************************************************************************************/
bool LORA_Receive(uint8_t impl_hdr_size_to_read, uint8_t *packet, uint8_t *len_was_read, int *rssi)
{
  gAssert(lora_rx_single_begin_packet(impl_hdr_size_to_read, len_was_read));

  if(*len_was_read)
  {
    gAssert(lora_rx_single_read(*len_was_read, packet));
    gAssert(lora_get_rssi(rssi));
  }

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
bool LORA_Process(uint8_t *buffer, size_t len)
{

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void LORA_api_periodic_checks(void)
{
  // Checa pinos
  if(LORA.Drv.monitor_dio0())
  {
    // Do something...

  }

  lora_api_check_queue();  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void lora_api_check_queue(void)
{
  GenericQueueData_st NewEvent;  

  if (xQueueReceive(LORA.Queue, &NewEvent, SX_RTOS_DEFAULT_DELAYS) != pdPASS)
  {
	  return;
  }

  switch(NewEvent.enEvent)
  {
	  case EvntFromATMtoLORA:
		  switch (NewEvent.byCmd)
		  {
        //----------------------------------------------
        case Cmd_SendEnergyData:
          if(lora_send_energy_data())
          {
            // Registra que tem uma nova mensagem esperando ACK e da um timeout para esse ack chegar

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
static bool lora_send_energy_data(void)
{
  uint32_t hex_data = 0, cntr = 0;
  char temp[5] = {0};

  // Retira todos os itens da fila de energia
  while (xQueueReceive(*Queue_ENERGY_HANDLE, &hex_data, 0) == pdPASS)
  {
    cntr++;
    GM_U32_TO_8ASCIIS(hex_data, temp);
    strcat(lora_buffer_energy_queue, temp);
    strcat(lora_buffer_energy_queue, ",");
    continue;
  }
  
  // Termina de montar a msg de envio (ID, sequence number)

  // Envia dados via Lora

  // Limpa buffer  
  memset(lora_buffer_energy_queue, 0, 50);

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_rx_single_begin_packet(uint8_t size, uint8_t *packetLen)
{
  uint8_t irqFlags = 0;

  *packetLen = 0; // Forces zero 

  gAssert(LORA.Drv.read_data(REG_IRQ_FLAGS, &irqFlags));

  if(size && LORA.Drv.header == header_is_explicit)
  {
    lora_enter_implicit_header_mode();

    gAssert(LORA.Drv.write_data(REG_PAYLOAD_LENGTH, size & 0xff));
  }
  else if (LORA.Drv.header == header_is_implicit)
  {
    lora_enter_explicit_header_mode();
  }

  // Limpa IRQ
  gAssert(LORA.Drv.write_data(REG_IRQ_FLAGS, irqFlags));
  
  // Recebeu RxDone
  if(irqFlags & IRQ_RX_DONE_MASK)
  {
    // Read packet len
    if(LORA.Drv.header == header_is_implicit)
    {
      gAssert(LORA.Drv.read_data(REG_PAYLOAD_LENGTH, packetLen));
    }
    else
    {
      gAssert(LORA.Drv.read_data(REG_RX_NB_BYTES, packetLen));
    }

    // Set fifo address to current rx address
    uint8_t aux = 0;
    gAssert(LORA.Drv.read_data(REG_FIFO_RX_CURRENT_ADDR, &aux));
    gAssert(LORA.Drv.write_data(REG_FIFO_ADDR_PTR, aux));

    // Entra em standby
    gAssert(lora_standby());
    return true;
  }

  uint8_t op_mode = 0;
  gAssert(LORA.Drv.read_data(REG_OP_MODE, &op_mode));
  // Se nao estou em Lora Rx -> Entro (Pode ser que eu tenha saído sozinho devido ao timeout do Rx anterior)
  if((op_mode & (MODE_RX_SINGLE | MODE_LONG_RANGE_MODE)) != (MODE_RX_SINGLE | MODE_LONG_RANGE_MODE))
  {
    gAssert(LORA.Drv.write_data(REG_FIFO_ADDR_PTR, BASE_RX_POINTER));
    gAssert(LORA.Drv.change_mode(MODE_RX_SINGLE));
  }

  vTaskDelay(SX_RTOS_DEFAULT_DELAYS);
  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_rx_single_read(uint8_t len, uint8_t *packet)
{
  for(uint8_t k = 0; k < len; k++)
  {
    gAssert(LORA.Drv.read_data(REG_FIFO, &packet[k]));
  }
  
  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_tx_begin_packet(uint8_t implicitHeader)
{
  if (lora_is_transmitting()) 
  {
    return 0;
  }

  // put in standby mode
  gAssert(lora_standby());

  if (implicitHeader) 
  {
    gAssert(lora_enter_implicit_header_mode());
  } 
  else 
  {
    gAssert(lora_enter_explicit_header_mode());
  }

  // reset FIFO address and payload length
  gAssert(LORA.Drv.write_data(REG_FIFO_ADDR_PTR, BASE_TX_POINTER));
  gAssert(LORA.Drv.write_data(REG_PAYLOAD_LENGTH, 0));


  return true;  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static size_t lora_tx_write(uint8_t *buffer, size_t len)
{
  uint8_t currentLength = 0;
  gAssert(LORA.Drv.read_data(REG_PAYLOAD_LENGTH, &currentLength));

  // check size
  if ((currentLength + len) > MAX_PKT_LENGTH) 
  {
    len = MAX_PKT_LENGTH - currentLength;
  }

  // write data to FIFO
  for(uint8_t k = 0; k < len; k++)
  {
    gAssert(LORA.Drv.write_data(REG_FIFO, buffer[k]));
  }

  // update length
  gAssert(LORA.Drv.write_data(REG_PAYLOAD_LENGTH, currentLength + len));

  return len;  
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_tx_end_packet(bool async, uint32_t timeout)
{
  uint8_t aux = 0;

  if ((async) && (LORA.tx_done_callback))
  {
    // DIO0 => TXDONE
    gAssert(LORA.Drv.write_data(REG_DIO_MAPPING_1, 0x40)); 
  }

  // put in TX mode
  gAssert(LORA.Drv.change_mode(MODE_TX));

  if (!async) 
  {
    // wait for TX done
    gAssert(LORA.Drv.read_data(REG_IRQ_FLAGS, &aux));

    // Enquanto nao deu TxDone, e o timeout nao acabou, fico rodando o loop
    while (((IRQ_TX_DONE_MASK & aux) != IRQ_TX_DONE_MASK) && (timeout)) 
    {
      gAssert(LORA.Drv.read_data(REG_IRQ_FLAGS, &aux));
      timeout = timeout >= 100 ? timeout - 100 : 0;

      if(!timeout)
      {
        return false;
      }

      vTaskDelay(RTOS_DELAY_MS(100));
    }
    // clear IRQ's
    gAssert(LORA.Drv.write_data(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK));
  }

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_sleep(void)
{
  gAssert(LORA.Drv.change_mode(MODE_SLEEP));
  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_standby(void)
{
  gAssert(LORA.Drv.change_mode(MODE_STDBY));

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_set_frequency(uint32_t freq)
{
	uint64_t frequency = ((uint64_t) freq << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (frequency >> 16);
	freq_reg[1] = (uint8_t) (frequency >> 8);
	freq_reg[2] = (uint8_t) (frequency >> 0);

  gAssert(LORA.Drv.write_burst_data(REG_FRF_MSB, freq_reg, 3));

  LORA.Drv.frequencia = freq;

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_set_tx_power(uint8_t level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) 
  {
    // RFO
    if (level < 0) 
    {
      level = 0;
    } 
    else if (level > 14) 
    {
      level = 14;
    }

    gAssert(LORA.Drv.write_data(REG_PA_CONFIG, 0x70 | level));
  } 
  else 
  {
    // PA BOOST
    if (level > 17) 
    {
      if (level > 20) 
      {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      gAssert(LORA.Drv.write_data(REG_PA_DAC, 0x87));
      gAssert(lora_set_over_current_protection(140));
    } 
    else 
    {
      if (level < 2) 
      {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      gAssert(LORA.Drv.write_data(REG_PA_DAC, 0x84));
      gAssert(lora_set_over_current_protection(100));
    }

    gAssert(LORA.Drv.write_data(REG_PA_CONFIG, PA_BOOST | (level - 2)));
    //gAssert(LORA.Drv.write_data(REG_PA_CONFIG, 0x48)); // Use isso para TxPower ~ 7dBm
  }

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_set_over_current_protection(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) 
  {
    ocpTrim = (mA - 45) / 5;
  } 
  else if (mA <=240) 
  {
    ocpTrim = (mA + 30) / 10;
  }

  gAssert(LORA.Drv.write_data(REG_OCP, 0x20 | (0x1F & ocpTrim)));

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_is_transmitting(void)
{
  uint8_t aux = 0;

  gAssert(LORA.Drv.read_data(REG_OP_MODE, &aux));
  if ((aux & MODE_TX) == MODE_TX) 
  {
    return true;
  }

  gAssert(LORA.Drv.read_data(REG_IRQ_FLAGS, &aux));
  if ( aux & IRQ_TX_DONE_MASK) 
  {
    // clear IRQ's
    gAssert(LORA.Drv.write_data(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK));
  }

  return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_enter_implicit_header_mode(void)
{
  LORA.Drv.header = header_is_implicit;

  uint8_t aux = 0;
  gAssert(LORA.Drv.read_data(REG_MODEM_CONFIG_1, &aux));
  gAssert(LORA.Drv.write_data(REG_MODEM_CONFIG_1, aux | 0x01));

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_enter_explicit_header_mode(void)
{
  LORA.Drv.header = header_is_explicit;

  uint8_t aux = 0;
  gAssert(LORA.Drv.read_data(REG_MODEM_CONFIG_1, &aux));
  gAssert(LORA.Drv.write_data(REG_MODEM_CONFIG_1, aux & 0xfe));

  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool lora_get_rssi(int *rssi)
{
  uint8_t rssi_reg_val = 0, rssi_offset = 0;
  gAssert(LORA.Drv.read_data(REG_RSSI_VALUE, &rssi_reg_val));

  rssi_offset = LORA.Drv.frequencia < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT;

  *rssi = rssi_reg_val - rssi_offset;
  
  return true;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static float lora_get_packet_snr(void)
{
  uint8_t aux = 0;
  LORA.Drv.read_data(REG_PKT_SNR_VALUE, &aux);

  return (aux * 0.25);
}

#pragma GCC diagnostic pop
