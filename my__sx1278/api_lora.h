/***************************************************************************************************
* @file   api_lora.h
* @brief  Adaptação do arquivo de referencia de Arduino
* @author Giuliano Motter
* @date   01/2021
* @note   Copyright (c) Sandeep Mistry.
***************************************************************************************************/

#ifndef __API_LORA_H__
#define __API_LORA_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rtos_utility.h"
#include "api_sx1278.h"

/***************************************************************************************************
* Defines
***************************************************************************************************/
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d // Defaults to 0x72
#define REG_MODEM_CONFIG_2       0x1e // Defaults to 0x70
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26 // Defaults to 0x00
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80
#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525000000
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

/* 
  - This is set to 128 to allow half of the RAM memory to be used for
    Tx and half for Rx operations. (Full RAM is 256 bytes)
  
  - The FIFO is cleared when the device is put in sleep mode
  
  See datasheet section 4.1.2.3

*/
#define MAX_PKT_LENGTH      128
#define BASE_RX_POINTER     0x00
#define BASE_TX_POINTER     0x80

//#define LORA_TEST_TX
//#define LORA_TEST_RX

#define LORA_RX_BUFFER_SIZE   50
/***************************************************************************************************
* Types
***************************************************************************************************/

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
bool LORA_api_init  (void);
bool LORA_config    (uint32_t frequency);

void LORA_api_periodic_checks(void);

bool LORA_Transmit  (uint8_t *buffer, size_t len, uint32_t timeout);
bool LORA_Receive   (uint8_t impl_hdr_size_to_read, uint8_t *packet, uint8_t *len_was_read, int *rssi);
bool LORA_Process   (uint8_t *buffer, size_t len);
#endif
