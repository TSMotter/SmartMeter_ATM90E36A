/***************************************************************************************************
* @file   protocolo.c
* @brief  Protocol defines and functions
* @author Giuliano Motter
* @date   01/2021
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include "protocolo.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static uint16_t   prot_crc                  (uint8_t *buffer, uint16_t size);
static void       prot_aplica_escape_code   (char *in, uint16_t len_in, char *out);
static void       prot_retira_escape_code   (char *in, uint16_t len_in);
static bool       prot_char_is_escapable    (uint8_t character);
/***************************************************************************************************
* Externals
***************************************************************************************************/

/***************************************************************************************************
* Vars
***************************************************************************************************/

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static uint16_t prot_crc(uint8_t* buffer, uint16_t size)
{
  unsigned short tmp;
  uint16_t crc = 0xffff;

  for (uint16_t i=0; i < size ; i++)
  {
    tmp = (crc >> 8) ^ buffer[i];
    crc = (crc << 8) ^ CRC_CCITT_TABLE[tmp];
  }

  return crc;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void prot_aplica_escape_code(char *in, uint16_t len_in, char *out)
{
	uint8_t byPosicaoEscape = 0;	

	for (int by = 0; by < len_in; by++)
	{
	  byPosicaoEscape = strlen(out);
	  if (prot_char_is_escapable(in[by]))
		{
	    out[byPosicaoEscape] = 0x10;
	    out[byPosicaoEscape+1] = in[by] + 0x20;
	  }
	  else 
		{
	    out[byPosicaoEscape] = in[by];
	  }
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static void prot_retira_escape_code(char *in, uint16_t len_in)
{
	//For para tirar escape code do payload         
  for(uint8_t by = 0, by2 = 0; in[by] != GM_EOF; by++, by2++)
	{
		//Se encontrou Escape Code
    if(in[by] == 0x10)
    {
      in[by2] = in[by+1] - 0x20;
      by++;
    }
    else
		{
      in[by2] = in[by];
    }
		
		// Breaks de loop automatically after 40 cycles (for safety)
		if(by2 >= 40)
		{
			break;
		}
  } 
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
static bool prot_char_is_escapable(uint8_t character)
{
	if (character == 0x00 || character == 0x10 || 
			character == 0x0D || character == 0x0A ||
			character == 0x24 || character == 0x2a || character == 0x2c)
	{
		return true;
	}
	return false;
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
rc_prot_en PROT_Parser(receive_command_st *RxStruct, char incoming_buffer[GM_Max_Command_Len])
{

  uint16_t RemoteCRC = 0, LocalCRC = 0, sizeTotalCmd = 0;

  uint8_t localCpy[GM_Max_Command_Len] = {0};
  memcpy(localCpy, incoming_buffer, GM_Max_Command_Len);
  memset(RxStruct, 0, sizeof(receive_command_st));

  char* str_id   		  = &incoming_buffer[1];  //++ para comecar a pegar apos o SOF
  char* str_sub_id 	  = PARSER(str_sub_id,   str_id,       GM_Separador);
  char* str_comando   = PARSER(str_comando,  str_sub_id,    GM_Separador);
  char* str_data_len 	= PARSER(str_data_len, str_comando,   GM_Separador);
  char* str_data    	= PARSER(str_data,     str_data_len, GM_Separador); 
  char* str_crc    	  = PARSER(str_crc,      str_data,     GM_Separador); 

  // - Valida CRC
  RemoteCRC = strtoul(str_crc, NULL, 16);

  sizeTotalCmd = str_crc - str_id;
  LocalCRC = prot_crc(&localCpy[1], sizeTotalCmd);

  if(RemoteCRC != LocalCRC)
  {
    return Prot_Erro_CRC;
  }

  // - Preenche struct
  RxStruct->ID = strtoul(str_id, NULL, 16);
  RxStruct->SubID = strtoul(str_sub_id, NULL, 16);
  RxStruct->Comando = strtoul(str_comando, NULL, 16);
  RxStruct->DataLen = strtoul(str_data_len, NULL, 16);
  
  if(RxStruct->DataLen)
  {
    uint8_t LocalData[GM_Max_Command_Len] = {0};
    uint16_t max_loops = RxStruct->DataLen > GM_Max_Command_Len ? GM_Max_Command_Len : RxStruct->DataLen;
    for(uint16_t k = 0; k < max_loops; k++)
    {
      char aa[3] = {str_data[2*k], str_data[(2*k)+1], '\0'};
      LocalData[k] = strtoul(aa, NULL, 16);    
    } 
    memcpy(RxStruct->Data, LocalData, max_loops);
  }
  
  return Prot_OK;  
}

#pragma GCC diagnostic pop
