/***************************************************************************************************
* @file 	gm_lib.c
* @brief  
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <string.h>
#include "gm_lib.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static bool character_is_escapable(uint8_t character);

/***************************************************************************************************
* @brief 
***************************************************************************************************/
//Converte um byte num ascii (eg: 0x35 = 5)
uint8_t GM_Byte_Coder(uint8_t stripped_nib)
{
	//se for numero, subtrai 0x30
	if ((stripped_nib >= 0x30 && stripped_nib<=0x39))
	{
		return (stripped_nib & 0x0F);
	}
	//se for letra, retorna ela mesma
	else if (((stripped_nib>=0x41)&(stripped_nib<=0x5A)) || ((stripped_nib>=0x61)&(stripped_nib<=0x7A)))
	{
		return (stripped_nib);
	}
	//se nao for nem letra nem numero, retorna null
	else
	{
		return (0x00);
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void GM_U8_TO_2ASCIIS(uint8_t hex, char *ascii)
{
	uint8_t temp = 0, i = 2;

	while(i != 0)						// 	2		 1
	{
		temp = hex>>(4*(i-1));	//0x0f  0xf1 
		temp = temp&0x0F;				//0x0f  0x01 

		if(temp <= 9)
		{
			temp += 0x30;
		}
		else
		{
			temp += 0x57;		// 0x66	 0x31	
		}

		ascii[2-i] = temp;
		i--;
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void GM_U16_TO_4ASCIIS(uint16_t hex, char *ascii)
{
	uint8_t temp = 0, i = 4;

	while(i != 0)				// 4	  3		2		 1
	{
		temp = hex>>(4*(i-1));	//0x0f  0xf1   0x1d		0xd2
		temp = temp&0x0F;		//0x0f  0x01   0x0d		0x02

		if(temp <= 9)
		{
			temp += 0x30;
		}
		else
		{
			temp += 0x57;		// 0x66	 0x31	0x64	0x02
		}

		ascii[4-i] = temp;
		i--;
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void GM_U32_TO_8ASCIIS(uint32_t hex, char *ascii)
{
	uint8_t temp = 0, i = 8;

	while(i != 0)				// 8           		4
	{
		temp = hex>>(4*(i-1));	//0x0000000f	0x000f1d2a
		temp = temp&0x0F;		//0x0000000f	0x0000000a

		if(temp <= 9)
		{
			temp += 0x30;
		}
		else
		{
			temp += 0x57;		// 0x66	 		0x61
		}

		ascii[8-i] = temp;
		i--;
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
void GM_U64_TO_16ASCIIS(uint64_t hex, char *ascii)
{
	uint8_t temp = 0, i = 16;

	while(i != 0)
	{
		temp = hex>>(4*(i-1));
		temp = temp&0x0F;

		if(temp <= 9)
		{
			temp += 0x30;
		}
		else
		{
			temp += 0x57;
		}

		ascii[16-i] = temp;
		i--;
	}
}

/***************************************************************************************************
* @brief 
***************************************************************************************************/
uint16_t GM_CRC_CCITT(uint8_t* buffer, uint16_t size)
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
static bool character_is_escapable(uint8_t character)
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
void aplica_escape_code(char *in, uint16_t len_in, char *out)
{
	uint8_t byPosicaoEscape = 0;	

	for (int by = 0; by < len_in; by++)
	{
	  byPosicaoEscape = strlen(out);
	  if (character_is_escapable(in[by]))
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
void retira_escape_code(char *in, uint16_t len_in)
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



/*

@brief Converte um byte em um ASCII maiusculo (eg: 9 = 0x39, 5 = 0x35, A = 0x41)
unsigned char GM_ASCII_Coder (uint8_t stripped_nib) 
{

	// se esta entre 0x00 e 0x09, soma 0x30
	if ((stripped_nib<=0x09))
	{
		return (stripped_nib + 0x30); 
	}
	else if ((stripped_nib>=0x0A)&(stripped_nib<=0x0F))
	{
		return (stripped_nib + 0x41 - 0x0A); // se entre 0x0A e 0x0F, soma (0x41-0x0A)
	}
	else
	{
		return (0x00); // retorna NULL caso contrario
	}
}

void GM_Byte_to_ASCII (uint8_t bt, uint8_t* Pstr){
	
	unsigned char nibble0, nibble1;

	// byte = 0 x nibble1 nibble0
	//Operacao bit a bit (mascara) 
	nibble0 = bt >> 4; // colocando & 0xF0 ficava tipo 0b00010000 (� necess�rio rolar pra direita)
	nibble1 = bt & 0x0F;

	Pstr[0] = GM_ASCII_Coder (nibble0);
	Pstr[1] = GM_ASCII_Coder (nibble1);
	Pstr[2] = '\0';
}

uint8_t GM_ByteVec_to_ASCIIStr (uint8_t** P_Pvec, uint8_t* P_N, uint8_t** P_Pstr)
{
	//pra acessar elemento do vetor usar **, para incrementar a contagem usar *
	if(*P_N > 0)
	{
		GM_Byte_to_ASCII(**P_Pvec, *P_Pstr);
		(*P_Pvec)++;
		(*P_Pstr)=(*P_Pstr)+2;
		(*P_N)--;
		return 1;
	}
	else 
	{
		return 0;
	}
}


int GM_ASSERT(void (*function))
{
	int _err = function();
	if (_err)
	{
		return _err;
	}

	return 0;
}

void GM_SET_PWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
	HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	HAL_TIM_Base_Stop_IT(&timer);

	TIM_OC_InitTypeDef sConfigOC;

	timer.Init.Prescaler = 16000000 / period;
	timer.Init.Period = period; // set the period duration

	HAL_TIM_PWM_Init(&timer); // reinititialise with new period value

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

	HAL_TIM_Base_Start_IT(&timer);
	HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}


void GM_RTC_PRINT(uint8_t vect_in[7], uint8_t* vect_out)
{
	
	
	
	//get_rtc_time[0] = 0x00; // seconds
	//get_rtc_time[1] = 0x00; // minutes
	//get_rtc_time[2] = 0x00; // hours
	//get_rtc_time[3] = 0x00; // days
	//get_rtc_time[4] = 0x00; // weekdays
	//get_rtc_time[5] = 0x00; // century_month
	//get_rtc_time[6] = 0x00; // year
	//
	//Quero imprimir: DIA-MES-ANO-DIA SEMANA-HORA-MINUTOS-SEGUNDOS
	
	
		
	//dia do mes
	vect_out[0] = GM_ASCII_Coder((vect_in[3]>>4 & 0x03));
	vect_out[1] = GM_ASCII_Coder(vect_in[3] & 0x0F);
	vect_out[2] = 0x2D;
	
	//mes
  vect_out[3] = GM_ASCII_Coder((vect_in[5]>>4) & 0x01);
	vect_out[4] = GM_ASCII_Coder(vect_in[5] & 0x0F);
	vect_out[5] = 0x2D;
	
	//ano
	vect_out[6] = GM_ASCII_Coder(vect_in[6]>>4);
	vect_out[7] = GM_ASCII_Coder(vect_in[6] & 0x0F);
	vect_out[8] = 0x2D;
	
	//dia da semana
	vect_out[9] = GM_ASCII_Coder(vect_in[4] & 0x07);
	vect_out[10] = 0x2A;
	vect_out[11] = 0x2D;
	
	//hora
	vect_out[12] = GM_ASCII_Coder((vect_in[2]>>4) & 0x03);
  vect_out[13] = GM_ASCII_Coder(vect_in[2] & 0xF);
	vect_out[14] = 0x3A;
	
	//minutos
	vect_out[15] = GM_ASCII_Coder(vect_in[1]>>4);
	vect_out[16] = GM_ASCII_Coder(vect_in[1] & 0x0F);
	vect_out[17] = 0x3A;
	
	//segundos
	vect_out[18] = GM_ASCII_Coder(vect_in[0]>>4);
	vect_out[19] = GM_ASCII_Coder(vect_in[0] & 0x0F);
	vect_out[20] = 0x0D;
	
}



*/
