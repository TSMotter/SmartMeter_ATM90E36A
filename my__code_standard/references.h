/*
***************************************************************************************************
LEDS:
	- Verde		  PD12	GPIOD, GPIO_PIN_12
	- Laranja	  PD13	GPIOD, GPIO_PIN_13
	- Vermelho	PD14	GPIOD, GPIO_PIN_14
	- Azul		  PD15	GPIOD, GPIO_PIN_15
BOTAO:
	- Botao		  PA0		GPIOA, GPIO_PIN_0
Pinos ATM:
	- SPI1
		. MISO	  PA6		GPIOA, GPIO_PIN_6
		. MOSI	  PA7		GPIOA, GPIO_PIN_7
		. CLK	    PA5		GPIOA, GPIO_PIN_5
		. CS	    PC2		GPIOC, GPIO_PIN_2
	- IRQ0		  PC4		GPIOC, GPIO_PIN_4
	- IRQ1		  PC5		GPIOC, GPIO_PIN_5
	- WarnOut	  PB0		GPIOB, GPIO_PIN_0
	- Reset		  PA4		GPIOA, GPIO_PIN_4
	
Pinos I2C:
	- I2C2
		. SDA	    PB11	GPIOB, GPIO_PIN_11
		. SCL	    PB10	GPIOB, GPIO_PIN_10
		
Pinos UART:
	- UART3
		. TX	    PD8		GPIOD, GPIO_PIN_8
		. RX	    PD9		GPIOD, GPIO_PIN_9

***************************************************************************************************

HAL_GPIO_WritePin( ,GPIO_PIN_SET)
HAL_GPIO_WritePin( ,GPIO_PIN_RESET)
HAL_GPIO_TogglePin()

@ref: Funcoes de leitura e escrita em registradores e bits @file stm32f4xx.h

***************************************************************************************************

Protocolo UART:

	$ID,SUBID,DATALEN,DATA,CRC*
	|| ||    ||      ||   ||  |_ end of frame [0x2a]
	|| ||    ||      ||   ||_ crc 16-CCITT 0xFFFF no formato %04x 
	|| ||    ||      ||   |_ separador [0x2c]
	|| ||    ||      ||_ dado em ascii
	|| ||    ||      |_ separador [0x2c]
	|| ||    ||_ quantidade de dados no campo dado no formato %02x
	|| ||    |_ separador [0x2c]
	|| ||_ sub id da msg no formato %02x
	|| |_ separador [0x2c]
	||_ id da msg no formato %02x
	|_ start of frame [0x24]

- Observacoes:
	. Datalen dira respeito apenas ao campo de dados
	. A CRC devera ser calculada considerando todos os caracteres entre o SOF e EOF 
		(inclusive os separadores)
	. Devera ser aplicado escape code nos caracteres:
		0x2c --> 0x10 0x4c
		0x2a --> 0x10 0x4a
		0x24 --> 0x10 0x44
		0x00 --> 0x10 0x20
		0x0a --> 0x10 0x2a
		0x0d --> 0x10 0x2d
		0x10 --> 0x10 0x20 (Que e o proprio escape)

- Exemplo:
	. <$81,01,0001,058934A8B4,fedf*> (valores tudo errado)

***************************************************************************************************
ATM90E36A:

Interface:
	SPI 
	16bits
	Clock trailing edge
	Clock iddle == high
	CS active == low
	MSB first

	0x0009 == write to reg. 0x09
	0x8009 == read from reg. 0x09

Configuracoes:

	- AtmState_ConfigFuncEn
  	1.1) FuncEn0
  		. Default

  	1.2) FuncEn1
  		. Default

	- AtmState_ConfigZx
    2.1) ZxConfig
    	. Default		

	- AtmState_ConfigThresholds
	  3.1) Voltage sag Th
			. Ref: Pg 34 of App. Note
			. Target: Vth = 80% of 127Vrms --> Vth aprox = 100Vrms
			. RegValue = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) --> ?? Quem e Ugain ??
			. Default value of Ugain = 0xce40
			. TxValue = 0x1124
			
    3.2) Voltage phase loss Th
			. Ref: Pg 34 of App. Note
			. Target: Vth = 10% of 127Vrms --> Vth aprox = 13Vrms
			. RegValue = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) 
			. 
			. TxValue = 0x023A

    3.3) In calculated Th (Ia+Ib+Ic)
			. Absolute value
			. 16 bits, unit = 1mA
			. TxValue = 36A --> 0x8CA0

    3.4) In sampled Th 
			. Absolute value
			. 16 bits, unit = 1mA
			. TxValue = 36A --> 0x8CA0

    3.5) Voltage THD+N Th
			. Absolute value
			. 16 bits, unit = 0.01%
			. TxValue = 10% --> 0x03E8

    3.6) Current THD+N Th
			. Absolute value
			. 16 bits, unit = 0.01%
			. TxValue = 10% --> 0x03E8

	- AtmState_ConfigCS0
		4.1) Set ConfigStart to initial value
			.

		4.2) PLConst High
			. Ref: Pg 20 of App. Note
			. Default

		4.3) PLConst Low
			. Default

		4.4) MMode0
			. Freq = 60Hz
			. Filtro HPF = On
			. Integrador di/dt = On
			. Registradores de energia lsb == 0.01CF
			. 3P4W
			. Energia total (soma de todas as fases) e Porencia total == Absolute Sum

		4.5) MMode1
			. Digital PGA = 1x
			. All PGA = 1x 

		4.6) P Start Th 
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032W
			. TxValue = 0.96W = 0x0bb8

		4.7) Q Start Th
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032Va
			. TxValue = 0.96Va = 0x0bb8

		4.8) S Start Th
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032Var
			. TxValue = 0.96Var = 0x0bb8

		4.9) P Start Phase Th
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032W
			. TxValue = 0.064W = 0x00c8

		4.10) Q Start Phase  Th
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032Va
			. TxValue = 0.064Va = 0x00c8

		4.11) S Start Phase  Th
			. Ref: Pg 33 of App. Note
			. Absolute value
			. 16 bits, unit - 0.00032Var
			. TxValue = 0.064Var = 0x00c8
		
		4.12) Configura CS0 register

		4.13) Set ConfigStart to final value

Medidas:

	1 	Phase Voltage RMS¹
	2 	Phase Current RMS¹
	3 	Active Power¹
	4 	Reactive Power¹
	5 	Apparent Power¹
	6 	Active Fundamental Power¹
	7 	Active Harmonic Power¹
	8 	Voltage THD+N
	9 	Current THD+N
	10	Frequency
	11	2th ~ 32th Voltage Harmonic
	12	2th ~ 32th Current Harmonic
	
	13	Forward Active Energy²
	14	Reverse Active Energy²
	15	Forward Reactive Energy²
	16	Reverse Reactive Energy²	
	17	Apparent Energy²
	18		. Total (arithmetic sum)
	19		. Total (vector sum)
	20	Apparent Power¹
	21		. Total (vector sum)
	22	Forward Active Fundamental Energy²
	23	Reverse Active Fundamental Energy²
	24	Forward Active Harmonic Energy²
	25	Reverse Active Harmonic Energy²


	¹ : Items that have normal reading (1 16bit register) but also extended reading (+LSB register)
	² : Registers unit is CF based

*/
