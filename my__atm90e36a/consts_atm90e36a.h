/***************************************************************************************************
* @file   consts_atm90e36a.h
* @brief  Presents the hardware constants for the ATM90E36A IC
* @author Giuliano Motter
* @date   09/2020
***************************************************************************************************/

#ifndef __CONSTS_ATM90E36A_H__
#define __CONSTS_ATM90E36A_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdbool.h>

/***************************************************************************************************
* Types
***************************************************************************************************/
// Registrador ATM_REG_ADD_FuncEn0
typedef struct
{
  bool bCS2ErrEn;           // Determina se o bit CS2Err deve ou nao gerar interrupcao em IRQ0
  bool bURevWnEn;           // Determina se o bit URevWn deve ou nao gerar interrupcao em IRQ0
  bool bIRevWnEn;           // Determina se o bit IRevWn deve ou nao gerar interrupcao em IRQ0
  bool bSagWnEn;            // Determina se o bit SagWarn deve ou nao gerar interrupcao em IRQ0
  bool bPhaseLoseWnEn;      // Determina se o bit PhaseLose deve ou nao gerar interrupcao em IRQ0
} type_stATM_REG_FuncEn0;


// Registrador ATM_REG_ADD_FuncEn1
typedef struct
{
  bool bINOv1En;    // Determina se o bit INOv1 deve ou nao gerar interrupcao em IRQ1
  bool bINOv0En;    // Determina se o bit INOv1 deve ou nao gerar interrupcao em IRQ1
  bool bTHDUOvEn;   // Determina se o bit THDUOv deve ou nao gerar interrupcao em IRQ1
  bool bTHDIOvEn;   // Determina se o bit THDIOv deve ou nao gerar interrupcao em IRQ1
  bool bDFTDone;    // Determina se o bit DFTDone deve ou nao gerar interrupcao em IRQ1

  /* Determina se as mudancas de direcao de energias ativas/reativas
   * total ou de alguma fase individual devem ou nao gerar interrupcao
   * em IRQ1
   * */
  bool bRevQchgTEn;
  bool bRevQchgAEn;
  bool bRevQchgBEn;
  bool bRevQchgCEn;

  bool bRevPchgTEn;
  bool bRevPchgAEn;
  bool bRevPchgBEn;
  bool bRevPchgCEn;
} type_stATM_REG_FuncEn1;


typedef enum
{
  Ua = 0, Ub, Uc, UFixed0, Ia, Ib, Ic, IFixed0
} ZXSrc_en;

typedef enum
{
  RisingEdge = 0, FallingEdge, AnyEdge, None
} ZXCon_en;


#define ALL_PHASE_SUM 0
typedef enum
{
  NEUTRAL = 0,
  PHASE_A,
  PHASE_B,
  PHASE_C,
}phase_en;


#define CS_REG_Value_PowerUp      0x6886
#define CS_REG_Value_Calibration  0x5678
#define CS_REG_Value_Operation    0x8765

/***************************************************************************************************
* Macros
***************************************************************************************************/

/***************************************************************************************************
* Defines
***************************************************************************************************/
#define		ATM_REG_SoftReset_Add	      0x00 // Status and Special Register

#define		ATM_REG_SysStatus0_Add		           0x01
#define		ATM_REG_SysStatus0_Pos_CS0Err        (14U)
#define		ATM_REG_SysStatus0_Msk_CS0Err        (0x1UL << ATM_REG_SysStatus0_Pos_CS0Err)
#define		ATM_REG_SysStatus0_Pos_CS1Err        (12U)
#define		ATM_REG_SysStatus0_Msk_CS1Err        (0x1UL << ATM_REG_SysStatus0_Pos_CS1Err)
#define		ATM_REG_SysStatus0_Pos_CS2Err        (10U)
#define		ATM_REG_SysStatus0_Msk_CS2Err        (0x1UL <<ATM_REG_SysStatus0_Pos_CS2Err)
#define		ATM_REG_SysStatus0_Pos_CS3Err        (8U)
#define		ATM_REG_SysStatus0_Msk_CS3Err        (0x1UL <<ATM_REG_SysStatus0_Pos_CS3Err)
#define		ATM_REG_SysStatus0_Pos_URevWn        (7U)
#define		ATM_REG_SysStatus0_Msk_URevWn        (0x1UL << ATM_REG_SysStatus0_Pos_URevWn)
#define		ATM_REG_SysStatus0_Pos_IRevWn        (6U)
#define		ATM_REG_SysStatus0_Msk_IRevWn        (0x1UL << ATM_REG_SysStatus0_Pos_IRevWn)
#define		ATM_REG_SysStatus0_Pos_SagWarn       (3U)
#define		ATM_REG_SysStatus0_Msk_SagWarn       (0x1UL << ATM_REG_SysStatus0_Pos_SagWarn)
#define		ATM_REG_SysStatus0_Pos_PhaseLoseWn   (2U)
#define		ATM_REG_SysStatus0_Msk_PhaseLoseWn   (0x1UL << ATM_REG_SysStatus0_Pos_PhaseLoseWn) 

#define		ATM_REG_SysStatus1_Add   	           0x02
#define		ATM_REG_SysStatus1_Pos_INOv1         (15U)  
#define		ATM_REG_SysStatus1_Msk_INOv1         (0x1UL << ATM_REG_SysStatus1_Pos_INOv1)
#define		ATM_REG_SysStatus1_Pos_INOv0         (14U)  
#define		ATM_REG_SysStatus1_Msk_INOv0         (0x1UL << ATM_REG_SysStatus1_Pos_INOv0)
#define		ATM_REG_SysStatus1_Pos_THDUOv        (11U)  
#define		ATM_REG_SysStatus1_Msk_THDUOv        (0x1UL << ATM_REG_SysStatus1_Pos_THDUOv)
#define		ATM_REG_SysStatus1_Pos_THDIOv        (10U)  
#define		ATM_REG_SysStatus1_Msk_THDIOv        (0x1UL << ATM_REG_SysStatus1_Pos_THDIOv)
#define		ATM_REG_SysStatus1_Pos_DFTDone       (9U)  
#define		ATM_REG_SysStatus1_Msk_DFTDone       (0x1UL << ATM_REG_SysStatus1_Pos_DFTDone)
#define		ATM_REG_SysStatus1_Pos_RevQchgT      (7U)  
#define		ATM_REG_SysStatus1_Msk_RevQchgT      (0x1UL << ATM_REG_SysStatus1_Pos_RevQchgT)
#define		ATM_REG_SysStatus1_Pos_RevQchgA      (6U)  
#define		ATM_REG_SysStatus1_Msk_RevQchgA      (0x1UL << ATM_REG_SysStatus1_Pos_RevQchgA)
#define		ATM_REG_SysStatus1_Pos_RevQchgB      (5U)  
#define		ATM_REG_SysStatus1_Msk_RevQchgB      (0x1UL << ATM_REG_SysStatus1_Pos_RevQchgB)
#define		ATM_REG_SysStatus1_Pos_RevQchgC      (4U)  
#define		ATM_REG_SysStatus1_Msk_RevQchgC      (0x1UL << ATM_REG_SysStatus1_Pos_RevQchgC)
#define		ATM_REG_SysStatus1_Pos_RevPchgT      (3U)  
#define		ATM_REG_SysStatus1_Msk_RevPchgT      (0x1UL << ATM_REG_SysStatus1_Pos_RevPchgT)
#define		ATM_REG_SysStatus1_Pos_RevPchgA      (2U)  
#define		ATM_REG_SysStatus1_Msk_RevPchgA      (0x1UL << ATM_REG_SysStatus1_Pos_RevPchgA)
#define		ATM_REG_SysStatus1_Pos_RevPchgB      (1U)  
#define		ATM_REG_SysStatus1_Msk_RevPchgB      (0x1UL << ATM_REG_SysStatus1_Pos_RevPchgB)
#define		ATM_REG_SysStatus1_Pos_RevPchgC      (0)  
#define		ATM_REG_SysStatus1_Msk_RevPchgC      (0x1UL << ATM_REG_SysStatus1_Pos_RevPchgC)
 
#define		ATM_REG_FuncEn0_Add                  0x03
#define   ATM_REG_FuncEn0_DefaultValue         0x0000
#define   ATM_REG_FuncEn0_Pos_CS2ErrEn         (10U)
#define   ATM_REG_FuncEn0_Msk_CS2ErrEn         (0x1UL << ATM_REG_FuncEn0_Pos_CS2ErrEn)
#define   ATM_REG_FuncEn0_Pos_URevWnEn         (7U)
#define   ATM_REG_FuncEn0_Msk_URevWnEn         (0x1UL << ATM_REG_FuncEn0_Pos_URevWnEn)
#define   ATM_REG_FuncEn0_Pos_IRevWnEn         (6U)
#define   ATM_REG_FuncEn0_Msk_IRevWnEn         (0x1UL << ATM_REG_FuncEn0_Pos_IRevWnEn)
#define   ATM_REG_FuncEn0_Pos_SagWnEn          (3U)
#define   ATM_REG_FuncEn0_Msk_SagWnEn          (0x1UL << ATM_REG_FuncEn0_Pos_SagWnEn)
#define   ATM_REG_FuncEn0_Pos_PhaseLoseWnEn    (2U)
#define   ATM_REG_FuncEn0_Msk_PhaseLoseWnEn    (0x1UL << ATM_REG_FuncEn0_Pos_PhaseLoseWnEn)

#define		ATM_REG_FuncEn1_Add                  0x04
#define		ATM_REG_FuncEn1_Pos_INOv1En          (15U)
#define		ATM_REG_FuncEn1_Msk_INOv1En          (0x1UL << ATM_REG_FuncEn1_Pos_INOv1En)
#define		ATM_REG_FuncEn1_Pos_INOv0En          (14U)
#define		ATM_REG_FuncEn1_Msk_INOv0En          (0x1UL << ATM_REG_FuncEn1_Pos_INOv0En)
#define		ATM_REG_FuncEn1_Pos_THDUOvEn         (11U)
#define		ATM_REG_FuncEn1_Msk_THDUOvEn         (0x1UL << ATM_REG_FuncEn1_Pos_THDUOvEn)
#define		ATM_REG_FuncEn1_Pos_THDIOvEn         (10U)
#define		ATM_REG_FuncEn1_Msk_THDIOvEn         (0x1UL << ATM_REG_FuncEn1_Pos_THDIOvEn)
#define		ATM_REG_FuncEn1_Pos_DFTDoneEn        (9U)
#define		ATM_REG_FuncEn1_Msk_DFTDoneEn        (0x1UL << ATM_REG_FuncEn1_Pos_DFTDoneEn)
#define		ATM_REG_FuncEn1_Pos_RevQchgTEn       (7U)
#define		ATM_REG_FuncEn1_Msk_RevQchgTEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevQchgTEn)
#define		ATM_REG_FuncEn1_Pos_RevQchgAEn       (6U)
#define		ATM_REG_FuncEn1_Msk_RevQchgAEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevQchgAEn)
#define		ATM_REG_FuncEn1_Pos_RevQchgBEn       (5U)
#define		ATM_REG_FuncEn1_Msk_RevQchgBEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevQchgBEn)
#define		ATM_REG_FuncEn1_Pos_RevQchgCEn       (4U)
#define		ATM_REG_FuncEn1_Msk_RevQchgCEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevQchgCEn)
#define		ATM_REG_FuncEn1_Pos_RevPchgTEn       (3U)
#define		ATM_REG_FuncEn1_Msk_RevPchgTEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevPchgTEn)
#define		ATM_REG_FuncEn1_Pos_RevPchgAEn       (2U)
#define		ATM_REG_FuncEn1_Msk_RevPchgAEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevPchgAEn)
#define		ATM_REG_FuncEn1_Pos_RevPchgBEn       (1U)
#define		ATM_REG_FuncEn1_Msk_RevPchgBEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevPchgBEn)
#define		ATM_REG_FuncEn1_Pos_RevPchgCEn       (0)
#define		ATM_REG_FuncEn1_Msk_RevPchgCEn       (0x1UL << ATM_REG_FuncEn1_Pos_RevPchgCEn)

#define		ATM_REG_ZXConfig_Add                0x07
#define		ATM_REG_ZXConfig_Pos_ZX2Src         (13U)
#define		ATM_REG_ZXConfig_Pos_ZX1Src         (10U)
#define		ATM_REG_ZXConfig_Pos_ZX0Src         (7U)
#define   ATM_REG_AxConfig_Val_Ua             (0x07 & 0x00)
#define   ATM_REG_AxConfig_Val_Ub             (0x07 & 0x01)
#define   ATM_REG_AxConfig_Val_Uc             (0x07 & 0x02)
#define   ATM_REG_AxConfig_Val_UFix           (0x07 & 0x03)
#define   ATM_REG_AxConfig_Val_Ia             (0x07 & 0x04)
#define   ATM_REG_AxConfig_Val_Ib             (0x07 & 0x05)
#define   ATM_REG_AxConfig_Val_Ic             (0x07 & 0x06)
#define   ATM_REG_AxConfig_Val_IFix           (0x07 & 0x07)
#define		ATM_REG_ZXConfig_Pos_ZX2Con         (5U)
#define		ATM_REG_ZXConfig_Pos_ZX1Con         (3U)
#define		ATM_REG_ZXConfig_Pos_ZX0Con         (1U)
#define   ATM_REG_AxConfig_Val_RiseEdge       (0x03 & 0x00)
#define   ATM_REG_AxConfig_Val_FallEdge       (0x03 & 0x01)
#define   ATM_REG_AxConfig_Val_BothEdge       (0x03 & 0x02)
#define   ATM_REG_AxConfig_Val_NoneEdge       (0x03 & 0x03)
#define		ATM_REG_ZXConfig_Pos_ZXdis          (0)
#define		ATM_REG_ZXConfig_Msk_ZXdis          (0x1UL << ATM_REG_ZXConfig_Pos_ZXdis)





#define		ATM_REG_SagTh_Add           0x08
#define		ATM_REG_PhaseLossTh_Add     0x09
#define		ATM_REG_INWarnTh0_Add       0x0A
#define		ATM_REG_INWarnTh1_Add       0x0B
#define		ATM_REG_THDNUTh_Add         0x0C
#define		ATM_REG_THDNITh_Add         0x0D
#define		ATM_REG_DMACtrl_Add         0x0E
#define		ATM_REG_LastSPIData_Add     0x0F

#define		ATM_REG_DetectCtrl_Add		  0x10 // Low Power Mode Register
#define		ATM_REG_DetectTh1_Add 		  0x11
#define		ATM_REG_DetectTh2_Add 		  0x12
#define		ATM_REG_DetectTh3_Add 		  0x13
#define		ATM_REG_PMOffsetA_Add 		  0x14
#define		ATM_REG_PMOffsetB_Add 		  0x15
#define		ATM_REG_PMOffsetC_Add 		  0x16
#define		ATM_REG_PMPGA_Add 			    0x17
#define		ATM_REG_PMIrmsA_Add 		    0x18
#define		ATM_REG_PMIrmsB_Add 		    0x19
#define		ATM_REG_PMIrmsC_Add 		    0x1A
#define		ATM_REG_PMConfig_Add		    0x1B
#define		ATM_REG_PMAvgSamples_Add	  0x1C
#define		ATM_REG_PMIrmsLSB_Add		    0x1D

#define 	ATM_REG_ConfigStart_Add 	  0x30 // Configuration Registers
#define 	ATM_REG_PLconstH_Add 	 	    0x31
#define 	ATM_REG_PLconstL_Add 	 	    0x32

#define 	ATM_REG_MMode0_Add  	 		    0x33
#define 	ATM_REG_MMode0_Pos_I1I3Swap   (13U)
#define 	ATM_REG_MMode0_Msk_I1I3Swap   (0x1UL << ATM_REG_MMode0_Pos_I1I3Swap)
#define 	ATM_REG_MMode0_Pos_Freq60Hz   (12U)
#define 	ATM_REG_MMode0_Msk_Freq60Hz   (0x1UL << ATM_REG_MMode0_Pos_Freq60Hz)
#define 	ATM_REG_MMode0_Pos_HPFOff     (11U)
#define 	ATM_REG_MMode0_Msk_HPFOff     (0x1UL <<ATM_REG_MMode0_Pos_HPFOff)
#define 	ATM_REG_MMode0_Pos_didtEn     (10U)
#define 	ATM_REG_MMode0_Msk_didtEn     (0x1UL << ATM_REG_MMode0_Pos_didtEn)
#define 	ATM_REG_MMode0_Pos_001LSB     (9U)
#define 	ATM_REG_MMode0_Msk_001LSB     (0x1UL << ATM_REG_MMode0_Pos_001LSB)
#define 	ATM_REG_MMode0_Pos_3P3W       (8U)
#define 	ATM_REG_MMode0_Msk_3P3W       (0x1UL << ATM_REG_MMode0_Pos_3P3W)
#define 	ATM_REG_MMode0_Pos_CF2varh    (7U)
#define 	ATM_REG_MMode0_Msk_CF2varh    (0x1UL << ATM_REG_MMode0_Pos_CF2varh)
#define 	ATM_REG_MMode0_Pos_CF2ESV     (6U)
#define 	ATM_REG_MMode0_Msk_CF2ESV     (0x1UL << ATM_REG_MMode0_Pos_CF2ESV)
#define 	ATM_REG_MMode0_Pos_ABSEnQ     (4U)
#define 	ATM_REG_MMode0_Msk_ABSEnQ     (0x1UL << ATM_REG_MMode0_Pos_ABSEnQ)
#define 	ATM_REG_MMode0_Pos_ABSEnP     (3U)
#define 	ATM_REG_MMode0_Msk_ABSEnP     (0x1UL << ATM_REG_MMode0_Pos_ABSEnP)
#define 	ATM_REG_MMode0_Pos_EnPA       (2U)
#define 	ATM_REG_MMode0_Msk_EnPA       (0x1UL << ATM_REG_MMode0_Pos_EnPA)
#define 	ATM_REG_MMode0_Pos_EnPB       (1U)
#define 	ATM_REG_MMode0_Msk_EnPB       (0x1UL << ATM_REG_MMode0_Pos_EnPB)
#define 	ATM_REG_MMode0_Pos_EnPC       (0U)
#define 	ATM_REG_MMode0_Msk_EnPC       (0x1UL << ATM_REG_MMode0_Pos_EnPC)


#define		ATM_REG_MMode1_Add                0x34
#define		ATM_REG_MMode1_Pos_DPGA_GAIN      (14U)
#define		ATM_REG_MMode1_Val_Gain1       	  (0x03 & 0x00)
#define		ATM_REG_MMode1_Val_Gain2       	  (0x03 & 0x01)
#define   ATM_REG_MMode1_Val_Gain4       	  (0x03 & 0x02)
#define   ATM_REG_MMode1_Val_Gain8       	  (0x03 & 0x03)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_V3    (12U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_V2    (10U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_V1    (8U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_I4    (6U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_I3    (4U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_I2    (2U)
#define		ATM_REG_MMode1_Pos_PGA_GAIN_I1    (0U)
#define		ATM_REG_MMode1_Val_1x       		  (0x03 & 0x00)
#define		ATM_REG_MMode1_Val_2x       		  (0x03 & 0x01)
#define   ATM_REG_MMode1_Val_4x       		  (0x03 & 0x02)
#define   ATM_REG_MMode1_Val_NA       		  (0x03 & 0x03)


#define 	ATM_REG_PStartTh_Add 		    0x35
#define 	ATM_REG_QStartTh_Add 		    0x36
#define 	ATM_REG_SStartTh_Add 		    0x37
#define 	ATM_REG_PPhaseTh_Add 		    0x38
#define 	ATM_REG_QPhaseTh_Add 		    0x39
#define 	ATM_REG_SPhaseTh_Add 		    0x3A
#define 	ATM_REG_CS0_Add 	 		      0x3B
#define		ATM_REG_CalStart_Add		    0x40 // Calibration Registers
#define		ATM_REG_PoffsetA      	0x41
#define		ATM_REG_QoffsetA      	0x42
#define		ATM_REG_POffsetB      	0x43
#define		ATM_REG_QOffsetB      	0x44
#define		ATM_REG_POffsetC      	0x45
#define		ATM_REG_QOffsetC      	0x46
#define		ATM_REG_GainA         	0x47
#define		ATM_REG_PhiA          	0x48
#define		ATM_REG_GainB         	0x49
#define		ATM_REG_PhiB          	0x4A
#define		ATM_REG_GainC         	0x4B
#define		ATM_REG_PhiC          	0x4C
#define		ATM_REG_CS1_Add           	0x4D
#define		ATM_REG_HarmStart_Add		    0x50 // Fundamental/ Harmonic Energy Calibration registers
#define		ATM_REG_POffsetAF     	0x51
#define		ATM_REG_POffsetBF     	0x52
#define		ATM_REG_POffsetCF     	0x53
#define		ATM_REG_PGainAF       	0x54
#define		ATM_REG_PGainBF       	0x55
#define		ATM_REG_PGainCF       	0x56
#define		ATM_REG_CS2_Add           	0x57
#define		ATM_REG_AdjStart_Add		    0x60 // Measurement Calibration
#define		ATM_REG_UgainA_Add         	0x61
#define		ATM_REG_IgainA_Add         	0x62
#define		ATM_REG_UoffsetA_Add       	0x63
#define		ATM_REG_IoffsetA_Add       	0x64
#define		ATM_REG_UgainB_Add         	0x65
#define		ATM_REG_IgainB_Add         	0x66
#define		ATM_REG_UoffsetB_Add       	0x67
#define		ATM_REG_IoffsetB_Add       	0x68
#define		ATM_REG_UgainC_Add         	0x69
#define		ATM_REG_IgainC_Add         	0x6A
#define		ATM_REG_UoffsetC_Add       	0x6B
#define		ATM_REG_IoffsetC_Add       	0x6C
#define		ATM_REG_IgainN         	0x6D
#define		ATM_REG_IoffsetN       	0x6E
#define		ATM_REG_CS3_Add            	0x6F
#define		ATM_REG_APenergyT		    0x80 // Energy Register
#define		ATM_REG_APenergyA       0x81
#define		ATM_REG_APenergyB       0x82
#define		ATM_REG_APenergyC       0x83
#define		ATM_REG_ANenergyT       0x84
#define		ATM_REG_ANenergyA       0x85
#define		ATM_REG_ANenergyB       0x86
#define		ATM_REG_ANenergyC       0x87
#define		ATM_REG_RPenergyT       0x88
#define		ATM_REG_RPenergyA       0x89
#define		ATM_REG_RPenergyB       0x8A
#define		ATM_REG_RPenergyC       0x8B
#define		ATM_REG_RNenergyT       0x8C
#define		ATM_REG_RNenergyA       0x8D
#define		ATM_REG_RNenergyB       0x8E
#define		ATM_REG_RNenergyC       0x8F
#define		ATM_REG_SAenergyT      	0x90
#define		ATM_REG_SenergyA       	0x91
#define		ATM_REG_SenergyB       	0x92
#define		ATM_REG_SenergyC       	0x93
#define		ATM_REG_SVenergyT      	0x94
#define		ATM_REG_EnStatus0      	0x95
#define		ATM_REG_EnStatus1      	0x96
#define		ATM_REG_SVmeanT        	0x98
#define		ATM_REG_SVmeanTLSB     	0x99
#define		ATM_REG_APenergyTF		  0xA0 // Fundamental / Harmonic Energy Register
#define		ATM_REG_APenergyAF      0xA1
#define		ATM_REG_APenergyBF      0xA2
#define		ATM_REG_APenergyCF      0xA3
#define		ATM_REG_ANenergyTF      0xA4
#define		ATM_REG_ANenergyAF      0xA5
#define		ATM_REG_ANenergyBF      0xA6
#define		ATM_REG_ANenergyCF      0xA7
#define		ATM_REG_APenergyTH      0xA8
#define		ATM_REG_APenergyAH      0xA9
#define		ATM_REG_APenergyBH      0xAA
#define		ATM_REG_APenergyCH      0xAB
#define		ATM_REG_ANenergyTH      0xAC
#define		ATM_REG_ANenergyAH      0xAD
#define		ATM_REG_ANenergyBH      0xAE
#define		ATM_REG_ANenergyCH      0xAF
#define		ATM_REG_PmeanT			    0xB0 // Power and Power Factor Registers
#define		ATM_REG_PmeanA          0xB1
#define		ATM_REG_PmeanB          0xB2
#define		ATM_REG_PmeanC          0xB3
#define		ATM_REG_QmeanT          0xB4
#define		ATM_REG_QmeanA          0xB5
#define		ATM_REG_QmeanB          0xB6
#define		ATM_REG_QmeanC          0xB7
#define		ATM_REG_SAmeanT         0xB8
#define		ATM_REG_SmeanA          0xB9
#define		ATM_REG_SmeanB          0xBA
#define		ATM_REG_SmeanC          0xBB
#define		ATM_REG_PFmeanT         0xBC
#define		ATM_REG_PFmeanA         0xBD
#define		ATM_REG_PFmeanB         0xBE
#define		ATM_REG_PFmeanC         0xBF
#define		ATM_REG_PmeanTLSB       0xC0
#define		ATM_REG_PmeanALSB       0xC1
#define		ATM_REG_PmeanBLSB       0xC2
#define		ATM_REG_PmeanCLSB       0xC3
#define		ATM_REG_QmeanTLSB       0xC4
#define		ATM_REG_QmeanALSB       0xC5
#define		ATM_REG_QmeanBLSB       0xC6
#define		ATM_REG_QmeanCLSB       0xC7
#define		ATM_REG_AmeanTLSB       0xC8
#define		ATM_REG_SmeanALSB       0xC9
#define		ATM_REG_SmeanBLSB       0xCA
#define		ATM_REG_SmeanCLSB       0xCB
#define		ATM_REG_PmeanTF			    0xD0 // Fundamental / Harmonic Power and Voltage / Current RMS Registers
#define		ATM_REG_PmeanAF	    	  0xD1
#define		ATM_REG_PmeanBF     	  0xD2
#define		ATM_REG_PmeanCF     	  0xD3
#define		ATM_REG_PmeanTH     	  0xD4
#define		ATM_REG_PmeanAH     	  0xD5
#define		ATM_REG_PmeanBH     	  0xD6
#define		ATM_REG_PmeanCH     	  0xD7
#define		ATM_REG_IrmsN1      	  0xD8
#define		ATM_REG_UrmsA_Add       	  0xD9
#define		ATM_REG_UrmsB_Add       	  0xDA
#define		ATM_REG_UrmsC_Add       	  0xDB
#define		ATM_REG_IrmsN0_Add      	  0xDC
#define		ATM_REG_IrmsA_Add       	  0xDD
#define		ATM_REG_IrmsB_Add       	  0xDE
#define		ATM_REG_IrmsC_Add       	  0xDF
#define		ATM_REG_PmeanTFLSB  	  0xE0
#define		ATM_REG_PmeanAFLSB  	  0xE1
#define		ATM_REG_PmeanBFLSB  	  0xE2
#define		ATM_REG_PmeanCFLSB  	  0xE3
#define		ATM_REG_PmeanTHLSB  	  0xE4
#define		ATM_REG_PmeanAHLSB  	  0xE5
#define		ATM_REG_PmeanBHLSB  	  0xE6
#define		ATM_REG_PmeanCHLSB  	  0xE7
#define		ATM_REG_UrmsALSB_Add    	  0xE9
#define		ATM_REG_UrmsBLSB_Add    	  0xEA
#define		ATM_REG_UrmsCLSB_Add    	  0xEB
#define		ATM_REG_IrmsALSB_Add    	  0xED
#define		ATM_REG_IrmsBLSB_Add    	  0xEE
#define		ATM_REG_IrmsCLSB_Add    	  0xEF
#define		ATM_REG_THDNUA			    0xF1 // THD+N, Frequency, Angle and Temperature Registers
#define		ATM_REG_THDNUB      	  0xF2
#define		ATM_REG_THDNUC      	  0xF3
#define		ATM_REG_THDNIA      	  0xF5
#define		ATM_REG_THDNIB      	  0xF6
#define		ATM_REG_THDNIC      	  0xF7
#define		ATM_REG_Freq_Add        0xF8
#define		ATM_REG_PAngleA     	  0xF9
#define		ATM_REG_PAngleB     	  0xFA
#define		ATM_REG_PAngleC     	  0xFB
#define		ATM_REG_Temp        	  0xFC
#define		ATM_REG_UangleA     	  0xFD
#define		ATM_REG_UangleB     	  0xFE
#define		ATM_REG_UangleC     	  0xFF


// Defines para trabalhar com leitura de tensao e corrente RMS
#define ATM_REG_LineVoltageRms_Offset            0xD8
#define ATM_REG_LineVoltageRmsLsb_Offset         0xE8
#define ATM_REG_LineCurrentRms_Offset            0xDC
//#define ATM_REG_LineCurrentRmsLsb_Offset        0xE8
#define ATM_REG_ActivePower_Offset               0xB0
#define ATM_REG_ReactivePower_Offset             0xB4
#define ATM_REG_AparantPower_Offset              0xB8
#define ATM_REG_PowerFactor_Offset               0xBC
#define ATM_REG_ActiveFundamentalPower_Offset    0xD0
#define ATM_REG_ActiveHarmonicPower_Offset       0xD4
#define ATM_REG_VoltageTHDN_Offset               0xF0
#define ATM_REG_CurrentTHDN_Offset               0xF4
#define ATM_REG_2_Order_HarmRatio_Current_Offset 0x100
#define ATM_REG_3_Order_HarmRatio_Current_Offset 0x101
#define ATM_REG_4_Order_HarmRatio_Current_Offset 0x102
#define ATM_REG_5_Order_HarmRatio_Current_Offset 0x103
#define ATM_REG_Total_HarmRatio_Current_Offset   0x11F
#define ATM_REG_2_Order_HarmRatio_Voltage_Offset 0x160
#define ATM_REG_3_Order_HarmRatio_Voltage_Offset 0x161
#define ATM_REG_4_Order_HarmRatio_Voltage_Offset 0x162
#define ATM_REG_5_Order_HarmRatio_Voltage_Offset 0x163
#define ATM_REG_Total_HarmRatio_Voltage_Offset   0x17F
#endif
