/**
  ******************************************************************************
  * @file    UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains interface of UserInterface class
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* Pre-compiler coherency check */
#define PROJECT_CHK
#include "CrossCheck.h" 
#undef PROJECT_CHK

#include "MC_Type.h"
#include "Parameters conversion.h"
#if defined(PFC_ENABLED)
  #include "PIRegulatorClass.h"
#endif

#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"

#if defined(PFC_ENABLED)
  #include "PFCApplication.h"
#endif

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #include "MC_type.h"
  #define MAX_UI_NUM 3u

  _CUI_t UIpool[MAX_UI_NUM];
  unsigned char UI_Allocated = 0u;
#endif
  
#define CLASS_VARS &((_CUI)this)->Vars_str

/**
  * @brief  Creates an object of the class UserInterface
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @retval CUI new instance of UserInterface object
  */
CUI UI_NewObject(pUserInterfaceParams_t pUserInterfaceParams)
{
  _CUI _oUI;
  
  #ifdef MC_CLASS_DYNAMIC
    _oUI = (_CUI)calloc(1u,sizeof(_CUI_t));
  #else
    if (UI_Allocated  < MAX_UI_NUM)
    {
      _oUI = &UIpool[UI_Allocated++];
    }
    else
    {
      _oUI = MC_NULL;
    }
  #endif
  
  _oUI->pParams_str = (pParams_t)pUserInterfaceParams;
  
  return ((CUI)_oUI);
}

/**
  * @brief  Initialization of UI object. It perform the link between the UI
  *         object and the MC interface and MC tuning objects. It must be called
  *         before the derived class initialization.
  * @param  this related object of class CUI.
  * @param  bMCNum  Is the total number of MC object presnet in the list.
  * @param  pMCI is the pointer of the list of MC interface objects to be linked
  *         with the UI.
  * @param  pMCT is the pointer of the list of MC tuning objects to be linked
  *         with the UI.
  * @param  pUICfg is the pointer of the user interface configuration list. Each 
  *         element of the list must be a bit field containing one (or more) of 
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  * @retval none.
  */
void UI_Init(CUI this, uint8_t bMCNum, CMCI* pMCI, CMCT* pMCT, uint32_t* pUICfg)
{
  pVars_t pVars = CLASS_VARS;
  pVars->bDriveNum = bMCNum;
  pVars->pMCI = pMCI;
  pVars->pMCT = pMCT;
  pVars->bSelectedDrive = 0u;
  pVars->pUICfg = pUICfg;
}

/**
  * @brief  It is used to select the MC on which UI operates.
  * @param  this related object of class CUI.
  * @param  bSelectMC The new selected MC, zero based, on which UI operates.
  * @retval bool It return true if the bSelectMC is valid oterwise return false.
  */
bool UI_SelectMC(CUI this,uint8_t bSelectMC)
{
  pVars_t pVars = CLASS_VARS;
  bool retVal = TRUE;
  if (bSelectMC  >= pVars->bDriveNum)
  {
    retVal = FALSE;
  }
  else
  {
    pVars->bSelectedDrive = bSelectMC;
  }
  return retVal;
}

/**
  * @brief  It is used to retrive the MC on which UI currently operates.
  * @param  this related object of class CUI.
  * @retval uint8_t It returns the currently selected MC, zero based, on which
  *         UI operates.
  */
uint8_t UI_GetSelectedMC(CUI this)
{
  return ((_CUI)this)->Vars_str.bSelectedDrive;
}

/**
  * @brief  It is used to retrive the configuration of the MC on which UI 
  *         currently operates.
  * @param  this related object of class CUI.
  * @retval uint16_t It returns the currently configuration of selected MC on 
  *         which UI operates.
  *         It represents a bit field containing one (or more) of 
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  */
uint32_t UI_GetSelectedMCConfig(CUI this)
{
  pVars_t pVars = CLASS_VARS;

  return pVars->pUICfg[pVars->bSelectedDrive];
}

/**
  * @brief  It is used to retrieve the current selected MC tuning object.
  * @param  this related object of class CUI.
  * @retval CMCT It returns the currently selected MC tuning object on which
  *         UI operates.
  */
CMCT UI_GetCurrentMCT(CUI this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->pMCT[pVars->bSelectedDrive]);
}

/**
  * @brief  It is used to execute a SetReg command coming from the user.
  * @param  this related object of class CUI.
  * @param  bRegID Code of register to be updated. Valid code is one of the 
  *         MC_PROTOCOL_REG_xxx values exported by UserOnterfaceClass.
  * @param  wValue is the new value to be set.
  * @retval uint8_t It returns the currently selected MC, zero based, on which
  *         UI operates.
  */
bool UI_SetReg(CUI this, MC_Protocol_REG_t bRegID, int32_t wValue)
{
  pVars_t pVars = CLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];

  bool retVal = TRUE;
  switch (bRegID)
  {
  case MC_PROTOCOL_REG_TARGET_MOTOR:
    {
      retVal = UI_SelectMC(this,(uint8_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_CONTROL_MODE:
    {
      if ((STC_Modality_t)wValue == STC_TORQUE_MODE)
      {
        MCI_ExecTorqueRamp(oMCI, MCI_GetTeref(oMCI),0);
      }
      if ((STC_Modality_t)wValue == STC_SPEED_MODE)
      {
        MCI_ExecSpeedRamp(oMCI, MCI_GetMecSpeedRef01Hz(oMCI),0);
      }
    }
    break;
      
  case MC_PROTOCOL_REG_SPEED_KP:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_SPEED_KI:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_SPEED_KD:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      PID_SetKD((CPID_PI)oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_TORQUE_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(oMCI);
      currComp.qI_Component1 = (int16_t)wValue;
      MCI_SetCurrentReferences(oMCI,currComp);
    }
    break;
    
  case MC_PROTOCOL_REG_TORQUE_KP:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_TORQUE_KI:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_TORQUE_KD:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      PID_SetKD((CPID_PI)oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUX_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(oMCI);
      currComp.qI_Component2 = (int16_t)wValue;
      MCI_SetCurrentReferences(oMCI,currComp);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUX_KP:
    {
      CPI oPI = MCT_GetIdLoopPID(oMCT);
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUX_KI:
    {
      CPI oPI = MCT_GetIdLoopPID(oMCT);
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUX_KD:
    {
      CPI oPI = MCT_GetIdLoopPID(oMCT);
      PID_SetKD((CPID_PI)oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_OBSERVER_C1:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetObserverGains((CSTO_SPD)oSPD,&hC1,&hC2);
        STO_SetObserverGains((CSTO_SPD)oSPD,(int16_t)wValue,hC2);
      }
    }
    break;

  case MC_PROTOCOL_REG_OBSERVER_CR_C1:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_CR_GetObserverGains((CSTO_CR_SPD)oSPD,&hC1,&hC2);
        STO_CR_SetObserverGains((CSTO_CR_SPD)oSPD,(int16_t)wValue,hC2);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_OBSERVER_C2:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetObserverGains((CSTO_SPD)oSPD,&hC1,&hC2);
        STO_SetObserverGains((CSTO_SPD)oSPD,hC1,(int16_t)wValue);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_OBSERVER_CR_C2:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_CR_GetObserverGains((CSTO_CR_SPD)oSPD,&hC1,&hC2);
        STO_CR_SetObserverGains((CSTO_CR_SPD)oSPD,hC1,(int16_t)wValue);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_PLL_KI:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetPLLGains((CSTO_SPD)oSPD,&hPgain,&hIgain);
        STO_SetPLLGains((CSTO_SPD)oSPD,hPgain,(int16_t)wValue);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_PLL_KP:
{
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetPLLGains((CSTO_SPD)oSPD,&hPgain,&hIgain);
        STO_SetPLLGains((CSTO_SPD)oSPD,(int16_t)wValue,hIgain);
      }
    }    
    break;
    
  case MC_PROTOCOL_REG_FLUXWK_KP:
    {
      CPI oPI = MCT_GetFluxWeakeningLoopPID(oMCT);
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUXWK_KI:
    {
      CPI oPI = MCT_GetFluxWeakeningLoopPID(oMCT);
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FLUXWK_BUS:
    {
      CFW oFW = MCT_GetFluxWeakeningCtrl(oMCT);
      if (oFW)
      {
        FW_SetVref(oFW,(uint16_t)wValue);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_IQ_SPEEDMODE:
    {
      MCI_SetIdref(oMCI,(int16_t)wValue);
    }
    break;
    
  case MC_PROTOCOL_REG_FF_1Q:
    {
      FF_TuningStruct_t sNewConstants;
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      sNewConstants = FF_GetFFConstants(oFF);
      sNewConstants.wConst_1Q = wValue;
      FF_SetFFConstants(oFF,sNewConstants);
    }
    break;
    
  case MC_PROTOCOL_REG_FF_1D:
    {
      FF_TuningStruct_t sNewConstants;
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      sNewConstants = FF_GetFFConstants(oFF);
      sNewConstants.wConst_1D = wValue;
      FF_SetFFConstants(oFF,sNewConstants);
    }
    break;
    
  case MC_PROTOCOL_REG_FF_2:
    {
      FF_TuningStruct_t sNewConstants;
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      sNewConstants = FF_GetFFConstants(oFF);
      sNewConstants.wConst_2 = wValue;
      FF_SetFFConstants(oFF,sNewConstants);
    }
    break;
    
  case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      MCI_ExecSpeedRamp(oMCI,(int16_t)(wValue/6),0);
    }
    break;
    
#if defined(PFC_ENABLED)
  case MC_PROTOCOL_REG_PFC_DCBUS_REF:
    {
      PFC_SetBoostVoltageReference((int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KP:
    {
      CPI oPI = PFC_GetCurrentLoopPI();
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KI:
    {
      CPI oPI = PFC_GetCurrentLoopPI();
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KD:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      if ((hUICfg & UI_CFGOPT_PFC_I_KD) != 0u)
      {
        CPI oPID = PFC_GetCurrentLoopPI();
        PID_SetKD((CPID_PI)oPID,(int16_t)wValue);
      }
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KP:
    {
      CPI oPI = PFC_GetVoltageLoopPI();
      PI_SetKP(oPI,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KI:
    {
      CPI oPI = PFC_GetVoltageLoopPI();
      PI_SetKI(oPI,(int16_t)wValue);
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KD:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      if ((hUICfg & UI_CFGOPT_PFC_V_KD) != 0u)
      {
        CPI oPID = PFC_GetVoltageLoopPI();
        PID_SetKD((CPID_PI)oPID,(int16_t)wValue);
      }
    }
    break;
  case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
    {
      PFC_SetStartUpDuration((int16_t)wValue);
    }
    break;
#endif

#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        HFI_FP_SetMinSaturationDifference(oHFI,(int16_t)wValue);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        CPI oPI = HFI_FP_GetPITrack(oHFI);
        PI_SetKP(oPI,(int16_t)wValue);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        CPI oPI = HFI_FP_GetPITrack(oHFI);
        PI_SetKI(oPI,(int16_t)wValue);
      }
    }
    break;
#endif
  
  default:
    retVal = FALSE;
    break;
  }
  
  return retVal;
}

/**
  * @brief  It is used to execute a GetReg command coming from the user.
  * @param  this related object of class CUI.
  * @param  bRegID Code of register to be updated. Valid code is one of the 
  *         MC_PROTOCOL_REG_xxx values exported by UserInterfaceClass.
  * @retval int32_t is the current value of register bRegID.
  */
int32_t UI_GetReg(CUI this, MC_Protocol_REG_t bRegID)
{
  pVars_t pVars = CLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  
  int32_t bRetVal = (int32_t)GUI_ERROR_CODE;
  switch (bRegID)
  {
  case MC_PROTOCOL_REG_TARGET_MOTOR:
    {
      bRetVal = (int32_t)UI_GetSelectedMC(this);
    }
    break;
  case MC_PROTOCOL_REG_FLAGS:
    {
      CSTM oSTM = MCT_GetStateMachine(oMCT);
      bRetVal = (int32_t)STM_GetFaultState(oSTM);
    }
    break;
  case MC_PROTOCOL_REG_STATUS:
    {
      CSTM oSTM = MCT_GetStateMachine(oMCT);
      bRetVal = (int32_t)STM_GetState(oSTM);
    }
    break;
  case MC_PROTOCOL_REG_CONTROL_MODE:
    {
      bRetVal = (int32_t)MCI_GetControlMode(oMCI);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_REF:
    {
      bRetVal = (int32_t)(MCI_GetMecSpeedRef01Hz(oMCI) * 6);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_KP:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_KP_DIV:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKPDivisor(oPI);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_KI:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_KI_DIV:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKIDivisor(oPI);
    }
    break;
  case MC_PROTOCOL_REG_SPEED_KD:
    {
      CPI oPI = MCT_GetSpeedLoopPID(oMCT);
      bRetVal = (int32_t)PID_GetKD((CPID_PI)oPI);
    }
    break;
  case MC_PROTOCOL_REG_TORQUE_REF:
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(oMCI);      
      bRetVal = (int32_t)currComp.qI_Component1;
    }
    break;
  case MC_PROTOCOL_REG_TORQUE_KP:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_TORQUE_KI:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      bRetVal = (int32_t)PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_TORQUE_KD:
    {
      CPI oPI = MCT_GetIqLoopPID(oMCT);
      bRetVal = (int32_t)PID_GetKD((CPID_PI)oPI);
    }
    break;
  case MC_PROTOCOL_REG_FLUX_REF:
  case MC_PROTOCOL_REG_IQ_SPEEDMODE: 
    {
      Curr_Components currComp;
      currComp = MCI_GetIqdref(oMCI);      
      bRetVal = (int32_t)currComp.qI_Component2;
    }
    break;
  case MC_PROTOCOL_REG_FLUX_KP:
    {
      CPI oPI = MCT_GetIdLoopPID (oMCT);
      bRetVal = (int32_t)PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_FLUX_KI:
    {
      CPI oPI = MCT_GetIdLoopPID (oMCT);
      bRetVal = (int32_t)PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_FLUX_KD:
    {
      CPI oPI = MCT_GetIdLoopPID (oMCT);
      bRetVal = (int32_t)PID_GetKD((CPID_PI)oPI);
    }
    break;
  case MC_PROTOCOL_REG_OBSERVER_C1:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetObserverGains((CSTO_SPD)oSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC1;
    }
    break;
  case MC_PROTOCOL_REG_OBSERVER_CR_C1:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_CR_GetObserverGains((CSTO_CR_SPD)oSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC1;
    }
    break;
  case MC_PROTOCOL_REG_OBSERVER_C2:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetObserverGains((CSTO_SPD)oSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC2;
    }
    break;
  case MC_PROTOCOL_REG_OBSERVER_CR_C2:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_CR_GetObserverGains((CSTO_CR_SPD)oSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC2;
    }
    break;
  case MC_PROTOCOL_REG_PLL_KP:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetPLLGains((CSTO_SPD)oSPD,&hPgain,&hIgain);
      }
      bRetVal = (int32_t)hPgain;
    }
    break;
  case MC_PROTOCOL_REG_PLL_KI:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        STO_GetPLLGains((CSTO_SPD)oSPD,&hPgain,&hIgain);
      }
      bRetVal = (int32_t)hIgain;
    }
    break;
  case MC_PROTOCOL_REG_FLUXWK_KP:
    {
      CPI oPI = MCT_GetFluxWeakeningLoopPID(oMCT);
      bRetVal = PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_FLUXWK_KI:
    {
      CPI oPI = MCT_GetFluxWeakeningLoopPID(oMCT);
      bRetVal = PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_FLUXWK_BUS:
    {
      CFW oFW = MCT_GetFluxWeakeningCtrl(oMCT);
      if (oFW)
      {
        bRetVal = (int32_t)FW_GetVref(oFW);
      }
    }
    break;
  case MC_PROTOCOL_REG_BUS_VOLTAGE:
    {
      CVBS oVBS = MCT_GetBusVoltageSensor(oMCT);
      bRetVal = (int32_t)VBS_GetAvBusVoltage_V(oVBS);
    }
    break;
  case MC_PROTOCOL_REG_HEATS_TEMP:
    {
      CTSNS oTSNS = MCT_GetTemperatureSensor(oMCT);
      bRetVal = (int32_t)TSNS_GetAvTemp_C(oTSNS);
    }
    break;
  case MC_PROTOCOL_REG_MOTOR_POWER:
    {
      CMPM oMPM = MCT_GetMotorPowerMeasurement(oMCT);
      bRetVal = MPM_GetAvrgElMotorPowerW(oMPM);
    }
    break;
  case MC_PROTOCOL_REG_DAC_OUT1:
    {
      MC_Protocol_REG_t value = UI_GetDAC(this, DAC_CH0);
      bRetVal = value;
    }
    break;
  case MC_PROTOCOL_REG_DAC_OUT2:
    {
      MC_Protocol_REG_t value = UI_GetDAC(this, DAC_CH1);
      bRetVal = value;
    }
    break;
  case MC_PROTOCOL_REG_SPEED_MEAS:
    {
      bRetVal = (int32_t)(MCI_GetAvrgMecSpeed01Hz(oMCI) * 6);
    }
    break;
  case MC_PROTOCOL_REG_TORQUE_MEAS:
  case MC_PROTOCOL_REG_I_Q:
    {
      bRetVal = MCI_GetIqd(oMCI).qI_Component1;
    }
    break;
  case MC_PROTOCOL_REG_FLUX_MEAS:
  case MC_PROTOCOL_REG_I_D:
    {
      bRetVal = MCI_GetIqd(oMCI).qI_Component2;
    }
    break;
  case MC_PROTOCOL_REG_FLUXWK_BUS_MEAS:
    {
      CFW oFW = MCT_GetFluxWeakeningCtrl(oMCT);
      if (oFW)
      {
        bRetVal = ((int32_t)FW_GetAvVPercentage(oFW));
      }
    }
    break;
  case MC_PROTOCOL_REG_RUC_STAGE_NBR:
    {
      CRUC oRUC = MCT_GetRevupCtrl(oMCT);
      bRetVal = (int32_t)RUC_GetNumberOfPhases(oRUC);
    }
    break;
  case MC_PROTOCOL_REG_I_A:
    {
      bRetVal = MCI_GetIab(oMCI).qI_Component1;
    }
    break;
  case MC_PROTOCOL_REG_I_B:
    {
      bRetVal = MCI_GetIab(oMCI).qI_Component2;
    }
    break;
  case MC_PROTOCOL_REG_I_ALPHA:
    {
      bRetVal = MCI_GetIalphabeta(oMCI).qI_Component1;
    }
    break;
  case MC_PROTOCOL_REG_I_BETA:
    {
      bRetVal = MCI_GetIalphabeta(oMCI).qI_Component2;
    }
    break;
  case MC_PROTOCOL_REG_I_Q_REF:
    {
      bRetVal = MCI_GetIqdref(oMCI).qI_Component1;
    }
    break;
  case MC_PROTOCOL_REG_I_D_REF:
    {
      bRetVal = MCI_GetIqdref(oMCI).qI_Component2;
    }
    break;
  case MC_PROTOCOL_REG_V_Q:
    {
      bRetVal = MCI_GetVqd(oMCI).qV_Component1;
    }
    break;
  case MC_PROTOCOL_REG_V_D:
    {
      bRetVal = MCI_GetVqd(oMCI).qV_Component2;
    }
    break;
  case MC_PROTOCOL_REG_V_ALPHA:
    {
      bRetVal = MCI_GetValphabeta(oMCI).qV_Component1;
    }
    break;
  case MC_PROTOCOL_REG_V_BETA:
    {
      bRetVal = MCI_GetValphabeta(oMCI).qV_Component2;
    }
    break;
  case MC_PROTOCOL_REG_MEAS_EL_ANGLE:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if ((MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
          (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if ((AUX_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
          (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetElAngle(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_MEAS_ROT_SPEED:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if ((MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
          (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if ((AUX_SCFG_VALUE(hUICfg) == UI_SCODE_ENC) ||
          (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_HALL))
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetS16Speed(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_EL_ANGLE:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetElAngle(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_EL_ANGLE:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetElAngle(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_ROT_SPEED:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetS16Speed(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_EL_ANGLE:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetElAngle(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_ROT_SPEED:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetS16Speed(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_ROT_SPEED:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = SPD_GetS16Speed(oSPD);
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_I_ALPHA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetEstimatedCurrent((CSTO_SPD)oSPD).qI_Component1;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_I_ALPHA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetEstimatedCurrent((CSTO_CR_SPD)oSPD).qI_Component1;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_I_BETA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetEstimatedCurrent((CSTO_SPD)oSPD).qI_Component2;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_I_BETA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetEstimatedCurrent((CSTO_CR_SPD)oSPD).qI_Component2;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_BEMF_ALPHA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetEstimatedBemf((CSTO_SPD)oSPD).qV_Component1;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetEstimatedBemf((CSTO_CR_SPD)oSPD).qV_Component1;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_BEMF_BETA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetEstimatedBemf((CSTO_SPD)oSPD).qV_Component2;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_BEMF_BETA:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetEstimatedBemf((CSTO_CR_SPD)oSPD).qV_Component2;
      }
    }
    break;
  case MC_PROTOCOL_REG_DAC_USER1:
    {
      if (((_CUI)this)->Methods_str.pUI_DACGetUserChannelValue)
      {
        bRetVal = (((_CUI)this)->Methods_str.pUI_DACGetUserChannelValue)
                  (this, 0);
      } 
      else
      {
        bRetVal = 0;
      }
    }
    break;
  case MC_PROTOCOL_REG_DAC_USER2:
    {
      if (((_CUI)this)->Methods_str.pUI_DACGetUserChannelValue)
      {
        bRetVal = (((_CUI)this)->Methods_str.pUI_DACGetUserChannelValue)
                  (this, 1);
      } 
      else
      {
        bRetVal = 0;
      }
    }
    break;
  case MC_PROTOCOL_REG_MAX_APP_SPEED:
    {
      CSTC oSTC = MCT_GetSpeednTorqueController(oMCT);
      bRetVal = STC_GetMaxAppPositiveMecSpeed01Hz(oSTC) * 6;
    }
    break;
  case MC_PROTOCOL_REG_MIN_APP_SPEED:
    {
      CSTC oSTC = MCT_GetSpeednTorqueController(oMCT);
      bRetVal = STC_GetMinAppNegativeMecSpeed01Hz(oSTC) * 6;
    }
    break;
  case MC_PROTOCOL_REG_EST_BEMF_LEVEL:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetEstimatedBemfLevel((CSTO_SPD)oSPD) >> 16;
      }
    }
    break;
  case MC_PROTOCOL_REG_OBS_BEMF_LEVEL:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_GetObservedBemfLevel((CSTO_SPD)oSPD) >> 16;
      }
    }
    break;
  case MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetEstimatedBemfLevel((CSTO_CR_SPD)oSPD) >> 16;
      }      
    }
    break;
  case MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CSPD oSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorMain(oMCT);
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_CR)
      {
        oSPD = MCT_GetSpeednPosSensorAuxiliary(oMCT);
      }
      if (oSPD != MC_NULL)
      {
        bRetVal = STO_CR_GetObservedBemfLevel((CSTO_CR_SPD)oSPD) >> 16;
      }      
    }
    break;
  case MC_PROTOCOL_REG_FF_1Q:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetFFConstants(oFF).wConst_1Q;
    }
    break;
  case MC_PROTOCOL_REG_FF_1D:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetFFConstants(oFF).wConst_1D;
    }
    break;
  case MC_PROTOCOL_REG_FF_2:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetFFConstants(oFF).wConst_2;
    }
    break;
  case MC_PROTOCOL_REG_FF_VQ:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetVqdff(oFF).qV_Component1;
    }
    break;
  case MC_PROTOCOL_REG_FF_VD:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetVqdff(oFF).qV_Component2;
    }
    break;
  case MC_PROTOCOL_REG_FF_VQ_PIOUT:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetVqdAvPIout(oFF).qV_Component1;
    }
    break;
  case MC_PROTOCOL_REG_FF_VD_PIOUT:
    {
      CFF oFF = MCT_GetFeedForwardCtrl(oMCT);
      bRetVal = FF_GetVqdAvPIout(oFF).qV_Component2;
    }
    break;
case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      if (MCI_GetControlMode(oMCI) == STC_SPEED_MODE)
      {
        bRetVal = (int32_t)(MCI_GetLastRampFinalSpeed(oMCI) * 6);
      }
      else
      {
        bRetVal = (int32_t)(MCI_GetMecSpeedRef01Hz(oMCI) * 6);
      }
    }
    break;
#if defined(PFC_ENABLED)
  case MC_PROTOCOL_REG_PFC_STATUS:
    {
      CSTM oSTM = PFC_GetStateMachine();
      bRetVal = (int32_t)STM_GetState(oSTM);
    }
    break;
  case MC_PROTOCOL_REG_PFC_FAULTS:
    {
      CSTM oSTM = PFC_GetStateMachine();
      bRetVal = (int32_t)STM_GetFaultState(oSTM);
    }
    break;
  case MC_PROTOCOL_REG_PFC_DCBUS_REF:
    {
      bRetVal = (int32_t)PFC_GetBoostVoltageReference();
    }
    break;
  case MC_PROTOCOL_REG_PFC_DCBUS_MEAS:
    {
      bRetVal = (int32_t)PFC_GetAvBusVoltage_V();
    }
    break;
  case MC_PROTOCOL_REG_PFC_ACBUS_FREQ:
    {
      bRetVal = (int32_t)PFC_GetMainsFrequency();
    }
    break;
  case MC_PROTOCOL_REG_PFC_ACBUS_RMS:
    {
      bRetVal = (int32_t)PFC_GetMainsVoltage();
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KP:
    {
      CPI oPI = PFC_GetCurrentLoopPI();
      bRetVal = (int32_t)PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KI:
    {
      CPI oPI = PFC_GetCurrentLoopPI();
      bRetVal = (int32_t)PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_PFC_I_KD:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      if ((hUICfg & UI_CFGOPT_PFC_I_KD) != 0u)
      {
        CPI oPID = PFC_GetCurrentLoopPI();
        bRetVal = (int32_t)PID_GetKD((CPID_PI)oPID);
      }
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KP:
    {
      CPI oPI = PFC_GetVoltageLoopPI();
      bRetVal = (int32_t)PI_GetKP(oPI);
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KI:
    {
      CPI oPI = PFC_GetVoltageLoopPI();
      bRetVal = (int32_t)PI_GetKI(oPI);
    }
    break;
  case MC_PROTOCOL_REG_PFC_V_KD:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      if ((hUICfg & UI_CFGOPT_PFC_V_KD) != 0u)
      {
        CPI oPID = PFC_GetVoltageLoopPI();
        bRetVal = (int32_t)PID_GetKD((CPID_PI)oPID);
      }
    }
    break;
  case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
    {
      bRetVal = (int32_t)PFC_GetStartUpDuration();
    }
    break;
  case MC_PROTOCOL_REG_PFC_ENABLED:
    {
      bRetVal = (int32_t)PFC_GetState();
    }
    break;
#endif

#if defined(HFINJECTION) || (defined(DUALDRIVE) && defined(HFINJECTION2))
  case MC_PROTOCOL_REG_HFI_CURRENT:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        bRetVal = HFI_FP_GetCurrent(oHFI);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_INIT_ANG_PLL:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        bRetVal = HFI_FP_GetRotorAngleLock(oHFI);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        bRetVal = HFI_FP_GetSaturationDifference(oHFI);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        CPI oPI = HFI_FP_GetPITrack(oHFI);
        bRetVal = PI_GetKP(oPI);
      }
    }
    break;
  case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
    {
      uint32_t hUICfg = pVars->pUICfg[pVars->bSelectedDrive];
      CHFI_FP oHFI = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_HFINJ)
      {
        oHFI = MCT_GetHFICtrl(oMCT);
      }
      if (oHFI != MC_NULL)
      {
        CPI oPI = HFI_FP_GetPITrack(oHFI);
        bRetVal = PI_GetKI(oPI);
      }
    }
    break;
#endif
    
#if defined(MOTOR_PROFILER)
  case MC_PROTOCOL_REG_SC_CHECK:
    {
      bRetVal = 1;
    }
    break;
    
  case MC_PROTOCOL_REG_SC_STATE:
    {
      uint8_t state = 0;
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      COTT oOTT = MCT_GetOneTouchTuning(oMCT);
      if (oSCC != MC_NULL)
      {
        state += SCC_GetState(oSCC);
      }
      if (oOTT != MC_NULL)
      {
        state += OTT_GetState(oOTT);
      }
      bRetVal = (int32_t)(state);
    }
    break;
    
  case MC_PROTOCOL_REG_SC_STEPS:
    {
      uint8_t state = 0;
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      COTT oOTT = MCT_GetOneTouchTuning(oMCT);
      if (oSCC != MC_NULL)
      {
        state += SCC_GetSteps(oSCC);
      }
      if (oOTT != MC_NULL)
      {
        state += OTT_GetSteps(oOTT);
      }
      bRetVal = (int32_t)(state - 1u);
    }
    break;
    
  case MC_PROTOCOL_REG_SC_RS:
    {
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      if (oSCC != MC_NULL)
      {
        bRetVal = SCC_GetRs(oSCC);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_SC_LS:
    {
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      if (oSCC != MC_NULL)
      {
        bRetVal = SCC_GetLs(oSCC);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_SC_KE:
    {
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      if (oSCC != MC_NULL)
      {
        bRetVal = SCC_GetKe(oSCC);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_SC_VBUS:
    {
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      if (oSCC != MC_NULL)
      {
        bRetVal = SCC_GetVbus(oSCC);
      }
    }
    break;
    
  case MC_PROTOCOL_REG_SC_NOMINALSPEED:
    {
      COTT oOTT = MCT_GetOneTouchTuning(oMCT);
      if (oOTT != MC_NULL)
      {
        bRetVal = OTT_GetNominalSpeed(oOTT);
      }
    }
    break;
#endif
    
  default:
    break;
  }
  return bRetVal;
}

/**
  * @brief  It is used to execute a command coming from the user.
  * @param  this related object of class CUI.
  * @param  bCmdID Code of register to be updated. Valid code is one of the 
  *         MC_PROTOCOL_CMD_xxx define exported by UserInterfaceClass.
  * @retval bool It returns true if the command has been performed 
  *         succesfully otherwise returns false.
*/
bool UI_ExecCmd(CUI this, uint8_t bCmdID)
{
  bool retVal = TRUE;
  pVars_t pVars = CLASS_VARS;
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  
  switch (bCmdID)
  {
  case MC_PROTOCOL_CMD_START_MOTOR:
    {
      /*  Call MCI Start motor; */
      MCI_StartMotor(oMCI);
    }
    break;
  case MC_PROTOCOL_CMD_STOP_MOTOR:
    {
      /* Call MCI Stop motor; */
      MCI_StopMotor(oMCI);
    }
    break;
  case MC_PROTOCOL_CMD_STOP_RAMP:
    {
      if (MCI_GetSTMState(oMCI) == RUN)
      {
        MCI_ExecSpeedRamp(oMCI,MCI_GetAvrgMecSpeed01Hz(oMCI),0);
      }
    }
    break;
  case MC_PROTOCOL_CMD_PING:
    {
    }
    break;
  case MC_PROTOCOL_CMD_START_STOP:
    {
      /* Queries the STM and a command start or stop depending on the state. */
      if (MCI_GetSTMState(oMCI) == IDLE)
      {
        MCI_StartMotor(oMCI);
      }
      else
      {
        MCI_StopMotor(oMCI);
      }
    }
    break;
  case MC_PROTOCOL_CMD_RESET:
    {
    }
    break;
  case MC_PROTOCOL_CMD_FAULT_ACK:
    {
      MCI_FaultAcknowledged(oMCI);
    }
    break;
  case MC_PROTOCOL_CMD_ENCODER_ALIGN:
    {
      MCI_EncoderAlign(oMCI);
    }
    break;
  case MC_PROTOCOL_CMD_IQDREF_CLEAR:
    {
      MCI_Clear_Iqdref(oMCI);
    }
    break;
#if defined(PFC_ENABLED)  
  case MC_PROTOCOL_CMD_PFC_ENABLE:
    {
      if ((pVars->pUICfg[pVars->bSelectedDrive] & UI_CFGOPT_PFC) == 0u)
      {
        // No PFC configured
        retVal = FALSE;
      }
      else
      {
        PFC_Enable();
      }
    }
    break;
  case MC_PROTOCOL_CMD_PFC_DISABLE:
    {
      if ((pVars->pUICfg[pVars->bSelectedDrive] & UI_CFGOPT_PFC) == 0u)
      {
        // No PFC configured
        retVal = FALSE;
      }
      else
      {
        PFC_Disable();
      }
    }
    break;
  case MC_PROTOCOL_CMD_PFC_FAULT_ACK:
    {
      if ((pVars->pUICfg[pVars->bSelectedDrive] & UI_CFGOPT_PFC) == 0u)
      {
        // No PFC configured
        retVal = FALSE;
      }
      else
      {
        PFC_FaultAcknowledged();
      }
    }
    break;
#endif
    
#if defined(MOTOR_PROFILER)
  case MC_PROTOCOL_CMD_SC_START:
    {
      CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
      CSCC oSCC = MCT_GetSelfCommissioning(oMCT);
      COTT oOTT = MCT_GetOneTouchTuning(oMCT);
      if (oSCC)
      {
        SCC_ForceProfile(oSCC);
      }
      if (oOTT)
      {
        OTT_ForceTuning(oOTT);
      }
      MCI_StartMotor(oMCI);
    }
    break;
  case MC_PROTOCOL_CMD_SC_STOP:
    {
      MCI_StopMotor(oMCI);
    }
    break;
#endif
  default:
    retVal = FALSE;
    break;
  }
  return retVal;
}

/**
  * @brief  It is used to execute a speed ramp command coming from the user.
  * @param  this related object of class CUI.
  * @param  wFinalMecSpeedRPM final speed value expressed in RPM.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It returns true if the command has been performed 
  *         succesfully otherwise returns false.
  */
bool UI_ExecSpeedRamp(CUI this, int32_t wFinalMecSpeedRPM, uint16_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  
  /* Call MCI Exec Ramp */
  MCI_ExecSpeedRamp(oMCI,(int16_t)(wFinalMecSpeedRPM/6),hDurationms);
  return TRUE;
}

/**
  * @brief  It is used to execute a torque ramp command coming from the user.
  * @param  this related object of class CUI.
  * @param  hTargetFinal final torque value. See MCI interface for more
            details.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It returns true if the command has been performed 
  *         succesfully otherwise returns false.
  */
bool UI_ExecTorqueRamp(CUI this, int16_t hTargetFinal, uint16_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  
  /* Call MCI Exec Ramp */
  MCI_ExecTorqueRamp(oMCI,hTargetFinal,hDurationms);
  return TRUE;
}

/**
  * @brief  It is used to execute a get Revup data command coming from the user.
  * @param  this related object of class CUI.
  * @param  bStage is the rev up phase, zero based, to be read.
  * @param  pDurationms is the pointer to an uint16_t variable used to retrieve 
  *         the duration of the Revup stage.
  * @param  pFinalMecSpeed01Hz is the pointer to an int16_t variable used to 
  *         retrieve the mechanical speed at the end of that stage expressed in
  *         0.1Hz.
  * @param  pFinalTorque is the pointer to an int16_t variable used to 
  *         retrieve the value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  * @retval bool It returns true if the command has been performed 
  *         succesfully otherwise returns false.
  */
bool UI_GetRevupData(CUI this, uint8_t bStage, uint16_t* pDurationms, 
                     int16_t* pFinalMecSpeed01Hz, int16_t* pFinalTorque )
{
  bool hRetVal = TRUE;
  pVars_t pVars = CLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CRUC oRUC = MCT_GetRevupCtrl(oMCT);
  if (oRUC)
  {
    *pDurationms = RUC_GetPhaseDurationms(oRUC, bStage);
    *pFinalMecSpeed01Hz = RUC_GetPhaseFinalMecSpeed01Hz(oRUC, bStage);
    *pFinalTorque = RUC_GetPhaseFinalTorque(oRUC, bStage);
  }
  else
  {
    hRetVal = FALSE;
  }
  return hRetVal;
}

/**
  * @brief  It is used to execute a set Revup data command coming from the user.
  * @param  this related object of class CUI.
  * @param  bStage is the rev up phase, zero based, to be modified.
  * @param  hDurationms is the new duration of the Revup stage.
  * @param  hFinalMecSpeed01Hz is the new mechanical speed at the end of that 
  *         stage expressed in 0.1Hz.
  * @param  hFinalTorque is the new value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  * @retval bool It returns true if the command has been performed 
  *         succesfully otherwise returns false.
  */
bool UI_SetRevupData(CUI this, uint8_t bStage, uint16_t hDurationms, 
                     int16_t hFinalMecSpeed01Hz, int16_t hFinalTorque )
{
  pVars_t pVars = CLASS_VARS;
  CMCT oMCT = pVars->pMCT[pVars->bSelectedDrive];
  CRUC oRUC = MCT_GetRevupCtrl(oMCT);
  RUC_SetPhaseDurationms(oRUC, bStage, hDurationms);
  RUC_SetPhaseFinalMecSpeed01Hz(oRUC, bStage, hFinalMecSpeed01Hz);
  RUC_SetPhaseFinalTorque(oRUC, bStage, hFinalTorque);
  return TRUE;
}

/**
  * @brief  It is used to execute a set current reference command coming from 
  *         the user.
  * @param  this related object of class CUI.
  * @param  hIqRef is the current Iq reference on qd reference frame. This value
  *         is expressed in digit. To convert current expressed in digit to 
  *         current expressed in Amps is possible to use the formula: 
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  hIdRef is the current Id reference on qd reference frame. This value
  *         is expressed in digit. See hIqRef param description.
  * @retval none.
  */
void UI_SetCurrentReferences(CUI this, int16_t hIqRef, int16_t hIdRef)
{
  pVars_t pVars = CLASS_VARS;
  CMCI oMCI = pVars->pMCI[pVars->bSelectedDrive];
  Curr_Components currComp;
  currComp.qI_Component1 = hIqRef;
  currComp.qI_Component2 = hIdRef;
  MCI_SetCurrentReferences(oMCI,currComp);
}

/**
  * @brief  Hardware and software initialization of the DAC object. This is a 
  *         virtual function and is implemented by related object.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
void UI_DACInit(CUI this)
{
  if (((_CUI)this)->Methods_str.pUI_DACInit)
  {
    (((_CUI)this)->Methods_str.pUI_DACInit)(this);
  }
}

/**
  * @brief  This method is used to update the DAC outputs. The selected 
  *         variables will be provided in the related output channels. This is a
  *         virtual function and is implemented by related object.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  * @retval none.
  */
void UI_DACExec(CUI this)
{
  if (((_CUI)this)->Methods_str.pUI_DACExec)
  {
    (((_CUI)this)->Methods_str.pUI_DACExec)(this);
  }
}

/**
  * @brief  This method is used to set up the DAC outputs. The selected
  *         variables will be provided in the related output channels after next
  *         DACExec.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  bChannel the DAC channel to be programmed. It must be one of the 
  *         exported channels Ex. DAC_CH0.
  * @param  bVariable the variables to be provided in out through the selected
  *         channel. It must be one of the exported UI register Ex. 
  *         MC_PROTOCOL_REG_I_A.
  * @retval none.
  */
void UI_SetDAC(CUI this, DAC_Channel_t bChannel, 
                         MC_Protocol_REG_t bVariable)
{
  if (((_CUI)this)->Methods_str.pUI_DACSetChannelConfig)
  {
    (((_CUI)this)->Methods_str.pUI_DACSetChannelConfig)(this, bChannel, bVariable);
  }
}

/**
  * @brief  This method is used to get the current DAC channel selected output.
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  bChannel the inspected DAC channel. It must be one of the 
  *         exported channels (Ex. DAC_CH0).
  * @retval MC_Protocol_REG_t The variables provided in out through the inspected
  *         channel. It will be one of the exported UI register (Ex. 
  *         MC_PROTOCOL_REG_I_A).
  */
MC_Protocol_REG_t UI_GetDAC(CUI this, DAC_Channel_t bChannel)
{
  MC_Protocol_REG_t retVal = MC_PROTOCOL_REG_UNDEFINED;
  if (((_CUI)this)->Methods_str.pUI_DACGetChannelConfig)
  {
    retVal = (((_CUI)this)->Methods_str.pUI_DACGetChannelConfig)(this, bChannel);
  }
  return retVal;
}

/**
  * @brief  This method is used to set the value of the "User DAC channel".
  * @param  this related object of class UI. It must be a DACx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  bUserChNumber the "User DAC channel" to be programmed.
  * @param  hValue the value to be put in output.
  * @retval none.
  */
void UI_SetUserDAC(CUI this, DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  if (((_CUI)this)->Methods_str.pUI_DACSetUserChannelValue)
  {
    (((_CUI)this)->Methods_str.pUI_DACSetUserChannelValue)(this, bUserChNumber,
                                                           hValue);
  }
}

/**
  * @brief  Initialization of LCD object. It must be called after the UI_Init.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  oDAC related DAC object upcasted to CUI. It can be MC_NULL.
  * @param  s_fwVer String contating firmware version.
  * @retval none.
  */
void UI_LCDInit(CUI this, CUI oDAC, const char* s_fwVer)
{
  if (((_CUI)this)->Methods_str.pUI_LCDInit)
  {
    (((_CUI)this)->Methods_str.pUI_LCDInit)(this, oDAC, s_fwVer);
  }
}

/**
  * @brief  Execute the LCD execution and refreshing. It must be called 
  *         periodically.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void UI_LCDExec(CUI this)
{
  if (((_CUI)this)->Methods_str.pUI_LCDExec)
  {
    (((_CUI)this)->Methods_str.pUI_LCDExec)(this);
  }
}

/**
  * @brief  It is used to force a refresh of all LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void UI_LCDUpdateAll(CUI this)
{
  if (((_CUI)this)->Methods_str.pUI_LCDUpdateAll)
  {
    (((_CUI)this)->Methods_str.pUI_LCDUpdateAll)(this);
  }
}

/**
  * @brief  It is used to force a refresh of only measured LCD values.
  * @param  this related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void UI_LCDUpdateMeasured(CUI this)
{
  if (((_CUI)this)->Methods_str.pUI_LCDUpdateMeasured)
  {
    (((_CUI)this)->Methods_str.pUI_LCDUpdateMeasured)(this);
  }
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
