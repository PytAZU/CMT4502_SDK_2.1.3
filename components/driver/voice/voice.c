/**************************************************************************************************
 
  Wuxi CMOSTEK Microelectronics Co Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Wuxi CMOSTEK Microelectronics Co 
  Limited ("CMOSTEK"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of CMOSTEK. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a CMOSTEK Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  CMOSTEK OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/*******************************************************************************
* @file   voice.c
* @brief  Contains all functions support for adc driver
* @version  0.0
* @date   16. Jun. 2018
* @author CMOSTEK team
*
* Copyright(C) 2018, Wuxi CMOSTEK Microelectronics Co
* All rights reserved.
*
*******************************************************************************/

#include "error.h"
#include "ap_cp.h"
#include "common.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "clock.h"
#include "adc.h"
#include <string.h>
#include "log.h"
#include "voice.h"

static voice_Ctx_t mVoiceCtx;

static uint32_t voice_data[HALF_VOICE_WORD_SIZE];

// Enable voice core
void hal_voice_enable(void)
{
	clk_gate_enable(MOD_ADCC);
	subWriteReg(0x40050000,0,0,1);
}

// Disable voice core
void hal_voice_disable(void)
{
	clk_gate_disable(MOD_ADCC);
	subWriteReg(0x40050000,0,0,0);
}

// Select DMIC
void hal_voice_dmic_mode(void)
{
  subWriteReg(0x4005000c,0,0,1);
}

// Select AMIC
void hal_voice_amic_mode(void)
{
  subWriteReg(0x4005000c,0,0,0);
	subWriteReg(0x4000f048,7,5,0);		//Connect ADC to PGA
	subWriteReg(0x4000f07c,4,4,1);
	subWriteReg(0x4000f07c,0,0,1);
	subWriteReg(0x4000F000 + 0x7c,2,1,HAL_ADC_CLOCK_320K);
}

// Open a GPIO pin for DMIC
void hal_voice_dmic_open(GPIO_Pin_e dmicDataPin, GPIO_Pin_e dmicClkPin)
{
	hal_gpio_fmux_set(dmicDataPin, (Fmux_Type_e)ADCC);
	hal_gpio_fmux_set(dmicClkPin, (Fmux_Type_e)CLK1P28M);
}

// Set PGA gain for AMIC
void hal_voice_amic_gain(uint8_t amicGain)
{
	uint8_t pgaGain1;
	uint8_t pgaGain2;
	
	if (amicGain > 14)
		amicGain = 14;
	
	if (amicGain > 8) {
		pgaGain1 = 2;
		pgaGain2 = amicGain - 8;
	}
	else if (amicGain > 4) {
		pgaGain1 = 1;
		pgaGain2 = amicGain - 4;
	}
	else {
		pgaGain1 = 0;
		pgaGain2 = amicGain;
	}
	subWriteReg(0x4000f048,18,17,(uint32_t)pgaGain1);
	subWriteReg(0x4000f048,21,19,(uint32_t)pgaGain2);
}

// Set voice process gain
void hal_voice_gain(uint8_t voiceGain)
{
  subWriteReg(0x4005000c,22,16,(uint32_t)voiceGain);
}

// Set voice encoding mode
void hal_voice_encode(VOICE_ENCODE_t voiceEncodeMode)
{
  subWriteReg(0x4005000c,13,12,voiceEncodeMode);
}

// Set voice data rate
void hal_voice_rate(VOICE_RATE_t voiceRate)
{
  subWriteReg(0x4005000c,9,8,voiceRate);
}

// INTERNAL: Set voice notch filter config
static void set_voice_notch(VOICE_NOTCH_t voiceNotch)
{
  subWriteReg(0x4005000c,3,2,voiceNotch);
}

// INTERNAL: Set voice data polarity
static void set_voice_polarity(VOICE_POLARITY_t voicePolarity)
{
  subWriteReg(0x4005000c,1,1,voicePolarity);
}

// Enable voice auto-mute
void hal_voice_amute_on(void)
{
	subWriteReg(0x40050014,0,0,0);
}

// Disable voice auto-mute
void hal_voice_amute_off(void)
{
	subWriteReg(0x40050014,0,0,1);
}

// INTERNAL: Set voice auto-mute configurations
static void set_voice_amute_cfg(
	uint16_t amutGainMax, 
	uint8_t amutGainBwMax, 
	uint8_t amutGdut, 
	uint8_t amutGst2, 
	uint8_t amutGst1, 
	uint16_t amutLvl2, 
	uint16_t amutLvl1, 
	uint8_t amutAlvl,
	uint8_t amutBeta,
	uint8_t amutWinl)
{
	subWriteReg(0x40050010,30,20,(uint32_t)amutGainMax);
	subWriteReg(0x40050010,19,16,(uint32_t)amutGainBwMax);
	subWriteReg(0x40050010,13,8,(uint32_t)amutGdut);
	subWriteReg(0x40050010,7,4,(uint32_t)amutGst2);
	subWriteReg(0x40050010,3,0,(uint32_t)amutGst1);
	subWriteReg(0x40050014,30,20,(uint32_t)amutLvl2);
	subWriteReg(0x40050014,18,8,(uint32_t)amutLvl1);
	subWriteReg(0x40050018,15,8,(uint32_t)amutAlvl);
	subWriteReg(0x40050018,6,4,(uint32_t)amutBeta);
	subWriteReg(0x40050018,3,0,(uint32_t)amutWinl);
}



/**************************************************************************************
 * @fn          hal_VOICE_IRQHandler
 *
 * @brief       This function process for adc interrupt
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_ADC_IRQHandler(void)
{
//  uint32_t voice_data[HALF_VOICE_SAMPLE_SIZE];
	
//	LOG("Voice interrupt processing\n");
  MASK_VOICE_INT;
	
	if (GET_IRQ_STATUS & BIT(8)) {
		int n;
		for (n = 0; n < HALF_VOICE_WORD_SIZE; n++) {
			voice_data[n] = (uint32_t)(read_reg(VOICE_BASE + n * 4));
		}
		CLEAR_VOICE_HALF_INT;
		while (IS_CLAER_VOICE_HALF_INT) {}
//    if(mVoiceCtx.enable == FALSE)
//			continue;
		if (mVoiceCtx.evt_handler) {
			voice_Evt_t evt;
			evt.type = HAL_VOICE_EVT_DATA;
			evt.data = voice_data;
			evt.size = HALF_VOICE_WORD_SIZE;
			mVoiceCtx.evt_handler(&evt);
//			LOG("Voice memory half full interrupt processing completed\n");
		}
	}
	else if (GET_IRQ_STATUS & BIT(9)) {
		int n;
		for (n = 0; n < HALF_VOICE_WORD_SIZE; n++) {
			voice_data[n] = (uint32_t)(read_reg(VOICE_MID_BASE + n * 4));
		}
		CLEAR_VOICE_FULL_INT;
		while (IS_CLAER_VOICE_FULL_INT) {}
//    if(mVoiceCtx.enable == FALSE)
//			continue;
		if (mVoiceCtx.evt_handler) {
			voice_Evt_t evt;
			evt.type = HAL_VOICE_EVT_DATA;
			evt.data = voice_data;
			evt.size = HALF_VOICE_WORD_SIZE;
			mVoiceCtx.evt_handler(&evt);
//			LOG("Voice memory full interrupt processing completed\n");
		}
	}
	
  ENABLE_VOICE_INT;
}

/**************************************************************************************
 * @fn          hal_voice_init
 *
 * @brief       This function process for adc initial
 *
 * input parameters
 *
 * @param       ADC_MODE_e mode: adc sample mode select;1:SAM_MANNUAL(mannual mode),0:SAM_AUTO(auto mode)
 *              ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: signle-ended mode negative side enable; 1:SINGLE_END(single-ended mode) 0:DIFF(Differentail mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_voice_init(void) {
  hal_pwrmgr_register(MOD_ADCC,NULL,NULL);
	hal_pwrmgr_register(MOD_VOC,NULL,NULL);
  memset(&mVoiceCtx, 0, sizeof(mVoiceCtx));;
}

int hal_voice_config(voice_Cfg_t cfg, voice_Hdl_t evt_handler)
{
  if(mVoiceCtx.enable)
    return PPlus_ERR_BUSY;
	
  if(evt_handler == NULL)
   return PPlus_ERR_INVALID_PARAM;
	
	clk_gate_enable(MOD_ADCC);//enable I2C clk gated

	mVoiceCtx.evt_handler = evt_handler; //evt_handler;
	
//	mVoiceCtx.evt_handler = evtHandler;
	
  if(cfg.voiceSelAmicDmic) {
		hal_voice_dmic_mode();
		hal_voice_dmic_open(cfg.dmicDataPin, cfg.dmicClkPin);		
	}
	else {
		hal_voice_amic_mode();
		hal_voice_amic_gain(cfg.amicGain);
	}
	
	hal_voice_gain(cfg.voiceGain);
	hal_voice_encode(cfg.voiceEncodeMode);
	hal_voice_rate(cfg.voiceRate);
	
	set_voice_notch(VOICE_NOTCH_1);
	set_voice_polarity(VOICE_POLARITY_POS);
	
	if(cfg.voiceAutoMuteOnOff) {
		hal_voice_amute_off();
	}
	else {
		hal_voice_amute_on();
	}
	
	set_voice_amute_cfg(64, 6, 9, 0, 1, 55, 10, 48, 3, 10);
	
  mVoiceCtx.cfg = cfg;
	
//	mVoiceCtx.evt_handler = evt_handler;
  
  
  //CLK_1P28M_ENABLE;
  AP_PCRM->CLKSEL |= BIT(6);
  
	//ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
  AP_PCRM->CLKHF_CTL0 |= BIT(18);
  
	//ENABLE_DLL;                  //enable DLL
  AP_PCRM->CLKHF_CTL1 |= BIT(7);
	
	//ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
  AP_PCRM->CLKHF_CTL1 &= ~BIT(21);
	
	//ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
  AP_PCRM->CLKHF_CTL1 |= BIT(13);

  //subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
  AP_PCRM->ADC_CTL4 |= BIT(4);
	//*(volatile unsigned int *) 0x4000f040=0x5014B820;
	//*(volatile unsigned int *) 0x4000f044=0x019028b0;
  //*(volatile unsigned int *) 0x4000f048=0x0000014b;
	
//	hal_pwrmgr_register(MOD_ADCC,NULL,NULL);
//  hal_pwrmgr_register(MOD_VOC,NULL,NULL);
	
	return PPlus_SUCCESS;
}

int hal_voice_start(void)
{
  clk_gate_enable(MOD_ADCC);
	
	mVoiceCtx.enable = TRUE;
  hal_pwrmgr_lock(MOD_ADCC);
	hal_pwrmgr_lock(MOD_VOC);
	

  if (mVoiceCtx.cfg.voiceSelAmicDmic) {
	}		
	else {
		AP_PCRM->ANA_CTL |= BIT(16);	//Power on PGA
		AP_PCRM->ANA_CTL |= BIT(3);		//Power on ADC
  }


  NVIC_SetPriority((IRQn_Type)ADCC_IRQ, IRQ_PRIO_HAL);//teddy add 20190121	
  //ADCC_IRQ_ENABLE;
  NVIC_EnableIRQ((IRQn_Type)ADCC_IRQ);
  
  
	//Enable voice core
	hal_voice_enable();
	
	//Enable VOICE IRQ
	ENABLE_VOICE_INT;
	
  return PPlus_SUCCESS;

}

int hal_voice_stop(void)
{
  MASK_VOICE_INT;
  
	//Disable voice core
	hal_voice_disable();
	
  if (mVoiceCtx.cfg.voiceSelAmicDmic) {
	}		
	else {
		AP_PCRM->ANA_CTL &= ~BIT(16);	//Power off PGA
  }
 	
  //Enable sleep
  hal_pwrmgr_unlock(MOD_VOC);
  hal_pwrmgr_unlock(MOD_ADCC);

  mVoiceCtx.enable = FALSE;
	
	return 0;
}

int hal_voice_clear(void)
{
  //MASK_VOICE_INT;
  MASK_VOICE_INT;
  
  NVIC_DisableIRQ((IRQn_Type)ADCC_IRQ);
	
	if (mVoiceCtx.cfg.voiceSelAmicDmic) {
		hal_gpioin_disable(mVoiceCtx.cfg.dmicDataPin);
		hal_gpioin_disable(mVoiceCtx.cfg.dmicClkPin);
  }
	else	{
	}
	
  //clk_gate_disable(MOD_ADCC);//disable I2C clk gated

	memset(&mVoiceCtx, 0, sizeof(mVoiceCtx));
	
  //enableSleep();
  hal_pwrmgr_unlock(MOD_VOC);
	
	return 0;
}
