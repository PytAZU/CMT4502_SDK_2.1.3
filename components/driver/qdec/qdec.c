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
* @file		qdec.c
* @brief	Contains all functions support for key scan driver
* @version	0.0
* @date		13. Nov. 2017
* @author	Ding
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include <string.h>
#include "ap_cp.h"
#include "OSAL.h"
#include "qdec.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"
#include "clock.h"


qdec_Ctx_t m_qdecCtx;


void __attribute__((used)) hal_QDEC_IRQHandler()
{	
	hal_gpio_pin_init(P20, OEN);
	hal_gpio_write(P20,1);
	
	osal_stop_timerEx(m_qdecCtx.qdec_task_id, m_qdecCtx.timeout_event);
	
	WaitMs(1);
	
	int32_t delta = GET_CNT_QUAN(m_qdecCtx.cfg.qdec_chn);

	m_qdecCtx.count += delta;
	
	if(m_qdecCtx.cfg.evt_handler){
		qdec_Evt_t evt;
		evt.count = m_qdecCtx.count;
		m_qdecCtx.cfg.evt_handler(&evt);
    }
	
	CLR_INT_QUAN(m_qdecCtx.cfg.qdec_chn);
	hal_pwrmgr_unlock(MOD_QDEC);
	
	hal_gpio_pin_init(P20, OEN);
	hal_gpio_write(P20,0);
}

void hal_qdec_timeout_handler()
{
	osal_stop_timerEx(m_qdecCtx.qdec_task_id, m_qdecCtx.timeout_event);
	
	hal_pwrmgr_unlock(MOD_QDEC);
}

/**************************************************************************************
 * @fn          hal_qdec_set_cha
 *
 * @brief       This function process for qdec initial
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
static void hal_qdec_set_cha(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin){
	
	hal_gpio_pull_set(pin, PULL_DOWN);
	hal_gpio_fmux_set(pin, (Fmux_Type_e)(CHAX + (qdecCHN*3)));
}

/**************************************************************************************
 * @fn          hal_qdec_set_chb
 *
 * @brief       This function process for qdec initial
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
static void hal_qdec_set_chb(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin){
	
	hal_gpio_pull_set(pin, PULL_DOWN);
	hal_gpio_fmux_set(pin, (Fmux_Type_e)(CHBX + (qdecCHN*3)));
}

/**************************************************************************************
 * @fn          hal_qdec_set_chi
 *
 * @brief       This function process for qdec initial
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
static void hal_qdec_set_chi(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin){
	
	hal_gpio_pull_set(pin, PULL_DOWN);
	hal_gpio_fmux_set(pin, (Fmux_Type_e)(CHIX + (qdecCHN*3)));
}

/**************************************************************************************
 * @fn          hal_qdec_init
 *
 * @brief       This function process for qdec initial
 *
 * input parameters
 *
 * @param       qdec_Cfg_t cfg
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int hal_qdec_init(qdec_Cfg_t cfg, uint8 task_id, uint16 event){
	
	if(m_qdecCtx.enable)
		return PPlus_ERR_INVALID_STATE;
	
	uint8_t pins[3] = {cfg.cha_pin, cfg.chb_pin, cfg.chi_pin};
	memcpy(m_qdecCtx.pin_arr, pins, sizeof(uint8_t)*3);
	
	m_qdecCtx.cfg = cfg;
	m_qdecCtx.qdec_task_id = task_id;
	m_qdecCtx.timeout_event = event;
	m_qdecCtx.enable = TRUE;
	
	qdec_hw_config();
	
	hal_pwrmgr_register(MOD_QDEC, qdec_sleep_handler, qdec_wakeup_handler);
	
	return PPlus_SUCCESS;
}

void qdec_hw_config()
{
	qdec_Cfg_t* cfg = &(m_qdecCtx.cfg);
	
	clk_gate_enable(MOD_QDEC);
	
	hal_qdec_set_cha(cfg->qdec_chn, cfg->cha_pin);
	hal_qdec_set_chb(cfg->qdec_chn, cfg->chb_pin);
	
	DIS_INT_INCN(cfg->qdec_chn);
	DIS_INT_QUAN(cfg->qdec_chn);
	DIS_INT_02F_QUAN(cfg->qdec_chn);
	DIS_INT_F20_QUAN(cfg->qdec_chn);
		
	SET_MODE_QUAN(cfg->qdec_chn, cfg->quaMode);
	
	hal_qdec_set_qua_irq(cfg->qdec_chn, cfg->intMode);
	
	if(cfg->use_inc){
		hal_qdec_set_chi(cfg->qdec_chn, cfg->chi_pin);
		
		if(cfg->use_inc_irq){
			hal_qdec_set_inc_irq(cfg->qdec_chn, cfg->incMode, cfg->intMode);
		}
	}
	
	QDEC_IRQ_ENABLE;
	ENABLE_CHN(cfg->qdec_chn);
}

static void qdec_sleep_handler(void)
{	
	uint8_t pin_num;
	pin_num = m_qdecCtx.cfg.use_inc ? 3:2;
	
	for(uint8_t i=0;i<pin_num;i++){
		IO_Wakeup_Pol_e pol;
		GPIO_Pin_e pin = m_qdecCtx.pin_arr[i];
		
		hal_gpio_pin_init(pin, IE);
		pol = hal_gpio_read(pin) ? NEGEDGE:POSEDGE;
		hal_gpio_wakeup_set(pin, pol);
		m_qdecCtx.pin_state[i] = pol;
	}
	
	DISABLE_CHN(m_qdecCtx.cfg.qdec_chn);
	hal_gpio_pin_init(P23, OEN);
	hal_gpio_write(P23,0);
}

static void qdec_wakeup_handler(void)
{
	hal_gpio_pin_init(P23, OEN);
	hal_gpio_write(P23,1);

	uint8_t pin_num;
	pin_num = m_qdecCtx.cfg.use_inc ? 3:2;
	
	IO_Wakeup_Pol_e pol;
	for(uint8_t i=0;i<pin_num;i++){
		hal_gpio_pin_init(m_qdecCtx.pin_arr[i], IE);
		pol = hal_gpio_read(m_qdecCtx.pin_arr[i]) ? POSEDGE:NEGEDGE;
		
		if(pol == m_qdecCtx.pin_state[i]){
			break;
		}
		else if(i == pin_num-1){
			return;
		}
	}
	
	int32_t delta = GET_CNT_QUAN(m_qdecCtx.cfg.qdec_chn);
	
	hal_gpio_pin_init(P31, OEN);
	hal_gpio_write(P31,0);
	
	hal_pwrmgr_lock(MOD_QDEC);
	qdec_hw_config();
	
	hal_gpio_pin_init(P31, OEN);
	hal_gpio_write(P31,1);
	
	osal_start_timerEx(m_qdecCtx.qdec_task_id, m_qdecCtx.timeout_event, 150);
}

/**************************************************************************************
 * @fn          hal_qdec_set_qua_irq
 *
 * @brief       This function process for setting qdecode counter interupt mode
 *
 * input parameters
 *
 * @param       QDEC_CHN_e chn
 * @param       QDEC_INT_MODE_e intMode
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void hal_qdec_set_qua_irq(QDEC_CHN_e chn, QDEC_INT_MODE_e intMode){
	
	SET_INT_MODE_QUAN(chn, intMode);
	EN_INT_QUAN(chn);
}

/**************************************************************************************
 * @fn          hal_qdec_set_inc_irq
 *
 * @brief       This function process for setting qdecode index count mode and interupt mode
 *
 * input parameters
 *
 * @param       QDEC_CHN_e chn
 * @param       QDEC_INT_MODE_e intMode
 * @param       QDEC_INC_MODE_e incMode
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void hal_qdec_set_inc_irq(QDEC_CHN_e chn, QDEC_INC_MODE_e incMode, QDEC_INT_MODE_e intMode){
	
	SET_MODE_INCN(chn, incMode);
	SET_INT_MODE_INCN(chn, intMode);
	EN_INT_INCN(chn);
}

