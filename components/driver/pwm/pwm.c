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
* @file		pwm.c
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "ap_cp.h"
#include "clock.h"
#include "pwm.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "common.h"

/**************************************************************************************
 * @fn          hal_pwm_init
 *
 * @brief       This function process for pwm initial
 *
 * input parameters
 *
 * @param       PWMN_e pwmN						: pwm channel
 *              PWM_CLK_DIV_e pwmDiv			: clock prescaler of PWM channel
 *				PWM_CNT_MODE_e pwmMode			: count mode of PWM channel
 *				PWM_POLARITY_e pwmPolarity		: output polarity setting of PWM channel
 *				unsigned short cmpVal			: the compare value of PWM channel 
 *				unsigned short cntTopVal		: the counter top value of PWM channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_init(
					PWMN_e pwmN,
					PWM_CLK_DIV_e pwmDiv,
					PWM_CNT_MODE_e pwmMode,
					PWM_POLARITY_e pwmPolarity)
{
		clk_gate_enable(MOD_PWM);
		PWM_DISABLE_CH(pwmN);
		
		PWM_SET_DIV(pwmN, pwmDiv);
		PWM_SET_MODE(pwmN, pwmMode);
		PWM_SET_POL(pwmN, pwmPolarity);
		
		//PWM_INSTANT_LOAD_CH(pwmN);

    hal_pwrmgr_register(MOD_PWM, NULL, NULL);
}

/**************************************************************************************
 * @fn          hal_pwm_open_channel
 *
 * @brief       This function process for pwm start working
 *
 * input parameters
 *
 * @param       PWMN_e pwmN						: pwm channel
 *              GPIO_Pin_e pwmPin				: pwm pin number
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_open_channel(PWMN_e pwmN,GPIO_Pin_e pwmPin){

		hal_gpio_fmux_set(pwmPin, (Fmux_Type_e)(PWM0 + pwmN));
		PWM_ENABLE_CH(pwmN);
}

/**************************************************************************************
 * @fn          hal_pwm_close_channel
 *
 * @brief       This function process for pwm stop working
 *
 * input parameters
 *
 * @param       PWMN_e pwmN						: pwm channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_close_channel(PWMN_e pwmN){
	
		PWM_DISABLE_CH(pwmN);
		//clk_gate_disable(MOD_PWM);
}

/**************************************************************************************
 * @fn          hal_pwm_destroy
 *
 * @brief       This function process for pwm clear and disable
 *
 * input parameters
 *
 * @param       PWMN_e pwmN						: pwm channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_destroy(PWMN_e pwmN){
	
		PWM_DISABLE_CH(pwmN);
		PWM_NO_LOAD_CH(pwmN);
		PWM_NO_INSTANT_LOAD_CH(pwmN);
	
		PWM_SET_DIV(pwmN, 0);
		PWM_SET_MODE(pwmN, 0);
		PWM_SET_POL(pwmN, 0);
		
		PWM_SET_TOP_VAL(pwmN, 0);
		PWM_SET_CMP_VAL(pwmN, 0);
}

/**************************************************************************************
 * @fn          hal_pwm_set_count_val
 *
 * @brief       This function process for change pwm count value
 *
 * input parameters
 *
 * @param       PWMN_e pwmN						: pwm channel
 *				uint16_t cmpVal					: the compare value of PWM channel 
 *				uint16_t cntTopVal				: the counter top value of PWM channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_set_count_val(PWMN_e pwmN, uint16_t cmpVal, uint16_t cntTopVal){

		PWM_NO_LOAD_CH(pwmN);
		PWM_SET_CMP_VAL(pwmN, cmpVal);
		PWM_SET_TOP_VAL(pwmN, cntTopVal);
		PWM_LOAD_CH(pwmN);
}

static unsigned int pwm_en = 0;
void hal_pwm_start(void)
{
	if(pwm_en == 0)
	{
		hal_pwrmgr_lock(MOD_PWM);
		PWM_ENABLE_ALL;
		pwm_en = 1;
	}
}

void hal_pwm_stop(void)
{
	if(pwm_en == 1)
	{
		hal_pwrmgr_unlock(MOD_PWM);
		PWM_DISABLE_ALL;
		pwm_en = 0;
	}
}
//------------------------------------------------------------
//new api,make use easily
typedef struct {
  bool          enable;
  bool          ch_en[6];
  pwm_ch_t      ch[6];
}pwm_Ctx_t;

static pwm_Ctx_t pwmCtx = 
{
	.enable = FALSE,
	.ch_en = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE},
};

void hal_pwm_module_init(void)
{
	int i = 0;
		
	if(pwmCtx.enable == TRUE)
		return;
	pwmCtx.enable = TRUE;
	
	for(i = 0;i < 6;i++)
	{
		pwmCtx.ch_en[i] = FALSE;
		
		pwmCtx.ch[i].pwmN = (PWMN_e)i;		
		pwmCtx.ch[i].pwmPin = GPIO_DUMMY;		
		pwmCtx.ch[i].pwmDiv = PWM_CLK_NO_DIV;
		pwmCtx.ch[i].pwmMode = PWM_CNT_UP;
		pwmCtx.ch[i].pwmPolarity = PWM_POLARITY_RISING;
		pwmCtx.ch[i].cmpVal = 0;
		pwmCtx.ch[i].cntTopVal = 0;
	}		
	
	hal_pwm_stop();
}

void hal_pwm_module_deinit(void)
{
	int i = 0;
		
	if(pwmCtx.enable == FALSE)
		return;
	pwmCtx.enable = FALSE;
	
	for(i = 0;i < 6;i++)
	{
		pwmCtx.ch_en[i] = FALSE;
		
		pwmCtx.ch[i].pwmN = (PWMN_e)i;		
		pwmCtx.ch[i].pwmPin = GPIO_DUMMY;		
		pwmCtx.ch[i].pwmDiv = PWM_CLK_NO_DIV;
		pwmCtx.ch[i].pwmMode = PWM_CNT_UP;
		pwmCtx.ch[i].pwmPolarity = PWM_POLARITY_RISING;
		pwmCtx.ch[i].cmpVal = 0;
		pwmCtx.ch[i].cntTopVal = 0;
		
		hal_pwm_close_channel((PWMN_e)i);
		hal_pwm_destroy((PWMN_e)i);	
	}	
	
	hal_pwm_stop();
}

void hal_pwm_ch_start(pwm_ch_t ch)
{
	if(pwmCtx.enable == FALSE)
		return;
	
	if(pwmCtx.ch_en[ch.pwmN] == TRUE)
	{
		hal_pwm_set_count_val(ch.pwmN,ch.cmpVal,ch.cntTopVal);
	}
	else
	{
		hal_pwm_init(ch.pwmN,ch.pwmDiv,ch.pwmMode,ch.pwmPolarity);
		hal_pwm_set_count_val(ch.pwmN,ch.cmpVal,ch.cntTopVal);
		hal_pwm_open_channel(ch.pwmN,ch.pwmPin);
		pwmCtx.ch_en[ch.pwmN] = TRUE;

		hal_pwm_start();
	}
}

void hal_pwm_ch_stop(pwm_ch_t ch)
{
	if(pwmCtx.ch_en[ch.pwmN] == FALSE)
		return;
	else
	{
		hal_pwm_stop();
		pwmCtx.ch_en[ch.pwmN] = FALSE;		
		hal_pwm_destroy(ch.pwmN);	
		hal_pwm_close_channel(ch.pwmN);		
	}	
}

bool hal_pwm_ch_enable(PWMN_e pwmN)
{
	return pwmCtx.ch_en[pwmN];
}

pwm_ch_t hal_pwm_ch_reg(PWMN_e pwmN)
{
	return pwmCtx.ch[pwmN];
}
