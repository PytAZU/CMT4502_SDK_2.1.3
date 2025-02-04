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
* @file		kscan.c
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
#include "kscan.h"
#include "pwrmgr.h"
#include "error.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "log.h"

typedef struct {
	bool         	enable;
	kscan_Cfg_t 	cfg;
	uint16_t     	key_state[18];
	uint8_t       	pin_state[NUM_KEY_ROWS];
	uint8_t 		kscan_task_id;
	uint16_t		timeout_event;
} kscan_Ctx_t;


static kscan_Ctx_t m_kscanCtx;
static kscan_Key_t m_keys[MAX_KEY_NUM];

static uint8_t reScan_flag=0;


//PRIVATE FUNCTIONS
static void kscan_hw_config(void);
static void hal_kscan_config_row(KSCAN_ROWS_e row);
static void hal_kscan_config_col(KSCAN_COLS_e col);
static void kscan_sleep_handler(void);
static void kscan_wakeup_handler(void);
static void get_key_matrix(uint16_t* key_matrix);
static void rmv_ghost_key(uint16_t* key_matrix);
static kscan_Evt_t 	kscan_compare_key(uint16_t* key_pre, uint16_t* key_nxt);
static void hal_kscan_clear_config(void);

#define	TIMEOUT_DELTA	10

/**************************************************************************************
 * @fn          hal_kscan_init
 *
 * @brief       This function process for key scan initial
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
int hal_kscan_init(kscan_Cfg_t cfg, uint8 task_id, uint16 event){
	
	if(m_kscanCtx.enable)
		return PPlus_ERR_INVALID_STATE;
	
	m_kscanCtx.cfg = cfg;
	m_kscanCtx.kscan_task_id = task_id;
	m_kscanCtx.timeout_event = event;
	m_kscanCtx.enable = TRUE;
	
	kscan_hw_config();
	
	hal_pwrmgr_register(MOD_KSCAN, kscan_sleep_handler, kscan_wakeup_handler);
	
	return PPlus_SUCCESS;
}


/**************************************************************************************
 * @fn          hal_kscan_clear_config
 *
 * @brief       This function process for key scan clear config
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
void hal_kscan_clear_config(){
	
	*(volatile unsigned int *) 0x4000384C &= 0x0;
	*(volatile unsigned int *) 0x40003850 &= 0x0;
	subWriteReg(0x400240F0, 15, 0, 0);
	subWriteReg(0x400240C0, 19, 2, 0);
}


void __attribute__((used)) hal_KSCAN_IRQHandler()
{		
   // LOG("key irq\n\r");
   if(reScan_flag==1)
   	reScan_flag=0;

   
    osal_stop_timerEx(m_kscanCtx.kscan_task_id, m_kscanCtx.timeout_event);

	uint16_t key_nxt[18];
	get_key_matrix(key_nxt);
	if(m_kscanCtx.cfg.ghost_key_state == IGNORE_GHOST_KEY){
		rmv_ghost_key(key_nxt);
	}

	if(m_kscanCtx.cfg.evt_handler){
		kscan_Evt_t evt = kscan_compare_key(m_kscanCtx.key_state, key_nxt);
		
		if(evt.num>0)
		m_kscanCtx.cfg.evt_handler(&evt);
    }
	
	memcpy(m_kscanCtx.key_state, key_nxt, sizeof(uint16_t)*18);
	
	CLEAR_KEY_PRESSED;
	CLEAR_INTERUPT;
   osal_start_timerEx(m_kscanCtx.kscan_task_id, m_kscanCtx.timeout_event, (m_kscanCtx.cfg.interval+TIMEOUT_DELTA));

}


void hal_kscan_timeout_handler()
{	
	
    if(reScan_flag==0)
    {
   // LOG("kscan_reScan\n\r");
     hal_kscan_clear_config();
     reScan_flag=1;
	 kscan_hw_config();
	 osal_start_timerEx(m_kscanCtx.kscan_task_id, m_kscanCtx.timeout_event, (m_kscanCtx.cfg.interval+TIMEOUT_DELTA));
	}
	else if(reScan_flag==1)
	{
	//LOG("kscan_timeout_handler\n\r");
     osal_stop_timerEx(m_kscanCtx.kscan_task_id, m_kscanCtx.timeout_event);

	 uint16_t key_nxt[18];
	 memset(&key_nxt[0],0,sizeof(uint16_t)*18); //all register must be 0.teedy add 2019/01/23
	
	//get_key_matrix(key_nxt); //no need to read the register,because keyScan didn't update the register .teedy add 2019/01/23
	
	if(m_kscanCtx.cfg.ghost_key_state == IGNORE_GHOST_KEY){
		rmv_ghost_key(key_nxt);
	}
	
	if(m_kscanCtx.cfg.evt_handler){
      kscan_Evt_t evt = kscan_compare_key(m_kscanCtx.key_state, key_nxt);
      m_kscanCtx.cfg.evt_handler(&evt);
    }	
	memcpy(m_kscanCtx.key_state, key_nxt, sizeof(uint16_t)*18);
	
    reScan_flag=0;
	hal_pwrmgr_unlock(MOD_KSCAN);

	}
	
}


static void kscan_hw_config(void)
{
	hal_kscan_clear_config();
	
	kscan_Cfg_t* cfg = &(m_kscanCtx.cfg);
	for(uint8_t i=0;i<NUM_KEY_ROWS;i++)
		hal_kscan_config_row(cfg->key_rows[i]);
	
	for(uint8_t i=0;i<NUM_KEY_COLS;i++)
		hal_kscan_config_col(cfg->key_cols[i]);
	
	SET_MULTI_KEY_STATE(NOT_IGNORE_MULTI_KEY);
	SET_POLARITY(SENCE_HIGH);
	SET_INTERVAL(cfg->interval);
	ENABLE_AUTO_SCAN;
	
	NVIC_SetPriority((IRQn_Type)KSCAN_IRQ, IRQ_PRIO_HAL);
	KSCAN_IRQ_ENABLE;
	ENABLE_KSCAN;
}


/**************************************************************************************
 * @fn          hal_kscan_config_row
 *
 * @brief       This function process for setting key row pin
 *
 * input parameters
 *
 * @param       KSCAN_ROWS_e row
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
 static void hal_kscan_config_row(KSCAN_ROWS_e row){
	
	GPIO_Pin_e row_pin = (GPIO_Pin_e)KSCAN_ROW_GPIO[row];
	
	hal_gpio_fmux(row_pin, Bit_DISABLE);
	hal_gpio_pull_set(row_pin, PULL_DOWN);
	
	EN_MUX_KSCAN_ROW(KSCAN_ROW_MK[row]);
	CONFIG_KEY_ROW(KSCAN_ROW_MK[row]);
}


/**************************************************************************************
 * @fn          hal_kscan_config_col
 *
 * @brief       This function process for setting key scan col pin
 *
 * input parameters
 *
 * @param       KSCAN_COLS_e col
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
 static void hal_kscan_config_col(KSCAN_COLS_e col){

	GPIO_Pin_e col_pin = (GPIO_Pin_e)KSCAN_COL_GPIO[col];
	
	if(col == KEY_COL_P16 || col == KEY_COL_P17){
		hal_gpio_cfg_analog_io(col_pin, Bit_DISABLE);
	}
	
	hal_gpio_fmux(col_pin, Bit_DISABLE);
	hal_gpio_pull_set(col_pin, PULL_DOWN);
	
	EN_MUX_KSCAN_COL(KSCAN_COL_MK[col]);
	CONFIG_KEY_COL(KSCAN_COL_MK[col]);
}


static void kscan_sleep_handler(void)
{	
	hal_kscan_clear_config();
	
	IO_Wakeup_Pol_e pol;
	for(uint8_t i=0;i<NUM_KEY_COLS;i++){	
		GPIO_Pin_e col_pin = (GPIO_Pin_e)KSCAN_COL_GPIO[m_kscanCtx.cfg.key_cols[i]];
		
		DIS_MUX_KSCAN_COL(KSCAN_COL_MK[m_kscanCtx.cfg.key_cols[i]]);
		hal_gpio_pin_init(col_pin, IE);
		hal_gpio_pull_set(col_pin, STRONG_PULL_UP);
	}
	
	for(uint8_t i=0;i<NUM_KEY_ROWS;i++){
		GPIO_Pin_e row_pin = (GPIO_Pin_e)KSCAN_ROW_GPIO[m_kscanCtx.cfg.key_rows[i]];
		
		DIS_MUX_KSCAN_ROW(KSCAN_ROW_MK[m_kscanCtx.cfg.key_rows[i]]);
	    hal_gpio_pull_set(row_pin, PULL_DOWN);
		hal_gpio_pin_init(row_pin, IE);
		pol = hal_gpio_read(row_pin) ? NEGEDGE:POSEDGE;

		hal_gpio_wakeup_set(row_pin, pol);
		m_kscanCtx.pin_state[i] = pol;
	}
}


static void kscan_wakeup_handler(void)
{
	for(uint8_t i=0;i<NUM_KEY_COLS;i++){
		GPIO_Pin_e col_pin = (GPIO_Pin_e)KSCAN_COL_GPIO[m_kscanCtx.cfg.key_cols[i]];
		
		hal_gpio_pin_init(col_pin, IE);
		hal_gpio_pull_set(col_pin, STRONG_PULL_UP);
	}
	
	for(uint8_t i=0;i<NUM_KEY_ROWS;i++){
		GPIO_Pin_e row_pin = (GPIO_Pin_e)KSCAN_ROW_GPIO[m_kscanCtx.cfg.key_rows[i]];

		hal_gpio_pin_init(row_pin, IE);
	    hal_gpio_pull_set(row_pin, PULL_DOWN);//teddy add 20190122
	}
	
	for(uint8_t i=0;i<NUM_KEY_ROWS;i++){
		
		GPIO_Pin_e row_pin = (GPIO_Pin_e)KSCAN_ROW_GPIO[m_kscanCtx.cfg.key_rows[i]];
		hal_gpio_pin_init(row_pin, IE);
		
		IO_Wakeup_Pol_e pol = hal_gpio_read(row_pin) ? POSEDGE:NEGEDGE;
		
		if(pol == m_kscanCtx.pin_state[i]){
			break;
		}
		else if(i == NUM_KEY_ROWS-1){
			return;
		}
	}
	
	hal_pwrmgr_lock(MOD_KSCAN);

	for(uint8_t i=0;i<NUM_KEY_COLS;i++)  //teddy add 20190122
	{
		GPIO_Pin_e col_pin = (GPIO_Pin_e)KSCAN_COL_GPIO[m_kscanCtx.cfg.key_cols[i]];

		hal_gpio_pin_init(col_pin, IE);
		hal_gpio_pull_set(col_pin, PULL_DOWN);
	}
	
	kscan_hw_config();
    reScan_flag=0;
	osal_start_timerEx(m_kscanCtx.kscan_task_id, m_kscanCtx.timeout_event, (m_kscanCtx.cfg.interval + TIMEOUT_DELTA));

}


/**************************************************************************************
 * @fn          get_key_matrix
 *
 * @brief       This function process for reading key row and col
 *
 * input parameters
 *
 * @param       uint16_t* key_matrix
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void get_key_matrix(uint16_t* key_matrix){

	for(uint8_t i=0; i<9; i++){
		
		uint16_t low  = (read_reg(MULTI_KEY_READ_ADDR + 4*i) & 0x0000FFFF);
		uint16_t high = (read_reg(MULTI_KEY_READ_ADDR + 4*i) & 0xFFFF0000) >> 16;
		
		key_matrix[i*2] = low;
		key_matrix[i*2+1] = high;
	}
}


/**************************************************************************************
 * @fn          rmv_ghost_key
 *
 * @brief       This function process for removing ghost key
 *
 * input parameters
 *
 * @param       uint16_t* key_matrix
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void rmv_ghost_key(uint16_t* key_matrix){

	uint16_t mix_final = 0;
	for (uint8_t i=0; i<18; ++i){
		for (uint8_t j=i+1; j<18; ++j){

			uint16_t mix = key_matrix[i] & key_matrix[j];
			uint8_t bit_is_pow2 = (mix&(mix-1)) == 0;
			if (mix && !bit_is_pow2){
				
				mix_final |= mix;
			}
		}
		key_matrix[i] &= ~mix_final;
	}
}


static kscan_Evt_t kscan_compare_key(uint16_t* key_pre, uint16_t* key_nxt)
{
	uint16_t multi_key_num = 0;
	
	for(uint8_t i=0; i<18; i++){

		uint16_t chg_key = key_pre[i] ^ key_nxt[i];
		uint16_t key_sta = chg_key & key_nxt[i];
		
		if(chg_key != 0){
			
			for(uint8_t j=0; j<16; j++){
				if((chg_key & BIT(j)) != 0){
					kscan_Key_t key_param;
					key_param.row = j;
					key_param.col = i;
					key_param.type = (key_sta & BIT(j)) ? KEY_PRESSED:KEY_RELEASED;
					m_keys[multi_key_num] = key_param;
					multi_key_num++;
				}
				if(multi_key_num == MAX_KEY_NUM){
					break;
				}
			}
		}
	}
	kscan_Evt_t evt;
	evt.keys = m_keys;
	evt.num  = multi_key_num;
	return evt;
}
