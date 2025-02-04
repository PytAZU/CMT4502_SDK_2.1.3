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
* @file		qdec.h
* @brief	Contains all functions support for key scan driver
* @version	0.0
* @date		13. Nov. 2017
* @author	Ding
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __QDEC__H__
#define __QDEC__H__

#include "types.h"
#include "gpio.h"

#define		QDEC_IRQ_ENABLE 			*(volatile unsigned int *) 0xe000e100 |= BIT(30)

#define		ENABLE_CHN(n)		    	*(volatile unsigned int *) 0x4000B000 |= BIT(4*n)
#define		DISABLE_CHN(n)		    	*(volatile unsigned int *) 0x4000B000 &= ~BIT(4*n)
	
#define		EN_INT_QUAN(n)		    	*(volatile unsigned int *) 0x4000B004 |= BIT(8*n)
#define		EN_INT_INCN(n)				*(volatile unsigned int *) 0x4000B004 |= BIT(4+8*n)
#define		DIS_INT_QUAN(n)		    	*(volatile unsigned int *) 0x4000B004 &= ~BIT(8*n)
#define		DIS_INT_INCN(n)				*(volatile unsigned int *) 0x4000B004 &= ~BIT(4+8*n)
#define		SET_INT_MODE_QUAN(chn,mod)	subWriteReg(0x4000B004,2+8*chn,2+8*chn,mod)
#define		SET_INT_MODE_INCN(chn,mod)	subWriteReg(0x4000B004,6+8*chn,6+8*chn,mod)

#define		EN_INT_F20_QUAN(n)		    *(volatile unsigned int *) 0x4000B004 |= BIT(24+2*n)
#define		EN_INT_02F_QUAN(n)	    	*(volatile unsigned int *) 0x4000B004 |= BIT(25+2*n)
#define		DIS_INT_F20_QUAN(n)			*(volatile unsigned int *) 0x4000B004 &= ~BIT(24+2*n)
#define		DIS_INT_02F_QUAN(n)			*(volatile unsigned int *) 0x4000B004 &= ~BIT(25+2*n)
	
#define		CLR_INT_F20_QUAN(n)			*(volatile unsigned int *) 0x4000B008 |= BIT(24+2*n)
#define		CLR_INT_02F_QUAN(n)			*(volatile unsigned int *) 0x4000B008 |= BIT(25+2*n)
#define		CLR_INT_QUAN(n)		    	*(volatile unsigned int *) 0x4000B008 |= BIT(8*n)	
#define		CLR_INT_INCN(n)		    	*(volatile unsigned int *) 0x4000B008 |= BIT(4+8*n)
	
#define		STATUS_INT_QUAN(n)		    read_reg(0x4000B00C) & BIT(8*n)
#define		STATUS_INT_INCN(n)		    read_reg(0x4000B00C) & BIT(4+8*n)
#define		STATUS_INT_F20_QUAN(n)	    read_reg(0x4000B00C) & BIT(24+2*n)
#define		STATUS_INT_02F_QUAN(n)	    read_reg(0x4000B00C) & BIT(25+2*n)

#define		SET_MODE_QUAN(chn,mod)	    subWriteReg(0x4000B010 + 0x14*chn,1,0, mod)
#define		SET_MODE_INCN(chn,mod)	    subWriteReg(0x4000B010 + 0x14*chn,17,16,mod)
#define		SET_HIT_QUAN(chn,mod)	    write_reg(0x4000B014 + 0x14*chn,mod)
#define		SET_HIT_INCN(chn,mod)	    write_reg(0x4000B018 + 0x14*chn,mod)
#define		GET_CNT_QUAN(n)	    		read_reg(0x4000B01C + 0x14*n)
#define		GET_CNT_INCN(n)	    		read_reg(0x4000B020 + 0x14*n)

/*************************************************************
*	@brief		enum variable used for setting channel
*
*/
typedef enum{

	QDEC_CHX   =   0,
	QDEC_CHY   =   1, 
	QDEC_CHZ   =   2
	
}QDEC_CHN_e;

/*************************************************************
*	@brief		enum variable used for setting quadrature count mode
*
*/
typedef enum{

	QDEC_MODE_1X = 1,
	QDEC_MODE_2X = 2,
	QDEC_MODE_4X = 3
	
}QDEC_QUA_MODE_e;

/*************************************************************
*	@brief		enum variable used for setting index count mode
*
*/
typedef enum{

	HIGH_LEVEL = 0,
	POS_EDGE = 1,
	NEG_EDGE = 2,
	POS_OR_NEG_EDGE = 3,
	
}QDEC_INC_MODE_e;

/*************************************************************
*	@brief		enum variable used for setting interupt mode
*
*/
typedef enum{

	INT_BY_CHANGE = 0,
	INT_BY_HIT = 1
	
}QDEC_INT_MODE_e;

typedef struct {
	int32_t		 	count; 
} qdec_Evt_t;

typedef void (*qdec_Hdl_t)(qdec_Evt_t* pev);

typedef struct {
	GPIO_Pin_e		cha_pin;
	GPIO_Pin_e		chb_pin;
	QDEC_CHN_e		qdec_chn;
	QDEC_QUA_MODE_e	quaMode;
	QDEC_INT_MODE_e	intMode;
	qdec_Hdl_t  	evt_handler;
	bool			use_inc;
	bool			use_inc_irq;
	GPIO_Pin_e 		chi_pin;
	QDEC_INC_MODE_e incMode;
} qdec_Cfg_t;

typedef struct {
	bool        enable;
	qdec_Cfg_t 	cfg;
	int32_t		count;
	uint8_t		pin_state[3];
	GPIO_Pin_e	pin_arr[3];
	uint8_t 	qdec_task_id;
	uint16_t	timeout_event;
} qdec_Ctx_t;

static void qdec_hw_config(void);
static void qdec_sleep_handler(void);
static void qdec_wakeup_handler(void);
static void hal_qdec_set_cha(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin);
static void hal_qdec_set_chb(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin);
static void hal_qdec_set_chi(QDEC_CHN_e qdecCHN,GPIO_Pin_e pin);
static void hal_qdec_set_qua_irq(QDEC_CHN_e chn, QDEC_INT_MODE_e intMode);
static void hal_qdec_set_inc_irq(QDEC_CHN_e chn, QDEC_INC_MODE_e incMode, QDEC_INT_MODE_e intMode);
	
int hal_qdec_init(qdec_Cfg_t cfg, uint8 task_id, uint16 event);
void hal_qdec_timeout_handler(void);
void hal_qdec_IRQHandler(void);

#endif
