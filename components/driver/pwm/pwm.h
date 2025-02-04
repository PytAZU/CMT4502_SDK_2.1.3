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
* @file		pwm.h
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __PWM__H__
#define __PWM__H__

#include "types.h"
#include "gpio.h"
#include "common.h"

#define    PWM_CH_BASE             	0x4000E004UL
#define    PWM_CTL0_ADDR(n)        	(PWM_CH_BASE + n*12)
#define    PWM_CTL1_ADDR(n)        	(PWM_CH_BASE + 4 + n*12)

#define    PWM_ENABLE_ALL          	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(0);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(4);\
									}while(0)
#define    PWM_DISABLE_ALL 			do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(0);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(4);\
									}while(0)
#define    PWM_ENABLE_CH_012      	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(8);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(9);\
									}while(0)
#define    PWM_DISABLE_CH_012       do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(8);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(9);\
									}while(0)
#define    PWM_ENABLE_CH_345    	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(10);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(11);\
									}while(0)
#define    PWM_DISABLE_CH_345      	do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(10);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(11);\
									}while(0)
#define    PWM_ENABLE_CH_01        	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(12);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(13);\
									}while(0)
#define    PWM_DISABLE_CH_01       	do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(12);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(13);\
									}while(0)
#define    PWM_ENABLE_CH_23        	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(14);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(15);\
									}while(0)
#define    PWM_DISABLE_CH_23       	do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(14);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(15);\
									}while(0)
#define    PWM_ENABLE_CH_45        	do{\
										*(volatile unsigned int *) 0x4000e000 |= BIT(16);\
										*(volatile unsigned int *) 0x4000e000 |= BIT(17);\
									}while(0)
#define    PWM_DISABLE_CH_45    	do{\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(16);\
										*(volatile unsigned int *) 0x4000e000 &= ~BIT(17);\
									}while(0)

						
#define    PWM_INSTANT_LOAD_CH(n)  	 subWriteReg(PWM_CTL0_ADDR(n),31,31,1)
#define    PWM_NO_INSTANT_LOAD_CH(n) subWriteReg(PWM_CTL0_ADDR(n),31,31,0)
#define    PWM_LOAD_CH(n) 			     subWriteReg(PWM_CTL0_ADDR(n),16,16,1)
#define    PWM_NO_LOAD_CH(n) 		     subWriteReg(PWM_CTL0_ADDR(n),16,16,0)																 
#define    PWM_SET_DIV(n,v) 		     subWriteReg(PWM_CTL0_ADDR(n),14,12,v)
#define    PWM_SET_MODE(n,v) 	   	   subWriteReg(PWM_CTL0_ADDR(n),8,8,v)
#define    PWM_SET_POL(n,v) 		     subWriteReg(PWM_CTL0_ADDR(n),4,4,v)
#define    PWM_ENABLE_CH(n)        	 subWriteReg(PWM_CTL0_ADDR(n),0,0,1)
#define    PWM_DISABLE_CH(n)       	 subWriteReg(PWM_CTL0_ADDR(n),0,0,0)
									
#define    PWM_SET_CMP_VAL(n,v) 	   subWriteReg(PWM_CTL1_ADDR(n),31,16,v)					
#define    PWM_SET_TOP_VAL(n,v) 	   subWriteReg(PWM_CTL1_ADDR(n),15,0,v)
#define    PWM_GET_CMP_VAL(n) 	   	 ((read_reg(PWM_CTL1_ADDR(n)) & 0xFFFF0000) >> 8)
#define    PWM_GET_TOP_VAL(n) 	   	 read_reg(PWM_CTL1_ADDR(n)) & 0x0000FFFF


/*************************************************************
*	@brief		enum variable, the number of PWM channels supported
*
*/
typedef enum{
	
	PWM_CH0 = 0,
	PWM_CH1 = 1,
	PWM_CH2 = 2,
	PWM_CH3 = 3,
	PWM_CH4 = 4,
	PWM_CH5 = 5
	
}PWMN_e;

/*************************************************************
*	@brief		enum variable used for PWM clock prescaler
*
*/
typedef enum{
	
	PWM_CLK_NO_DIV = 0,
	PWM_CLK_DIV_2 = 1,
	PWM_CLK_DIV_4 = 2,
	PWM_CLK_DIV_8 = 3,
	PWM_CLK_DIV_16 = 4,
	PWM_CLK_DIV_32 = 5,
	PWM_CLK_DIV_64 = 6,
	PWM_CLK_DIV_128 = 7
	
}PWM_CLK_DIV_e;

/*************************************************************
*	@brief		enum variable used for PWM work mode setting
*
*/
typedef enum{
	
	PWM_CNT_UP = 0,
	PWM_CNT_UP_AND_DOWN = 1
	
}PWM_CNT_MODE_e;

/*************************************************************
*	@brief		enum variable used for PWM output polarity setting
*
*/
typedef enum{
	
	PWM_POLARITY_RISING = 0,
	PWM_POLARITY_FALLING = 1
	
}PWM_POLARITY_e;

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
void hal_pwm_init(PWMN_e pwmN, PWM_CLK_DIV_e pwmDiv,
	PWM_CNT_MODE_e pwmMode, PWM_POLARITY_e pwmPolarity);

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
void hal_pwm_open_channel(PWMN_e pwmN,GPIO_Pin_e pwmPin);

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
void hal_pwm_close_channel(PWMN_e pwmN);

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
void hal_pwm_destroy(PWMN_e pwmN);

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
void hal_pwm_set_count_val(PWMN_e pwmN, uint16_t cmpVal, uint16_t cntTopVal);

void hal_pwm_start(void);
void hal_pwm_stop(void);

//new api,make use easily
typedef struct
{  
	PWMN_e pwmN;
	GPIO_Pin_e pwmPin;
	PWM_CLK_DIV_e pwmDiv;
	PWM_CNT_MODE_e pwmMode;
	PWM_POLARITY_e pwmPolarity;
	uint16_t cmpVal;
	uint16_t cntTopVal;
  
} pwm_ch_t;

/**************************************************************************************
 * @fn          hal_pwm_module_init
 *
 * @brief       init pwm global variables
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
void hal_pwm_module_init(void);

/**************************************************************************************
 * @fn          hal_pwm_module_deinit
 *
 * @brief       deinit pwm global variables
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
void hal_pwm_module_deinit(void);

/**************************************************************************************
 * @fn          hal_spi_pin_init
 *
 * @brief       config and make a pwm start to work 
 *
 * input parameters
 *
 * @param       pwm_ch_t ch: pwm channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_ch_start(pwm_ch_t ch);

/**************************************************************************************
 * @fn          hal_pwm_ch_stop
 *
 * @brief       make a pwm stop form working
 *
 * input parameters
 *
 * @param       pwm_ch_t ch: pwm channel
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_pwm_ch_stop(pwm_ch_t ch);

#endif
