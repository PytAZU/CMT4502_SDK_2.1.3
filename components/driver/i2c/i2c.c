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
* @file		i2c.c
* @brief	Contains all functions support for i2c driver
* @version	0.0
* @date		25. Oct. 2017
* @author	CMOSTEK team
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/

/*******************************************************************************
*@ Module    		:  pre-compiler
*@ Description    :  NULL
*******************************************************************************/
#define _I2C_CMD_

/*******************************************************************************
*@ Module    			:  Includes
*@ Description    :  None
*******************************************************************************/
#include "types.h"
#include "hal_mcu.h"
#include "i2c.h"
#include "clock.h"
#include "log.h"
#include "error.h"
#include "OSAL.h"
#include "pwrmgr.h"

#define I2C_OP_TIMEOUT  100   //100ms for an Byte operation

extern  uint32_t pclk;


/**************************************************************************************
 * @fn          hal_master_send_read_cmd
 *
 * @brief       This function process for master send read command;It's vaild when the chip act as master
 *
 * input parameters
 *
 * @param       uint8_t len: read length
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void hal_master_send_read_cmd(void* pi2c, uint8_t len){
  uint8_t i;
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
	for(i=0;i<len;i++){
		I2C_READ_CMD(pi2cdev);
	}
}

static void _hal_i2c_send_byte(void* pi2c, uint8_t data)
{
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  pi2cdev->IC_DATA_CMD = data;  //data
}

/**************************************************************************************
 * @fn          hal_i2c_send
 *
 * @brief       This function process for send a serial(programe length) data by i2c interface
 *
 * input parameters
 *
 * @param       unsigned char* str: send data(string)
 *              uint32_t len: send length
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int hal_i2c_send(void* pi2c, uint8_t* str,uint8_t len)
{
  uint8_t i;
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  for(i=0;i<len;i++){
		_hal_i2c_send_byte(pi2c, str[i]);
	}
	return PPlus_SUCCESS;
}


int hal_i2c_wait_tx_completed(void* pi2c)
{
  int cnt = 0;
  //uint32 st0, st;
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  I2C_INIT_TOUT(to);
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  //st0 = pi2cdev->IC_INTR_STAT;
  //st = pi2cdev->IC_RAW_INTR_STAT;
  while(1){
    cnt++;
    if(pi2cdev->IC_RAW_INTR_STAT&0x200)//check tx empty
      break;

    I2C_CHECK_TOUT(to, I2C_OP_TIMEOUT, "hal_i2c_wait_tx_completed TO\n");
  }
  //LOG("I2c Tx empty:%x, %x\n", st0,st);
  return PPlus_SUCCESS;
}


void* hal_i2c_init(i2c_dev_t dev, I2C_CLOCK_e i2c_clock_rate)
{
  AP_I2C_TypeDef * pi2cdev = NULL;
  if(dev == I2C_0){
    pi2cdev = AP_I2C0;
    clk_gate_enable(MOD_I2C0);
  }
  else if(dev == I2C_1){
    pi2cdev = AP_I2C1;
    clk_gate_enable(MOD_I2C1);
  }
  else{
    return NULL;
  }

  pi2cdev->IC_ENABLE=0;
	pi2cdev->IC_CON=0x61;
  if(i2c_clock_rate==I2C_CLOCK_100K){
    pi2cdev->IC_CON= ((pi2cdev->IC_CON) & 0xfffffff9)|(0x01 << 1);
    if(pclk==16000000){
      pi2cdev->IC_SS_SCL_HCNT=70;  //16
      pi2cdev->IC_SS_SCL_LCNT=76;  //32)
    }else if(pclk==32000000){
      pi2cdev->IC_SS_SCL_HCNT=148;  //16
      pi2cdev->IC_SS_SCL_LCNT=154;  //32)
    }else if(pclk==48000000){
      pi2cdev->IC_SS_SCL_HCNT=230;  //16
      pi2cdev->IC_SS_SCL_LCNT=236;  //32)
    }else if(pclk==64000000){
      pi2cdev->IC_SS_SCL_HCNT=307;  //16
      pi2cdev->IC_SS_SCL_LCNT=320;  //32)
    }else if(pclk==96000000){
      pi2cdev->IC_SS_SCL_HCNT=460;  //16
      pi2cdev->IC_SS_SCL_LCNT=470;  //32)
    }
  }else if(i2c_clock_rate==I2C_CLOCK_400K){
    pi2cdev->IC_CON= ((pi2cdev->IC_CON) & 0xfffffff9)|(0x02 << 1);
    if(pclk==16000000){
      pi2cdev->IC_FS_SCL_HCNT=10;  //16
      pi2cdev->IC_FS_SCL_LCNT=17;  //32)
    }else if(pclk==32000000){
      pi2cdev->IC_FS_SCL_HCNT=30;  //16
      pi2cdev->IC_FS_SCL_LCNT=35;  //32)
    }else if(pclk==48000000){
      pi2cdev->IC_FS_SCL_HCNT=48;  //16
      pi2cdev->IC_FS_SCL_LCNT=54;  //32)
    }else if(pclk==64000000){
      pi2cdev->IC_FS_SCL_HCNT=67;  //16
      pi2cdev->IC_FS_SCL_LCNT=75;  //32)
    }else if(pclk==96000000){
      pi2cdev->IC_FS_SCL_HCNT=105;  //16
      pi2cdev->IC_FS_SCL_LCNT=113;  //32)
    }
  }

	pi2cdev->IC_TAR = I2C_MASTER_ADDR_DEF;
	pi2cdev->IC_INTR_MASK=0;
	pi2cdev->IC_RX_TL=0x0;
	pi2cdev->IC_TX_TL=0x1;
		
  pi2cdev->IC_ENABLE=1;
  return (void*)pi2cdev;
}


int hal_i2c_deinit(void* pi2c){
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev == AP_I2C0){
    pi2cdev->IC_ENABLE=0;
    clk_gate_disable(MOD_I2C0);
  }
  else if(pi2cdev == AP_I2C1){
    pi2cdev->IC_ENABLE=0;
    clk_gate_enable(MOD_I2C1);
  }
  else{
    return PPlus_ERR_INVALID_PARAM;
  }
  return PPlus_SUCCESS;
}


/**************************************************************************************
 * @fn          hal_i2c_pin_init
 *
 * @brief       This function process for i2c pin initial(2 lines);You can use two i2c,i2c0 and i2c1,should programe by USE_AP_I2CX 
 *
 * input parameters
 *
 * @param       GPIO_Pin_e pin_sda: define sda_pin
 *              GPIO_Pin_e pin_clk: define clk_pin
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int hal_i2c_pin_init(i2c_dev_t dev, GPIO_Pin_e pin_sda, GPIO_Pin_e pin_clk){
  if(dev == I2C_0){
		hal_gpio_fmux_set(pin_clk, IIC0_SCL);
		hal_gpio_fmux_set(pin_sda, IIC0_SDA);
  }
  else if(dev == I2C_1){
		hal_gpio_fmux_set(pin_clk, IIC1_SCL);
		hal_gpio_fmux_set(pin_sda, IIC1_SDA);
  }
  else{
    return PPlus_ERR_INVALID_PARAM;
  }
	hal_gpio_pull_set(pin_sda,STRONG_PULL_UP);
	hal_gpio_pull_set(pin_clk,STRONG_PULL_UP);
	return PPlus_SUCCESS;
}

int hal_i2c_tx_start(void* pi2c)
{
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  pi2cdev->IC_ENABLE=1;
  return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_i2c_addr_update
 *
 * @brief       This function process for tar update 
 *
 * input parameters
 *
 * @param       uint8_t addr: address
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int hal_i2c_addr_update(void* pi2c, uint8_t addr){
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  pi2cdev->IC_ENABLE=0;
  pi2cdev->IC_TAR = addr;
  pi2cdev->IC_ENABLE=0;
  return PPlus_SUCCESS;
}


int _hal_i2c_read_s(void* pi2c, uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t size)
{
  I2C_INIT_TOUT(to);
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  hal_i2c_addr_update(pi2c, slave_addr);
  
  HAL_ENTER_CRITICAL_SECTION();
  hal_i2c_tx_start(pi2c);
  _hal_i2c_send_byte(pi2c, reg);
  hal_master_send_read_cmd(pi2c, size);
  HAL_EXIT_CRITICAL_SECTION();
  
  while(1){
    if(I2C_RX_FIFO_NOT_EMPTY(pi2cdev)){
      *data = (pi2cdev->IC_DATA_CMD&0xff);
      data++;
      size --;
      if(size == 0)
        break;
    }
    I2C_CHECK_TOUT(to, I2C_OP_TIMEOUT*size, "I2C RD TO\n");
  }
	return PPlus_SUCCESS;

}



int hal_i2c_read(
  void* pi2c,
  uint8_t slave_addr,
  uint8_t reg,
  uint8_t* data,
  uint8_t size)
{
  uint8_t cnt;
  int ret = PPlus_SUCCESS;
  AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
  if(pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1){
    return PPlus_ERR_INVALID_PARAM;
  }
  while(size){
    cnt = (size >7) ? 7 : size;
    size -= cnt;
    ret = _hal_i2c_read_s(pi2c, slave_addr, reg, data , cnt);
    if(ret != PPlus_SUCCESS)
      break;
    data += cnt;
  }
	return ret;
}



