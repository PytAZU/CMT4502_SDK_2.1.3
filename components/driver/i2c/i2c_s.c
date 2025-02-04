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


#include "ap_cp.h"
#include "gpio.h"
#include "hal_mcu.h"
#include "clock.h"
#include "i2c_s.h"
#include "i2c.h"
#include "error.h"
#include "log.h"

typedef struct{
  uint8_t           id;  //0: uninit, 1: i2c0, 2:i2c1
  uint8_t           mode; //(1)I2CS_MODE_REG_8BIT,  (2)I2CS_MODE_REG_16BIT,(3)I2CS_MODE_RAW
  uint8_t           saddr;
  GPIO_Pin_e        cs;  //cs pin, used to  wakeup or release
  GPIO_Pin_e        sda;
  GPIO_Pin_e        scl;
  AP_I2C_TypeDef*   dev;
  i2cs_hdl_t        evt_handler;
  uint8_t           rxoffset;
  uint8_t           rxbuf[I2CS_RX_MAX_SIZE];
  uint8_t           txoffset;
  uint8_t           txbuf[I2CS_TX_MAX_SIZE];
}i2cs_ctx_t;

static uint8_t s_i2cs_state = I2CSST_IDLE;

static i2cs_ctx_t s_i2cs_ctx;

static void i2cs_irq_rx_handler(AP_I2C_TypeDef* pdev)
{
  uint32_t val;
	while(1){
		if((pdev->IC_STATUS & BV(3)) == 0)
		  break;
		val = pdev->IC_DATA_CMD;
		LOG("Rx %x\n",val);
	}
}
static uint32_t tx_dummy = 0; 
static void i2cs_irq_tx_handler(AP_I2C_TypeDef* pdev)
{
  pdev->IC_DATA_CMD = tx_dummy &0xff;
	tx_dummy ++;
  pdev->IC_DATA_CMD = tx_dummy &0xff;
	tx_dummy ++;
  //LOG("Tx\n");
}
static void i2cs_irq_handler(AP_I2C_TypeDef* pdev)
{
  uint32_t int_status = pdev->IC_INTR_STAT;
	uint32_t clr = pdev->IC_CLR_INTR;
	//LOG("i2cs_irq_handler %x\n",int_status);
	if(int_status & I2C_MASK_START_DET)
	{
	  
	}
  if(int_status & I2C_MASK_RX_FULL)
  {
    i2cs_irq_rx_handler(pdev);
  }
  if(int_status & I2C_MASK_RD_REQ)
  {
    i2cs_irq_tx_handler(pdev);

  }
}

void __attribute__((used)) hal_I2C0_IRQHandler(void)
{
  i2cs_ctx_t* pctx = &s_i2cs_ctx;
  if(pctx->id == I2CS_0)
    i2cs_irq_handler(AP_I2C0);
}
void __attribute__((used)) hal_I2C1_IRQHandler(void)
{
  i2cs_ctx_t* pctx = &s_i2cs_ctx;
  if(pctx->id == I2CS_1)
    i2cs_irq_handler(AP_I2C1);
}



int i2cs_init(
  i2cs_channel_t  ch_id,
  i2cs_mode_t     mode,
  uint8_t         saddr,  //slave address
  GPIO_Pin_e      cs,  //if need not cs, choose GPIO_DUMMY_PIN
  GPIO_Pin_e      sda,
  GPIO_Pin_e      scl,
  i2cs_hdl_t      evt_handler)
{
  i2cs_ctx_t* pctx = &s_i2cs_ctx;
  Fmux_Type_e   fmux;
  MODULE_e      module;
  AP_I2C_TypeDef* pdev = NULL;
  int    irqid;
  if(pctx->id){
    return PPlus_ERR_IO_CONFILCT;
  }

  //parameter validate check


  //set device
  irqid = (ch_id == I2CS_0) ? I2C0_IRQ : I2C1_IRQ;
  module = (ch_id == I2CS_0) ? MOD_I2C0 : MOD_I2C1;
  clk_gate_enable(module);
  
  pdev = (ch_id == I2CS_0) ? AP_I2C0 : AP_I2C1;
	pctx->id = ch_id;
  pctx->dev = pdev;
  pctx->saddr = saddr;
  pctx->cs = cs;
  pctx->scl = scl;
  pctx->sda = sda;
  pctx->evt_handler = evt_handler;
  
  fmux = (ch_id == I2CS_0) ? IIC0_SCL : IIC1_SCL;
  hal_gpio_fmux_set(scl, fmux);
  
  fmux = (ch_id == I2CS_0) ? IIC0_SDA : IIC1_SDA;
  hal_gpio_fmux_set(sda, fmux);
  
  hal_gpio_pull_set(scl , STRONG_PULL_UP);
  hal_gpio_pull_set(sda , STRONG_PULL_UP);

  pdev->IC_ENABLE = 0; //disable
  pdev->IC_CON = (SPEED_FAST) << 1;
  pdev->IC_SAR = saddr;
  pdev->IC_RX_TL = 1;
  pdev->IC_TX_TL = 1;
  pdev->IC_INTR_MASK = 0xfef;//(I2C_MASK_TX_ABRT | I2C_MASK_RD_REQ | I2C_MASK_RX_FULL | I2C_MASK_RX_DONE);

  pdev->IC_ENABLE = 1; //disable

  NVIC_EnableIRQ((IRQn_Type)irqid);
		
  NVIC_SetPriority(irqid, IRQ_PRIO_HAL);
  
  return PPlus_SUCCESS;
}


int i2cs_deinit(void)
{
  i2cs_ctx_t* pctx = &s_i2cs_ctx;
  if(pctx->id == 0)
    return PPlus_ERR_IO_FAIL;

  //release io
	
	return PPlus_SUCCESS;
}

