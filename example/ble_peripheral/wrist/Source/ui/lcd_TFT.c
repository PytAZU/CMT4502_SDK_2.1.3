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


#if(CFG_DISP==DISP_TFT)

#include  "OSAL.h"
#include  "gpio.h"
#include  "error.h"
#include  "pwrmgr.h"
#include  "lcd_TFT.h"
#include  "spi.h"
#include  "common.h"
#include  "log.h"

extern void pad_ds_control(GPIO_Pin_e pin, BitAction_e value);

static hal_spi_t lcd_spi_handle = {SPI0};

//lcd reset
static void Reset(void)
{
  hal_gpio_write(RESET_PIN,0);
  WaitMs(100);
  hal_gpio_write(RESET_PIN,1);
  WaitMs(100);
}

//write cmd to lcd
static void  write_cmd(uint8_t Data)
{
  hal_gpio_write(DC_PIN,0);
  hal_spi_send_byte(&lcd_spi_handle, Data);
}

//write uint8 data to lcd
static void  write_data(uint8_t Data)
{ 
  hal_gpio_write(DC_PIN,1);
  hal_spi_send_byte(&lcd_spi_handle, Data);
}

//lcd initial
static void lcd_config(void){
  Reset();//Reset before LCD Init.
  
  //LCD Init For 1.44Inch LCD Panel with ST7735R.
  write_cmd(0x36); 
  write_data(0x00);

  write_cmd(0x3A); 
  write_data(0x05);

  write_cmd(0xB2);
  write_data(0x0C);
  write_data(0x0C);
  write_data(0x00);
  write_data(0x33);
  write_data(0x33);

  write_cmd(0xB3);
  write_data(0x10);
  write_data(0x0f);
  write_data(0x0f);

  write_cmd(0xB7); 
  write_data(0x35);  

  write_cmd(0xBB);
  write_data(0x19);

  write_cmd(0xC0);
  write_data(0x2C);

  write_cmd(0xC2);
  write_data(0x01);

  write_cmd(0xC3);
  write_data(0x12);   

  write_cmd(0xC4);
  write_data(0x20);  

  write_cmd(0xC6); 
  write_data(0x0F);    

  write_cmd(0xD0); 
  write_data(0xA4);
  write_data(0xA1);

  write_cmd(0xE0);
  write_data(0xD0);
  write_data(0x04);
  write_data(0x0D);
  write_data(0x11);
  write_data(0x13);
  write_data(0x2B);
  write_data(0x3F);
  write_data(0x54);
  write_data(0x4C);
  write_data(0x18);
  write_data(0x0D);
  write_data(0x0B);
  write_data(0x1F);
  write_data(0x23);

  write_cmd(0xE1);
  write_data(0xD0);
  write_data(0x04);
  write_data(0x0C);
  write_data(0x11);
  write_data(0x13);
  write_data(0x2C);
  write_data(0x3F);
  write_data(0x44);
  write_data(0x51);
  write_data(0x2F);
  write_data(0x1F);
  write_data(0x1F);
  write_data(0x20);
  write_data(0x23);

  write_cmd(0x21); 

  write_cmd(0x11); 
  WaitMs(100);

  write_cmd(0x29); 
  
}

#define X_OFFSET 0
#define Y_OFFSET 0
static void set_region(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end)
{
  write_cmd(0x2a);
  hal_spi_TxComplete(&lcd_spi_handle);
  //hal_gpio_write(DC_PIN,1);
  write_data(0);
  write_data(x_start);
  write_data(0);
  write_data(x_end);
  hal_spi_TxComplete(&lcd_spi_handle);

  write_cmd(0x2b);
  hal_spi_TxComplete(&lcd_spi_handle);
  //hal_gpio_write(DC_PIN,1);
  write_data(0);
  write_data(y_start);
  write_data(0);
  write_data(y_end);
  hal_spi_TxComplete(&lcd_spi_handle);
  write_cmd(0x2c);
  hal_spi_TxComplete(&lcd_spi_handle);
  
}

int lcd_setscn_TFT(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
  uint8_t i,j;
  uint8_t xe = x+w-1, ye = y+ h-1;
  if(x > SCN_WIDTH-1 || y > SCN_HEIGHT-1)
    return PPlus_ERR_INVALID_PARAM;
  xe = (xe > SCN_WIDTH -1)? SCN_WIDTH -1 : xe;
  ye = (ye > SCN_HEIGHT-1)? SCN_HEIGHT -1 : ye;
  
  set_region(x, y, x+w-1, y+h-1);
  //write_cmd(0x2c);
  hal_gpio_write(DC_PIN,1);
	//hal_spi_send_buff(color, w*h);
  for(i = 0; i< w; i++){
    for(j = 0; j< h; j++){
      //if(((x + i)>xe)||((y+j)>ye))
      //  continue;
      hal_spi_send_byte(&lcd_spi_handle,(uint8_t)(color>>8));
      hal_spi_send_byte(&lcd_spi_handle, (uint8_t)color);
    }
  }
  hal_spi_TxComplete(&lcd_spi_handle);
  return 0;
}

int lcd_draw_TFT(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint16_t* data)
{
  uint8_t i,j;
  uint8_t xe = x+w-1, ye = y+ h-1;
  uint16_t val;
  if(x > SCN_WIDTH-1 || y > SCN_HEIGHT-1)
    return PPlus_ERR_INVALID_PARAM;
  xe = (xe > SCN_WIDTH -1)? SCN_WIDTH -1 : xe;
  ye = (ye > SCN_HEIGHT-1)? SCN_HEIGHT -1 : ye;
  
  set_region(x, y, x+w-1, y+h-1);
  //write_cmd(0x2c);
  hal_gpio_write(DC_PIN,1);
  for(j = 0; j< h; j++){
    for(i = 0; i< w; i++){
      if(((x + i)>xe)||((y+j)>ye))
        continue;
      val = data[j*w+i];
      hal_spi_send_byte(&lcd_spi_handle, (uint8_t)(val>>8));
      hal_spi_send_byte(&lcd_spi_handle, (uint8_t)val);
    }
  }
  hal_spi_TxComplete(&lcd_spi_handle);
  return 0;
}

void lcd_on_TFT(void)
{
  hal_gpio_pull_set(LIGHT_LEDK, PULL_DOWN);
  hal_gpio_pull_set(LIGHT_LEDA, PULL_DOWN);
  write_cmd(0x29);//Display on
}

void lcd_off_TFT(void)
{
  hal_gpio_pull_set(LIGHT_LEDK, PULL_DOWN);
  hal_gpio_pull_set(LIGHT_LEDA, STRONG_PULL_UP);
  write_cmd(0x28);//Display off
}

int lcd_bus_init(void)
{
  int ret;
  spi_Cfg_t cfg = {
      .sclk_pin = P24,
      .ssn_pin = P23,
      .MOSI = P15,
      .MISO = P32,
      .baudrate = 12000000,
      .spi_tmod = SPI_TRXD,
      .force_cs = TRUE,
      .int_mode = FALSE,
    };
  
    ret = hal_spi_bus_init(&lcd_spi_handle,cfg);//spi init
    hal_gpio_DS_control(cfg.sclk_pin,Bit_ENABLE);
    hal_gpio_DS_control(cfg.MOSI,Bit_ENABLE);
    LOG("lcd_bus_init:%d\n",ret);
    return ret;
}

int lcd_bus_deinit(void)
{
  hal_spi_bus_deinit(&lcd_spi_handle);
  return PPlus_SUCCESS;
}

int lcd_init(void){
  
  lcd_bus_init();
    
  lcd_config();
  lcd_setscn_TFT(0,0,160,80,0);

	lcd_off_TFT();
  //lcd_bus_deinit();
    
  return PPlus_SUCCESS;
  }

#endif /*CFG_DISP=DISP_TFT*/



