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

/**************************************************************************************************
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "spiflash_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "spi.h"
#include "error.h"
#include "spiflash.h"
#include "pwrmgr.h"
//#include  "lcd_ST7789VW.h"

static uint8 spi_TaskID; 

/*********************************************************************
 * @fn      spi_demo_Init
 *
 * @brief   This function shows how to initialize a timer in OSAL.
 *
 * @param   
 *
 * @return  
 */
void spi_demo_Init( uint8 task_id ){
	spi_TaskID = task_id;
	osal_start_reload_timer( spi_TaskID, TIMER_1000MS_CYCLE , 1000);
}

/*********************************************************************
 * @fn      spi_demo_ProcessEvent
 *
 * @brief   This function shows how to answer the timerout event in OSAL.
 *
 * @param   task_id
 *
 * @return  none
 */
void spi_cb(spi_evt_t* evt)
{
	//LOG("(SPI finish:%d %d)	",evt.spi,evt.type);
}

hal_spi_t spi ={
	.spi_index = SPI0,
};

spi_Cfg_t spi_cfg = {
	.sclk_pin = GPIO_P34,
	.ssn_pin = GPIO_P31,
	.MOSI = GPIO_P32,
	.MISO = GPIO_P33,
	
	.baudrate = 12000000,//12M
	//.baudrate = 4000000,//4M
	.spi_tmod = SPI_TRXD,
	.spi_scmod = SPI_MODE3,
	
	.int_mode = false,
	.force_cs = true,
	.evt_handler = spi_cb,
};

#define BUF_LEN 512
uint8_t tx_buf[BUF_LEN];
uint8_t rx_buf[BUF_LEN];
uint8_t spi_tx_buf[BUF_LEN];

void spi_gd25q16_read_test_set_buffer(bool cs_force,uint32_t addr,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t transmit_len)
{
	uint16_t i;
	
	hal_setMem(tx_buf,0x00,transmit_len);
	hal_setMem(rx_buf,0x00,transmit_len);

	if(cs_force)
	{
		*(tx_buf+0) = FLASH_READ;
		*(tx_buf+1) = (addr>>16)&0xff;
		*(tx_buf+2) = (addr>>8)&0xff;
		*(tx_buf+3) = addr & 0xff;
	}
	else
	{
			for(i=0;i<transmit_len/4;i++)
			{
				*(tx_buf+0+i*8) = FLASH_READ;
				*(tx_buf+1+i*8) = (addr>>16)&0xff;
				*(tx_buf+2+i*8) = (addr>>8)&0xff;
				*(tx_buf+3+i*8) = addr & 0xff;
				addr += 4;
			}
	}
}

void spi_gd25q16_read_test_set_buffer_2(bool cs_force,uint32_t addr,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t transmit_len)
{
	uint16_t i;
	uint8_t rem;
	
	hal_setMem(tx_buf,0x00,transmit_len);
	hal_setMem(rx_buf,0x00,transmit_len);
	{
		rem = transmit_len%4;
		if(rem == 0)
		{
			for(i=0;i<transmit_len/4;i++)
			{
				*(tx_buf+0+i*8) = FLASH_READ;
				*(tx_buf+1+i*8) = (addr>>16)&0xff;
				*(tx_buf+2+i*8) = (addr>>8)&0xff;
				*(tx_buf+3+i*8) = addr & 0xff;
				addr += 4;
			}
		}
		else
		{
				*(tx_buf+0) = FLASH_READ;
				*(tx_buf+1) = (addr>>16)&0xff;
				*(tx_buf+2) = (addr>>8)&0xff;
				*(tx_buf+3) = addr & 0xff;
				addr += 4;
				for(i=1;i<(transmit_len/8+1);i++)
				{		
					*(tx_buf+(i)*8+0) = FLASH_READ;
					*(tx_buf+(i)*8+1) = (addr>>16)&0xff;
					*(tx_buf+(i)*8+2) = (addr>>8)&0xff;
					*(tx_buf+(i)*8+3) = addr & 0xff;
					
					addr += 4;
				}
		}
	}
}

void spi_gd25q16_write_test_set_buffer(bool cs_force,uint32_t addr,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t transmit_len)
{
	uint16_t i = 0;
	
	hal_setMem(tx_buf,0x00,transmit_len);
	hal_setMem(rx_buf,0x00,transmit_len);

	if(cs_force)
	{
		tx_buf[0] = FLASH_PP;
		tx_buf[1] = (addr>>16)&0xff;
		tx_buf[2] = (addr>>8)&0xff;
		tx_buf[3] = addr & 0xff;
		
		for(i=4;i<transmit_len;i++)
		{
			*(tx_buf+i) = (i-4);
		}
	}
	else
	{
		//when wrtie data in several frames,it is complex in this mode,because we need check busy flag every time.
		//we just write 4 byte for example.
		tx_buf[0] = FLASH_PP;
		tx_buf[1] = (addr>>16)&0xff;
		tx_buf[2] = (addr>>8)&0xff;
		tx_buf[3] = addr & 0xff;
		tx_buf[4] = 0x11;
		tx_buf[5] = 0x22;
		tx_buf[6] = 0x33;
		tx_buf[7] = 0x44;
	}
}

void spi_gd25q16_write_test_set_buffer_2(bool cs_force,uint32_t addr,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t transmit_len)
{
	uint16_t i;
	
	hal_setMem(tx_buf,0x00,transmit_len);
	hal_setMem(rx_buf,0x00,transmit_len);

	if(cs_force)
	{
		tx_buf[0] = FLASH_PP;
		tx_buf[1] = (addr>>16)&0xff;
		tx_buf[2] = (addr>>8)&0xff;
		tx_buf[3] = addr & 0xff;
		
		for(i=4;i<transmit_len;i++)
		{
			*(tx_buf+i) = (i-4);
		}
	}
	else
	{
		//when wrtie data in several frames,it is complex in this mode,because we need check busy flag every time.
		//we just write 4 byte for example.
		tx_buf[0] = FLASH_PP;
		tx_buf[1] = (addr>>16)&0xff;
		tx_buf[2] = (addr>>8)&0xff;
		tx_buf[3] = addr & 0xff;
		tx_buf[4] = 0x11;
		tx_buf[5] = 0x22;
		tx_buf[6] = 0x33;
		tx_buf[7] = 0x44;
	}
}

void spi_gd25q16_print_buffer(uint8_t* tx_buf,uint8_t *rx_buf,uint16_t transmit_len,uint8_t testCase)
{
	uint16_t i = 0;
	uint8_t temp;

	for(i=0;i<transmit_len;i++)
	{
		switch(testCase)
		{
			case 0:
				if(i>=4)
				{
					if(*(rx_buf+i) != 0xFF)//check the original data
					{
						LOG("error:%d\n",testCase);
							while(1);
					}
					//LOG("%x ",*(rx_buf+i));
				}
				break;
			
			case 5:
				if(i>=4)
				{
					LOG("%x ",*(rx_buf+i));//print the original data
					if((*(rx_buf+4) != 0x11) || (*(rx_buf+5) != 0x22) || (*(rx_buf+6) != 0x33) || (*(rx_buf+7) != 0x44))
					{
						LOG("error:%d\n",testCase);
						while(1);
					}
				}
				break;
			
			case 1:
			case 3://00000123(addr+0123) 456789ab(data:1~8) c~13(data:1~8)...
				if(i>=4)
				{
					if((i-4)!= *(rx_buf+i))
					{
						LOG("error:%d\n",testCase);
						while(1);
					}
				}
				break;
				
			case 2:
			case 4://00000123(addr+0123) 00004567(data:1~4) 000089ab...
				if((i%8)>=4)
				{
					temp =(i/8)*4+ (i%8) - 4;
					if(temp!= *(rx_buf+i))
					{
						LOG("\nerror:%d\n",testCase);
						while(1);
					}
				}
			default:
				break;
		}
	}
}

void spi_gd25q16_write_enable(void)
{
	uint16_t ret16;
	
	ret16 = spiflash_read_status_register(0);
	if(ret16 != 0)
	{
		spiflash_write_status_register(0x00);
		while(spiflash_bus_busy() == TRUE);
	}
	spiflash_write_enable();
}

int spi_gd25q16_write_test(uint16_t addr,uint16_t len,uint16_t testCase)
{
	uint16_t transmit_len;
	
	switch(testCase)
	{
		case 1:
			//LOG("\n\nmode=polling,cs_force=true\n");
			hal_spi_set_force_cs(&spi,true);
			hal_spi_set_int_mode(&spi,false);
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = (4+len);
				spi_gd25q16_write_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
				
				spi_gd25q16_write_enable();
				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
			}
			break;
		
		case 2:
			//LOG("\n\nmode=polling,cs_force=false\n");
			//LOG("[be care]before write we should check busy flag,so just write three data to test\n");
		
			hal_spi_set_force_cs(&spi,false);
			hal_spi_set_int_mode(&spi,false);
		
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = ((len/4)*8 + (len%4==0?0:4) + len%4);
				spi_gd25q16_write_test_set_buffer(false,addr,tx_buf,rx_buf,transmit_len);
				
				spi_gd25q16_write_enable();
				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
			}
			break;
			
		case 3:
			//LOG("\n\nmode=int,cs_force=true\n");	
			hal_spi_set_force_cs(&spi,true);
			hal_spi_set_int_mode(&spi,false);
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = (4+len);
				spi_gd25q16_write_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
				spi_gd25q16_write_enable();
				
				hal_spi_set_force_cs(&spi,true);
				hal_spi_set_int_mode(&spi,true);
				hal_spi_set_tx_buffer(&spi,spi_tx_buf,512);//care when use int mode,please give spi a tx buf
				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
				SPI_INIT_TOUT(to);
				while(hal_spi_get_transmit_bus_state(&spi) == true)
				{                   
                    SPI_CHECK_TOUT(to,1000000,"timeout\n")
				}
			}
			break;
		
		case 4:
			//LOG("\n\nmode=int,cs_force=false\n");
			//LOG("[be care]before write we should check busy flag,so just write three data to test\n");
			hal_spi_set_force_cs(&spi,false);
			hal_spi_set_int_mode(&spi,false);
		
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = ((len/4)*8 + (len%4==0?0:4) + len%4);
				spi_gd25q16_write_test_set_buffer(false,addr,tx_buf,rx_buf,transmit_len);
				spi_gd25q16_write_enable();
				
				hal_spi_set_force_cs(&spi,false);
				hal_spi_set_int_mode(&spi,true);				
				hal_spi_set_tx_buffer(&spi,spi_tx_buf,512);//care when use int mode,please give spi a tx buf
				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);

				SPI_INIT_TOUT(to);
				while(hal_spi_get_transmit_bus_state(&spi) == true)
				{
                    SPI_CHECK_TOUT(to,1000000,"timeout\n")
				}
			}
			break;
			
			default:
				break;
	}
    return PPlus_SUCCESS;
}

/*
int flash_test(void)
{
	uint16_t i,transmit_len = 260;
	uint32_t addr = 0x800;
	
	LOG("\nmode=polling,cs_force=true len=%d ",(transmit_len-4));
	hal_spi_set_force_cs(&spi,true);
	hal_spi_set_int_mode(&spi,false);
	if(hal_spi_get_transmit_bus_state(&spi) == false)
	{
		spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
		hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);		
		for(i=4;i<260;i++)
		{
			LOG("%x ",*(rx_buf+i));
			if((i>4)&&((i-4)%16==0))
			{
				LOG("\n");
			}
		}
	}
	return PPlus_SUCCESS;
}


int flash_test(void)
{
	uint16_t i,transmit_len = 260;
	uint32_t addr = 0x800;
	
	LOG("\nmode=int,cs_force=true len=%d ",(transmit_len-4));
	hal_spi_set_force_cs(&spi,true);
	hal_spi_set_int_mode(&spi,true);	
	if(hal_spi_get_transmit_bus_state(&spi) == false)
	{
		spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);

		hal_spi_set_tx_buffer(&spi,spi_tx_buf,512);//care when use int mode,please give spi a tx buf
		hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);

		SPI_INIT_TOUT(to);
		while(hal_spi_get_transmit_bus_state(&spi) == true)
		{
			SPI_CHECK_TOUT(to,1000000,"timeout\n")		
		}
		
		for(i=4;i<260;i++)
		{
			LOG("%x ",*(rx_buf+i));
			if((i>4)&&((i-4)%16==0))
			{
				LOG("\n");
			}
		}	
	}
	return PPlus_SUCCESS;
}
*/

int spi_gd25q16_read_test(uint16_t addr,uint16_t len)
{
	uint16_t testCase = 0,transmit_len;
	
	while(1)
	{
		testCase++;
		switch(testCase)
		{
			case 1:
			{
				//LOG("\nmode=polling,cs_force=true len=%d ",len);
				hal_spi_set_force_cs(&spi,true);
				hal_spi_set_int_mode(&spi,false);
				if(hal_spi_get_transmit_bus_state(&spi) == false)
				{
					transmit_len = (4+len);
					spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
					hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
					spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,1);
				}
			}
			break;
			
			case 2:
			//LOG("\nmode=polling,cs_force=false len=%d ",len);
			hal_spi_set_force_cs(&spi,false);
			hal_spi_set_int_mode(&spi,false);
			
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = ((len/4)*8 + (len%4==0?0:4) + len%4);
				spi_gd25q16_read_test_set_buffer(false,addr,tx_buf,rx_buf,transmit_len);

				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
				spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,2);
			}
			break;

			case 3:
				//LOG("\nmode=int,cs_force=true len=%d ",len);
				hal_spi_set_force_cs(&spi,true);
				hal_spi_set_int_mode(&spi,true);
				if(hal_spi_get_transmit_bus_state(&spi) == false)
				{
					transmit_len = (4+len);
					spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);

					hal_spi_set_tx_buffer(&spi,spi_tx_buf,512);//care when use int mode,please give spi a tx buf
					hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
					
					SPI_INIT_TOUT(to);
					while(hal_spi_get_transmit_bus_state(&spi) == true)
					{
                        SPI_CHECK_TOUT(to,1000000,"timeout\n")
					}
				
					spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,3);
				}
				break;
			
			case 4:
			//LOG("\nmode=int,cs_force=false len=%d ",len);
			hal_spi_set_force_cs(&spi,false);
			hal_spi_set_int_mode(&spi,true);
			if(hal_spi_get_transmit_bus_state(&spi) == false)
			{
				transmit_len = ((len/4)*8 + (len%4==0?0:4) + len%4);
				spi_gd25q16_read_test_set_buffer_2(false,addr,tx_buf,rx_buf,transmit_len);
				
				hal_spi_set_tx_buffer(&spi,spi_tx_buf,512);
				hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
				
				SPI_INIT_TOUT(to);
				while(hal_spi_get_transmit_bus_state(&spi) == true)
				{
                    SPI_CHECK_TOUT(to,1000000,"timeout\n")
				}
				
				spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,4);
			}
			break;
			
			default:
				break;
		}

		if(testCase >= 3)
			break;
	}
    return PPlus_SUCCESS;    
}

void spi_spi_gd25q16_read_chip(bool check,uint16_t addr_para)
{
	uint16_t len = 256,transmit_len;
	uint32_t addr = 0;
	
	//LOG("\nmode=polling,cs_force=true len=%d ",len);
	if(check == false)
	{
		hal_spi_set_force_cs(&spi,true);
		hal_spi_set_int_mode(&spi,false);
		
		if(hal_spi_get_transmit_bus_state(&spi) == false)
		{
			transmit_len = (4+len);
			spi_gd25q16_read_test_set_buffer(true,addr_para,tx_buf,rx_buf,transmit_len);
			hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
			spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,5);
		}
		return;
	}
	
	//LOG("\nmode=polling,cs_force=true len=%d ",len);
	while(addr < 0x200000)//2M byte
	{
		//LOG("\naddr:%x\n",addr);
		hal_spi_set_force_cs(&spi,true);
		hal_spi_set_int_mode(&spi,false);
		if(hal_spi_get_transmit_bus_state(&spi) == false)
		{
			transmit_len = (4+len);
			spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
			hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
			
			spi_gd25q16_print_buffer(tx_buf,rx_buf,transmit_len,0);
		}
		addr += 256;
	}
}

void spi_spi_gd25q16_read_check(uint32_t addr,uint16_t t_len,uint8_t check_case)
{
	uint16_t i = 0;
	uint16_t transmit_len = 260;
	const uint8_t data_check[] = {
	0,0,0,0,
	0,0,1,0,1,2,0,1,2,3,0,1,2,3,4,0,1,2,3,4,5,0,1,2,3,4,5,6,0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7,8,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,10,0,1,2,3,4,5,6,7,8,9,10,11,0,1,2,3,4,5,6,7,8,9,10,11,12,0,1,2,3,4,5,6,7,8,9,10,11,12,13,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21
	};
	
	hal_spi_set_force_cs(&spi,true);
	hal_spi_set_int_mode(&spi,false);
	if(hal_spi_get_transmit_bus_state(&spi) == false)
	{
		spi_gd25q16_read_test_set_buffer(true,addr,tx_buf,rx_buf,transmit_len);
		hal_spi_transmit(&spi,tx_buf,rx_buf,transmit_len);
		
		if(check_case == 0)
		{
			for(i=4;i<(253+4);i++)
			{
				if(data_check[i] != rx_buf[i])
				{
					LOG("check_case=0 error:%d\n",i);
					while(1);
				}
				else
				{
					LOG("%x ",rx_buf[i]);
				}
			}
		}
		else//00~f9,00~ff
		{
			for(i=4;i<(t_len+4);i++)
			{
				if((i-4) != rx_buf[i])
				{
					LOG("check_case=1 error:%d\n",i);
					while(1);
				}
				else
				{
					LOG("%x ",rx_buf[i]);
				}
			}
		}
	}
}

/*
1page
0 01 012 0123.....0~21
*/
void spi_write_read_test_1(const uint16_t addr,uint8_t mode)
{
	uint8_t len;
	uint16_t temp;
	
	LOG("write data:\n");
	temp = addr;
	for(len=1;len<23;len++)
	{
		spi_gd25q16_write_test(temp,len,mode);
		temp += len;
	}
	LOG("read and check data:\n");
	spi_spi_gd25q16_read_check(addr,253,0);
	LOG("\n");
}

/*
8page
249~256
*/
void spi_write_read_test_2(const uint16_t addr,uint8_t mode)
{
	uint16_t len,i;
	uint16_t temp;
	
	LOG("write data:\n");
	for(i=0;i<8;i++)
	{
		temp = 0x100*i + addr;
		len=249+i;
		spi_gd25q16_write_test(temp,len,mode);
	}
	
	LOG("read and check data:\n");
	for(i=0;i<8;i++)
	{
		LOG("\n%d\n",(248+i));
		temp = 0x100*i + addr;
		len=249+i;
		spi_spi_gd25q16_read_check(temp,len,mode);
	}
	LOG("\n");
}
/*
page0:0+01+012+0123+(0~22)
page1~page8:249 250 251 252 253 254 255 256

page:9:0x11 0x22 0x33 0x44

page10:0+01+012+0123+(0~22)
page11~page18:249 250 251 252 253 254 255 256

page:19:0x11 0x22 0x33 0x44
*/
void spi_test_on_gd25q16(void)
{
	static uint16_t testCase = 0;
	uint16_t len = 256;
	uint32_t addr = 0;
//	uint32_t ret;
	//LOG("test spi on gd25q16b,there may be some differences between differenct slaves\n");
	//LOG("when use int mode to transmit data,please assign buffer first\n");
	
	if(hal_spi_bus_init(&spi,spi_cfg) != PPlus_SUCCESS)
	{
		LOG("spi init err!please check it!\n");
		while(1);
	}
		
	//ret = flash_test();
	
	while(testCase < 5)
	{
		LOG("testCase:%d\n",testCase);
		switch(testCase)
		{
			case 0:
				GD25_erase(0,2*1024*1024);
				spi_spi_gd25q16_read_chip(true,0);
				LOG("erase_ok\n\n\n");
				break;
			
			case 1:
				LOG("write_test(mode 1):");
				spi_write_read_test_1(0x100*0,1);//1 page
				spi_write_read_test_2(0x100*1,1);//8 page
				LOG("case 1 ok\n\n\n");
				break;
				
			case 2:
				LOG("write_test(mode 2):");
				addr = 0x100*9;
				spi_gd25q16_write_test(addr,4,2);
				spi_spi_gd25q16_read_chip(false,addr);
				LOG("\ncase 2 ok\n\n\n");
				break;
				
			case 3:
				LOG("write_test(mode 3)\n");
				spi_write_read_test_1(0x100*10,3);//1 page
				spi_write_read_test_2(0x100*11,3);//8 page
				LOG("case 3 ok\n\n\n");
				break;
			
			case 4:
				LOG("write_test(mode 4):");
				addr = 0x100*19;
				spi_gd25q16_write_test(addr,4,4);
				spi_spi_gd25q16_read_chip(false,addr);
				LOG("\ncase 4 ok\n\n\n");
				break;
			
			default:
				break;
		}
		
		testCase++;
	}
	if((testCase>=5) && (testCase<10))
	{
		LOG("testCase:%d\n",testCase);//read a page which data is 00~ff
		addr = 0x100*8;
		for(len=1;len<256;len++)
		spi_gd25q16_read_test(addr,len);		
		//ret = flash_test();
		testCase++;
	}
	else
	{
		LOG("test end,no problem\n");
		while(1);;
	}
	hal_spi_bus_deinit(&spi);
}

uint16 spi_demo_ProcessEvent( uint8 task_id, uint16 events )
{
	if (events & TIMER_1000MS_CYCLE )
	{
		LOG("\n2s:recycle mode\n");
		spi_test_on_gd25q16();
		return (events ^ TIMER_1000MS_CYCLE);
	}  

	return 0;
}
