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



#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"

#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "ll_sleep.h"

//#include "touch_key.h"
#include "uart.h"
#include "gpio.h"
#include "clock.h"
#include "common.h"
#include "log.h"
#include "adc.h"
#include "pwrmgr.h"
#include "string.h"
#include "phy_console.h"
#include "flash.h"

extern void  ble_main(void);

extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);

extern  uint32_t pclk;

#define     LARGE_HEAP_SIZE  6*1024
uint8       g_largeHeap[LARGE_HEAP_SIZE];
volatile uint8 g_clk32K_config;
extern uint8_t rom_MID;

static void rf_wakeup_handler(void){
  NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
  NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);
}


void cons_callback(uint16_t cmd_id, uint8_t argc, char** argv)
{
	LOG("cmd id is 0x%x, parameter num is %d\n", cmd_id, argc);
	for(uint8_t i = 0; i<argc; i++){
		LOG("param %d : %s\n", i, argv[i]);
	}
}

const cons_cmd_t s_cmd_list[] = {
	{0x0010, "cmd1"},
	{0x0011, "cmd2"},
	{0x0012, "cmd3"},
	{0x0013, "cmd4"},
	{0x0014, "cmd5"},
	{0x0015, "cmd6"},
	{0, NULL},
};

static void hal_init(void)
{
  
  hal_system_init(g_system_clk);//(SYS_CLK_DLL_32M);//SYS_CLK_XTAL_16M);   //system init

  //hal_rtc_clock_config(CLK_32K_XTAL);//CLK_32K_RCOSC)CLK_32K_XTAL;
    hal_rtc_clock_config(g_clk32K_config);
  
  hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);
  
  //LOG_INIT();
  hal_gpio_init();
  hal_adc_init();
	
	console_init(s_cmd_list, cons_callback);
	
  LOG("all driver init OK!\n");

}

static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;

    hal_rom_code_ini();
    
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump. 

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw

    //========= UART RX Pull up
    hal_gpio_pull_set(P10,WEAK_PULL_UP);    

    DCDC_CONFIG_SETTING(0x0d);


    NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);

    hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);

}

int  main(void)  
{     
    g_system_clk = SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC

	read_deviceID();  
	if(rom_MID == 0xb3)
	{
		g_system_clk = SYS_CLK_XTAL_16M;
	}
	
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
	
    init_config();

    hal_pwrmgr_init();

    hal_rfphy_init();  
    							
    hal_init();

    app_main();	

}


///////////////////////////////////  end  ///////////////////////////////////////

