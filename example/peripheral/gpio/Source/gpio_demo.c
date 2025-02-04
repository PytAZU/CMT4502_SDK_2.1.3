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
  Filename:       gpio_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"


/*********************************************************************
 * pulseMeasure_Task
 * Task pulseMeasure sample code,we can use p04~p07 and p11~p15 easily.
 */
static uint8 pulseMeasure_TaskID;
 
typedef struct {
	bool          enable;
	bool          pinstate;
	uint32_t      edge_tick;
}gpioin_Trig_t;

typedef struct {
	GPIO_Pin_e    pin;
	bool          type;
	uint32_t      ticks;
}gpioin_pulse_Width_measure_t;

gpioin_pulse_Width_measure_t measureResult ={
	.pin = GPIO_P14,
};

static gpioin_Trig_t gpioTrig = {
  .enable = FALSE,
  .edge_tick = 0,
};

void plus_edge_callback(void){
	LOG("pulse:%d %d\n",measureResult.type,measureResult.ticks);
}

void pulse_measure_callback(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(gpioTrig.enable == FALSE)
	{
		gpioTrig.enable = TRUE;
		gpioTrig.edge_tick = hal_systick();
		return;
	}
	measureResult.type = type;
	measureResult.ticks = hal_ms_intv(gpioTrig.edge_tick);
	plus_edge_callback();
	gpioTrig.edge_tick = hal_systick();
}


void Pulse_Measure_Init( uint8 task_id )
{
	pulseMeasure_TaskID = task_id;

	hal_gpioin_register(measureResult.pin,pulse_measure_callback,pulse_measure_callback);
	gpioTrig.pinstate = hal_gpio_read(measureResult.pin);
}

uint16 Pulse_Measure_ProcessEvent( uint8 task_id, uint16 events )
{
	if(task_id != pulseMeasure_TaskID){
		return 0;
	}
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * timer_Task
 * Task timer sample code
 */
static uint8 timer_TaskID; 
#define TIMER_1S_ONCE                                0x0001
#define TIMER_2S_CYCLE                               0x0004

void Timer_Demo_Init( uint8 task_id ){
	timer_TaskID = task_id;
	
	osal_start_timerEx( timer_TaskID, TIMER_1S_ONCE , 1000);
  osal_start_reload_timer( timer_TaskID, TIMER_2S_CYCLE , 2000);
}

uint16 Timer_Demo_ProcessEvent( uint8 task_id, uint16 events ){
	static uint8 count1 = 0,count2 = 0;
	static bool  timer_cycle_enable = TRUE;
	
	if(task_id != timer_TaskID){
		return 0;
	}
	
  if ( events & TIMER_1S_ONCE ){
		LOG("1s:once only mode\n");
		osal_start_timerEx( timer_TaskID, TIMER_1S_ONCE , 1000);
		
		if(timer_cycle_enable == FALSE){
			if(++count1 >= 10 ){
				osal_start_reload_timer( timer_TaskID, TIMER_2S_CYCLE , 2000);
				
				LOG("2s:recycle mode start\n");
				timer_cycle_enable = TRUE;
				count1 = 0;
			}
		}
    return (events ^ TIMER_1S_ONCE);
  }  
  
	  if ( events & TIMER_2S_CYCLE ){
			LOG("2s:recycle mode\n");
			if(++count2 >= 5 ){
				osal_stop_timerEx(timer_TaskID, TIMER_2S_CYCLE);
				
				LOG("2s:recycle mode stop\n");
				timer_cycle_enable = FALSE;
				count2 = 0;
			}
			
    return (events ^ TIMER_2S_CYCLE);
  }  
	
  return 0;
}

/*********************************************************************
 * gpio_wakeup_Task
 * Task gpio wakeup sample code
 * The followinng code shows P14 wakeup the system when there is a posedge or negedge.
 */
static uint8 gpio_wakeup_TaskID; 
void posedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type){
	if(type == POSEDGE){
		LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
	}
	else{
		LOG("error\n");
	}
}

void negedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type){
	if(type == NEGEDGE){
		LOG("wakeup(neg):gpio:%d type:%d\n",pin,type);
	}
	else{
	LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
	}
}

/*
	*P00~P03:default jtag,we can use it as wakeup pin when no debug.
	*P04~P07,P11~P15,P18~P30:default gpio,use it easily.
	*P08:mode select pin,cannot used as other usage.
	*P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
	*P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
	*P31~P34:default spif,we can use it as wakeup pin directly,we driver have completed its multiplex config.
*/
typedef struct gpioin_wakeup_t{
	GPIO_Pin_e pin;
	gpioin_Hdl_t posedgeHdl; 
	gpioin_Hdl_t negedgeHdl;
}gpioin_wakeup;

#define GPIO_WAKEUP_PIN_NUM 3
gpioin_wakeup gpiodemo[GPIO_WAKEUP_PIN_NUM] = {
	GPIO_P14,posedge_callback_wakeup,negedge_callback_wakeup,
	GPIO_P23,posedge_callback_wakeup,negedge_callback_wakeup,
	GPIO_P31,posedge_callback_wakeup,negedge_callback_wakeup,
};

void Gpio_Wakeup_Init(uint8 task_id ){
	uint8_t i = 0;
	static bool gpioin_state[GPIO_WAKEUP_PIN_NUM];
	
	gpio_wakeup_TaskID = task_id;
	LOG("gpio wakeup demo start...\n");
	
	for(i = 0;i<GPIO_WAKEUP_PIN_NUM;i++){
		hal_gpioin_register(gpiodemo[i].pin,gpiodemo[i].posedgeHdl,gpiodemo[i].negedgeHdl);
		gpioin_state[i] = hal_gpio_read(gpiodemo[i].pin);
		LOG("gpioin_state:%d %d\n",i,gpioin_state[i]);
	}		
}

uint16 Gpio_Wakeup_ProcessEvent( uint8 task_id, uint16 events ){
	if(task_id != gpio_wakeup_TaskID){
		
		return 0;
	}
	return 0;
}

/*********************************************************************
 * key_Task:gpio config as key
 * 
 */
static uint8 key_TaskID;

#define KEY_DEMO_ONCE_TIMER      0x0001
#define KEY_DEMO_CYCLE_TIMER     0x0002
//#define HAL_KEY_EVENT	         0x0100//assign short key event in your app event process

#ifdef HAL_KEY_SUPPORT_LONG_PRESS
#define KEY_DEMO_LONG_PRESS_EVT   0x0200 //if use long key,assign long key event in your app process
#endif

static void key_press_evt(uint8_t i,key_evt_t key_evt)
{
	LOG("\nkey index:%d gpio:%d ",i,key_state.key[i].pin);
	switch(key_evt)
	{
		case HAL_KEY_EVT_PRESS:
			LOG("key(press down)\n");		
#ifdef HAL_KEY_SUPPORT_LONG_PRESS
			osal_start_timerEx(key_TaskID,KEY_DEMO_LONG_PRESS_EVT,HAL_KEY_LONG_PRESS_TIME);
#endif
			break;
		
		case HAL_KEY_EVT_RELEASE:
			LOG("key(press release)\n");
			break;

#ifdef HAL_KEY_SUPPORT_LONG_PRESS		
		case HAL_KEY_EVT_LONG_RELEASE:
			hal_pwrmgr_unlock(MOD_USR1);
			LOG("key(long press release)\n");
			break;
#endif
		
		default:
			LOG("unexpect\n");
			break;
	}
}

static void P16_wakeup_handler(void){
	hal_gpio_cfg_analog_io(P16,Bit_DISABLE);
}

static void P17_wakeup_handler(void){
	hal_gpio_cfg_analog_io(P17,Bit_DISABLE);
}

typedef struct _uart_Context{
  bool        enable;
  uint8_t     tx_state;
  uart_Tx_Buf_t tx_buf;
  uart_Cfg_t  cfg;
}uart_Ctx_t;


//extern uart_Ctx_t m_uartCtx;
void uart_port_reconfig(void)
{
  uart_Cfg_t cfg_user = {
	.tx_pin = P14,
	.rx_pin = P15,
		
	.rts_pin = GPIO_DUMMY,
	.cts_pin = GPIO_DUMMY,
	.baudrate = 115200,
	.use_fifo = TRUE,
	.hw_fwctrl = FALSE,
	.use_tx_buf = FALSE,
	.parity     = FALSE,
	.evt_handler = NULL,
  };
	
	hal_gpio_fmux(P9,Bit_DISABLE);
	hal_gpio_fmux(P10,Bit_DISABLE);

	hal_gpio_pin_init(P14,OEN);
	hal_gpio_pin_init(P15,IE);
	//m_uartCtx.enable = FALSE;
  hal_uart_init(cfg_user);//uart init
	
	LOG("uart new port...\n");
}

void Key_Demo_Init(uint8 task_id)
{
	uint8_t i = 0;
	
	key_TaskID = task_id;
	LOG("gpio key demo start...\n");
	
	/*
	when use key,please set the following parameters:
	1.key number,config KEY_NUM in key.h
	2.gpio used,config key_state.pin
		*P00~P03:default jtag,we can use it as key when no debug.
		*P04~P07,P11~P15:default gpio,use it easily.
		*P08:mode select pin,cannot used as other usage.
	  *P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
		*P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
		*P18~P34:wakeup is supported,but interrupt is not supported,so config it as key is not suggested.
	3.idle level,config key_state.idle_level
	4.key type,if only use press and release,ignore the long press and release code
	5.taskID and callback function
	*/
	
//	key_state.key[0].pin = GPIO_P14;//default gpio
//	key_state.key[1].pin = GPIO_P15;
//	key_state.key[2].pin = GPIO_P00;//default jtag
//	key_state.key[3].pin = GPIO_P01;
//	key_state.key[4].pin = GPIO_P02;
//	key_state.key[5].pin = GPIO_P03;
//	key_state.key[6].pin = GPIO_P16;//default xtal
//	key_state.key[7].pin = GPIO_P17;//default xtal
//	
//	key_state.key[0].pin = GPIO_P09;
//	key_state.key[1].pin = GPIO_P10;

	key_state.key[0].pin = GPIO_P14;
	key_state.key[1].pin = GPIO_P15;
	
	for(i = 0; i < HAL_KEY_NUM; ++i){
		key_state.key[i].state = HAL_STATE_KEY_IDLE;
		key_state.key[i].idle_level = HAL_LOW_IDLE;
	
		if(key_state.key[i].pin == GPIO_P16){
			hal_pwrmgr_register(MOD_USR2,NULL,P16_wakeup_handler);
			hal_gpio_cfg_analog_io(key_state.key[i].pin,Bit_DISABLE);
			LOG("P16 is used\n");
		}
			else if(key_state.key[i].pin == GPIO_P17){
			hal_pwrmgr_register(MOD_USR3,NULL,P17_wakeup_handler);
			hal_gpio_cfg_analog_io(key_state.key[i].pin,Bit_DISABLE);
			LOG("P17 is used\n");
		}
		else if((key_state.key[i].pin == GPIO_P09) || (key_state.key[i].pin == GPIO_P10)){
			uart_port_reconfig();
		}
	}
		
//	key_state.key[0].idle_level = HAL_LOW_IDLE;	
//	key_state.key[1].idle_level = HAL_HIGH_IDLE;
	
	key_state.task_id = key_TaskID;
	key_state.key_callbank = key_press_evt;
	key_init();
	
	osal_start_timerEx(key_TaskID, KEY_DEMO_ONCE_TIMER, 5000);
	osal_start_reload_timer(key_TaskID, KEY_DEMO_CYCLE_TIMER , 5000);
}

uint16 Key_ProcessEvent( uint8 task_id, uint16 events )
{
	if(task_id != key_TaskID){
		return 0;
	}
	
	if( events & KEY_DEMO_ONCE_TIMER){		
		//LOG("once timer\n");
		osal_start_timerEx( key_TaskID, KEY_DEMO_ONCE_TIMER , 5000);
		return (events ^ KEY_DEMO_ONCE_TIMER);
	}
	
	if( events & KEY_DEMO_CYCLE_TIMER){		
		//LOG("recycle timer\n");
		return (events ^ KEY_DEMO_CYCLE_TIMER);
	}
	
	if( events & HAL_KEY_EVENT){													//do not modify,key will use it
		for (uint8 i = 0; i < HAL_KEY_NUM; ++i){
			if ((key_state.temp[i].in_enable == TRUE)||
			(key_state.key[i].state == HAL_STATE_KEY_RELEASE_DEBOUNCE)){
				gpio_key_timer_handler(i);
			}
		}
		return (events ^ HAL_KEY_EVENT);
	}

#ifdef HAL_KEY_SUPPORT_LONG_PRESS	
	if( events & KEY_DEMO_LONG_PRESS_EVT){
		
		for (int i = 0; i < HAL_KEY_NUM; ++i){
			if(key_state.key[i].state == HAL_KEY_EVT_PRESS){
				LOG("key:%d gpio:%d	",i,key_state.key[i].pin);
				LOG("key(long press down)");
				osal_start_timerEx(key_TaskID,KEY_DEMO_LONG_PRESS_EVT,HAL_KEY_LONG_PRESS_TIME);//2s
				
				//user app code long press down process 	
			}
		}
		return (events ^ KEY_DEMO_LONG_PRESS_EVT);
	}
#endif
	
  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
