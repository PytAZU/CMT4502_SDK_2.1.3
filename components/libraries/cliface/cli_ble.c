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
  Filename:       cli_ble.c
  Revised:        
  Revision:       

  Description:    This file contains the cli ble sample application
                  

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#ifndef CLI_ROLE

#else

#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "log.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"



#include "gapbondmgr.h"
#include "flash.h"
#include "ll.h"
#include "clock.h"
#include "common.h"
#include "rflib.h"


#include "string.h"
#include "cli_ble.h"


/*********************************************************************
 * CLI BLE VARIABLES
 */

#define START_ADVERTISING_EVT         0x0001

// Advertising timeouts in sec
#define CLI_LIM_ADV_TIMEOUT           5  

// High duty Advertising interval   uint:625us
#define CLI_HIGH_ADV_INT_MIN           4  
#define CLI_HIGH_ADV_INT_MAX           6 

#define CLI_PERIPHERAL_ROLE 1
#define CLI_CLIENT_ROLE 2
#define CLI_MULTI_ROLE 3

#if(CLI_ROLE == CLI_MULTI_ROLE)
#include "multi.h"
#include "flash.h"
extern uint8 gapMultiRole_TaskID;
extern uint16 cli_conn_handle;
extern uint16_t cli_mtu_size;
static uint8_t save_peer_addr[6];
#else
#include "peripheral.h"
#include "central.h"
#include "gap.h"
#include "att.h"
#endif
#define SLA_DATA_LENGTH_EVT         0x0800
#define UPD_DATA_LENGTH_EVT         0x0100

extern uint8 gapDeviceAddrMode;
extern uint8 gapRole_TaskID;


///Advertising  Type
enum
{
    CLI_ADV_IND,
    CLI_ADV_DIRECT_IND_HIGH_DUTY,
    CLI_ADV_SCAN_IND,
    CLI_ADV_NONCONN_IND,
    CLI_ADV_DIRECT_IND_LOW_DUTY,
    CLI_ADV_MAX_TYPE,
}__attribute__((packed));

unsigned char cmdstr[256];
uint16_t cmdlen;
static uint8_t cli_ble_osal_id;

static uint16 cli_set_system_clk(uint32 argc, unsigned char *argv[]);

static uint16 cli_set_random_addr(uint32 argc, unsigned char *argv[]);

static uint16 cli_white_list(uint32 argc, unsigned char *argv[]);

static uint16 cli_adv_parameters(uint32 argc, unsigned char *argv[]);

static uint16 cli_adv_enable(uint32 argc, unsigned char *argv[]);

static uint16 cli_adv_data(uint32 argc, unsigned char *argv[]);

static uint16 cli_scan_rsp_data(uint32 argc, unsigned char *argv[]);

static uint16 cli_scan_param(uint32 argc, unsigned char *argv[]);

static uint16 cli_scan_enable(uint32 argc, unsigned char *argv[]);

static uint16 cli_scan_cancel(uint32 argc, unsigned char *argv[]);

static uint16 cli_creat_conn(uint32 argc, unsigned char *argv[]);

static uint16 cli_conn_cancel(uint32 argc, unsigned char *argv[]);

static uint16 cli_terminate_conn(uint32 argc, unsigned char *argv[]);

static uint16 cli_slave_conn_param_update(uint32 argc, unsigned char *argv[]);

static uint16 cli_mtu_exchange(uint32 argc, unsigned char *argv[]);

static uint16 cli_dlu_enable(uint32 argc, unsigned char *argv[]);

static uint16 cli_phy_set(uint32 argc, unsigned char *argv[]);

static uint16 cli_gatt_read(uint32 argc, unsigned char *argv[]);

static uint16 cli_gatt_notify(uint32 argc, unsigned char *argv[]);

static uint16 cli_write_req(uint32 argc, unsigned char *argv[]);

static uint16 cli_authen_enable(uint32 argc, unsigned char *argv[]);

static uint16 cli_ll_info_show(uint32 argc, unsigned char *argv[]);

const CLI_COMMAND cli_cmd_list[] =
{
  { "set_system_clk", "set system clk", (CLI_CMD_HANDLER)cli_set_system_clk},
  { "set_random_addr", "set mac", (CLI_CMD_HANDLER)cli_set_random_addr},
  { "white_list", "white list", (CLI_CMD_HANDLER)cli_white_list},
  { "adv_parameters", "adv parameters", (CLI_CMD_HANDLER)cli_adv_parameters},
  { "adv_data", "adv data", (CLI_CMD_HANDLER)cli_adv_data},
  { "scan_rsp_data", "scan rsp data", (CLI_CMD_HANDLER)cli_scan_rsp_data},
  { "adv_enable", "adv enable", (CLI_CMD_HANDLER)cli_adv_enable},
  { "scan_param", "scan param", (CLI_CMD_HANDLER)cli_scan_param},
  { "scan_enable", "scan enable", (CLI_CMD_HANDLER)cli_scan_enable},
  { "scan_cancel", "scan cancel", (CLI_CMD_HANDLER)cli_scan_cancel},
  { "creat_conn", "creat connation", (CLI_CMD_HANDLER)cli_creat_conn},
  { "conn_cancel", "connation cancel", (CLI_CMD_HANDLER)cli_conn_cancel},
  { "terminate_conn", "terminate connation", (CLI_CMD_HANDLER)cli_terminate_conn},
  { "slave_conn_param_update", "slave conn param update", (CLI_CMD_HANDLER)cli_slave_conn_param_update},
  { "mtu_exchange", "mtu exchange", (CLI_CMD_HANDLER)cli_mtu_exchange},
  { "dlu_enable", "dlu enable", (CLI_CMD_HANDLER)cli_dlu_enable},
  { "phy_set", "phy set", (CLI_CMD_HANDLER)cli_phy_set},
  { "gatt_read", "gatt read", (CLI_CMD_HANDLER)cli_gatt_read},
  { "gatt_notify", "gatt notify", (CLI_CMD_HANDLER)cli_gatt_notify},
  { "write_req", "write request", (CLI_CMD_HANDLER)cli_write_req},
  { "authen_enable", "authen enable", (CLI_CMD_HANDLER)cli_authen_enable},
  { "ll_info_show", "ll info show", (CLI_CMD_HANDLER)cli_ll_info_show},

};

static void ProcessUartData(uart_Evt_t *evt)
{
    uint16_t c_len = cmdlen + evt->len;
    if(c_len < sizeof(cmdstr))
    {
        osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
        cmdlen += evt->len;
        osal_set_event( cli_ble_osal_id, CLI_UART_RX_EVT );
    }
    else
    {
        cmdlen = 0;
    }
}

void sbp_uart_init(void)
{
  hal_uart_deinit();
  uart_Cfg_t cfg = {
  .tx_pin = P9,
  .rx_pin = P10,
  .rts_pin = GPIO_DUMMY,
  .cts_pin = GPIO_DUMMY,
  .baudrate = 115200,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = FALSE,
  .parity     = FALSE,
  .evt_handler = ProcessUartData,
  };
  hal_uart_init(cfg);//uart init
}

void cli_init(void)
{
    CLI_init((CLI_COMMAND*)cli_cmd_list,(sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)));
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
      extern uint8 simpleBLEPeripheral_TaskID;
      cli_ble_osal_id = simpleBLEPeripheral_TaskID;
#elif(CLI_ROLE == CLI_CLIENT_ROLE)
      extern uint8 simpleBLETaskId;
      cli_ble_osal_id = simpleBLETaskId;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	  extern uint8 multiRole_TaskId;
      cli_ble_osal_id = multiRole_TaskId;
#endif
}

/// 2 parameters: set system clock / clk32K_config
static uint16 cli_set_system_clk(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  if(argc != 2)
  {
    AT_LOG("Invaild parameters numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
#if (SYS_CLK_TEST_UCDS)
  uint8_t cli_system_clk,cli_clk32K_config;
  cli_system_clk = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  cli_clk32K_config = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 8);

  if(cli_system_clk < 7 &&  cli_clk32K_config < 2 )
  {
    AT_LOG("[SYS CLK CHG] %d %d\n",cli_clk32K_config, cli_system_clk);
    flash_erase_ucds(SYS_CLK_TEST_UCDS);

    uint8_t return_status = flash_write_ucds(SYS_CLK_TEST_UCDS, (cli_clk32K_config<<8) | cli_system_clk );
    if(return_status == CLI_BLE_RETURN_SUCCESS)
    {
      NVIC_SystemReset();
      return CLI_BLE_RETURN_SUCCESS;
    }
    else
    {
      AT_LOG("[RETURN SYS CLK CHG ERR: %d] %d %d\n",return_status,cli_clk32K_config, cli_system_clk);
      return CLI_BLE_INVAILD_PARAMS;
    }
    
  }
  else
  {
    AT_LOG("[SYS CLK PARAMS CHG ERR] %d %d\n",cli_clk32K_config, cli_system_clk);
    return CLI_BLE_INVAILD_PARAMS;
  }
#else
  LOG("[SYS CLK CHG NO SUPPORT]\n");
  return CLI_BLE_INVAILD_PARAMS;
#endif
  
}

static uint16 cli_set_random_addr(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  uint8_t random_address[6] = {0};
  if(argc != 1)
  {
    AT_LOG("Invaild parameters numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint8_t return_status = CLI_strtoarray(argv[0], CLI_strlen(argv[0]), random_address,6);
  if(return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild parameters\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
  HCI_LE_SetRandomAddressCmd(random_address);
  return CLI_BLE_RETURN_SUCCESS;

}

/// cmd:add remove clear 
static uint16 cli_white_list(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  uint8_t addr_type,address[6] = {0};

  if(0 == CLI_STR_COMPARE(argv[0], "clear"))
  {
    if(argc != 1)
    {
      AT_LOG("Invaild parameters numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    ///clear white list
    HCI_LE_ClearWhiteListCmd();
  }
  else if(0 == CLI_STR_COMPARE(argv[0], "add"))
  {
    if(argc != 3)
    {
      AT_LOG("Invaild parameters numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    addr_type =  CLI_strtoi(argv[1], CLI_strlen(argv[1]), 8);
    uint8_t return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), address,6);
    if(addr_type > 3 || return_status != CLI_BLE_RETURN_SUCCESS)
    {
      AT_LOG("Invaild parameters\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
    HCI_LE_AddWhiteListCmd(addr_type,address);
  }
  else if(0 == CLI_STR_COMPARE(argv[0], "remove"))
  {
    if(argc != 3)
    {
      AT_LOG("Invaild  parameters numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    addr_type =  CLI_strtoi(argv[1], CLI_strlen(argv[1]), 8);
    uint8_t return_status = CLI_strtoarray(argv[2], CLI_strlen(argv[2]), address,6);
    if(addr_type > 3 || return_status != CLI_BLE_RETURN_SUCCESS)
    {
      AT_LOG("Invaild parameters\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
    HCI_LE_RemoveWhiteListCmd(addr_type,address);
  }
  else
  {
    AT_LOG("Invaild parameters\n");
    return CLI_BLE_INVAILD_PARAMS;
  }

  return CLI_BLE_RETURN_SUCCESS;

}

/// min_interval  max_interval adv_type own_addr_type peer_addr_type peer_addr adv_channle_map adv_filter_policy
static uint16 cli_adv_parameters(uint32 argc, unsigned char *argv[])
{
    AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
    if( argc != 8)
    {
        AT_LOG("Invaild Paraments numbers\n");
        return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    uint16_t min_adv_interval,max_adv_interval;
    uint8_t adv_type,own_addr_type,peer_addr_type,peer_addr[6],adv_channle_map,adv_filter_policy,return_status;
    min_adv_interval = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    max_adv_interval = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    /// Advertising event properties
    adv_type = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 8);
    /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    own_addr_type = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 8);
    /// Peer address type: public=0 / random=1
    peer_addr_type = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 8);
    /// Primary advertising channel map
    adv_channle_map = CLI_strtoi(argv[6], CLI_strlen(argv[6]), 8);
    /// Advertising filter policy
    adv_filter_policy= CLI_strtoi(argv[7], CLI_strlen(argv[7]), 8);
    /// Peer Bluetooth device address
    return_status = CLI_strtoarray(argv[5], CLI_strlen(argv[5]), peer_addr,6);
    if(adv_type >= CLI_ADV_MAX_TYPE || own_addr_type > 3 || peer_addr_type > 1 || adv_channle_map > 7  || adv_filter_policy > 3 || return_status != CLI_BLE_RETURN_SUCCESS)
    {
      AT_LOG("invalid parameters\r\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
    AT_LOG("min_adv_interval = %d max_adv_interval = %d adv_type = %d own_addr_type = %d \
    peer_addr_type = %d chan_map = %d adv_fl = %d\r\n",min_adv_interval,max_adv_interval,
    adv_type,own_addr_type,peer_addr_type,adv_channle_map,adv_filter_policy);
    for(uint8_t i = 0; i < 6; i++)
    {
      AT_LOG("%2x",peer_addr[i]);
    }
    AT_LOG("\n");

    if(adv_type == CLI_ADV_DIRECT_IND_HIGH_DUTY)
    {
      ///high duty direct adv interval shall be less than or equal to 3.75 ms and set  5 sec adv timeout
      VOID GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, CLI_HIGH_ADV_INT_MIN );
      VOID GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, CLI_HIGH_ADV_INT_MAX );
      VOID GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, CLI_LIM_ADV_TIMEOUT );
    }
    else
    {
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, min_adv_interval );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, max_adv_interval );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, min_adv_interval );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, max_adv_interval );
    }
	gapDeviceAddrMode = own_addr_type;
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &adv_type );
    GAPRole_SetParameter( GAPROLE_ADV_DIRECT_TYPE, sizeof( uint8 ), &peer_addr_type );
    GAPRole_SetParameter( GAPROLE_ADV_DIRECT_ADDR, sizeof(peer_addr), peer_addr);
    GAPRole_SetParameter( GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &adv_channle_map);
    GAPRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy ); 
    
    ///set adv parameters???
    VOID osal_set_event( gapRole_TaskID, START_ADVERTISING_EVT );
    return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
   GAPMultiRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &adv_type ,0);
   GAPMultiRole_SetParameter( GAPROLE_ADV_DIRECT_TYPE, sizeof( uint8 ), &peer_addr_type ,0 );
   GAPMultiRole_SetParameter( GAPROLE_ADV_DIRECT_ADDR, sizeof(peer_addr), peer_addr,0 );
   GAPMultiRole_SetParameter( GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &adv_channle_map,0);
   GAPMultiRole_SetParameter( GAPROLE_ADV_FILTER_POLICY, sizeof( uint8 ), &adv_filter_policy ,0 );
   return CLI_BLE_RETURN_SUCCESS; 
#endif 
#else
   return CLI_BLE_INVAILD_PARAMS;
#endif    
    
}

static uint16 cli_adv_data(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
  uint8_t data_length,adv_data_value[31] = {0};
  if(argc != 2)
  {
    AT_LOG("Invaild parameters numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  data_length = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  uint8_t return_status = CLI_strtoarray(argv[1], CLI_strlen(argv[1]), adv_data_value,data_length);
  if(data_length > 31 || data_length == 0 || return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild parameters\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, data_length, adv_data_value );
  return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
  GAPMultiRole_SetParameter( GAPROLE_ADVERT_DATA, data_length, adv_data_value ,0);
  return CLI_BLE_RETURN_SUCCESS;
#endif
#else
  return CLI_BLE_INVAILD_PARAMS;
#endif
  
}

static uint16 cli_scan_rsp_data(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
  uint8_t data_length,scan_rsp_data_value[31] = {0};
  if(argc != 2)
  {
    AT_LOG("Invaild paremeters numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  data_length = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  uint8_t return_status = CLI_strtoarray(argv[1], CLI_strlen(argv[1]), scan_rsp_data_value,data_length);
  if(data_length > 31 || data_length == 0 || return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild parameters\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
  GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, data_length, scan_rsp_data_value );
  return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
  GAPMultiRole_SetParameter( GAPROLE_SCAN_RSP_DATA, data_length, scan_rsp_data_value ,0);
  return CLI_BLE_RETURN_SUCCESS;
#endif
#else
  return CLI_BLE_INVAILD_PARAMS;
#endif
  
}
static uint16 cli_adv_enable(uint32 argc, unsigned char *argv[])
{
    AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
    if( argc != 1 )
    {
        AT_LOG("Invaild Paraments numbers\n");
        return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    uint8_t adv_enable;
    adv_enable =  CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
    AT_LOG("adv_enable = %d\r\n",adv_enable);
    if(adv_enable == true)
    {
      uint8 initial_advertising_enable = TRUE;//true
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
			return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
 	  GAPMultiRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable,0 );
    osal_set_event( gapMultiRole_TaskID, ADV_ENABLE_EVT );
		return CLI_BLE_RETURN_SUCCESS;
#endif
    }
    else
    {
      uint8 initial_advertising_enable = false;//false
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
      return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	  GAPMultiRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable,0 );
    osal_set_event( gapMultiRole_TaskID, ADV_OFF_EVT );
    return CLI_BLE_RETURN_SUCCESS;
#endif
    }
#else
  return CLI_BLE_INVAILD_PARAMS;
#endif  
    
}

#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
static uint16_t scan_interval,scan_window;
static uint8_t scan_mode,scan_fliter_policy;
#endif
/// scan_mode interval  window  own_addr_type  scan_fliter_policy
static uint16 cli_scan_param(uint32 argc, unsigned char *argv[])
{
    AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
    if( argc != 5)
    {
        AT_LOG("Invaild Paraments numbers\n");
        return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    } 
    uint8_t own_addr_type;
    /// passive_scan=0 / active_scan=1  
    scan_mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
    scan_interval = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    scan_window = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
    /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    own_addr_type = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 8);
    /// allow_all_adv=0 / use_white_list=1
    scan_fliter_policy = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 8);
    if(scan_mode > 1 || own_addr_type > 3 || scan_fliter_policy > 1)
    {
      AT_LOG("invalid scan parameters\r\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
#if(CLI_ROLE == CLI_MULTI_ROLE)
	
	extern uint8_t activeScamode;
	extern uint8_t scanWhiteListMode;
	extern uint8 gapDeviceAddrMode;
	activeScamode= scan_mode;
	scanWhiteListMode = scan_fliter_policy;
	gapDeviceAddrMode = own_addr_type;
#endif
    AT_LOG("scan_mode = %d scan_interval = %d scan_window = %d own_addr_type = %d scan_fliter_policy = %d\r\n"\
    ,scan_mode,scan_interval,scan_window,own_addr_type,scan_fliter_policy);
    ///Scan parameters are processed in Scan Enable
    return CLI_BLE_RETURN_SUCCESS;
#else
    return CLI_BLE_INVAILD_PARAMS;
#endif  
    
}

/// scan_disc_mode  filter_duplic_en
static uint16 cli_scan_enable(uint32 argc, unsigned char *argv[])
{
    AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
    if( argc != 2 )
    {
        AT_LOG("Invaild Paraments numbers\n");
        return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    uint8_t disc_mode,filter_duplic_en;

    /// GAP Device Discovery Modes: no_disc=0 / general_disc=1 / limited_disc=2 / not_filtered=3
    disc_mode =  CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
    /// filter_duplic_enble=1
    filter_duplic_en = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 8);

    AT_LOG("disc_mode = %d filter_duplic_en = %d\r\n",disc_mode,filter_duplic_en);
    if(disc_mode > 3 || filter_duplic_en > 1)
    {
      AT_LOG("invalid parameter\r\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
    if(disc_mode == DEVDISC_MODE_LIMITED)
    {
      GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, scan_interval);
      GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, scan_window);

    }
    else
    {
      GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, scan_interval);
      GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, scan_window);
    }
    GAP_SetParamValue(TGAP_FILTER_ADV_REPORTS, filter_duplic_en);
#if(CLI_ROLE == CLI_CLIENT_ROLE)
    GAPCentralRole_StartDiscovery(disc_mode,scan_mode,scan_fliter_policy);
    return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	  extern uint8_t scan_disc_mode;
	  scan_disc_mode = disc_mode;
    osal_set_event( gapMultiRole_TaskID, SCAN_ENABLE_EVT );
    return CLI_BLE_RETURN_SUCCESS;
#endif
#else
    return CLI_BLE_INVAILD_PARAMS;
#endif
    
}

static uint16 cli_scan_cancel(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
#if (CLI_ROLE == CLI_CLIENT_ROLE)
  GAPCentralRole_CancelDiscovery();
	return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
  extern bStatus_t GAPMultiRole_CancelDiscovery(void);
  GAPMultiRole_CancelDiscovery();
	return CLI_BLE_RETURN_SUCCESS;
#endif
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif
}


/// no high duty conn 
/// 10 parameters/   no MIN_CE_LEN and MAX_CE_LEN
static uint16 cli_creat_conn(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)  
  if(argc != 10)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  ///Scan interval (N * 0.625 ms)
  uint16_t       scan_intv;
  ///Scan window size (N * 0.625 ms)
  uint16_t       scan_window;
  ///Initiator filter policy
  uint8_t        init_filt_policy;
  ///Peer address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
  uint8_t        peer_addr_type;
  ///Peer BD address
  uint8_t        peer_addr[6];
  ///Own address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
  uint8_t        own_addr_type;
  ///Minimum of connection interval (N * 1.25 ms)
  uint16_t       con_intv_min;
  ///Maximum of connection interval (N * 1.25 ms)
  uint16_t       con_intv_max;
  ///Connection latency
  uint16_t       con_latency;
  ///Link supervision timeout
  uint16_t       superv_to;

  scan_intv = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
  scan_window = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
  init_filt_policy = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 8);
  peer_addr_type = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 8);
  uint8_t return_status = CLI_strtoarray(argv[4], CLI_strlen(argv[4]), peer_addr,6);
  own_addr_type = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 8);
  con_intv_min = CLI_strtoi(argv[6], CLI_strlen(argv[6]), 16);
  con_intv_max = CLI_strtoi(argv[7], CLI_strlen(argv[7]), 16);
  con_latency = CLI_strtoi(argv[8], CLI_strlen(argv[8]), 16);
  superv_to = CLI_strtoi(argv[9], CLI_strlen(argv[9]), 16);

  if(init_filt_policy > 1 || peer_addr_type > 3 || own_addr_type > 3 || return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
  GAP_SetParamValue(TGAP_CONN_SCAN_INT, scan_intv);
  GAP_SetParamValue(TGAP_CONN_SCAN_WIND, scan_window);
  gapDeviceAddrMode = own_addr_type;
  GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, con_intv_min);
  GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, con_intv_max);
  GAP_SetParamValue(TGAP_CONN_EST_LATENCY, con_latency);
  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, superv_to);
#if(CLI_ROLE == CLI_CLIENT_ROLE)
  GAP_SetParamValue(TGAP_CONN_SCAN_INT, scan_intv);
  GAP_SetParamValue(TGAP_CONN_SCAN_INT, scan_intv);
  extern gapDevRec_t simpleBlePeerTarget;
  simpleBlePeerTarget.addrType = peer_addr_type;
  memcpy(simpleBlePeerTarget.addr,peer_addr,sizeof(peer_addr));
  GAPCentralRole_EstablishLink(false,init_filt_policy,peer_addr_type,peer_addr);
  return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)

  extern uint8_t highDutyCycleMode;
  extern uint8_t estConnWhiteList;
  extern uint8 GAPMultiRole_addPeerAddr(uint8_t *pAddr,uint8 addrType,uint8 en_connect );
  estConnWhiteList = init_filt_policy;
  highDutyCycleMode = false;
  memcpy(save_peer_addr,peer_addr,sizeof(peer_addr));
  GAPMultiRole_addPeerAddr(peer_addr,peer_addr_type,TRUE);
  osal_set_event( gapMultiRole_TaskID, ESTABLISH_CONN_EVT );
  return CLI_BLE_RETURN_SUCCESS;
#endif
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif
}

static uint16 cli_conn_cancel(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_MULTI_ROLE)
  extern uint8 GAPMultiRole_delPeerAddr(uint8_t *pAddr );
  GAPMultiRole_delPeerAddr(save_peer_addr);
#endif
  HCI_LE_CreateConnCancelCmd();

  return CLI_BLE_RETURN_SUCCESS;
}

/// 2 parameters: conn_handle
static uint16 cli_terminate_conn(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  if( argc != 1 )
  {
      AT_LOG("Invaild Paraments numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint16 conn_handle;
  conn_handle = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
  extern bStatus_t GAPRole_TerminateConnection( void );
  GAPRole_TerminateConnection();
#elif(CLI_ROLE == CLI_CLIENT_ROLE)
  extern bStatus_t GAPCentralRole_TerminateLink( uint16 connHandle );
  GAPCentralRole_TerminateLink(conn_handle);

#elif(CLI_ROLE == CLI_MULTI_ROLE)
  extern uint8 GAPMultiRole_delPeerAddr(uint8_t *pAddr );
  GAPMultiRole_delPeerAddr(save_peer_addr);

#endif
  return CLI_BLE_RETURN_SUCCESS;
}

/// 6 parameters --  mode:enable=1 / conn_handle / min_conn_interval / max_conn_interval / slave_latency / supervision_timeout
static uint16 cli_slave_conn_param_update(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  #if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE) 
  if(argc != 6)
  {
    AT_LOG("Invaild parameters numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint8_t mode;
  uint16_t conn_handle,min_conn_interval,max_conn_interval,slave_latency,supervision_timeout;
  mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
  ///0x0006 < conn_interval < 0x0C80
  min_conn_interval = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
  max_conn_interval = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
  ///0x0000 < slave_latency < 0x01F3
  slave_latency = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 16);
  ///0x000A < supervision_timeout < 0x0C80
  supervision_timeout = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 16);
  if(mode > 1 || conn_handle == 0xFFFF)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }

#if(CLI_ROLE == CLI_PERIPHERAL_ROLE)
  extern uint16 gapRole_MinConnInterval;
  extern uint16 gapRole_MaxConnInterval;
  extern uint16 gapRole_SlaveLatency;
  extern uint16 gapRole_TimeoutMultiplier;
  gapRole_MinConnInterval = min_conn_interval;
  gapRole_MaxConnInterval = max_conn_interval;
  gapRole_SlaveLatency = slave_latency;
  gapRole_TimeoutMultiplier = supervision_timeout;
  GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &min_conn_interval );
  GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &max_conn_interval );
  GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &slave_latency );
  GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &supervision_timeout );
  GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &mode );
  return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
  cli_conn_handle = conn_handle;
  GAPMultiRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &min_conn_interval,0 );
  GAPMultiRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &max_conn_interval,0  );
  GAPMultiRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &slave_latency,0  );
  GAPMultiRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &supervision_timeout,0  );
  GAPMultiRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8), &mode,0 );
  osal_set_event( gapMultiRole_TaskID, SLAVE_CONN_PARAM_UPDATE );
  return CLI_BLE_RETURN_SUCCESS;
#endif
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif

}

/// 2 parameters : conn_handle  /  mut_size 23--517
static uint16 cli_mtu_exchange(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
#if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE) 
    if(argc != 2)
    {
      AT_LOG("Invaild Paraments numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    uint16 conn_handle;
    uint16_t mtu_size;
    conn_handle = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    mtu_size = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    if(mtu_size < 23 || mtu_size > 517 || conn_handle == 0xFFFF)
    {
      AT_LOG("Invaild Paraments\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
#if(CLI_ROLE == CLI_CLIENT_ROLE)
    ATT_SetMTUSizeMax(mtu_size);
    attExchangeMTUReq_t pReq;
    pReq.clientRxMTU = mtu_size;
    uint8_t status=GATT_ExchangeMTU(conn_handle,&pReq, cli_ble_osal_id);
    AT_LOG( "[MTU Req]%d %d conn_handle = %d\n",status,pReq.clientRxMTU,conn_handle);
	return CLI_BLE_RETURN_SUCCESS;
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	cli_conn_handle = conn_handle;
	cli_mtu_size = mtu_size;
	osal_set_event( gapMultiRole_TaskID, MTU_EXCHANGE_EVT );
	return CLI_BLE_RETURN_SUCCESS;
#endif
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif
   
}

/// 4 parameters  :  mode = enable or disable / conn_handle / tx_octets = 0x001B-0x00FB  / tx_times = 0x0148-0x0848
static uint16 cli_dlu_enable(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");

    if(argc != 4)
    {
      AT_LOG("Invaild Paraments numbers\n");
      return CLI_BLE_INVAILD_PARAMS_NUMBERS;
    }
    uint8_t dlu_mode;
    uint16_t conn_handle,tx_octets,tx_times;
    dlu_mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
    conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    tx_octets = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
    tx_times = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
    if(dlu_mode > 1 || 0x001B > tx_octets || tx_octets > 0x00FB || 0x0148 > tx_times || tx_times > 0x0848)
    {
      AT_LOG("Invaild Paraments\n");
      return CLI_BLE_INVAILD_PARAMS;
    }
    if(dlu_mode == false)
    {
      llInitFeatureSetDLE(FALSE);
    }
    else
    {
      llInitFeatureSetDLE(TRUE);
      HCI_LE_SetDataLengthCmd(conn_handle,tx_octets,tx_times );
      AT_LOG("DLE[ %2d %2d ]\r\n",tx_octets,tx_times);
    }
    
    return CLI_BLE_RETURN_SUCCESS;

}

/// 5 parameters : conn_handle / all_phys / tx_phys / rx_phys / phy_options 
static uint16 cli_phy_set(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");

  if(argc != 5)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint16_t conn_handle,all_phys,tx_phys,rx_phys,phy_options;
  conn_handle = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
  ///Preferred PHYS selection  3 means tx and rx no perferred
  all_phys = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
  ///Preferred PHYS for TX
  tx_phys = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
  ///Preferred PHYS for RX
  rx_phys = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
  phy_options = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 16);
  if(conn_handle == 0xFFFF || all_phys > 3 || tx_phys > 7 || rx_phys > 7 || phy_options > 3)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
  llInitFeatureSet2MPHY(TRUE);
  HCI_LE_SetDefaultPhyMode(0,0x00,0x03,0x03);
  bStatus_t status = HCI_LE_SetPhyMode(conn_handle,all_phys,tx_phys,rx_phys,phy_options);
  if(status!=SUCCESS)
  {
  	AT_LOG("error:%d\n",status);
    return CLI_BLE_INVAILD_PARAMS;
  }

  return CLI_BLE_RETURN_SUCCESS;

}


/// 2 parameters : conn_handle  /  read_char_handle
static uint16 cli_gatt_read(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  #if(CLI_ROLE == CLI_CLIENT_ROLE ||CLI_ROLE == CLI_MULTI_ROLE)
  if(argc != 2)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint16_t conn_handle;
  attReadReq_t pReq_read_char;
  conn_handle = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
  pReq_read_char.handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
  if(conn_handle == 0xFFFF)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
#if(CLI_ROLE == CLI_CLIENT_ROLE) 
  bStatus_t status = GATT_ReadCharValue(conn_handle,&pReq_read_char,cli_ble_osal_id);
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	bStatus_t status = GATT_ReadCharValue(conn_handle,&pReq_read_char,gapMultiRole_TaskID);
#endif
  if(status == SUCCESS)
  {
    AT_LOG("send success\r\n");
    return CLI_BLE_RETURN_SUCCESS;
  }
  else
  {
    AT_LOG("send  error %d\r\n",status);
    return CLI_BLE_INVAILD_PARAMS;
  } 
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif
	

}

///4 parameters : conn_handle / ntf_char_handle / length / value
static uint16 cli_gatt_notify(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  #if(CLI_ROLE == CLI_PERIPHERAL_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
  if(argc != 4)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  uint16_t conn_handle;
  attHandleValueNoti_t ntf;
  conn_handle = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
  ntf.handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 8);
  ntf.len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
  uint8_t return_status = CLI_strtoarray(argv[3], CLI_strlen(argv[3]), ntf.value,ntf.len);
  if(conn_handle == 0xFFFF || return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
  bStatus_t status = GATT_Notification(conn_handle,&ntf,FALSE);
  if(status == SUCCESS)
  {
    AT_LOG("send success\r\n");
    return CLI_BLE_RETURN_SUCCESS;
  }
  else
  {
    AT_LOG("send error %d\r\n",status);
    return CLI_BLE_INVAILD_PARAMS;
  }
#else
	 return CLI_BLE_INVAILD_PARAMS;
#endif

}

/// 5 parameters : write_mode(req=1 or cmd=0) / conn_handle / char_handle / length / value
static uint16 cli_write_req(uint32 argc, unsigned char *argv[])
{
 AT_LOG("match\r\n");
  #if(CLI_ROLE == CLI_CLIENT_ROLE || CLI_ROLE == CLI_MULTI_ROLE)
  if(argc != 5)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  /// req=1   cmd=0
  uint8_t write_mode;
  uint16_t conn_handle;
  attWriteReq_t write_req_or_rsp;
  bStatus_t status;
  write_mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  conn_handle = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
  write_req_or_rsp.handle = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
  write_req_or_rsp.len = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
  uint8_t return_status = CLI_strtoarray(argv[4], CLI_strlen(argv[4]), write_req_or_rsp.value,write_req_or_rsp.len);
  write_req_or_rsp.sig = FALSE;
  if(conn_handle == 0xFFFF || return_status != CLI_BLE_RETURN_SUCCESS)
  {
    AT_LOG("Invaild Paraments\n");
    return CLI_BLE_INVAILD_PARAMS;
  }
  if(write_mode == true)
  {
    write_req_or_rsp.cmd = false;
#if(CLI_ROLE == CLI_CLIENT_ROLE)
    status = GATT_WriteCharValue(conn_handle,&write_req_or_rsp,cli_ble_osal_id);
#elif(CLI_ROLE == CLI_MULTI_ROLE)
	  status = GATT_WriteCharValue(conn_handle,&write_req_or_rsp,gapMultiRole_TaskID);
#endif 
  }
  else
  {
    write_req_or_rsp.cmd = true;
    status = GATT_WriteNoRsp(conn_handle,&write_req_or_rsp);
  }
  
  if(status == SUCCESS)
  {
    AT_LOG("send success\r\n");
    return CLI_BLE_RETURN_SUCCESS;
  }
  else
  {
    AT_LOG("send  error %d\r\n",status);
    return CLI_BLE_INVAILD_PARAMS;
  } 
#else
	return CLI_BLE_INVAILD_PARAMS;
#endif
	
	
}

/// 5 : pairMode / passkey / mitm / ioCap / bonding 
static uint16 cli_authen_enable(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  if(argc != 5)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }
  ///GAPBOND_PAIRING_MODE_NO_PAIRING / GAPBOND_PAIRING_MODE_WAIT_FOR_REQ / GAPBOND_PAIRING_MODE_INITIATE
  uint8 pairMode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 8);
  uint32 passkey = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 32);
  uint8 mitm = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 8);
  ///GAPBOND_IO_CAP_DISPLAY_ONLY/ GAPBOND_IO_CAP_DISPLAY_YES_NO/ GAPBOND_IO_CAP_KEYBOARD_ONLY/ GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT/ GAPBOND_IO_CAP_KEYBOARD_DISPLAY
  uint8 ioCap = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 8);
  uint8 bonding = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 8);

  GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
  GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
  GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
  GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
  GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );

  return CLI_BLE_RETURN_SUCCESS;

}

extern int phy6212_ll_info_show(void);
static uint16 cli_ll_info_show(uint32 argc, unsigned char *argv[])
{
  AT_LOG("match\r\n");
  if(argc != 0)
  {
    AT_LOG("Invaild Paraments numbers\n");
    return CLI_BLE_INVAILD_PARAMS_NUMBERS;
  }

  phy6212_ll_info_show();
  return CLI_BLE_RETURN_SUCCESS;

}


#endif



