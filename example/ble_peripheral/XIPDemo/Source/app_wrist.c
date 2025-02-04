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
  Filename:       wrist.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "ota_app_service.h"
#include "gatt_profile_uuid.h"
#include "wristservice.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "app_wrist.h"
//#include "ui_page.h"
//#include "ui_task.h"
//#include "ui_display.h"
//#include "touch_key.h"
//#include "em70xx.h"
//#include "KX023.h"
//#include "battery.h"
#include "led_light.h"
#include "kscan.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// How often to perform heart rate periodic event
#define DEFAULT_HEARTRATE_PERIOD              2000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000



// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 AppWrist_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
/*static uint8 scanData[] =
{
  0x10,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'C',
  'm',
  't',
  '+',
  ' ',
  'W',
  'r',
  'i',
  's',
  't',
  ' ',
  'D',
  'e',
  'm',
  'o',
};*/

static uint8 scanData[] =
{
  0xa,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'X',
  'I',
  'P',
  ' ',
  'D',
  'e',
  'm',
  'o',
  ' ',
};


static uint8 advertData[] = 
{ 
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUIDs
  0x03,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HEARTRATE_SERV_UUID),
  HI_UINT16(HEARTRATE_SERV_UUID),
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "XIP Demo ";

// GAP connection handle
static uint16 gapConnHandle;

// Advertising user-cancelled state
static bool WristAdvCancelled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void appWristProcOSALMsg( osal_event_hdr_t *pMsg );
static void WristGapStateCB( gaprole_States_t newState);
//static void HeartRateGapStateCB( gaprole_States_t newState );
//static void heartRatePeriodicTask( void );
//static void heartRateMeasNotify(void);
static void wristCB(uint8 event, uint8 param_size, uint8* param);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t WristPeripheralCB =
{
  WristGapStateCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
#if 0
static const gapBondCBs_t WristBondCB =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};
#endif



//static KSCAN_ROWS_e kscan_rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18}; 
//static KSCAN_COLS_e kscan_cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};



//static void on_HeartRateValueUpdate(hr_ev_t* pev)
//{
//  switch(pev->ev){
//  case HR_EV_HR_VALUE:
//    {
//      ui_ev_t ev = {
//          .ev = UI_EV_HEARTRATE,
//          .param = (uint16)pev->value,
//          .data = NULL
//      };
//      wristProfileResponseHRValue(pev->value);
//      ui_fsm_run(&ev);
//    }
//    break;
//  case HR_EV_RAW_DATA:
//    {
//      wristProfileResponseHRRawData(pev->value, pev->data);
//    }
//    break;
//  default:
//    break;
//  }
//}









//void on_touchKey(touch_evt_t key_evt)
//{
//  
//  osal_set_event(AppWrist_TaskID, TOUCH_PRESS_EVT);
//}

//void on_kscan_evt(kscan_Evt_t* kscan_evt)
//{
//	uint8_t key[16];
//  uint8_t r,c,k;
//  osal_memset(key, 0, 16);
//	LOG("KSE %d\n",kscan_evt->num);
//	for(uint8_t i=0;i<kscan_evt->num;i++){
//    k = kscan_evt->keys[i].type == KEY_PRESSED ? 1:2;
//    r = kscan_evt->keys[i].row &0x7;
//    c = kscan_evt->keys[i].col &0x7;
//    key[i] = (k << 6)|(c << 3)| r;
//	}
//  wristProfileResponseKScan(kscan_evt->num, key);
//}

//void on_kx023_evt(kx023_ev_t* pev)
//{
//  int g_acc_value[3];
//  if(pev->ev == wmi_event){
//    int i;
//    int gx = 0,gy = 0,gz = 0;
//  	int16_t *acc_data = (int16_t *)pev->data;
//    for(i = 0; i < pev->size/(sizeof(int16_t)*3); i++)
//    {
//      gx += (int)acc_data[0];
//      gy += (int)acc_data[1];
//      gz += (int)acc_data[2];
//      acc_data+=3;
//    }
//    //LOG("X%d,Y%d,Z%d\n",gx,gy,gz);
//    
//    gx = gx*6/pev->size;
//    gy = gy*6/pev->size;
//    gz = gz*6/pev->size;
//    //LOG("X%d,Y%d,Z%d\n",gx,gy,gz);
//    g_acc_value[0] = gx;
//    g_acc_value[1] = gy;
//    g_acc_value[2] = gz;
//    
//    ui_accelerator_event(g_acc_value);
//    wristProfileResponseAccelerationData(gx/4,gy/4,gz/4);
//    
//  }
//}



static void loop(void)
{
  volatile int lc = 5000;
  while(lc>0){
    lc--;
  }
}

static void xip_test(void)
{
  int i,cnt = 0;;
  LOG("when run in RAM: GPIO Toggle case");

  hal_gpio_write(P15, 1);
  for(i = 0; i< 10; i++){
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 1);
    loop();
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    hal_gpio_write(P14, 0);
    LOG("Cnt %d\n",cnt);
  }	
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_Init
 *
 * @brief   Initialization function for the Heart Rate App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
//  static int uipg_msg_key_press(void);
void appWristInit( uint8 task_id )
{
  AppWrist_TaskID = task_id;
	LOG("\n\n\nappWristInit\n\n\n");
	xip_test();
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    uint8 peerPublicAddr[] = {
			0x01,
			0x02,
			0x03,
			0x04,
			0x05,
			0x06
		};

    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
    // set adv channel map
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );


  // Set advertising interval
  {
      uint16 advInt = 400;   // actual time = advInt * 625us
  
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService( );
  ota_app_AddService();
  wristProfile_AddService(wristCB);
  
  app_datetime_init();

  // Setup a delayed profile startup
  osal_set_event( AppWrist_TaskID, START_DEVICE_EVT );
  light_init();
  //hal_gpio_pull_set(P31, PULL_DOWN);
//  light_ctrl(0,10);
//  light_ctrl(1,50);
//  light_ctrl(2,80);
//  light_ctrl(0,0);
//  light_ctrl(1,0);
//  light_ctrl(2,0);
//  em70xx_register(on_HeartRateValueUpdate);
//  touch_init(on_touchKey);
//  ui_init();
//  batt_init();
//	kx023_init(on_kx023_evt);
//  {
//    kscan_Cfg_t kcfg;
//    kcfg.ghost_key_state = NOT_IGNORE_GHOST_KEY;
//    kcfg.key_rows = kscan_rows;
//    kcfg.key_cols = kscan_cols;
//    kcfg.interval = 50;
//    kcfg.evt_handler = on_kscan_evt;
//    //hal_kscan_init(kcfg, AppWrist_TaskID, TIMER_KSCAN_DEBOUNCE_EVT);
//  }

}

/*********************************************************************
 * @fn      HeartRate_ProcessEvent
 *
 * @brief   Heart Rate Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 appWristProcEvt( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( AppWrist_TaskID )) != NULL )
    {
      appWristProcOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 
  

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &WristPeripheralCB );

    // Register with bond manager after starting device
//    GAPBondMgr_Register( (gapBondCBs_t *) &WristBondCB );
    
    return ( events ^ START_DEVICE_EVT );
  }

  if( events & TIMER_DT_EVT)
  {
    app_datetime_sync_handler();
    return ( events ^ TIMER_DT_EVT );
  }
//  if( events & TIMER_UI_EVT)
//  {
//    ui_page_timer_evt(NULL);

//    return ( events ^ TIMER_UI_EVT );
//  }
//  if( events & TOUCH_PRESS_EVT)
//  {
//    ui_key_evt(NULL);

//    return ( events ^ TOUCH_PRESS_EVT );
//  }
//  if( events & TIMER_HR_EVT)
//  {
//    em70xx_timer_handler();
//    return ( events ^ TIMER_HR_EVT);
//  }
//  
//  if( events & TIMER_BATT_EVT)
//  {
//    batt_meas_timeout_handler();
//    return ( events ^ TIMER_BATT_EVT);
//  }
//  if( events & BATT_CHARGE_EVT)
//  {
//    batt_evt_t evt = batt_charge_detect() ? BATT_CHARGE_PLUG : BATT_CHARGE_UNPLUG;
//    LOG("batt_evt_t %d\n", evt);
//    ui_batt_event(evt);
//    return ( events ^ BATT_CHARGE_EVT);
//  }
//  if( events & BATT_VALUE_EVT)
//  {
//    ui_batt_event(BATT_VOLTAGE);
//    return ( events ^ BATT_VALUE_EVT);
//  }
//  if( events & ACC_DATA_EVT)
//  {
//		drv_kx023_event_handle();
//    return ( events ^ ACC_DATA_EVT);
//  }
  
  if( events & TIMER_LIGHT_EVT)
  {
		light_timeout_handle();
    return ( events ^ TIMER_LIGHT_EVT);
  }
  
//  if( events & TIMER_KSCAN_DEBOUNCE_EVT)
//  {
//    hal_kscan_timeout_handler();
//    return ( events ^ TIMER_KSCAN_DEBOUNCE_EVT);
//  }
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      appWristProcOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void appWristProcOSALMsg( osal_event_hdr_t *pMsg )
{
}


/*********************************************************************
 * @fn      WristGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void WristGapStateCB( gaprole_States_t newState )
{
  LOG("WristGapStateCB: %d", newState);
  // if connected
  if (newState == GAPROLE_CONNECTED)
  {
    // get connection handle
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
  }
  // if disconnected
  else if (gapProfileState == GAPROLE_CONNECTED && 
           newState != GAPROLE_CONNECTED)
  {
    uint8 advState = TRUE;

    // reset client characteristic configuration descriptors
    wristProfile_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );    
  }    
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( WristAdvCancelled )
    {
      WristAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    }  
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      heartRateCB
 *
 * @brief   Callback function for Wrist service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void wristCB(uint8 event, uint8 param_size, uint8* param)
{
  switch(event){

		case WRIST_NOTI_ENABLED:
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      //osal_start_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
    }
    break;
  }
  case WRIST_NOTI_DISABLED:
  {
    // stop periodic measurement
    //osal_stop_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT );
    break;
  }

  }
}



/*********************************************************************
*********************************************************************/
