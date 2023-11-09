
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"
#include "log.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "sbpProfile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "flash.h"
#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "clock.h"
#include "common.h"
#include "rflib.h"
#include "config.h"


#define ADVERTISING_INTERVAL_MS(x) (x / 0.625) // convert milliseconds to 0.625ms units


static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static uint8 iBeaconAdvData[] =
{	
    0x02, // length of data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of data
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0xFF, // Company ID
    0xFF, // Company ID
    0x02, // Data Type
    0x15, // Data Length
    0xBC, // UUID
    0x5F, // UUID 
    0x06, // UUID
    0x62, // UUID
    0x8C, // UUID
    0xD4, // UUID
    0x48, // UUID
    0x38, // UUID
    0xB6, // UUID
    0xB7, // UUID
    0x52, // UUID
    0x8B, // UUID
    0xBE, // UUID
    0x93, // UUID
    0x9A, // UUID
    0x9F, // UUID
    0x00, // Major
    0x00, // Major
    DEVICE_ID_MSB, // Minor
    DEVICE_ID_LSB, // Minor
    (uint8_t)DEVICE_CALIBRATED_RSSI_1M // Power - The 2's complement of the calibrated Tx Power
};


static void simpleBLEPeripheral_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gaprole_States_t newState);
static void peripheralStateReadRssiCB(int8 rssi);
static void setBleMacAddress(void);


// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SimpleBLEPeripheral_Init(uint8 task_id)
{
    simpleBLEPeripheral_TaskID = task_id;
	
		setBleMacAddress();

		uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;
		GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advType);
		
		uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
		GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        

		uint8 initial_advertising_enable = TRUE;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);

		uint16 advInt = ADVERTISING_INTERVAL_MS(BLE_ADV_INTERVAL_MS);
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
		
		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(iBeaconAdvData), iBeaconAdvData);

    // Setup a delayed profile startup
    osal_set_event(simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID );

        if ( pMsg != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            osal_msg_deallocate( pMsg );
        }

        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

        return ( events ^ SBP_START_DEVICE_EVT );
    }
		
    if ( events & SBP_RESET_ADV_EVT )
    {  
        return ( events ^ SBP_RESET_ADV_EVT );
    }  
		
    if (events & SBP_SCAN_RSP_EVT )
    {
        return ( events ^ SBP_SCAN_RSP_EVT );
    }

    if ( events & SBP_PERIODIC_EVT )
    {
        return ( events ^ SBP_PERIODIC_EVT );
    } 

    // Discard unknown events
    return 0;
}

static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{

}

static void peripheralStateReadRssiCB( int8  rssi )
{

}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
   
}

/* Sets device BLE MAC address.
 * 
 * The address of memory to be written is found in one of example sources in SDK.
 * It is not documented anywhere.
 */
static void setBleMacAddress(void)
{
		const uint32_t BLE_MAC_MEMORY_ADDRESS = 0x1fff1231;
	
	  const uint8 DEVICE_MAC_ADDRESS[B_ADDR_LEN] = BLE_MAC_ADDRESS;
		
		volatile uint8_t * p_ownPublicAddr = (volatile uint8_t*)BLE_MAC_MEMORY_ADDRESS;
		
		for (int i = 0; i < B_ADDR_LEN; i++)
				p_ownPublicAddr[i] = DEVICE_MAC_ADDRESS[i];
}
