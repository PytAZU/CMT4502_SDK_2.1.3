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
  Filename:       gpio_demo.h
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

#ifndef __GPIO_DEMO_H__
#define __GPIO_DEMO_H__

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * FUNCTIONS
 */
void Pulse_Measure_Init( uint8 task_id);
uint16 Pulse_Measure_ProcessEvent( uint8 task_id, uint16 events);


void Timer_Demo_Init( uint8 task_id );
uint16 Timer_Demo_ProcessEvent( uint8 task_id, uint16 events);


void Gpio_Wakeup_Init( uint8 task_id );
uint16 Gpio_Wakeup_ProcessEvent( uint8 task_id, uint16 events);


void Key_Demo_Init( uint8 task_id );
uint16 Key_ProcessEvent( uint8 task_id, uint16 events);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
