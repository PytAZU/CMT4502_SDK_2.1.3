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


/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "hal_mcu.h"
#include "slboot.h"
#include "OSAL_Tasks.h"
#include "slb.h"
#include "ota_flash.h"
#include "error.h"
//const uint8 tasksCnt = 0;
//uint16 *tasksEvents;
//extern void jump_to_application(uint32_t run_addr);
__asm void jump_to_application(int address)
{
	MOV PC, R0
}


void jump2app(void)
{
  uint32_t* prunpc = (uint32_t*)(0x1fff4800);
  jump_to_application(prunpc[1]);
}

int run_application(void)
{
  int ret;
  HAL_ENTER_CRITICAL_SECTION();
  ret = ota_flash_load_app();
  if(ret == PPlus_SUCCESS){
    jump2app();
  }
  
  HAL_EXIT_CRITICAL_SECTION();
	return PPlus_SUCCESS;

}

void slboot_main(void)
{
  //check firmware update (exchange area)
	slb_boot_load_exch_zone();
	
	//boot firmware
	run_application();
	
	while(1){
			;
	}
}





/*********************************************************************
*********************************************************************/
