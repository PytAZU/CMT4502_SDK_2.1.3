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


#ifndef _CRC16_H__
#define _CRC16_H__

#include <stdint.h>

unsigned short crc16_CCITT(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_CCITT_FALSE(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_XMODEM(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_X25(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_IBM(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_MAXIM(unsigned char *puchMsg, unsigned int usDataLen);
unsigned short crc16_USB(unsigned char *puchMsg, unsigned int usDataLen);
uint16_t crc16(uint16_t seed, const volatile void * p_data, uint32_t size);

#endif // _CRC16_H__

