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

/*******************************************************************************
* @file		fs.h
* @brief	Contains all functions support for spi driver
* @version	0.0
* @date		18. Oct. 2017
* @author	
* 
* Copyright(C) 2016, CMOSTEK Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __FS_H__
#define __FS_H__

#include "types.h"

/**************************************************************************************
 * @fn          hal_fs_init
 *
 * @brief       initialize fs.
 *              if fs is new,use fs_start_address and sector_num to config fs.
 *              if fs is not new,read fs every sector head data and check with fs_start_address and sector_num,
 *              if same,the fs is valid,else is invalid.
 *
 * input parameters
 *
 * @param       fs_start_address:
 *              fs zone start address,4Kbyte align.
 *              fs zone should not cover phy code and app code.
 *
 *              sector_num:
 *              fs zone sector number,one sector = 4Kbyte,its minimal size is 3.
 *              fs zone should not cover phy code and app code.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS								fs init success.
 *							PPlus_ERR_FS_UNINITIALIZED	fs has not been inited.
 *							PPlus_ERR_INVALID_PARAM			parameter error,check your parameter.
 *							PPlus_ERR_FS_CONTEXT				fs has data but different with your parameter.
 *							PPlus_ERR_FS_WRITE_FAILED		flash cannot write.
 *							PPlus_ERR_FS_RESERVED_ERROR	reserved error.
 **************************************************************************************/
int hal_fs_init(uint32_t fs_start_address,uint8_t sector_num);

/**************************************************************************************
 * @fn          hal_fs_item_read
 *
 * @brief       read a file from fs.
 *
 * input parameters
 *
 * @param       id:file id,it should be unique.
 *              
 *              buf:file buf to be read.            
 *
 *              buf_len:file buf len.            
 *
 *              len:*len is the file length after read from fs.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS									file write success.
 *							PPlus_ERR_FS_IN_INT						write later beyond int processing.
 *							PPlus_ERR_FS_UNINITIALIZED		fs has not been inited.
 *							PPlus_ERR_FS_PARAMETER				parameter error,check it.
 *							PPlus_ERR_FS_BUFFER_TOO_SMALL	buf is too small.
 *							PPlus_ERR_FS_NOT_FIND_ID			there is no this file in fs.
 **************************************************************************************/
int hal_fs_item_read(uint16_t id,uint8_t* buf,uint16_t buf_len,uint16_t* len);

/**************************************************************************************
 * @fn          hal_fs_item_write
 *
 * @brief       write a file to fs.
 *
 * input parameters
 *
 * @param       id:file id,it should be unique.
 *
 *              buf:file buf.
 *              
 *              len:file length.
 *              
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS									file write success
 *							PPlus_ERR_FS_IN_INT						write later beyond int processing
 *							PPlus_ERR_FS_UNINITIALIZED		fs has not been inited
 *							PPlus_ERR_FS_PARAMETER				parameter error,check it
 *							PPlus_ERR_FS_NOT_ENOUGH_SIZE	there is not enouth size to write this file
 *							PPlus_ERR_FATAL								there is a same id file,when delete it,occur a error
 **************************************************************************************/
int hal_fs_item_write(uint16_t id,uint8_t* buf,uint16_t len);

/**************************************************************************************
 * @fn          hal_fs_get_free_size
 *
 * @brief       get fs free size.
 *              just file data not include file head. 
 *              for example,16bytes=4byte+12byte,free size is 12byte.
 *
 * input parameters
 *
 * @param       None.

 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							free size
 **************************************************************************************/
uint32_t hal_fs_get_free_size(void);

/**************************************************************************************
 * @fn          hal_fs_get_garbage_size
 *
 * @brief       get fs garbage size.
 *              some deleted files is in garbage,its size is garbage sise.
 *              only after garbage collect the garbage will be released to free. 
 *              just file data not include file head. 
 *              for example,16bytes=4byte+12byte,garbage size is 12byte.
 *
 * input parameters
 *
 * @param       None.

 * output parameters
 *
 * @param       garbage_file_num:deleted file number
 *
 * @return      
 *							garbage size
 **************************************************************************************/
int hal_fs_get_garbage_size(uint32_t* garbage_file_num);
/**************************************************************************************
 * @fn          hal_fs_item_del
 *
 * @brief       delete a file from fs.
 *
 * input parameters
 *
 * @param       id:file id,it should be unique.

 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS									file delete success
 *							PPlus_ERR_FS_IN_INT						delete later beyond int processing
 *							PPlus_ERR_FS_UNINITIALIZED		fs has not been inited
 *							PPlus_ERR_FS_NOT_FIND_ID			not find this file in fs
 **************************************************************************************/
int hal_fs_item_del(uint16_t id);

/**************************************************************************************
 * @fn          hal_fs_garbage_collect
 *
 * @brief       release all deleted file zone to free
 *
 * input parameters
 *
 * @param       None.

 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS									collect success
 *							PPlus_ERR_FS_IN_INT						delete later beyond int processing
 *							PPlus_ERR_FS_UNINITIALIZED		fs has not been inited
 *							PPlus_ERR_FS_WRITE_FAILED			flash cannot write.
 *							PPlus_ERR_FS_UNINITIALIZED	  fs has not been inited.
 *							PPlus_ERR_FS_CONTEXT					fs has data but different with your parameter.
 *							PPlus_ERR_FS_RESERVED_ERROR		reserved error.
 **************************************************************************************/
int hal_fs_garbage_collect(void);

/**************************************************************************************
 * @fn          hal_fs_format
 *
 * @brief       format fs.all fs data will be clean.
 *
 * input parameters
 *
 * @param       fs_start_address:
 *              fs zone start address,4Kbyte align.
 *              fs zone should not cover phy code and app code.
 *
 *              sector_num:
 *              fs zone sector number,one sector = 4Kbyte,its minimal size is 3.
 *              fs zone should not cover phy code and app code.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *							PPlus_SUCCESS								fs format and init success.
 *							PPlus_ERR_FS_IN_INT         delete later beyond int processing
 *							PPlus_ERR_FS_UNINITIALIZED	fs has not been inited.
 *							PPlus_ERR_INVALID_PARAM			parameter error,check your parameter.
 *							PPlus_ERR_FS_CONTEXT				fs has data but different with your parameter.
 *							PPlus_ERR_FS_WRITE_FAILED		flash cannot write.
 *							PPlus_ERR_FS_RESERVED_ERROR	reserved error.
 **************************************************************************************/
int hal_fs_format(uint32_t fs_start_address,uint8_t sector_num);

/**************************************************************************************
 * @fn          hal_fs_initialized
 *
 * @brief       fs has been initialized or not.
 *              if not,fs is disable,please use hal_fs_init to initialize it.
 *
 * input parameters
 *
 * @param       	none.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *              TRUE
 *              FALSE
 **************************************************************************************/
bool hal_fs_initialized(void);

#endif
