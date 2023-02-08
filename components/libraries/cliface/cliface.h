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


#ifndef _H_CLIFACE_
#define _H_CLIFACE_

/* --------------------------------------------- Header File Inclusion */
#include "types.h"
#include "error.h" 
#include "string.h"


/* --------------------------------------------- Macros */

/* Debug Macros */
/* TBD: Mapped with debug sub-system */
#define CLI_ERR(...)       /* printf(__VA_ARGS__) */
#define CLI_TRC(...)       /* printf(__VA_ARGS__) */
#define CLI_INF(...)       /* printf(__VA_ARGS__) */

#define CLI_STR_COMPARE(s0, s1)    strcmp((const char *)(s0), (const char *)(s1))

#define CLI_NULL_CHECK(ptr) \
    if (NULL == (ptr)) \
    {\
        CLI_ERR( \
                 "[CLI] NULL Pointer\n"); \
        \
        return PPlus_ERR_INVALID_DATA; \
    }

#define CLI_IS_WHITE_SPACE(ch)   ((' ' == (ch)) || ('\t' == (ch))|| ('\r' == (ch)))
#define CLI_IS_CMD_SEPARATOR(ch) ((' ' == (ch)) || (',' == (ch)) || ('\t' == (ch)) || ('\r' == (ch)) || ('\n' == (ch)))

/** TBD: Move to limits/configuration header file */
#define CLI_MAX_ARGS    16

#define CLI_strlen(s)   strlen((const char *)(s))

/* --------------------------------------------- Data Types/ Structures */
/**
    CLI command handler.

    CLI will call the handler for the received command.

    \param [in] argc    Number of arguments.
    \param [in] argv    List of arguments.
*/
typedef uint16_t (* CLI_CMD_HANDLER)
(
    uint32_t        argc,
    uint8_t*        argv[]
) ;

/** This data structure represents a CLI command */
typedef struct _cli_command
{
    /** Command name */
    const uint8_t*            cmd;

    /* Command description */
    const uint8_t*            desc;

    /** Command handler */
    const CLI_CMD_HANDLER   cmd_hdlr;

} CLI_COMMAND;

/* --------------------------------------------- Global Definitions */
extern CLI_COMMAND* g_cli_cmd_list;
extern uint8_t g_cli_cmd_len;



/* --------------------------------------------- Functions */
uint16_t CLI_init
(
    CLI_COMMAND* list,
    uint8_t   len
);


uint16_t CLI_process_line
(
    /* IN */ uint8_t*        buffer,
    /* IN */ uint32_t        buffer_len
//    /* IN */ CLI_COMMAND* cmd_list,
//    /* IN */ UINT32        cmd_count
);

int32_t CLI_strtoi
(
    /* IN */ uint8_t* data,
    /* IN */ uint16_t data_length,
    /* IN */ uint8_t base
);

uint16_t CLI_strtoarray
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
);

uint16_t CLI_strtoarray_le
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
);

#endif /* _H_CLIFACE_ */


