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


/* --------------------------------------------- Header File Inclusion */
#include "cliface.h"
#include "log.h"
//#ifdef HAVE_CLI
CLI_COMMAND* g_cli_cmd_list;
uint8_t g_cli_cmd_len;

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Function */
/**
    \fn CLI_init

    \brief Initialize CLI

    \Description
    This routine intializes CLI.

    \return EM_SUCCESS or an error code indicating reason for failure

*/
uint16_t CLI_init
(
    CLI_COMMAND* list,
    uint8_t   len
)
{
    g_cli_cmd_list = list;
    g_cli_cmd_len = len;
    return 0;
}

/**
    \brief Process a command line instruction

    \Description
    This routine processes a command line instruction.

    \param [in] buffer        Buffer containing a command
    \param [in] buffer_len    Length of command in buffer
    \param [in] cmd_list      Command List
    \param [in] cmd_count     Number of command in the list

    \return EM_SUCCESS or an error code indicating reason for failure
*/
uint16_t CLI_process_line
(
    /* IN */ uint8_t*        buffer,
    /* IN */ uint32_t        buffer_len
//    /* IN */ CLI_COMMAND* cmd_list,
//    /* IN */ UINT32        cmd_count
)
{
    uint32_t  argc;
    uint8_t*   argv[CLI_MAX_ARGS];
    uint8_t*   cmd;
    uint32_t   index;
    /* TBD: Parameter Validation */
    // AT_LOG("come in process cli buffer_len = %d\r\n",buffer_len);
    CLI_NULL_CHECK(buffer);

    /* Skip initial white spaces & Skip the first invalid data received after power-on */
    for (; buffer[0] > 128 || CLI_IS_WHITE_SPACE(*buffer) && (0 != buffer_len); buffer++, buffer_len--);

    /* Check this is not an empty command line */
    if (0 == buffer_len)
    {
        CLI_ERR(
            "[CLI] Empty command line\n");
        return PPlus_ERR_INVALID_LENGTH;
    }

    /**
        Got the initial command.
        Parse the remaining command line to get the arguments.
    */
    argc = 0;

    for (cmd = buffer + 1; cmd < (buffer + buffer_len); cmd++)
    {
        /**
            If command argument separator is detected, replace with '\0'
            to create each as separate strings.
        */
        if (CLI_IS_CMD_SEPARATOR(*cmd))
        {
            *cmd = '\0';
        }
        /* Check if this is start of a new argument */
        else if ('\0' == (*(cmd - 1)))
        {
            argv[argc++] = cmd;
        }
        else
        {
            /* Nothing to do */
        }
    }

    CLI_TRC(
        "[CLI] Command %s, Number of arguments %d\n", buffer, argc);
    {
        uint8_t ai;

        for (ai = 0; ai < argc; ai++)
        {
            CLI_TRC(
                "Arg [%02X] %s\n", ai, argv[ai]);
        }
    }
    /* Identified command name */
    cmd = buffer;

    /* Search command and call associated callback */
    for (index = 0; index < g_cli_cmd_len; index++)
    {
        // AT_LOG("--found cli list index = %d --\r\n",index);
        if (0 == CLI_STR_COMPARE(buffer, g_cli_cmd_list[index].cmd))
        {
            AT_LOG("-- match cli fun --\r\n");
            g_cli_cmd_list[index].cmd_hdlr(argc, argv);
            break;
        }
    }

    return PPlus_SUCCESS;
}

/* TODO: Create a separe utility module or move to a common utility module */
/* Supporting Macros */
#define IS_SPACE(c) ((' ' == (c)) || ('\t' == (c)))
#define IS_DIGIT(c) (('0' <= (c)) && ('9' >= (c)))
#define IS_UPPER(c) (('A' <= (c)) && ('F' >= (c)))
#define IS_LOWER(c) (('a' <= (c)) && ('f' >= (c)))
#define IS_ALPHA(c) IS_LOWER(c) || IS_UPPER(c)

/* Convert string to Integer */
int32_t CLI_strtoi
(
    /* IN */ uint8_t* data,
    /* IN */ uint16_t data_length,
    /* IN */ uint8_t base
)
{
    int32_t  value;
    uint16_t index;
    int8_t   sign_adj;
    uint8_t  c;
    c = 0;

    /* Skip Whitespaces */
    for (index = 0; index < data_length; index++)
    {
        c = data[index];

        if (IS_SPACE(c))
        {
            continue;
        }
        else
        {
            break;
        }
    }

    value = 0;
    sign_adj = 1;

    /* Check Sign */
    if ('-' == c)
    {
        sign_adj = (int8_t)-1;
        index++;
    }

    /* Not handling spaces after '-' or '0x' etc. */
    for (; index < data_length; index++)
    {
        c = data[index];

        /* Check if Digit */
        if (IS_DIGIT(c))
        {
            value *= base;
            value += (c - '0');
        }
        else if (IS_LOWER(c))
        {
            value *= base;
            value += (c - 'a' + 10);
        }
        else if (IS_UPPER(c))
        {
            value *= base;
            value += (c - 'A' + 10);
        }
        else
        {
            break;
        }
    }

    return (sign_adj * value);
}

/* Convert string to Integer Array */
uint16_t CLI_strtoarray
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
)
{
    int32_t  index;
    uint8_t  c0, c1;
    uint8_t  base;
    uint16_t output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    memset(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return PPlus_ERR_INVALID_LENGTH;
    }

    /* Process from end */
    output_index = output_array_len - 1;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        output_array[output_index] = c0 * base + c1;
        output_index--;
    }

    return PPlus_SUCCESS;
}

/* Convert string to Integer Array in Little Endian Packing */
uint16_t CLI_strtoarray_le
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
)
{
    int32_t  index;
    uint8_t  c0, c1;
    uint8_t  base;
    uint16_t output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    memset(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return PPlus_ERR_INVALID_LENGTH;
    }

    /* Process from end */
    output_index = 0;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        output_array[output_index] = c0 * base + c1;
        output_index++;
    }

    return PPlus_SUCCESS;
}

//#endif /* HAVE_CLI */

