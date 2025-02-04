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



/************************************************************** 
 *
 * Module Name:	display substance
 * File name:	ui_disp_substance.c 
 * Brief description:
 *    UI display substance.
 * Author:	Eagle.Lao
 * Revision:V0.02
****************************************************************/
#if(CFG_DISP==DISP_OLED)

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ui_display.h"
#include "ui_disp_substance.h"
#include "res_tab.h"
#include "log.h"

//static disp_rect_t disp_battery_bar(disp_point_t point)
//{
//	
//	uint8_t percent = 0;//batt_percent();
//	disp_rect_t  rect = disp_idb_f(point, IDB_ICON_batt);
//	disp_rect_t  rect_ret = rect;
//	rect.x = point.x+1;
//	rect.y = point.y+1;
//	rect.h = 5;
//	rect.w = 8*percent/100;

//	RECT_V2H(rect);
//	
//	disp_draw_rect(rect);
//	
//	return rect_ret;
//}



void ui_normal_start_ota(const char* mac)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 0);
	disp_string(point, "Start OTA:");
	DISP_POINT(point, 6, 20);
	disp_string(point, "Connecting AppFW");
	DISP_POINT(point, 6, 40);
	disp_string(point, mac);
	disp_reflash_all();

}
void ui_normal_ota_running(const char* mac)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 0);
	disp_string(point, "OTAing:");
	DISP_POINT(point, 6, 20);
	disp_string(point, "0 %%");
	DISP_POINT(point, 6, 40);
	disp_string(point, mac);
	disp_reflash_all();

}
void ui_normal_ota_percent(uint8_t percent)
{
	disp_rect_t rect;
	disp_point_t point;
  char str[10];
	DISP_RECT(rect, 6,20, 64, 20);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 20);
	sprintf(str,"%d %%", percent);
	disp_string(point, str);
	disp_reflash_all();

}
void ui_normal_ota_finished(void)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 20);
	disp_string(point, "Finished!");
	disp_reflash_all();

}
void ui_normal_acc_init(void)
{
	char str[15];
  disp_rect_t rect;
	disp_point_t point;
  
  DISP_RECT_FULL(rect);
  disp_clrscn(rect);
  
	sprintf(str, "x:0");
	DISP_POINT(point, 4, 4);
  disp_string_f(point, str);
  
	sprintf(str, "y:0");
	DISP_POINT(point, 4, 40);
  disp_string_f(point, str);
  
	sprintf(str, "z:0");
	DISP_POINT(point, 4, 76);
  disp_string_f(point, str);

  DISP_RECT(rect, 4, 20, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect);
  DISP_RECT(rect, 5, 21, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32, 23, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  DISP_RECT(rect, 4, 56, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect);
  DISP_RECT(rect, 5, 57, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32, 58, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  DISP_RECT(rect, 4, 92, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect);
  DISP_RECT(rect, 5, 93, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32, 95, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  
	disp_reflash_all();
  
}

void ui_normal_acc(int x, int y, int z)
{
	char str[32];
  disp_rect_t rect;
	disp_point_t point;
	float fx,fy,fz;
  //LOG("X%d,Y%d,Z%d\n",x,y,z);
  fx = x/200.0;
  fy = z/200.0;
  fz = y/200.0;

  
	sprintf(str, "x: %.2f       ",fx/20);
	DISP_POINT(point, 4, 4);
  disp_string_f(point, str);
  
	sprintf(str, "y: %.2f       ",fy/20);
	DISP_POINT(point, 4, 40);
  disp_string_f(point, str);
  
	sprintf(str, "z: %.2f       ",fz/20);
	DISP_POINT(point, 4, 76);
  disp_string_f(point, str);

  DISP_RECT(rect, 6, 23, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32 + (int)fx, 23, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  DISP_RECT(rect, 6, 58, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32 + (int)fy, 58, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  DISP_RECT(rect, 6, 95, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32 + (int)fz, 95, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect);

  
	disp_reflash_all();



}

void ui_normal_pedometer_steps(uint16_t steps, uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t length;
	uint16_t type = 0;
	uint16_t offset = 0;

	sprintf(str, "%d", steps);
	length = strlen(str);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 7, 30);
	if(frame_id%2)
		disp_idb_f(point, IDB_ICON_steps);
	else
		disp_idb_f(point, IDB_ICON_steps1);

	DISP_POINT(point, 0, 64);
	//display icon
	switch(length){
	case 1:
		type = DIGI_1;
		offset = 12;
		break;
	case 2:
		type = DIGI_1;
		offset = 6;
		break;
	case 3:
		type = DIGI_1;
		offset = 1;
		break;
	case 4:
		type = DIGI_2;
		offset = 1;
		break;
	case 5:
		type = DIGI_3;
		offset = 0;
		break;
	default:
		break;

	}

	point.x = offset;	
	disp_digi_f(point, type, str);
	disp_reflash_all();

}


void ui_normal_calorie_value(uint16_t calorie)
{
	disp_rect_t rect;
	disp_point_t point;

	char str[10];
	uint8_t length;
	uint16_t type = 0;
	uint16_t offset = 0;
	
	calorie = 0;//motion_get_calorie();

	sprintf(str, "%d", calorie);
	length = strlen(str);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);


	//display icon
	DISP_POINT(point, 7, 30);
	
	disp_idb_f(point, IDB_ICON_kalorie);

	DISP_POINT(point, 0, 64);
	//display icon
	switch(length){
	case 1:
		type = DIGI_1;
		offset = 12;
		break;
	case 2:
		type = DIGI_1;
		offset = 6;
		break;
	case 3:
		type = DIGI_1;
		offset = 1;
		break;
	case 4:
		type = DIGI_2;
		offset = 1;
		break;
	case 5:
		type = DIGI_3;
		offset = 0;
		break;
	default:
		break;

	}

	point.x = offset;	
	disp_digi_f(point, type, str);
	disp_reflash_all();

}




void ui_normal_hr_blink(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	if(frame_id%2){
		DISP_POINT(point, 5, 30);
		disp_idb_f(point, IDB_ICON_heartrate);
	}
	else
	{
		DISP_POINT(point, 6, 32);
		disp_idb_f(point, IDB_ICON_heartrate1);
	}

	DISP_POINT(point, 6, 68);

	disp_idb_f(point, IDB_ICON_heartrate_w);


	DISP_POINT(point, 4, 88);
	disp_idb_f(point, IDB_SUFFIX_bpm);


	
	disp_reflash_all();

}

void ui_normal_hr_failure(void)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 5, 30);
	disp_idb_f(point, IDB_ICON_heartrate);

	DISP_POINT(point, 11, 67);
	disp_idb_f(point, IDB_ICON_heartrate_x);


	DISP_POINT(point, 4, 88);
	disp_idb_f(point, IDB_SUFFIX_bpm);

	
	disp_reflash_all();

}


void ui_normal_hr_heartrate(uint8_t value)
{
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t length;
	//uint16_t type = 0;
	uint16_t offset = 7;

	sprintf(str, "%d", value);
	length = strlen(str);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 5, 30);
	disp_idb_f(point, IDB_ICON_heartrate);
	
	if(length > 2)
		offset = 1;
	DISP_POINT(point, offset, 64);
	disp_digi_f(point, DIGI_1, str);

	DISP_POINT(point, 4, 88);
	disp_idb_f(point, IDB_SUFFIX_bpm);

	
	disp_reflash_all();


}

void ui_normal_humiture_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 10, (0 - frame_id*8));
	
	disp_idb(point, IDB_ICON_blood_pressure);
	disp_reflash();

}

void ui_normal_cup_icon(uint8_t frame_id){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	if(frame_id%2){
		DISP_POINT(point, 6, 46);
		disp_idb_f(point, IDB_ICON_press0);
	}
	else
	{
		DISP_POINT(point, 6, 48);
		disp_idb_f(point, IDB_ICON_press1);
	}
	disp_reflash_all();

}

void ui_normal_batt_reflash(uint8_t frame_id, uint8_t percent){
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t len;

	sprintf(str, "%d%%",0);//batt_percent());
	len = strlen(str);
	
	if(frame_id == 0){
		DISP_RECT_FULL(rect);
		disp_clrscn(rect);
		DISP_POINT(point, 2, 52);
		disp_idb_f(point, IDB_ICON_batt0);
	}
	else
	{
		DISP_RECT(rect, 4, 54, 20, 13);
		RECT_V2H(rect);
		disp_clrscn(rect);
	}
	percent = (percent <100)? percent:100;
	DISP_RECT(rect, 6,57,(16*percent/100), 8);

	RECT_V2H(rect);
	
	disp_draw_rect(rect);

	if(len == 2)
	{
		DISP_POINT(point, 8, 78);
	}
	else if(len == 3)
	{
		DISP_POINT(point, 4, 78);
	}
	else
	{
		DISP_POINT(point, 0, 78);
	}
	disp_string_f(point, str);
	
	disp_reflash_all();

}

void ui_normal_call_icon(void){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_call);
	disp_reflash_all();
}

void ui_normal_call_info(uint16_t frame_id, const char* title){
	disp_rect_t rect;
	disp_point_t point;
	uint16_t idx = frame_id;

	if(idx >= 14)
		idx = 14;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46 - idx*3);
	disp_idb_f(point, IDB_ICON_call);

	//display title
	DISP_POINT(point, 68 - idx*3, 8);
	disp_string_f(point, title);
	
	disp_reflash_all();
}



void ui_normal_alarm_clk_icon(void){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_alarmclock);
	disp_on();
	disp_reflash_all();
}

void ui_firmware_upgrade(void){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_fw_upgrade);
	//disp_on();
	disp_reflash_all();
	disp_on();
}

#endif/*CFG_DISP==DISP_OLED*/


