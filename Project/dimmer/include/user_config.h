/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "stdlib.h"

#include "esp_common.h"
#include "pwm.h"
#include "ota_config.h"
#include "esp_common.h"
#include "lwip/mem.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "upgrade.h"
#include "uart.h"

#define DELAYTASK1MS  (1/portTICK_RATE_MS)
 // os_printf("[Dbg][%d](%s): ",__LINE__, __FILE__);\ //config in esp_libc.h
#define DBG_PRINT(fmt,...)	do{os_printf(fmt,##__VA_ARGS__);}while(0)

#define ERR_PRINT(fmt,...) do{\
	    os_printf("[Err] Fun:%s Line:%d file:%s: ",__FUNCTION__,__LINE__, __FILE__);\
	    os_printf(fmt,##__VA_ARGS__);\
	}while(0)
#define DBG_LINES(v) os_printf("------------------%s---------------\n",v)

#define DEMO_AP_SSID       "HLSoft"
#define DEMO_AP_PASSWORD   "mothaiba"
#define SOFT_AP_SSID       "RasHomeIoT"
#define SOFT_AP_PASSWORD   "12345678"

#define IR
#ifdef RGB
#define DEVICE_TYPE           "LED_DIMMER_RGB_WF"
#endif
#ifdef CW
#define DEVICE_TYPE           "LED_DIMMER_CW_WF"
#endif
#ifdef IR 
#define DEVICE_TYPE           "IR_CONTROL_WF"
#endif 

#define FW_VERSION            "V42"
#define FLASH_SECTOR_SIZE     (4*1024)
#define FLASH_PAGE_SIZE       (4)  
#define FLASH_START_ADD       0xfb000
#define FLASH_END_ADD         0xfbFFF
#define FLASH_TEST_ADD        0xfb000


#define BUF_SIZE (1024)
#define MSG_SER_SIZE_MAX  (256)
/**************PIN SETTINGS FOR PWM***************/
#ifdef CW
/*Cold pin 12*/
#define PWM_0_OUT_IO_MUX PERIPHS_IO_MUX_MTDI_U
#define PWM_0_OUT_IO_NUM 12
#define PWM_0_OUT_IO_FUNC  FUNC_GPIO12

// #define PWM_1_OUT_IO_MUX PERIPHS_IO_MUX_MTCK_U
// #define PWM_1_OUT_IO_NUM 13
// #define PWM_1_OUT_IO_FUNC  FUNC_GPIO13

/*Warm pin 14*/
#define PWM_2_OUT_IO_MUX PERIPHS_IO_MUX_MTMS_U
#define PWM_2_OUT_IO_NUM 14
#define PWM_2_OUT_IO_FUNC  FUNC_GPIO14

// #define PWM_3_OUT_IO_MUX PERIPHS_IO_MUX_MTDO_U
// #define PWM_3_OUT_IO_NUM 15
// #define PWM_3_OUT_IO_FUNC  FUNC_GPIO15

// #define PWM_4_OUT_IO_MUX PERIPHS_IO_MUX_GPIO4_U
// #define PWM_4_OUT_IO_NUM 4
// #define PWM_4_OUT_IO_FUNC  FUNC_GPIO4

#define PWM_NUM_CHANNEL_NUM    2  //number of PWM Channels
#define COLD_CHANNEL  0
#define WARM_CHANNEL  1
#endif
/********************* END *************************/

typedef enum {
  UNINSTALL = 0, 
  LEARN_WF, 
  RST_FAC, 
  INACTIVE_PAIRING, 
  INACTIVE_PAIRING_W,  
  ACTIVE_PAIRING, 
  ACTIVE_PAIRING_W, 
  WORKING
} deviceWorkState;

typedef enum { 
  ON = 0,
  OFF, 
  SYN
} statusComm;

#if defined (CW) || defined (RGB)
typedef struct { 
  deviceWorkState state; 
  deviceWorkState preState; 
  statusComm startup; 
  statusComm status; 
  uint8 secretKey; 
  uint8 id; 
  uint8 warm_val; 
  uint8 cold_val;
  uint8 color_val; 
  uint8 red_val;
  uint8 green_val;
  uint8 blue_val;
  uint8 brightness_val;
  uint8 rainbow_val;
  bool rgb_state;
} deviceWorkInfo;
#endif
#ifdef IR
typedef struct { 
  deviceWorkState state; 
  deviceWorkState preState; 
  statusComm startup; 
  statusComm status; 
  uint8 secretKey; 
  uint8 id; 
  uint8 power; 
  uint8 temp;
  uint8 cool; 
  uint8 swing;
  uint8 fan;
  uint8 heat;
  uint8 mode;
  uint8 manufactuer;
} deviceWorkInfo;
#endif
deviceWorkInfo device1;

extern char msgSer[MSG_SER_SIZE_MAX];
extern char msgSerRe[MSG_SER_SIZE_MAX];
extern uint8 idxMsgCom;

extern bool flagUpdateFW;
extern bool flagConnect;
extern bool flagWifiConnected;
extern bool flagSendCbSer;

extern void show_info(void);
#if defined (CW)|| defined (RGB)
extern void brightnessUpdate(bool rgb, uint8 clr, uint8 percent);
#endif

#endif

