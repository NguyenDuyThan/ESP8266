/* export PATH=/opt/xtensa-lx106-elf/bin:$PATH
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

//start server in linux:  python3 -m http.server 8091
#include "user_config.h"

#if defined (CW)|| defined (RGB)
uint8 brn_current_val = 0;
uint8 color_current_val = 0;
uint8 rainbow_current_val = 0;
#endif

char msgSer[MSG_SER_SIZE_MAX];
char msgSerRe[MSG_SER_SIZE_MAX];
uint8 idxMsgCom = 0;

bool flagConnect = false;
bool flagUpdateFW = false;
bool flagWifiConnected = false;
bool flagSendCbSer = false;

char *ssid = "HLSoft";
char *pwd = "mothaiba";
char *ip = "192.168.1.252";
char *gw = "192.168.1.1";
char *nestmask = "255.255.255.0";

void fota_event_cb(void);
void config_wifi(char *ssid, char *pwd, char *ip, char *gw, char *nestmask);
void readFlash(uint32 src_addr, uint32 *des_addr, uint32 size);
void eraseFlash(uint32 addr);
void writeFlash(uint32 des_addr, uint32 *src_addr, uint32 size);
/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void){
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

/******************************************************************************
 * FunctionName : TCP_Server_Task and UDP_Server_Task
 * Description  : some function for network task
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void TCP_Server_Task(){
  TcpLocalServer();
}

void UDP_Server_Task(){
  udpServer();
}

/*--------------------------------------------------------------
 Access MSG json
 -------------------------------------------------------------*/
void returnDataSer(char *msg){
  memset(msgSerRe, '\0', sizeof(msgSerRe));
  if((msg!=NULL)||(strlen(msg)>0)){
    sprintf(msgSerRe, "%s", msg);
  } else sprintf(msgSerRe, "%s", "{\"res\":\"OK\"}\n");
  flagSendCbSer = true;
}

void processMsgServer(char *msg){
  if(idxMsgCom > 0){
    if(strlen(msg)>0){
      char *text;
      text = (char*)calloc(strlen(msg) + 1, sizeof(char));
      sprintf(text, "%s", msg);
      idxMsgCom--;
      cJSON *json;
      json=cJSON_Parse(text);
      if (!json) {
        DBG_PRINT("Error before: [%s]\n",cJSON_GetErrorPtr());
      } else {
        // out=cJSON_Print(json);
        char *req = cJSON_GetObjectItem(json,"req")->valuestring;
        char *res;
        res = (char*)calloc(MSG_SER_SIZE_MAX,sizeof(char));
        DBG_PRINT("req: %s\n",req);
        if(device1.state == LEARN_WF){
          if(strcmp(req, "DEVICE") == 0){
            sprintf(res,"{\"res\":\"DEVICE\", \"type\":\"%s\"}\n", DEVICE_TYPE);
          } else if(strcmp(req, "NETWORK") == 0){
            char *ipState_ser = cJSON_GetObjectItem(json, "ip")->valuestring;
            if(strcmp(ipState_ser,"STATIC")==0){
              char *prelen = cJSON_GetObjectItem(json, "network_prefix_len")->valuestring;
              char *ssid = cJSON_GetObjectItem(json, "ssid")->valuestring;
              char *pwd = cJSON_GetObjectItem(json, "password")->valuestring;
              char *ip = cJSON_GetObjectItem(json, "ip_static")->valuestring;
              char *gw = cJSON_GetObjectItem(json, "gateway")->valuestring;
              if(strcmp(prelen,"8")==0){
                char *nestmask = "255.0.0.0";
                config_wifi(ssid, pwd, ip, gw, nestmask);
              } else if(strcmp(prelen,"16")==0){
                char *nestmask = "255.255.0.0";
                config_wifi(ssid, pwd, ip, gw, nestmask);
              } else if(strcmp(prelen,"24")==0){
                char *nestmask = "255.255.255.0";
                config_wifi(ssid, pwd, ip, gw, nestmask);
              }
              free(prelen);
              free(ssid);
              free(pwd);
              free(ip);
              free(gw);
            }
            free(ipState_ser);
          } 
        } else {
          if(strcmp(req, "CONTROL") == 0){
            uint8 color = 0;
            uint8 brn = 0;
            bool _rgb = false;
            char *status = cJSON_GetObjectItem(json, "status")->valuestring;
            if(strcmp(status, "ON") == 0){
              device1.status = ON;
            } else {
              device1.status = OFF;
            }
            free(status);
            color = cJSON_GetObjectItem(json,"color")->valueint;
            brn = cJSON_GetObjectItem(json,"brn")->valueint;
            if(1 == cJSON_GetObjectItem(json,"rgb")->valueint){
              _rgb = true;
            }
            brightnessUpdate(_rgb, color, brn);
          } else if (strcmp(req, "ERASE_FLASH") == 0){
            char *pos = cJSON_GetObjectItem(json,"pos")->valuestring;
            int posInt = atoi(pos);
            DBG_PRINT("start erase flash\n");
            eraseFlash(FLASH_TEST_ADD + posInt);
            free(pos);
          } else if (strcmp(req, "READ_FLASH") == 0){
            char *pos = cJSON_GetObjectItem(json,"pos")->valuestring;
            int posInt = atoi(pos);
            char *sz = cJSON_GetObjectItem(json,"size")->valuestring;
            int szInt = atoi(sz);
            DBG_PRINT("start read flash\n");
            int bufSize = szInt + 1;
            char bufRead[bufSize];
            memset(bufRead, '\0', sizeof(bufRead));
            readFlash(FLASH_TEST_ADD + posInt, (uint32 *)bufRead, szInt);
            int i = 0;
            DBG_PRINT("value of byte: \n");
            for( i=0; i<bufSize; i++){
              printf("\t%d",i);
            }
            printf("\n");
            for( i=0; i<bufSize; i++){
              printf("\t%02X",bufRead[i]);
            }
            printf("\n");
            free(pos);
            free(sz);
          } else if (strcmp(req, "WRITE_FLASH") == 0){
            DBG_PRINT("start write flash\n");
            char *value = cJSON_GetObjectItem(json,"val")->valuestring;
            char *pos = cJSON_GetObjectItem(json,"pos")->valuestring;
            char *sz = cJSON_GetObjectItem(json,"size")->valuestring;
            int szInt = atoi(sz);
            int posInt = atoi(pos);
            writeFlash(FLASH_TEST_ADD + posInt, (uint32 *)value, szInt);
            free(value);
            free(sz);
            free(pos);
          } else if (strcmp(req, "UPDATE_FW") == 0){
            DBG_PRINT("Receive command update firmware\n");
            flagUpdateFW = true;
          } else if (strcmp(req, "INFO") == 0){
            DBG_PRINT("Show INFO:\n");
            show_info();
          } else if (strcmp(req, "RESTART") == 0){
            DBG_PRINT("Reboot...\n");
            system_restart();
          }
          cJSON_Delete(json);
          // DBG_PRINT("%s\n",out);
        }
        returnDataSer(res);
        free(req);
        free(res);
      }
      free(text);
    }
  }
}

void msg_server_task(void* arg){
  DBG_PRINT("Start process msg server task\n");
  for(;;){
    processMsgServer(msgSer);
    vTaskDelay(400/portTICK_RATE_MS);
  }
}

/******************************************************************************
 * FunctionName : lightInit channelControl brightnessUpdate processLight led_task
 * Description  : setup pwm channel for control dimming led
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
#if defined (CW)|| defined (RGB)
void lightInit(void){
  uint32 io_info[][3] = {
            { PWM_0_OUT_IO_MUX, PWM_0_OUT_IO_FUNC, PWM_0_OUT_IO_NUM }, //Channel 0
            // { PWM_1_OUT_IO_MUX, PWM_1_OUT_IO_FUNC, PWM_1_OUT_IO_NUM }, //Channel 1
            { PWM_2_OUT_IO_MUX, PWM_2_OUT_IO_FUNC, PWM_2_OUT_IO_NUM }, //Channel 2
            // { PWM_3_OUT_IO_MUX, PWM_3_OUT_IO_FUNC, PWM_3_OUT_IO_NUM }, //Channel 3
            // { PWM_4_OUT_IO_MUX, PWM_4_OUT_IO_FUNC, PWM_4_OUT_IO_NUM }, //Channel 4
    };

	u32 duty[2] = {
    0, 
    // 512, 
    // 512, 
    // 512, 
    0}; //Max duty cycle is 1023
    
	pwm_init(1000, duty , PWM_NUM_CHANNEL_NUM, io_info);
  DBG_PRINT("Setup led success\n");
}

/*
 directly control pwm channel
*/

void channelControl(uint8 channel, uint32 value){
  pwm_set_duty(value, channel);
  pwm_start();   //Call this: every time you change duty/period
}

void brightnessUpdate(bool rgb, uint8 clr, uint8 brn){
  if (rgb == false){
    device1.brightness_val = brn;
    device1.color_val = clr;
    device1.rainbow_val = 0;
    device1.rgb_state = false;
  }  
#ifdef RGB
  else {
    brn_current_val = device1.brightness_val = 0;
    device1.rainbow_val = clr;
    device1.color_val = 0;
    device1.rgb_state = true;
  }
#endif
}

#ifdef  RGB
void rainbow(unsigned char index) {
    if (flagRGB) {
        if (index < 85) {
            _my92xx->setChannel(MY92XX_RED, index * 3);
            _my92xx->setChannel(MY92XX_GREEN, 255 - index * 3);
            _my92xx->setChannel(MY92XX_BLUE, 0);
        } else if (index < 170) {
            index -= 85;
            _my92xx->setChannel(MY92XX_RED, 255 - index * 3);
            _my92xx->setChannel(MY92XX_GREEN, 0);
            _my92xx->setChannel(MY92XX_BLUE, index * 3);
        } else {
            index -= 170;
            _my92xx->setChannel(MY92XX_RED, 0);
            _my92xx->setChannel(MY92XX_GREEN, index * 3);
            _my92xx->setChannel(MY92XX_BLUE, 255 - index * 3);
        }
    } else {
        _my92xx->setChannel(MY92XX_RED, 0);
        _my92xx->setChannel(MY92XX_GREEN, 0);
        _my92xx->setChannel(MY92XX_BLUE, 0);
    }
    _my92xx->update();
}
#endif

void processLight(void){
  if (device1.status == ON){
#ifdef RGB
    while (rainbow_current_val != device1.rainbow_val){
      if (rainbow_current_val > device1.rainbow_val){
        uint8 delta = rainbow_current_val - device1.rainbow_val;
        if (delta > 127){
          rainbow_current_val +=1;
          if (rainbow_current_val >= 255){
            rainbow_current_val = 0;
          }
        } else {
          rainbow_current_val -=1;
          if (rainbow_current_val <= 0){
            rainbow_current_val = 255;
          }
        }
      } else if (rainbow_current_val < device1.rainbow_val){
        uint8 delta = device1.rainbow_val - rainbow_current_val;
        if (delta > 127){
          rainbow_current_val -=1;
          if (rainbow_current_val <= 0){
            rainbow_current_val = 255;
          }
        } else {
          rainbow_current_val +=1;
          if (rainbow_current_val >= 255){
            rainbow_current_val = 0;
          }
        }
      }
      rainbow(rainbow_current_val);
      vTaskDelay(4/portTICK_RATE_MS);
    }
#endif
    if ((brn_current_val != device1.brightness_val) || (color_current_val != device1.color_val)){
      uint32 brn_t, color_t;
      uint8 i = 0;
      for(i=0;i<128;i++){
        if((brn_current_val == device1.brightness_val) && (color_current_val == device1.color_val)){
          break;
        } else {
          if (brn_current_val > device1.brightness_val){
            brn_current_val -=1;
          } else if (brn_current_val < device1.brightness_val){
            brn_current_val +=1;
          } 
          if (color_current_val > device1.color_val){
            color_current_val -=1;
          } else if (color_current_val < device1.color_val){
            color_current_val +=1;
          }
    #ifdef  CW
          brn_t = brn_current_val*1023/100;
          color_t = color_current_val*brn_t/255;
          channelControl(WARM_CHANNEL, color_t);
          channelControl(COLD_CHANNEL, brn_t - color_t);
    #endif
    #ifdef  RGB
          brn_t = brn_current_val*255/100;
          color_t = color_current_val*brn_t/255;
          // _my92xx->setChannel(MY92XX_WARM, color_t);
          // _my92xx->setChannel(MY92XX_COLD, brn_t - color_t);
          // _my92xx->update();
    #endif
          if((abs(brn_current_val - device1.brightness_val) > 63)||(abs(color_current_val - device1.color_val) > 63)){
            vTaskDelay(5*DELAYTASK1MS);
          } else {
            vTaskDelay(8*DELAYTASK1MS);
          }
        }
      }
    }
  } else if (device1.status == OFF){
    if ((brn_current_val != 0) || (color_current_val != 0)){
      uint8 i = 0;
      uint32 brn_t, color_t;
      for(i=0;i<255;i++){
        if ((brn_current_val == 0) && (color_current_val == 0)){
          break;
        } else {
          if(brn_current_val > 0){
            brn_current_val -=1;
          }
          if(color_current_val > 0){
            color_current_val -=1;
          }
    #ifdef  CW
          brn_t = brn_current_val*1023/100;
          color_t = color_current_val*brn_t/255;
          channelControl(WARM_CHANNEL, color_t);
          channelControl(COLD_CHANNEL, brn_t - color_t);
    #endif
    #ifdef  RGB
          brn_t = brn_current_val*255/100;
          color_t = color_current_val*brn_t/255;
          // _my92xx->setChannel(MY92XX_WARM, color_t);
          // _my92xx->setChannel(MY92XX_COLD, brn_t - color_t);
          // _my92xx->update();
    #endif
          if((abs(brn_current_val - device1.brightness_val) > 63)||(abs(color_current_val - device1.color_val) > 63)){
            vTaskDelay(5*DELAYTASK1MS);
          } else {
            vTaskDelay(8*DELAYTASK1MS);
          }
        }
      }
  #ifdef  RGB
      // _my92xx->setChannel(MY92XX_WARM, 0);
      // _my92xx->setChannel(MY92XX_COLD, 0);
      // _my92xx->setChannel(MY92XX_RED, 0);
      // _my92xx->setChannel(MY92XX_GREEN, 0);
      // _my92xx->setChannel(MY92XX_BLUE, 0);
      // _my92xx->update();
  #endif
    }
  }
}

void led_task(void* arg){
  DBG_PRINT("Start led task\n");
  lightInit();
	for(;;){
    processLight();
		vTaskDelay(400*DELAYTASK1MS); //400 milliSec Delay
	}
} 
#endif
/******************************************************************************
 * FunctionName : config_wifi
 * Description  : setup wifi with info input
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void config_wifi(char *ssid, char *pwd, char *ip, char *gw, char *nestmask){
  uint8 ipStatic[4] = {192,168,1,100};
  uint8 gwStatic[4] = {192,168,1,1};
  uint8 nestmaskStatic[4] = {255,255,255,0};
  uint8 idx = 0;
  char *token;
  token = strtok(ip, ".");
  while( token != NULL ){
    if(idx>3) break;
    ipStatic[idx] = atoi(token);
    token = strtok(NULL, ".");
    idx++;
  }
  idx = 0;
  token = strtok(gw, ".");
  while( token != NULL ){
    if(idx>3) break;
    gwStatic[idx] = atoi(token);
    token = strtok(NULL, ".");
    idx++;
  }
  idx = 0;
  token = strtok(nestmask, ".");
  while( token != NULL ){
    if(idx>3) break;
    nestmaskStatic[idx] = atoi(token);
    token = strtok(NULL, ".");
    idx++;
  }
  free(token);
  DBG_PRINT("ip: %d.%d.%d.%d\n",ipStatic[0],ipStatic[1],ipStatic[2],ipStatic[3]);
  DBG_PRINT("gw: %d.%d.%d.%d\n",gwStatic[0],gwStatic[1],gwStatic[2],gwStatic[3]);
  DBG_PRINT("nm: %d.%d.%d.%d\n",nestmaskStatic[0],nestmaskStatic[1],nestmaskStatic[2],nestmaskStatic[3]);
  struct ip_info info;
  IP4_ADDR(&info.ip, ipStatic[0], ipStatic[1], ipStatic[2], ipStatic[3]); // set IP
  IP4_ADDR(&info.gw, gwStatic[0], gwStatic[1], gwStatic[2], gwStatic[3]); // set gateway
  IP4_ADDR(&info.netmask, nestmaskStatic[0], nestmaskStatic[1], nestmaskStatic[2], nestmaskStatic[3]); // set netmask
  
  wifi_set_opmode(STATION_MODE);
  wifi_station_dhcpc_stop();
  wifi_softap_dhcps_status();
  if(!wifi_set_ip_info(STATION_IF, &info)){
    DBG_PRINT("set info ip error!!!\n");
  }
  struct station_config config;
  memset(&config, 0, sizeof(config));  //set value of config from address of &config to width of size to be value '0'
  sprintf(config.ssid, ssid);
  sprintf(config.password, pwd);
  wifi_station_set_config(&config);
  conn_ap_init();
}

/******************************************************************************
 * FunctionName : setUpSys sys_task
 * Description  : common function main of device
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void setUpSys(void){
  device1.state = WORKING;
  DBG_PRINT("Setup your system first here.\n\t\tInclue: readrom, check state,...\n");
}

void sys_task(void* arg){
  bool connectRun = false;
  bool flagStartAP = false;
  uint32 timeStartConnectWifi = system_get_time();
  setUpSys();
  DBG_PRINT("Start system task\n");
  for(;;){
    if((device1.state == WORKING)||(device1.state == ACTIVE_PAIRING)){
      if(flagWifiConnected == false){
        flagConnect = true;
      }
      if(flagConnect){
        config_wifi(ssid, pwd, ip, gw, nestmask);
        timeStartConnectWifi = system_get_time();
        vTaskDelay(1000);
        connectRun = true;
        flagConnect = false;
        // DBG_PRINT("Reboot...\n");
        // system_restart();
      }
      if(connectRun){
        if(system_get_time() - timeStartConnectWifi>5000000){
          if(flagWifiConnected == false){
            flagConnect = true;
          } else {
            connectRun = false;
          }
        }
      }
      if (flagUpdateFW){
        DBG_PRINT("Start update process...\n");
        fota_event_cb();
        flagUpdateFW = false;
      }
    } else { //state == LEARN_WF
      if((flagWifiConnected == true)|| (wifi_get_opmode() == STATION_MODE) || (wifi_get_opmode() == STATIONAP_MODE)){
        if(wifi_station_disconnect()){
          flagStartAP = true;
        }
      } else {
        flagStartAP = true;
      }
      if(flagStartAP){
        soft_ap_init();
        vTaskDelay(1000*DELAYTASK1MS);
        flagStartAP = false;
        // DBG_PRINT("Reboot...\n");
        // system_restart();
      }
    }
		vTaskDelay(400*DELAYTASK1MS); //400 milliSec Delay
  }
}

/******************************************************************************
 * FunctionName : all flash function
 * Description  : setup customer flash
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
/*--------------------------------------------------------------
 READ FLASH not complete
 -------------------------------------------------------------*/
 
void readFlash(uint32 src_addr, uint32 *des_addr, uint32 size){
  if(src_addr < FLASH_START_ADD || src_addr > FLASH_END_ADD){
    DBG_PRINT("addr not use for flash\n");
  } else { 
    int ret = 0;
    if (ret = spi_flash_read(src_addr, des_addr, size) != SPI_FLASH_RESULT_OK){
      DBG_PRINT("read flash error: %d\n", ret);
    }
  }
}

/*--------------------------------------------------------------
 EARSE FLASH
 -------------------------------------------------------------*/
 
void eraseFlash(uint32 addr){
  if(addr < FLASH_START_ADD || addr > FLASH_END_ADD){
    DBG_PRINT("addr not use for flash\n");
  } else { 
    uint16 sector = 0;
    sector = addr/FLASH_SECTOR_SIZE;
    int ret =0 ;
    if (ret = spi_flash_erase_sector(sector) != SPI_FLASH_RESULT_OK){
      DBG_PRINT("clear flash error: %d\n", ret);
    }
  }
}

/*--------------------------------------------------------------
 WRITE FLASH  not complette
 -------------------------------------------------------------*/
 
void writeFlash(uint32 des_addr, uint32 *src_addr, uint32 size){
  if(des_addr < FLASH_START_ADD || des_addr > FLASH_END_ADD){
    DBG_PRINT("addr not use for flash\n");
  } else { 
    uint16 insector = des_addr/FLASH_SECTOR_SIZE;
    uint32 startAdd = FLASH_SECTOR_SIZE*insector;
    // unsigned char *tempFlash;
    // tempFlash = (unsigned char*)calloc(FLASH_SECTOR_SIZE, sizeof(unsigned char));
    char tempFlash[100];
    memset(tempFlash, '\0', 100);
    readFlash(startAdd, (uint32 *)tempFlash, 100);
    eraseFlash(des_addr);
    int ret = 0;
    uint32 fsize = des_addr - startAdd;
    uint32 msize = size;
    uint32 esize = (uint32)(100 - fsize - msize);
    uint32 fdaddr = startAdd;
    uint32 mdaddr = fdaddr + fsize;
    uint32 edaddr = mdaddr + msize;
    uint32 *fsaddr = (uint32 *)tempFlash;
    uint32 *msaddr = src_addr;
    uint32 *esaddr = (uint32 *)tempFlash + fsize + msize;
    if (ret = spi_flash_write(fdaddr, fsaddr, fsize) != SPI_FLASH_RESULT_OK){
      DBG_PRINT("write flash bk1 error: %d\n", ret);
    }
    if (ret = spi_flash_write(mdaddr, msaddr, msize) != SPI_FLASH_RESULT_OK){
      DBG_PRINT("write new data to flash error: %d\n", ret);
    }
    if (ret = spi_flash_write(edaddr, esaddr, esize) != SPI_FLASH_RESULT_OK){
      DBG_PRINT("write flash bk2 error: %d\n", ret);
    }
    // free(tempFlash);
  }
}

/******************************************************************************
 * FunctionName : update function
 * Description  : setup customer flash
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
/*********************global param define start ******************************/
LOCAL os_timer_t upgrade_timer;
LOCAL uint32 totallength = 0;
LOCAL uint32 sumlength = 0;
LOCAL bool flash_erased = false;
LOCAL xTaskHandle *ota_task_handle = NULL;
/*********************global param define end *******************************/

/******************************************************************************
 * FunctionName : upgrade_recycle
 * Description  : recyle upgrade task, if OTA finish switch to run another bin
 * Parameters   :
 * Returns      : none
 *******************************************************************************/
LOCAL void upgrade_recycle(void){
  totallength = 0;
  sumlength = 0;
  flash_erased = false;

  system_upgrade_deinit();
  vTaskDelete(ota_task_handle);
  ota_task_handle = NULL;
  if (system_upgrade_flag_check() == UPGRADE_FLAG_FINISH) {
      system_upgrade_reboot(); // if need
  }
}

/******************************************************************************
 * FunctionName : upgrade_download
 * Description  : parse http response ,and download remote data and write in flash
 * Parameters   : int sta_socket : ota client socket fd
 *                char *pusrdata : remote data
 *                length         : data length
 * Returns      : none
 *******************************************************************************/
void upgrade_download(int sta_socket, char *pusrdata, unsigned short length){
    char *ptr = NULL;
    char *ptmp2 = NULL;
    char lengthbuffer[32];
    if (totallength == 0&& (ptr = (char *)strstr(pusrdata, "\r\n\r\n")) != NULL &&
    (ptr = (char *)strstr(pusrdata, "Content-Length")) != NULL) {
        ptr = (char *) strstr(pusrdata, "\r\n\r\n");
        length -= ptr - pusrdata;
        length -= 4;
        DBG_PRINT("upgrade file download start.\n");

        ptr = (char *) strstr(pusrdata, "Content-Length: ");
        if (ptr != NULL) {
            ptr += 16;
            ptmp2 = (char *) strstr(ptr, "\r\n");

            if (ptmp2 != NULL) {
                memset(lengthbuffer, 0, sizeof(lengthbuffer));
                memcpy(lengthbuffer, ptr, ptmp2 - ptr);
                sumlength = atoi(lengthbuffer);
                if (sumlength > 0) {
                    if (false == system_upgrade(pusrdata, sumlength)) {
                        system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
                        goto ota_recycle;
                    }
                    flash_erased = true;
                    ptr = (char *) strstr(pusrdata, "\r\n\r\n");
                    if (false == system_upgrade(ptr + 4, length)) {
                        system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
                        goto ota_recycle;
                    }
                    totallength += length;
                    DBG_PRINT("sumlength = %d\n", sumlength);
                    return;
                }
            } else {
                DBG_PRINT("sumlength failed\n");
                system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
                goto ota_recycle;
            }
        } else {
            DBG_PRINT("Content-Length: failed\n");
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            goto ota_recycle;
        }
    } else {
        totallength += length;
        if(totallength%1000 == 0){
          DBG_PRINT("totallen = %d\n", totallength);
        }
        if (false == system_upgrade(pusrdata, length)) {
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            goto ota_recycle;
        }
        if (totallength == sumlength) {
            DBG_PRINT("upgrade file download finished.\n");

            if (upgrade_crc_check(system_get_fw_start_sec(), sumlength) != true) {
                DBG_PRINT("upgrade crc check failed !\n");
                system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
                goto ota_recycle;
            }

            system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
            goto ota_recycle;
        } else {
            return;
        }
    }

    ota_recycle: DBG_PRINT("go to ota recycle\n");
    close(sta_socket);
    upgrade_recycle();

}
/******************************************************************************
 * FunctionName : fota_begin
 * Description  : ota_task function
 * Parameters   : task param
 * Returns      : none
 *******************************************************************************/
void fota_begin(void *pvParameters){
    int recbytes;
    int sin_size;
    int sta_socket;
    char recv_buf[1460];
    uint8 user_bin[9] = { 0 };
    struct sockaddr_in remote_ip;
    DBG_PRINT("Hello, welcome to client!\r\n");
    while (1) {
        sta_socket = socket(PF_INET, SOCK_STREAM, 0)
        ;
        if (-1 == sta_socket) {

            close(sta_socket);
            DBG_PRINT("socket fail !\r\n");
            continue;
        }
        DBG_PRINT("socket ok!\r\n");
        bzero(&remote_ip, sizeof(struct sockaddr_in));
        remote_ip.sin_family = AF_INET;
        remote_ip.sin_addr.s_addr = inet_addr(DEMO_SERVER);
        remote_ip.sin_port = htons(DEMO_SERVER_PORT);

        if(0 != connect(sta_socket,(struct sockaddr *)(&remote_ip),sizeof(struct sockaddr)))
        {
            close(sta_socket);
            DBG_PRINT("connect fail!\r\n");
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            upgrade_recycle();
        }
        DBG_PRINT("connect ok!\r\n");
        char *pbuf = (char *) zalloc(512);
        if (system_upgrade_userbin_check() == UPGRADE_FW_BIN1) {
            memcpy(user_bin, "user2.bin", 10);
        } else if (system_upgrade_userbin_check() == UPGRADE_FW_BIN2) {
            memcpy(user_bin, "user1.bin", 10);
        }

        sprintf(pbuf, "GET /%s HTTP/1.0\r\nHost: \"%s\":%d\r\n"pheadbuffer"", user_bin, DEMO_SERVER, 80);

        printf(pbuf);
        if (write(sta_socket,pbuf,strlen(pbuf)+1) < 0) {
            close(sta_socket);
            DBG_PRINT("send fail\n");
            free(pbuf);
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            upgrade_recycle();
        }
        DBG_PRINT("send success\n");
        free(pbuf);

        while ((recbytes = read(sta_socket, recv_buf, 1460)) >= 0) {
            if (recbytes != 0) {
                upgrade_download(sta_socket, recv_buf, recbytes);
            }
        }
        DBG_PRINT("recbytes = %d\n", recbytes);
        if (recbytes < 0) {
            DBG_PRINT("read data fail!\r\n");
            close(sta_socket);
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            upgrade_recycle();
        }
    }
}

/******************************************************************************
 * FunctionName : fota_begin
 * Description  : ota_task function
 * Parameters   : task param
 * Returns      : none
 *******************************************************************************/
// void fota_event_cb(System_Event_t *event)
void fota_event_cb(void){
    // if (event == NULL) {
        // return;
    // }

    // switch (event->event_id) {
        // case EVENT_STAMODE_GOT_IP:
            os_printf("creat fota task\n");
            if (ota_task_handle == NULL) {
                system_upgrade_flag_set(UPGRADE_FLAG_START);
                system_upgrade_init();
                xTaskCreate(fota_begin, "fota_task", 1024, NULL, 1, ota_task_handle);
            }
            os_timer_disarm(&upgrade_timer);
            os_timer_setfn(&upgrade_timer, (os_timer_func_t *) upgrade_recycle, NULL);
            os_timer_arm(&upgrade_timer, OTA_TIMEOUT, 0);
            // break;
        // case EVENT_SOFTAPMODE_STADISCONNECTED:
            // DBG_PRINT("sta disconnect from AP\n");
            // DBG_PRINT("recyle ota task");
            // system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            // upgrade_recycle();
        // default:
            // break;
    // }
}

/******************************************************************************
 * FunctionName : show_info
 * Description  : show status HW, version FW
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void show_info(void){
  DBG_PRINT("SDK version:%s\n", system_get_sdk_version());
  DBG_PRINT("Heap:%u\n", system_get_free_heap_size());
  DBG_PRINT("Chip ID: %u\n", system_get_chip_id());
  DBG_PRINT("Flash ID: %u\n", spi_flash_get_id());
  DBG_PRINT("App TEST Version: %s\n", FW_VERSION);
  DBG_PRINT("Device status: %d\n", device1.status);
#if defined (CW) || defined (RGB)
  DBG_PRINT("Device brightness_val: %d\n", device1.brightness_val);
  DBG_PRINT("Device color_val: %d\n", device1.color_val);
  DBG_PRINT("Device brn_current_val: %d\n", brn_current_val);
  DBG_PRINT("Device color_current_val: %d\n", color_current_val);
#endif
  rst_reason whyreset = system_get_rst_info()->reason;
  DBG_PRINT("Reason reset: \n");
  switch(whyreset){
    case REASON_WDT_RST:
      DBG_PRINT("Watchdog rst\n");
      break;
    case REASON_EXCEPTION_RST:
      DBG_PRINT("Exception rst\n");
      break;
    case REASON_SOFT_WDT_RST:
      DBG_PRINT("Soft watchdog rst\n");
      break;
    case REASON_SOFT_RESTART:
      DBG_PRINT("Soft restart\n");
      break;
    case REASON_DEEP_SLEEP_AWAKE:
      DBG_PRINT("Deep sleep wakeup\n");
      break;
    case REASON_EXT_SYS_RST:
      DBG_PRINT("Ext sys rst\n");
      break;
    default:
      DBG_PRINT("Not found reason: %d\n", whyreset);
      break;
  }
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void){
  // uint32 x = FLASH_SECTOR_SIZE;
  show_info();
  uart_init_conf(); //uart task
  espconn_init(); //call this for use espconn function.
  xTaskCreate(TCP_Server_Task, "TCP_Server_Task", 512, NULL, 5, NULL);
#if defined (CW)|| defined (RGB)
  xTaskCreate(led_task, "led_task", 1024, NULL, 6, NULL);
#endif
  xTaskCreate(UDP_Server_Task, "UDP_Server_Task", 512, NULL, 7, NULL);
  xTaskCreate(sys_task, "sys_task", 1024, NULL, 8, NULL);
  xTaskCreate(msg_server_task, "msg_server_task", 1024, NULL, 9, NULL);
}