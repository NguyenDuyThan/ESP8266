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

#include "esp_common.h"
#include "espconn.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "conn_ap.h"

#include "user_config.h"

void wifi_handle_event_cb(System_Event_t *evt){
  // printf("event %x\n", evt->event_id);
  switch (evt->event_id) {
    case EVENT_STAMODE_CONNECTED:
      DBG_PRINT("connect to ssid %s, channel %d\n", evt->event_info.connected.ssid, evt->event_info.connected.channel);
      break;
    case EVENT_STAMODE_DISCONNECTED:
      flagWifiConnected = false;
      DBG_PRINT("disconnect from ssid %s, reason %d\n", evt->event_info.disconnected.ssid, evt->event_info.disconnected.reason);
        break;
    case EVENT_STAMODE_AUTHMODE_CHANGE:
      DBG_PRINT("mode: %d -> %d\n", evt->event_info.auth_change.old_mode, evt->event_info.auth_change.new_mode);
      break;
    case EVENT_STAMODE_GOT_IP:
      flagWifiConnected = true;
      DBG_PRINT("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR "\n", IP2STR(&evt->event_info.got_ip.ip), IP2STR(&evt->event_info.got_ip.mask), IP2STR(&evt->event_info.got_ip.gw));
      break;
    case EVENT_SOFTAPMODE_STACONNECTED:
      DBG_PRINT("station: " MACSTR " join, AID = %d\n", MAC2STR(evt->event_info.sta_connected.mac), evt->event_info.sta_connected.aid);
      break;
    case EVENT_SOFTAPMODE_STADISCONNECTED:
      DBG_PRINT("station: " MACSTR " leave, AID = %d\n", MAC2STR(evt->event_info.sta_disconnected.mac), evt->event_info.sta_disconnected.aid);
      break;
    default:
      break;
  }
}

void conn_ap_init(void){
  // wifi_set_opmode(STATION_MODE);
  // wifi_station_dhcpc_stop(); //add for static ip Mr.Than
  // wifi_softap_dhcps_stop();
  // struct ip_info info;
  // IP4_ADDR(&info.ip, 192, 168, 1, 252); // set IP
  // IP4_ADDR(&info.gw, 192, 168, 1, 1); // set gateway
  // IP4_ADDR(&info.netmask, 255, 255, 255, 0); // set netmask
  // wifi_set_ip_info(SOFTAP_IF, &info);
  // struct station_config config;
  // memset(&config, 0, sizeof(config));  //set value of config from address of &config to width of size to be value '0'
  // sprintf(config.ssid, DEMO_AP_SSID);
  // sprintf(config.password, DEMO_AP_PASSWORD);
  // wifi_station_set_config(&config); //this function will save to flash 
  // wifi_station_set_config_current(&config); //not save to flash
  wifi_set_event_handler_cb(wifi_handle_event_cb);
  wifi_set_sleep_type(NONE_SLEEP_T);
  wifi_station_connect();
  //STATION_STATUS wifi_station_get_connect_status(void);
}

void soft_ap_init(void)
{
  wifi_set_opmode(SOFTAP_MODE);
  struct softap_config *config = (struct softap_config *) zalloc(sizeof(struct softap_config)); // initialization
  // wifi_softap_set_config_current(config); 
  wifi_softap_get_config(config); // Get soft-AP config first.
  sprintf(config->ssid, SOFT_AP_SSID);
  sprintf(config->password, SOFT_AP_PASSWORD);
  config->authmode = AUTH_WPA_WPA2_PSK;
  config->ssid_len = 0; // or its actual SSID length
  config->max_connection = 4;
  wifi_softap_set_config(config); // Set ESP8266 soft-AP config
  free(config);
  struct station_info * station = wifi_softap_get_station_info();
  while (station) {
    DBG_PRINT("bssid :" MACSTR ", ip: " IPSTR "\n", MAC2STR(station->bssid), IP2STR(&station->ip));
    station = STAILQ_NEXT(station, next);
  }
  wifi_softap_free_station_info(); // Free it by calling functionss
  wifi_softap_dhcps_stop(); // disable soft-AP DHCP server
  struct ip_info info;
  IP4_ADDR(&info.ip, 192, 168, 4, 1); // set IP
  IP4_ADDR(&info.gw, 192, 168, 1, 1); // set gateway
  IP4_ADDR(&info.netmask, 255, 255, 255, 0); // set netmask
  wifi_set_ip_info(SOFTAP_IF, &info);
  struct dhcps_lease dhcp_lease;
  IP4_ADDR(&dhcp_lease.start_ip, 192, 168, 4, 100);
  IP4_ADDR(&dhcp_lease.end_ip, 192, 168, 4, 105);
  wifi_softap_set_dhcps_lease(&dhcp_lease);
  wifi_softap_dhcps_start(); // enable soft-AP DHCP server
}

