/*
//mosquitto_pub -h ece.ddns.net -p 1883 -u "iotmqttcenter" -P "Fhsj5@GDhjw@TH2fdh" -t "iotmqttcenter/feeds/onoff" -m "{CONTROL:OFF}"
//mosquitto_pub -h ece.ddns.net -p 1883 -u "iotmqttcenter" -P "Fhsj5@GDhjw@TH2fdh" -t "iotmqttcenter/feeds/onoff" -m "{UPDATEFW:V}"

This firmware V01 use for series of manual funtion on off from 1 to 3 channel with ESP8266 V12F. May be comlict with another device.
***Added function update OTA: server:ece.ddns.net/firmwareESP8266OnOff/*.bin
*The OTA local server : 192.168.1.234/firmwareESP8266OnOff/*.bin
*CPU:80MHz
*Crystal: 26MHz
*Flash Size: 4M(1M SPIFFS)
*Flash mode: QIO
 */
/*NOTE EEPROM:
  status = ON  => Relay = HIGH & Led = LOW
  status = OFF  => Relay = LOW & Led = HIGH
*/
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <Hash.h>
//#include <ESP8266Ping.h>
#include <stdio.h>
#include "configure.h"
#include "Timer.h"
#include <string.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()
#include <PubSubClient.h>

#define AIO_SERVER      "ece.ddns.net"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "iotmqttcenter"
#define AIO_KEY         "Fhsj5@GDhjw@TH2fdh"

#define TZ              6       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries

#define NTP0_OR_LOCAL1  0       // 0:use NTP  1:fake external RTC
#define RTC_TEST     1510592825 // 1510592825 = Monday 13 November 2017 17:07:05 UTC

#define MQTT_ID                      "MAC_ID"

////////////////////////////////////////////////////////

#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)

// void mylogFc(const char *fmt, ...){
  // if(flagDebug){
    // char log[SIZE_OF_LOG];
    // va_list ap;
    // va_start(ap, fmt);
    // vsnprintf(log, SIZE_OF_LOG, fmt, ap);
    // va_end(ap);
    // Serial.print(__LINE__); Serial.print(": ");
    // Serial.print(log);
  // }
// }

timeval cbtime;      // time set in callback
bool cbtime_set = false;

Timer timeDelay(SYS_100HZ);  // khoi tao timer delay 0.1s
String DEVICE_TYPE = String("MOTOR_TREE");
char newVerFW[5];
boolean Mode_Station = false;

char *Ssid = "";
char *Password = "";
char *Ssid_old = "";
char *Password_old = "";

char *HOST_UPDATE = "http://ece.ddns.net/firmwareESP8266OnOff/";

IPAddress RashomeIP;
IPAddress deviceIp ;
IPAddress default_gateway;    //when connect with modem static
IPAddress subnet_mark;    //when connect with modem static

int SyncNumber = 0;
String seriesNumber="";
String req_fr_server = "";  //string of request from server
String res_of_device = "";  //string of response of device to sent to server
// other informations of Gang1
bool flagUpdateFw = false;
boolean isStatic = false; //trang thai mang DHCP or STATIC
boolean isRasHomeAlive = false; //if server online is will be set true
boolean connectFirst = false; // check for connect with wifi in the first time
boolean reconnectWifi = false; // true is wifi reconnting to modem
boolean flagEnableLedOff = false;   //flag notice enable turn off led is true
boolean flagEnableLedOn = false;    //flag notice enable turn on led is true
boolean flagSyncToRsh   = false;    //flag enable sync to rashome is true
boolean flagEnableRstFc = false;    //flag enable reset factory when submitted by gang master
boolean flagChangeWifi = false;     //is true when new ssid same as old ssid
boolean flagSsidMatched = false;    //is true enable flagChangeWifi is true    
boolean flagTimeView = false;         //is true enable view time current in serial
boolean flagAlarm = false;
boolean flagRelayState = false;
boolean flagStartOpenDeviceByAlarm = false;

unsigned long timeDetectVir = 0;
unsigned long timeStartLearnWF = 0; //time recent start learn wifi mode
unsigned long timeStartServerLearnWF = 0; //time start sever local again if not receive data response of mode learn wifi
unsigned long timeStartCheckConnect = 0; // time begin of checking connect to rashome
unsigned long timeStartCheckConnectWifi = 0; //time begin of checking connect to modem wifi
unsigned long timeStartReConnect = 0; //time begin of restart connect to modem wifi
unsigned long timeStartActivePairing = 0; //time begin of start mode active pairing to pair with device control
unsigned long timeStartRSTFac = 0; //time begin of mode reset factory
unsigned long timePreLedOn = 0; //time begin led on for control led off
unsigned long timePreLedOff = 0; //time begin led of for control led on
unsigned long timeCounterLed = 0; //counter for times led blink
unsigned long timeRunDeviceByAlarm = 0;
unsigned long timeStopAlarm = 0;
unsigned long timeStartOpenDeviceByAlarm = 0;

String warm_brn_val = ""; // value of warm light
String cold_brn_val = ""; // value of cold light
// const size_t bufferSize = JSON_OBJECT_SIZE(4) + 80; //define for decode string json
// DynamicJsonBuffer jsonBuffer(bufferSize);
WiFiUDP Udp;
WiFiServer server(TCP_PORT_SERVER);
WiFiClient client;

/****************************** time zone ****-***************************/
int dayCurrent  = 0;
int dayAlarm =0;
int hourCurrent = 0;
int hourAlarm =0;
int minuteCurrent = 0;
int minuteAlarm = 0;
char strCTime[20];

int sec;
int minu;
int hour;

int timeZone = +7; // utc-4 17eastern daylight time (nyc)
// for testing purpose:
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
timeval tv;
timespec tp;
time_t now;
uint32_t now_ms, now_us;
#define PTM(w) \
  Serial.print(":" #w "="); \
  Serial.print(tm->tm_##w);

void printTm(const char* what, const tm* tm) {
  Serial.print(what);
  PTM(isdst); PTM(yday); PTM(wday);
  PTM(year);  PTM(mon);  PTM(mday);
  PTM(hour);  PTM(min);  PTM(sec);
}

/*************************************************************************/
void button3ProcessCb(void);  //callback for interrupt button 3 defined in congigure.h file
void button2ProcessCb(void);//callback for interrupt button 2 defined in congigure.h file
void button1ProcessCb(void);//callback for interrupt button 1 defined in congigure.h file
void button3Process(void); //process button 3(some mode)
void button2Process(void); //process button 2(some mode)
void button1Process(void); //process button 1(some mode)

void relayControl(device_struct device, relay_state_t relayStatus); //only control relay 
void controlLed(void); 
void feedbackStatus(device_struct device);
void storeIP_tmp(String ip, int offset);
void storeIP(String StrIP, String StrGateway, String StrSubnet, String ipState);
void RHPing(String request);
void RHchangestartup(String request);
void RHchangeWifi(String request);
void RhChangeIP(String request);
String String2Hash(String HashString);
void ClearEeprom();
void iotDiscovery();
void ReadEpprom();
void setupWifiAccessPoint(char *ssid, char* password);
boolean sendDataServer(char *data);
void processDataFromServer(void);
void startupStatusProcess();
void WiFiReconnect();
boolean ResetFactory();
void checkConnectToRasHome();
void checkConnectWifi(void);
void checkCmd(void);
void accessUpdate(String ver);
String splitStringJson(String strSignal, String scr);
void onoffcallback(char *data, uint16_t len);
void timecallback(uint32_t current);
void time_is_set(void);
void check_req_mqtt(char *req);
void MQTT_connect();
void callback(char* topic, byte* payload, unsigned int length);


PubSubClient mqttclient(AIO_SERVER, AIO_SERVERPORT, callback, client);
// void setupSPIFFS();
#define MAX_BUF_REC_SERIAL   128
char sbuf[MAX_BUF_REC_SERIAL];
uint32_t x=0;

bool flagViewTest = false;


void setup() {
    ESP.wdtDisable();
    ESP.wdtEnable(TIME_WTD);
    Serial.begin(115200);
    while(!Serial){
      delay(1);
    }
    EEPROM.begin(EEPROM_SIZE);
    delay(100);
    ReadEpprom();
    delay(10);
    String mac = WiFi.macAddress();
    String date = __DATE__;
    int j,i = 0;
    int len = mac.length();
    date.toUpperCase();
    for (i = 0; i<len; i++){
      if(mac[i] != ':'){
        seriesNumber +=mac[i];
      }
    }
    for(j=0;j<date.length();j++){
      if (date[j] !=' '){
        seriesNumber+=date[j];
      }
    }
    DBGT("\n>>>>> This test "); DBG(DEVICE_TYPE);
    DBGT(">>>>> Version: "); DBG(VERSION);
    DBGT(">>>>> Series: "); DBG(seriesNumber);
#if defined(SW1G) || defined(SW2G) || defined(SW3G)
    device1.relay = Relay;
    device1.button = Button;
    pinMode(Relay, OUTPUT);
    pinMode(Button, INPUT);
    attachInterrupt(Button, button1ProcessCb, CHANGE);
    relayControl(device1, RELAYOFF);
#endif
#if defined(SW2G) || defined(SW3G)
    device2.relay = Relay2;
    device2.button = Button2;
    pinMode(Relay2, OUTPUT);
    pinMode(Button2, INPUT);
    attachInterrupt(Button2, button2ProcessCb, CHANGE);//CHANGE, RISING, FALLING
    relayControl(device2, RELAYOFF);
#endif
#if defined(SW3G)
    device3.relay = Relay3;
    device3.button = Button3;
    pinMode(Relay3, OUTPUT);
    pinMode(Button3, INPUT);
    attachInterrupt(Button3, button3ProcessCb, CHANGE);
    relayControl(device3, RELAYOFF);
#endif
    /******************************************/
    pinMode(Led, OUTPUT);
    digitalWrite(Led, LED_ST_OFF);
    /*******************************************/
    // setupSPIFFS();
    WiFi.hostname("ESP_" MQTT_ID);
    if (Mode_Station && (device1.state == WORKING || device2.state == WORKING || device3.state == WORKING)) {
        WiFi.config(deviceIp, default_gateway, subnet_mark);
        WiFi.begin(Ssid, Password);
        WiFi.mode(WIFI_STA);
        int timeout = 0;
        server.begin();
        while(WiFi.status() != WL_CONNECTED){
            ESP.wdtFeed();
            DBG(".");
            delay(500);
            timeout+=1;
            if (timeout > 20){
                return;
            }
        }
        if (WiFi.status() == WL_CONNECTED){
            DBGT("Connected to wifi : ");  DBG(Ssid);
            DBGT("My IP : "); DBG(WiFi.localIP());
        } else {
            DBGT("Can not connect to wifi : ");  DBG(Ssid);
        }
        //////////////////////////////////////////////////
        if (mqttclient.connect(MQTT_ID, AIO_USERNAME, AIO_KEY)) {
          mqttclient.publish(AIO_USERNAME "/feeds/onoff","hello world");
          mqttclient.subscribe(AIO_USERNAME "/feeds/onoff");
          mqttclient.subscribe(AIO_USERNAME "/" MQTT_ID "/feeds/control");
        }
        
        //set time
        settimeofday_cb(time_is_set);
        configTime(TZ_SEC, DST_SEC, "pool.ntp.org");
    } else {
      setupWifiAccessPoint(AP_SSID, AP_PW);
    }
//    startupStatusProcess();
    DBG("Setup DONE");
    ESP.wdtFeed();
    DBGT("HOUR: "); DBG(hourAlarm);
    hourAlarm = 23;
    DBGT("HOUR: "); DBG(hourAlarm);
}

void loop() {
    ESP.wdtFeed();
    mqttclient.loop();
    // checkConnectToRasHome();
    checkCmd();
    ESP.wdtFeed();
    checkConnectWifi();
    ESP.wdtFeed();
    // checkConnectToRasHome();
    controlLed();
    ESP.wdtFeed();
#if defined(SW2G) || defined(SW3G)
    button2Process();
    ESP.wdtFeed();
#endif
#if defined(SW3G)
    button3Process();
    ESP.wdtFeed();
#endif
#if defined(SW1G) || defined(SW2G) || defined(SW3G)
    button1Process();
    ESP.wdtFeed();
#endif
    processDataFromServer();
    ESP.wdtFeed();

    if(flagUpdateFw){
      accessUpdate(newVerFW);
    }
    gettimeofday(&tv, nullptr);
    clock_gettime(0, &tp);
    now = time(nullptr);
    now_ms = millis();
    now_us = micros();
    if(flagAlarm){
      memset(strCTime, '\0', sizeof(strCTime));
      sprintf(strCTime, "%s", ctime(&now));
      if(strstr(strCTime, "Sun") != NULL){
        dayCurrent = SUNDAY;
      } else if(strstr(strCTime, "Mon") != NULL){
        dayCurrent = MONDAY;
      } else if(strstr(strCTime, "Tue") != NULL){
        dayCurrent = TUEDAY;
      } else if(strstr(strCTime, "Wed") != NULL){
        dayCurrent = WEDDAY;
      } else if(strstr(strCTime, "Thu") != NULL){
        dayCurrent = THUDAY;
      } else if(strstr(strCTime, "Fri") != NULL){
        dayCurrent = FRIDAY;
      } else if(strstr(strCTime, "Sat") != NULL){
        dayCurrent = SATDAY;
      }
      if(flagViewTest){
        DBGT("Day: ");DBG(dayCurrent);
        hourAlarm = 22;
      }
      if (dayCurrent == dayAlarm){
        char *temp;
        temp = (char *)calloc(3,sizeof(char));
        strncpy(temp, strchr(strCTime, ':') - 2, 2);
        if(flagViewTest){
          DBGT("hour alarm: "); DBG(hourAlarm);
          DBGT("hour now: ");DBG(atoi(temp));
        }
        if(atoi(temp) == hourAlarm){
          memset(temp, '\0', sizeof(temp));
          strncpy(temp, strchr(strCTime, ':') + 1, 2);
          if(flagViewTest){
            DBGT("minute alarm:"); DBG(minuteAlarm);
            DBGT("minute now: ");DBG(atoi(temp));
          }
          if (flagStartOpenDeviceByAlarm == false){
            if(atoi(temp) >= minuteAlarm && atoi(temp) <= timeRunDeviceByAlarm + minuteAlarm){
              DBG("Run\n");
              flagStartOpenDeviceByAlarm = true;
              timeStartOpenDeviceByAlarm = millis() + timeRunDeviceByAlarm*60;
              relayControl(device1, RELAYON);
            }
          } else if (timeStartOpenDeviceByAlarm <= millis()){
            flagStartOpenDeviceByAlarm = false;
            relayControl(device1, RELAYOFF);
            EEPROM.write(ADD_ALARM_0, 0);
            EEPROM.commit();
            delay(5);
            flagAlarm = false;
          }
        }
        free(temp);
      }
      flagViewTest = false;
    }
  // localtime / gmtime every second change
//  static time_t lastv = 0;
//  if (lastv != tv.tv_sec) {
//    lastv = tv.tv_sec;
//    Serial.println();
//    printTm("localtime", localtime(&now));
//    Serial.println();
//    printTm("gmtime   ", gmtime(&now));
//    Serial.println();
//    Serial.println();
//  }
  if (flagTimeView){
    flagTimeView = false;
    Serial.print("clock:");
    Serial.print((uint32_t)tp.tv_sec);
    Serial.print("/");
    Serial.print((uint32_t)tp.tv_nsec);
    Serial.print("ns");
  
    // time from boot
    Serial.print(" millis:");
    Serial.print(now_ms);
    Serial.print(" micros:");
    Serial.print(now_us);
  
    // EPOCH+tz+dst
    Serial.print(" gtod:");
    Serial.print((uint32_t)tv.tv_sec);
    Serial.print("/");
    Serial.print((uint32_t)tv.tv_usec);
    Serial.print("us");
  
    // EPOCH+tz+dst
    Serial.print(" time:");
    Serial.print((uint32_t)now);
  
    // human readable
    Serial.print(" ctime:(UTC+");
    Serial.print((uint32_t)(TZ * 60 + DST_MN));
    Serial.print("mn)");
    Serial.print(ctime(&now));
  }
}

#if defined(SW2G) || defined(SW3G)
void button2ProcessCb() {
    if(digitalRead(Button2) == PRESS) {
        bt2.pressed = PRESS;
        bt2.timestart = millis();
        bt2.change = true;
    } else {
        bt2.pressed = RELEASE;
        bt2.timestop = millis();
        bt2.change = true;
        if (device2.state == UNINSTALL) {
            relayControl(device2, RELAY);
        } else if (device2.state == WORKING) {
            if (millis() -  bt2.timestart < TIME_TO_RST_FAC) {
                relayControl(device2, RELAY);
            }
        }
    }
}

void button2Process(){
    if (device1.state == RST_FAC){
#ifdef SW2G
        if (bt2.change) {
            if (bt2.pressed == PRESS){
                bt2.change = false;
            } else {
                bt2.change = false;
                unsigned long time_t = bt2.timestop - bt2.timestart;
                DBG(time_t);
                if (time_t < TIME_ENABLE_RST_FAC){
                    device1.state = device1.preState;
                    DBG("Break change mode to previous mode");
                } else {
                    flagEnableRstFc = true;
                } 
            }
        }
#endif
        if (bt2.change) {
            if (bt2.pressed == PRESS){
                bt2.change = false;
            } else {
                DBG("Waiting reset factory pa...");
                bt2.change = false;
            }
        }
    } else if (device2.state == UNINSTALL) {
        if (bt2.change) {
            if (bt2.pressed == PRESS){
                bt2.change = false;
            } else {
                bt2.change = false;
                unsigned long time_t = bt2.timestop - bt2.timestart;
                DBG(time_t);
                bt2.led = true;
                if (digitalRead(Relay2)== RELAY_ST_ON) {
                    DBG("UNINSTALL: Channel 2 ON");
                } else {
                    DBG("UNINSTALL: Channel 2 OFF");
                }
            }
        }
    } else if (device2.state == WORKING) {
        if (bt2.change) {
            if (bt2.pressed == PRESS){
                bt2.change = false;
            } else {
                bt2.change = false;
                unsigned long time_t = bt2.timestop - bt2.timestart;
                DBG(time_t);
                bt2.led = true;
                if (digitalRead(Relay2)== RELAY_ST_ON) {
                    DBG("WORKING: Channel 2 ON");
                } else {
                    DBG("WORKING: Channel 2 OFF");
                }
                feedbackStatus(device2);
            }
        }
    }
}

#endif
#if defined(SW3G)
void button3ProcessCb() {
    if(digitalRead(Button3) == PRESS) {
        bt3.pressed = PRESS;
        bt3.timestart = millis();
        bt3.change = true;
    } else {
        bt3.pressed = RELEASE;
        bt3.timestop = millis();
        bt3.change = true;
        if (device1.state == UNINSTALL) {
            relayControl(device3, RELAY);
        } else if (device3.state == WORKING) {
            if (millis() -  bt3.timestart < TIME_TO_RST_FAC) {
                relayControl(device3, RELAY);
            }
        }
    }
}

void button3Process(){
    if (device1.state == RST_FAC){
        if (bt3.change) {
            if (bt3.pressed == PRESS){
                bt3.change = false;
            } else {
                bt3.change = false;
                unsigned long time_t = bt3.timestop - bt3.timestart;
                DBG(time_t);
                if (time_t < TIME_ENABLE_RST_FAC){
                    device1.state = device1.preState;
                    DBG("Break change mode to previous mode");
                } else {
                    flagEnableRstFc = true;
                } 
            }
        }
    } else if (device3.state == UNINSTALL) {
        if (bt3.change) {
            if (bt3.pressed == PRESS){
                bt3.change = false;
            } else {
                bt3.change = false;
                unsigned long time_t = bt3.timestop - bt3.timestart;
                DBG(time_t);
                bt3.led = true;
                if (digitalRead(Relay3)== RELAY_ST_ON) {
                    DBG("UNINSTALL: Channel 3 ON");
                } else {
                    DBG("UNINSTALL: Channel 3 OFF");
                }
            }
        }
    } else if (device3.state == WORKING) {
        if (bt3.change) {
            if (bt3.pressed == PRESS){
                bt3.change = false;
            } else {
                bt3.change = false;
                unsigned long time_t = bt3.timestop - bt3.timestart;
                DBG(time_t);
                bt3.led = true;
                if (digitalRead(Relay3)== RELAY_ST_ON) {
                    DBG("WORKING: Channel 3 ON");
                } else {
                    DBG("WORKING: Channel 3 OFF");
                }
                feedbackStatus(device3);
            }
        }
    }
}

#endif
#if defined(SW1G) || defined(SW2G) || defined(SW3G)
void button1ProcessCb() {
  if(millis() - timeDetectVir > BOUNDTIME_BUTTON) {
    if(digitalRead(Button) == PRESS) {
        bt1.pressed = PRESS;
        bt1.timestart = millis();
        bt1.change = true;
    } else {
      timeDetectVir = millis();
        bt1.pressed = RELEASE;
        bt1.timestop = millis();
        bt1.change = true;
        if (device1.state == UNINSTALL) {
            relayControl(device1, RELAY);
        } else if (device1.state == WORKING) {
            if (millis() -  bt1.timestart < TIME_TO_RST_FAC) {
                relayControl(device1, RELAY);//relayControl(device1, RELAYOFF);
            }
        }
    }
  }
}

void button1Process(){
    if (device1.state == UNINSTALL) {
        if (bt1.change) {
            if (bt1.pressed == PRESS){
                bt1.change = false;
            } else {
                bt1.change = false;
                unsigned long time_temp = bt1.timestop - bt1.timestart;
                // DBG(time_temp);
                if (time_temp < TIME_TO_RST_FAC){
                    bt1.led = true;
                    if (digitalRead(Relay)== RELAY_ST_ON) {
                        DBG("UNINSTALL: Channel 1 ON");
                    } else {
                        DBG("UNINSTALL: Channel 1 OFF");
                    }
                } else {
                    DBG("Wait for RST Factory ...");
                    timeStartRSTFac = millis();
                    bt1.counter = 0;
                    device1.preState = device1.state;
                    // device1.state = RST_FAC;
                }
            }
        }
    } else if (device1.state == RST_FAC) {
        if (millis() - timeStartRSTFac > TIME_WAIT_RST_FAC){
            device1.state = device1.preState;
            timeCounterLed = 0;
            DBG("Overtime Reset factory -> Change mode to previous mode");
        } else {
            if (bt1.change) {
                if (bt1.pressed == PRESS){
                    bt1.change = false;
                } else {
                    bt1.change = false;
                    unsigned long time_temp = bt1.timestop - bt1.timestart;
                    // DBG(time_temp);
                    if (time_temp < TIME_ENABLE_RST_FAC){
                        device1.state = device1.preState;
                        DBG("Break change mode to previous mode");
                    } else {
                        flagEnableRstFc = true;
                    } 
                }
            } else {
                if (flagEnableRstFc) {
                    DBG("OK RESET FACTORY >>>>>");
                    ResetFactory();
                    flagEnableRstFc = false;
                    DBG("<<<<<< DONE >>>>>");
                }
            }
        }
    } else if (device1.state == WORKING) {
        if (bt1.change) {
            if (bt1.pressed == PRESS){
                bt1.change = false;
            } else {
                bt1.change = false;
                unsigned long time_temp = bt1.timestop - bt1.timestart;
                // DBG(time_temp);
                if (time_temp != TIME_TO_RST_FAC) { // if (time_t < TIME_TO_UNWORKING) {
                    bt1.led = true;
                    if (digitalRead(Relay)== RELAY_ST_ON) {
                        DBG("WORKING: Channel 1 ON");
                    } else {
                        DBG("WORKING: Channel 1 OFF");
                    }
                    feedbackStatus(device1);
                } else {
                    DBG("WORKING: Wait for RST Factory ...");
                    timeStartRSTFac = millis();
                    bt1.counter = 0;
                    device1.preState = device1.state;
                    device1.state = RST_FAC;
                }
            }
        }
    }
}

#endif

void checkCmd(void) {
    if (Serial.available()){
        memset(sbuf, '\0', MAX_BUF_REC_SERIAL);
        Serial.readBytes(sbuf, MAX_BUF_REC_SERIAL);
        String cmdbk = "";
        String cmdCk = "";
        cmdbk = sbuf;
        DBGT("Received cmd: "); DBG(cmdbk);
        if(cmdbk == "endbg"){
          flagDebug = true;
          EEPROM.write(ADD_DBG, 1);
          EEPROM.commit();
          delay(5);
          DBG("Start debug");
        } else if(cmdbk == "disdbg"){
          DBG("Stop debug");
          flagDebug = false;
          EEPROM.write(ADD_DBG, 0);
          EEPROM.commit();
          delay(5);
        } else if(cmdbk == "myip"){
            DBGT("\tIP : "); DBG(deviceIp);
        } else if(cmdbk == "time"){
            flagTimeView = true;
        } else if(cmdbk == "ssid"){
            DBGT("\tSSID : "); DBG(Ssid);
            DBGT("\tPassword : "); DBG(Password);
        } else if(cmdbk == "restart"){
            ESP.restart();
        } else if(cmdbk == "test"){
            flagViewTest = true;
        } else {
            DBGT("NOT FOUND\n");
        }
    }
}

void relayControl(device_struct device, relay_state_t relayStatus) {
    switch (relayStatus) {
        case RELAY:
            digitalWrite(device.relay, digitalRead(device.relay) ? RELAY_ST_OFF : RELAY_ST_ON);
            if(flagRelayState){
              digitalWrite(device.relay, RELAY_ST_ON);
              flagRelayState = false;
            } else {
              digitalWrite(device.relay, RELAY_ST_OFF);
              flagRelayState = true;
            }
            break;
        case RELAYON:
            digitalWrite(device.relay, RELAY_ST_ON);
            flagRelayState = true;
            break;
        case RELAYOFF:
            digitalWrite(device.relay, RELAY_ST_OFF);
            flagRelayState = false;
            break;
        default :
            break;
    }
}

void controlLed(void){
    if (device1.state == WORKING) {
        if (bt1.led == true){
            timeCounterLed = 1;
            bt1.led = false;
            digitalWrite(Led, LED_ST_ON);
            timePreLedOn = millis();
        } else {
            if ( timeCounterLed <= 0) {
                digitalWrite(Led, LED_ST_OFF);
            } else {
                if (digitalRead(Led) == LED_ST_ON) {
                    if ( millis() - timePreLedOn > TIME_DELAY_LED_SIGNAL_ON ) {
                        timeCounterLed--;
                        digitalWrite(Led, LED_ST_OFF);
                        timePreLedOff = millis();
                    }
                } else {
                    if ( millis() - timePreLedOff > TIME_DELAY_LED_SIGNAL_OFF ) {
                        digitalWrite(Led, LED_ST_ON);
                        timePreLedOn = millis();
                    }
                }
            }
        }
    } else if ( device1.state == UNINSTALL) {
        if (bt1.led == true){
            timeCounterLed = 2;
            bt1.led = false;
            digitalWrite(Led, LED_ST_ON);
            timePreLedOn = millis();
        } else {
            if ( timeCounterLed <= 0) {
                digitalWrite(Led, LED_ST_OFF);
            } else {
                if (digitalRead(Led) == LED_ST_ON) {
                    if ( millis() - timePreLedOn > TIME_DELAY_LED_SIGNAL_ON ) {
                        timeCounterLed--;
                        digitalWrite(Led, LED_ST_OFF);
                        timePreLedOff = millis();
                    }
                } else {
                    if ( millis() - timePreLedOff > TIME_DELAY_LED_SIGNAL_OFF ) {
                        digitalWrite(Led, LED_ST_ON);
                        timePreLedOn = millis();
                    }
                }
            }
        }
    } 
    if ( device1.state == RST_FAC || ((millis() - bt1.timestart > TIME_TO_RST_FAC)&&(bt1.timestop < bt1.timestart))) {
        if ( timeCounterLed <= 0) {
            digitalWrite(Led, LED_ST_OFF);
            if (millis() - timePreLedOff > TIME_DELAY_LED_ERROR) {
                timeCounterLed = 2;
            }   
        } else {
            if (digitalRead(Led) == LED_ST_ON) {
                if ( millis() - timePreLedOn > TIME_DELAY_LED_ERROR ) {
                    timeCounterLed--;
                    digitalWrite(Led, LED_ST_OFF);
                    timePreLedOff = millis();
                }
            } else {
                if ( millis() - timePreLedOff > TIME_DELAY_LED_ERROR ) {
                    digitalWrite(Led, LED_ST_ON);
                    timePreLedOn = millis();
                }
            }
        }
    } 
//    else if ( device1.state == UNINSTALL || device2.state == UNINSTALL || device3.state == UNINSTALL){
//        if ( timeCounterLed <= 0) {
//            digitalWrite(Led, LED_ST_OFF);
//            if (millis() - timePreLedOff > TIME_DELAY_LED_SOFF) {
//                timeCounterLed = 2;
//            }   
//        } else {
//            if (digitalRead(Led) == LED_ST_ON) {
//                if ( millis() - timePreLedOn > TIME_DELAY_LED_SIGNAL_ON ) {
//                    timeCounterLed--;
//                    digitalWrite(Led, LED_ST_OFF);
//                    timePreLedOff = millis();
//                }
//            } else {
//                if ( millis() - timePreLedOff > TIME_DELAY_LED_SIGNAL_OFF ) {
//                    digitalWrite(Led, LED_ST_ON);
//                    timePreLedOn = millis();
//                }
//            }
//        }
//    }
}

//*************READ EEPROM**************
void ReadEpprom() {
    DBG("Read Epprom");
    String tempStr = "";
    Mode_Station = false;
    device1.id ="";
    device2.id ="";
    device3.id ="";
    device1.secretKey = "";
    device2.secretKey = "";
    device3.secretKey = "";
    device1.state = EEPROM.read(ADD_STATE1);
    if (device1.state == WORKING) {
        for (int i = ADD_ID1_F; i <= ADD_ID1_E; i++) {
            device1.id += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tID device gang 1 : \t"); DBG(device1.id);
        for (int i = ADD_KEY1_F; i <= ADD_KEY1_E; i++) {
            device1.secretKey += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tSecretkey gang 1 :  \t"); DBG(device1.secretKey);
        if (EEPROM.read(ADD_STARTUP1) == 0) {
            device1.startup = "ON";
        } else if (EEPROM.read(ADD_STARTUP1) == 1) {
            device1.startup = "OFF";
        } else if (EEPROM.read(ADD_STARTUP1) == 2) {
            device1.startup = "SYN";
        }
        DBGT("\tStartup gang 1 : \t"); DBG(device1.startup);
        tempStr = "WORKING";
    } else {
        device1.state = UNINSTALL;
        tempStr = "Not matched => change to : UNINSTALL";
    }
    DBGT("\tState gang 1 : \t"); DBG(tempStr);
    
    device2.state = EEPROM.read(ADD_STATE2);
    if (device2.state == WORKING) {
        for (int i = ADD_ID2_F; i <= ADD_ID2_E; i++) {
            device2.id += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tID device gang 2 : \t"); DBG(device2.id);
        for (int i = ADD_KEY2_F; i <= ADD_KEY2_E; i++) {
            device2.secretKey += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tSecretkey gang 2 :  \t"); DBG(device2.secretKey);
        if (EEPROM.read(ADD_STARTUP2) == 0) {
            device2.startup = "ON";
        } else if (EEPROM.read(ADD_STARTUP2) == 1) {
            device2.startup = "OFF";
        } else if (EEPROM.read(ADD_STARTUP2) == 2) {
            device2.startup = "SYN";
        }
        DBGT("\tStartup gang 2 : \t"); DBG(device2.startup);
        tempStr = "WORKING";
    } else {
        device2.state = UNINSTALL;
        tempStr = "Not matched => change to : UNINSTALL";
    }
    DBGT("\tState gang 2 : \t"); DBG(tempStr);

    device3.state = EEPROM.read(ADD_STATE3);
    if (device3.state == WORKING) {
        for (int i = ADD_ID3_F; i <= ADD_ID3_E; i++) {
            device3.id += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tID device gang 3 : \t"); DBG(device3.id);
        for (int i = ADD_KEY3_F; i <= ADD_KEY3_E; i++) {
            device3.secretKey += String(EEPROM.read(i) - 0x30);
        }
        DBGT("\tSecretkey gang 3 :  \t"); DBG(device3.secretKey);
        if (EEPROM.read(ADD_STARTUP3) == 0) {
            device3.startup = "ON";
        } else if (EEPROM.read(ADD_STARTUP3) == 1) {
            device3.startup = "OFF";
        } else if (EEPROM.read(ADD_STARTUP3) == 2) {
            device3.startup = "SYN";
        }
        DBGT("\tStartup gang 3 : \t"); DBG(device3.startup);
        tempStr = "WORKING";
    } else {
        device3.state = UNINSTALL;
        tempStr = "Not matched => change to : UNINSTALL";
    }
    DBGT("\tState gang 3 : \t"); DBG(tempStr);

    // DOC STATION MODE
    if (EEPROM.read(ADD_MODE) == 1) {
        Mode_Station = true;
        tempStr = "STA";
    } else {
        Mode_Station = false;
        tempStr = "AP";
    }
    DBGT("\tMode : "); DBG(tempStr);
    // IP STATIC OR DHCP
    if (EEPROM.read(ADD_STATUS) == 1) {
        isStatic = true;
        tempStr = "STATIC";
    } else {
        isStatic = false;
        tempStr = "DHCP";
    }
    DBGT("\tIP Satus : "); DBG(tempStr);
    if (Mode_Station && isStatic) {
        IPAddress ip_(EEPROM.read(ADD_IP_STA_F), EEPROM.read((ADD_IP_STA_F + 1)), EEPROM.read((ADD_IP_STA_F + 2)), EEPROM.read(ADD_IP_STA_E));
        deviceIp = ip_;
        DBGT("\tDevice IP : "); DBG(deviceIp);
        IPAddress gateway(EEPROM.read(ADD_GETWAY_F), EEPROM.read((ADD_GETWAY_F + 1)), EEPROM.read((ADD_GETWAY_F + 2)), EEPROM.read(ADD_GETWAY_E));
        default_gateway = gateway;
        DBGT("\tDEfault Gateway : "); DBG(default_gateway);
        IPAddress subnet(EEPROM.read(ADD_SUBNETMARK_F), EEPROM.read(ADD_SUBNETMARK_F + 1), EEPROM.read(ADD_SUBNETMARK_F + 2), EEPROM.read(ADD_SUBNETMARK_E));
        subnet_mark = subnet;
        DBGT("\tSubnet mark : "); DBG(subnet_mark);
    }
  // READ SSID AND PASSWORD
    if (device1.state == INACTIVE_PAIRING || device2.state == INACTIVE_PAIRING || device3.state == INACTIVE_PAIRING || device1.state == WORKING || device2.state == WORKING || device3.state == WORKING) {
        if ((EEPROM.read(ADD_LEN_SSID) > 0)&&(EEPROM.read(ADD_LEN_SSID) < (ADD_SSID_F))) {
            String Ssid_ = "";
            String Password_ = "";
            for (int i = 0; i < EEPROM.read(ADD_LEN_SSID); i++) {
                Ssid_ = Ssid_ + "" + char(EEPROM.read(ADD_SSID_F + i));
            }
            if (EEPROM.read(ADD_LEN_PW) > 0) {
                for (int i = 0; i < EEPROM.read(ADD_LEN_PW); i++) {
                    Password_ = Password_ + "" + char(EEPROM.read(ADD_PW_F + i));
                }
            }
            Ssid = (char*)malloc((Ssid_.length()) * sizeof(char));
            Password = (char*)malloc((Password_.length()) * sizeof(char));
            Ssid_.toCharArray(Ssid, Ssid_.length() + 1);
            Password_.toCharArray(Password, Password_.length() + 1);
            DBGT("\tSSID : "); DBG(Ssid);
            DBGT("\tPassword : "); DBG(Password);
        } else {
            device1.state = UNINSTALL;
            device2.state = UNINSTALL;
            device3.state = UNINSTALL;
            DBG("Lose SSID and PW");
            DBGT("state 1 = state 2 = state 3 = UNINSTALL");
        }
        IPAddress rashomeip_(EEPROM.read(ADD_RSH_IP_F), EEPROM.read((ADD_RSH_IP_F + 1)), EEPROM.read((ADD_RSH_IP_F + 2)), EEPROM.read(ADD_RSH_IP_E));
        RashomeIP = rashomeip_;
        DBGT("\tRashomeIP : "); DBG(RashomeIP);
    }
    for (int i = 0; i < 5; i++){
      if(EEPROM.read(ADD_ALARM_0 + i) == 1){
        flagAlarm = true;
        timeRunDeviceByAlarm = EEPROM.read(ADD_ALARM_TIME + ADD_ALARM_STEP*i);
        dayAlarm = EEPROM.read(ADD_ALARM_DAY + ADD_ALARM_STEP*i);
        hourAlarm = EEPROM.read(ADD_ALARM_HOUR + ADD_ALARM_STEP*i);
        minuteAlarm = EEPROM.read(ADD_ALARM_MINUTE + ADD_ALARM_STEP*i);
        DBGT("TIME ON: "); DBG(timeRunDeviceByAlarm);
        DBGT("DAY: "); DBG(dayAlarm);
        DBGT("HOUR: "); DBG(hourAlarm);
        DBGT("MINUTE: "); DBG(minuteAlarm);
        DBGT("REPEAT: "); DBG(timeRunDeviceByAlarm);
      }
    }
}

void checkConnectWifi(void) {
    if (millis() - timeStartCheckConnectWifi > TIME_TO_CHECK_CONNECT_WIFI) {    
        timeStartCheckConnectWifi = millis();
        if (device1.state == WORKING) {
            if (WiFi.status() != WL_CONNECTED) {
                WiFiReconnect();
            } else {
                if (reconnectWifi) {
                    reconnectWifi = false;
                    connectFirst = true;
                }
            }
        }
    }
    if (connectFirst) {
        connectFirst = false;
        DBGT("Connected to wifi : ");  DBG(Ssid);
        DBGT("My IP : "); DBG(WiFi.localIP());
    }
}

boolean ResetFactory() { 
    device1.state = UNINSTALL;
    device2.state = UNINSTALL;
    device3.state = UNINSTALL;
    Mode_Station = false;
    ClearEeprom();
    ReadEpprom();
    relayControl(device1, RELAYOFF);
    relayControl(device2, RELAYOFF);
    relayControl(device3, RELAYOFF);
    server.close();
    WiFi.softAPdisconnect();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    ESP.restart();
    return false;
}

void WiFiReconnect() {
  if (millis() - timeStartReConnect > TIME_TO_RECONNECT_WIFI) {
    timeStartReConnect = millis();
    DBG("Reconnect wifi");
    // DBGT("SSID: "); DBG(Ssid);
    // DBGT("PW: "); DBG(Password);
    // WiFi.softAPdisconnect();
    // WiFi.disconnect();
    // delay(5);
    WiFi.mode(WIFI_STA);
    WiFi.begin(Ssid, Password);
    server.begin();
    reconnectWifi = true;
  }
}

void feedbackStatus(device_struct device){
    if (digitalRead(device.relay) == RELAY_ST_ON) {
        char *Status_t = "ON";
      if (!sendDataServer(Status_t)) {
          DBG("Send FB error");
      }
    } else {
        char *Status_t = "OFF";
        if (!sendDataServer(Status_t)) {
            DBG("Send FB error");
        }
    }
}

void startupStatusProcess() {
    if (device1.state == WORKING) {
        if (device1.startup == "ON") {
            relayControl(device1, RELAYON);
            feedbackStatus(device1);
        } else if (device1.startup == "OFF") {
            relayControl(device1, RELAYOFF);
            feedbackStatus(device1);
        }
    } else {
        relayControl(device1, RELAYOFF);
    }
    if (device2.state == WORKING) {
        if (device2.startup == "ON") {
            relayControl(device2, RELAYON);
            feedbackStatus(device2);
        } else if (device2.startup == "OFF") {
            relayControl(device2, RELAYOFF);
            feedbackStatus(device2);
        }
    } else {
        relayControl(device2, RELAYOFF);
    }
    if (device3.state == WORKING) {
        if (device3.startup == "ON") {
            relayControl(device3, RELAYON);
            feedbackStatus(device3);
        } else if (device3.startup == "OFF") {
            relayControl(device3, RELAYOFF);
            feedbackStatus(device3);
        }
    } else {
        relayControl(device3, RELAYOFF);
    }
}

boolean sendDataServer(char *data){
    if (WiFi.status() == WL_CONNECTED) {
        // if (! myputty.publish(data)) {
          // DBG(F("Failed"));
          // return false;
        // } else {
          DBG(F("OK!"));
          return true;
        // }
    } else {
        DBG("Wifi disconnect");
        WiFiReconnect();
        return false;
    }
}

void processDataFromServer(void) {
    WiFiClient clientConnect = server.available();
    clientConnect.setNoDelay(true);
    if (!clientConnect) {
        return;
    }
    // Wait until the client sends some data
    DBG("New client connect");
    DBGT(" IP client : "); DBG(clientConnect.remoteIP());
    unsigned long timeout = millis() + 3000;
    while(!clientConnect.available() && millis() < timeout){
        delay(1);
    }
    if (millis() > timeout) {
        DBGT("timeout");
        clientConnect.flush();
        clientConnect.stop();
        return;
    }
    // Read the first line of the request
    req_fr_server = clientConnect.readStringUntil('\r');
    // clientConnect.print("OK\r\n");
    // req_fr_server = clientConnect.read();
    DBG(req_fr_server);
    // clientConnect.flush();
    if (req_fr_server != "") {
        if (device1.state == UNINSTALL) {
          //GET /req/DEVICE/ssid/ThienSG/pwd/36622111h HTTP/1.1
            String matchReq = "";
            String temp = req_fr_server;
            temp.remove(0,req_fr_server.indexOf('/')+1);
            matchReq = "{\"" + temp.substring(0,temp.indexOf('/')) + "\":";
            temp.remove(0,temp.indexOf('/')+1);
            matchReq += "\"" + temp.substring(0,temp.indexOf('/')) + "\",";
            temp.remove(0,temp.indexOf('/')+1);
            matchReq += "\"" + temp.substring(0,temp.indexOf('/')) + "\":";
            temp.remove(0,temp.indexOf('/')+1);
            matchReq += "\"" + temp.substring(0,temp.indexOf('/')) + "\",";
            temp.remove(0,temp.indexOf('/')+1);
            matchReq += "\"" + temp.substring(0,temp.indexOf('/')) + "\":";
            temp.remove(0,temp.indexOf('/')+1);
            matchReq += "\"" + temp.substring(0,temp.indexOf('/')) + "\"}\n";
            DBG(matchReq);
//            String req = splitStringJson("req", req_fr_server); // "DEVICE"
            String req = splitStringJson("req", matchReq);
            if ( req == "NETWORK") {
                String ssid_ser = splitStringJson("ssid", matchReq);
                String password_ser = splitStringJson("pwd", matchReq);
                Ssid = (char*)malloc((ssid_ser.length()) * sizeof(char));
                Password = (char*)malloc((password_ser.length()) * sizeof(char));
                ssid_ser.toCharArray(Ssid, ssid_ser.length() + 1);
                password_ser.toCharArray(Password, password_ser.length() + 1);
                Mode_Station = true;
                EEPROM.write(ADD_MODE, 1);
                delay(1);
                EEPROM.write(ADD_LEN_SSID, strlen(Ssid));
                delay(1);
                for (int i = 0; i < strlen(Ssid); i++) {
                    int charNumber = Ssid[i];
                    EEPROM.write(ADD_SSID_F + i, charNumber);
                    delay(1);
                }
                EEPROM.write(ADD_LEN_PW, strlen(Password));
                delay(1);
                for (int i = 0; i < strlen(Password); i++) {
                    int charNumber = Password[i];
                    EEPROM.write(ADD_PW_F + i, charNumber);
                    delay(1);
                }
                // state = "PAIRING"; me
                DBG("device.state =  WORKING");
                EEPROM.write(ADD_STATE1, WORKING);
                EEPROM.write(ADD_STATE2, WORKING);
                EEPROM.write(ADD_STATE3, WORKING);
                EEPROM.commit();
                delay(5);
                DBG("Configure DONE");
                ReadEpprom();
                DBG("Close server");
                server.close();
                WiFi.softAPdisconnect();
                DBG("disconnect Wifi");
                WiFi.disconnect();
                DBG("Setup wifi and server");
                WiFi.mode(WIFI_STA);
                WiFi.begin(Ssid, Password);
                delay(10000);
                server.begin();
                delay(5000);
                ESP.restart();
                return;
            }
        } else {
            String SHA1 = "";
            // String RandomInt;
            String req = splitStringJson("req", req_fr_server); // "FEED_BACK"
            if ( req == "PAIR") {
                String gang = splitStringJson("gang", req_fr_server); // "FEED_BACK"
                String id = splitStringJson("id", req_fr_server); // "FEED_BACK"
                String secretkey = splitStringJson("secretkey", req_fr_server); // "FEED_BACK"
                String startup_t = splitStringJson("startup", req_fr_server); // "FEED_BACK"
                DBGT("GANG: "); DBG(gang);
                DBGT("NewID:"); DBG(id);
                DBGT("secret key: "); DBG(secretkey);
                DBGT("Startup :"); DBG(startup_t);
                
                String tempStr = "";
                IPAddress tempip = clientConnect.remoteIP();
                tempStr = String(tempip[0]) + '.' + String(tempip[1]) + '.' + String(tempip[2]) + '.' + String(tempip[3]);
                if (RashomeIP != tempip) {
                    DBGT("\tRashome IP old: "); DBG(RashomeIP);
                    DBGT("\tRashome IP new: "); DBG(tempip);
                    storeIP_tmp(tempStr, ADD_RSH_IP_F);
                    ReadEpprom();
                } else {
                    DBGT("\tRashome IP same: "); DBG(tempip);
                }
                if (gang == "1") {
                    EEPROM.write(ADD_ID1_F, id[0]);
                    EEPROM.write(ADD_ID1_F + 1, id[1]);
                    EEPROM.write(ADD_ID1_E, id[2]);
                    device1.id = id;

                    EEPROM.write(ADD_KEY1_F, secretkey[0]);
                    EEPROM.write(ADD_KEY1_F + 1, secretkey[1]);
                    EEPROM.write(ADD_KEY1_F + 2, secretkey[2]);
                    EEPROM.write(ADD_KEY1_E, secretkey[3]);
                    device1.secretKey = secretkey;

                    EEPROM.write(ADD_STATE1, WORKING);  // BIEN TRANG THAI
                    if (startup_t == "ON") {
                        EEPROM.write(ADD_STARTUP1, 0);
                        device1.startup = "ON";  // trang thai khoi dong cua gang 1
                    } else if (startup_t == "OFF") {
                        EEPROM.write(ADD_STARTUP1, 1);
                        device1.startup = "OFF";
                    } else if (startup_t == "SYN") {
                        EEPROM.write(ADD_STARTUP1, 2);
                        device1.startup = "SYN";
                    }
                    EEPROM.commit();
                    delay(5);
                    device1.state = WORKING;
                    DBG("-----------GANG 1 WORKING --------");
                    feedbackStatus(device1);
                    SyncNumber = 1;
                    Udp.stop();
                    clientConnect.println("{\"res\":\"PAIR\"}");
                    DBG("{\"res\":\"PAIRED\"}");
                } else if (gang == "2") {
                    EEPROM.write(ADD_ID2_F, id[0]);
                    EEPROM.write(ADD_ID2_F + 1, id[1]);
                    EEPROM.write(ADD_ID2_E, id[2]);
                    device2.id = id;

                    EEPROM.write(ADD_KEY2_F, secretkey[0]);
                    EEPROM.write(ADD_KEY2_F + 1, secretkey[1]);
                    EEPROM.write(ADD_KEY2_F + 2, secretkey[2]);
                    EEPROM.write(ADD_KEY2_E, secretkey[3]);
                    device2.secretKey = secretkey;

                    EEPROM.write(ADD_STATE2, WORKING);  // BIEN TRANG THAI
                    if (startup_t == "ON") {
                        EEPROM.write(ADD_STARTUP2, 0);
                        device2.startup = "ON";  // trang thai khoi dong cua gang 1
                    } else if (startup_t == "OFF") {
                        EEPROM.write(ADD_STARTUP2, 1);
                        device2.startup = "OFF";
                    } else if (startup_t == "SYN") {
                        EEPROM.write(ADD_STARTUP2, 2);
                        device2.startup = "SYN";
                    }
                    EEPROM.commit();
                    delay(5);
                    device2.state = WORKING;
                    DBG("-----------GANG 2 WORKING --------");
                    feedbackStatus(device2);
                    SyncNumber = 1;
                    Udp.stop();
                    clientConnect.println("{\"res\":\"PAIR\"}");
                    DBG("{\"res\":\"PAIRED\"}");
                } else if (gang == "3") {
                    EEPROM.write(ADD_ID3_F, id[0]);
                    EEPROM.write(ADD_ID3_F + 1, id[1]);
                    EEPROM.write(ADD_ID3_E, id[2]);
                    device3.id = id;

                    EEPROM.write(ADD_KEY3_F, secretkey[0]);
                    EEPROM.write(ADD_KEY3_F + 1, secretkey[1]);
                    EEPROM.write(ADD_KEY3_F + 2, secretkey[2]);
                    EEPROM.write(ADD_KEY3_E, secretkey[3]);
                    device3.secretKey = secretkey;

                    EEPROM.write(ADD_STATE3, WORKING);  // BIEN TRANG THAI
                    if (startup_t == "ON") {
                        EEPROM.write(ADD_STARTUP3, 0);
                        device3.startup = "ON";  // trang thai khoi dong cua gang 1
                    } else if (startup_t == "OFF") {
                        EEPROM.write(ADD_STARTUP3, 1);
                        device3.startup = "OFF";
                    } else if (startup_t == "SYN") {
                        EEPROM.write(ADD_STARTUP3, 2);
                        device3.startup = "SYN";
                    }
                    EEPROM.commit();
                    delay(5);
                    device3.state = WORKING;
                    DBG("-----------GANG 3 WORKING --------");
                    feedbackStatus(device1);
                    SyncNumber = 1;
                    Udp.stop();
                    clientConnect.println("{\"res\":\"PAIR\"}");
                    DBG("{\"res\":\"PAIRED\"}");
                }
            } else if (req == "CHANGE_IP") {
            #if defined(SW3G)
                clientConnect.println("{\"res\":\"CHANGE_IP\", \"id\":{\"" + device1.id + "\":\"\",\"" + device2.id + "\":\"\",\"" + device3.id + "\":\"\"}}");
            #elif defined(SW2G)
                clientConnect.println("{\"res\":\"CHANGE_IP\", \"id\":{\"" + device1.id + "\":\"\",\"" + device2.id + "\":\"\"}}");
            #elif defined(SW1G)
                clientConnect.println("{\"res\":\"CHANGE_IP\", \"id\":{\"" + device1.id + "\":\"\"}}");
            #endif
                String ip_address_new = splitStringJson("ip_address", req_fr_server);
                String netmask_new = splitStringJson("netmask", req_fr_server);
                String gateway_new = splitStringJson("gateway", req_fr_server);
                String ipState_ser = "STATIC";
                storeIP(ip_address_new, gateway_new, netmask_new, ipState_ser);
                DBG("Close server");
                server.close();
                WiFi.softAPdisconnect();
                DBG("disconnect Wifi");
                WiFi.disconnect();
                DBG("Setup wifi and server");
                ReadEpprom();
                WiFi.config(deviceIp, default_gateway, subnet_mark);
                // WiFi.begin(Ssid, Password);
                // server.begin();
                DBG("Setup DONE");
            } else if (req == "CHANGE_RASHOME_IP") {
                String Id = splitStringJson("id", req_fr_server);
                String IP = splitStringJson("ip", req_fr_server);
                String CheckSum = splitStringJson("checksum", req_fr_server);
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                bool flag = true;
                device_struct device;
                if (device1.id == Id) {
                    device = device1;
                } else if (device2.id == Id) {
                    device = device2;
                } else if (device3.id == Id) {
                    device = device3;
                } else {
                    flag = false;
                }
                if (flag) {
                    String SHA1 = String2Hash("CHANGE_RASHOME_IP" + device.id + IP + RandomInt + device.secretKey);
                    // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                        SyncNumber = RandomInt;
                        storeIP_tmp(IP, ADD_RSH_IP_F);
                        ReadEpprom();
                    // } else {
                        // DBG("Wrong security");
                    // }
                }
            } else if (req == "CHANGE_WIFI") {
                String Id = splitStringJson("id", req_fr_server);
                String ssid_ser = splitStringJson("ssid", req_fr_server);
                String password_ser = splitStringJson("password", req_fr_server);
                String CheckSum = splitStringJson("checksum", req_fr_server);
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                device_struct device;
                bool flagcheck= true;
                if (device1.id == Id) {
                    device = device1;
                } else if (device2.id == Id) {
                    device = device2;
                } else if (device3.id == Id) {
                    device = device3;
                } else {
                    flagcheck = false;
                }
                if (flagcheck) {
                    DBG("Close server");
                    server.close();
                    WiFi.softAPdisconnect();
                    DBG("disconnect old Wifi");
                    DBGT("Old SSID: "); DBG(Ssid_old);
                    DBGT("Old Password: ");DBG(Password_old);
                    WiFi.disconnect();

                    String SHA1 = String2Hash("CHANGE_WIFI" + device.id + ssid_ser + password_ser + RandomInt + device.secretKey);
                    // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                        SyncNumber = RandomInt;
                        Ssid = (char*)malloc((ssid_ser.length()) * sizeof(char));
                        Password = (char*)malloc((password_ser.length()) * sizeof(char));
                        ssid_ser.toCharArray(Ssid, ssid_ser.length() + 1);
                        password_ser.toCharArray(Password, password_ser.length() + 1);
                        EEPROM.write(ADD_LEN_SSID, strlen(Ssid));
                        delay(1);
                        for (int i = 0; i < strlen(Ssid); i++) {
                            int charNumber = Ssid[i];
                            EEPROM.write(ADD_SSID_F + i, charNumber);
                            delay(1);
                        }
                        EEPROM.write(ADD_LEN_PW, strlen(Password));
                        delay(1);
                        for (int i = 0; i < strlen(Password); i++) {
                            int charNumber = Password[i];
                            EEPROM.write(ADD_PW_F + i, charNumber);
                            delay(1);
                        }
                        EEPROM.commit();
                        ReadEpprom();
                        WiFi.config(deviceIp, default_gateway, subnet_mark);
                        WiFi.mode(WIFI_STA);
                        // WiFi.begin(Ssid, Password);
                        // delay(1000);
                        // server.begin();
                        delay(10);
                        DBG("Changed Wifi OK");
                    // } else {
                        // DBG("Wrong security");
                    // }
                }
                /*This function ok
                if new SSID or PASSWORD wrong it make return before SSID and old PASSWORD*/
/*
            }else if (req == "CHANGE_WIFI") {
                String Id = root["id"];
                String ssid_ser = root["ssid"];
                String password_ser = root["password"];
                String CheckSum = root["checksum"];
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                String idOfDevice_t = "";
                String secretKey_t = "";
                if (device1.id == Id) {
                    idOfDevice_t = device1.id;
                    secretKey_t = device1.secretKey;
                } else if (device2.id == Id) {
                    idOfDevice_t = device2.id;
                    secretKey_t = device2.secretKey;
                } else if (device3.id == Id) {
                    idOfDevice_t = device3.id;
                    secretKey_t = device3.secretKey;
                }
                if (idOfDevice_t != "") {
                    DBGT("New SSID: "); DBG(ssid_ser);
                    DBGT("New Password: ");DBG(password_ser);
                    DBGT("scan start...");
                    // WiFi.scanNetworks will return the number of networks found
                    int n = WiFi.scanNetworks();
                    DBGT("scan done");
                    if (n == 0) {
                        DBG("no networks found");
                    } else {
                        DBGT(n);
                        DBGT(" networks found");
                        for (int i = 0; i < n; ++i) {
                            // Print SSID and RSSI for each network found
                            if (WiFi.SSID(i) == ssid_ser) {
                                flagSsidMatched = true;
                                DBGT(i + 1);
                                DBGT(": ");
                                DBGT(WiFi.SSID(i));
                                DBGT(" (");
                                DBGT(WiFi.RSSI(i));
                                DBGT(")");
                                DBGT((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
                                break;
                            }
                            delay(1);
                        }
                        if (flagSsidMatched) {
                            flagChangeWifi = true;
                            sprintf(Ssid_old, Ssid);
                            sprintf(Password_old, Password);
                            DBG("Close server");
                            server.close();
                            WiFi.softAPdisconnect();
                            DBG("disconnect old Wifi");
                            DBGT("Old SSID: "); DBG(Ssid_old);
                            DBGT("Old Password: ");DBG(Password_old);
                            WiFi.disconnect();
                            String SHA1 = String2Hash("CHANGE_WIFI" + idOfDevice_t + ssid_ser + password_ser + RandomInt + secretKey_t);
                            // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                            SyncNumber = RandomInt;
                            Ssid = (char*)malloc((ssid_ser.length()) * sizeof(char));
                            Password = (char*)malloc((password_ser.length()) * sizeof(char));
                            ssid_ser.toCharArray(Ssid, ssid_ser.length() + 1);
                            password_ser.toCharArray(Password, password_ser.length() + 1);
                            WiFi.mode(WIFI_STA);
                            WiFi.begin(Ssid, Password);
                            for (int i = 0; i<5; i++){
                                if (WiFi.status() != WL_CONNECTED) {
                                    server.begin();
                                    DBG("Connect to new Wifi OK");
                                    EEPROM.write(ADD_LEN_SSID, strlen(Ssid));
                                    delay(1);
                                    for (int i = 0; i < strlen(Ssid); i++) {
                                        int charNumber = Ssid[i];
                                        EEPROM.write(ADD_SSID_F + i, charNumber);
                                        delay(1);
                                    }
                                    EEPROM.write(ADD_LEN_PW, strlen(Password));
                                    delay(1);
                                    for (int i = 0; i < strlen(Password); i++) {
                                        int charNumber = Password[i];
                                        EEPROM.write(ADD_PW_F + i, charNumber);
                                        delay(1);
                                    }
                                    EEPROM.commit();
                                    delay(10);
                                    DBG("Changed Wifi SUCCESS");
                                    flagChangeWifi = false;
                                    break;
                                }
                                if(i==3){
                                    WiFi.begin(Ssid, Password);
                                }
                                delay(100);
                            }
                            if (flagChangeWifi){
                                DBG("Something Wrong");
                                DBG("Setup to Wifi before");
                                sprintf(Ssid, Ssid_old);
                                sprintf(Password, Password_old);
                                WiFi.begin(Ssid, Password);
                            }
                        } else {
                            DBG(">>>>> SSID New not found");
                        }
                    }
                // } else {
                    // DBG("Wrong security");
                // }
                }
*/
            } else if (req == "STARTUP") {
                String Id = splitStringJson("id", req_fr_server);
                String Status = splitStringJson("status", req_fr_server);
                String CheckSum = splitStringJson("checksum", req_fr_server);
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                device_struct device;
                int startup_add = 0;
                bool flag = true;
                if (device1.id == Id) {
                    device = device1;
                    startup_add = ADD_STARTUP1;
                } else if (device2.id == Id) {
                    device = device2;
                    startup_add = ADD_STARTUP2;
                } else if (device3.id == Id) {
                    device = device3;
                    startup_add = ADD_STARTUP3;
                } else {
                    flag = false;
                }
                if (flag) {
                    String SHA1 = String2Hash("startup" + device.id + Status + RandomInt + device.secretKey);
                    // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                        SyncNumber = RandomInt;
                        if (Status == "ON") {
                            EEPROM.write(startup_add, 0);
                        } else if (Status == "OFF") {
                            EEPROM.write(startup_add, 1);
                        } else if (Status == "SYN") {
                            EEPROM.write(startup_add, 2);
                        }
                        EEPROM.commit();
                        delay(1);
                        DBG("Changed startup status DONE");
                    // } else {
                        // DBG("Wrong security");
                    // }
                }
            } 
        }
    }
}

/*****FUNCTION SETUP WIFI ACCESS POINT******/
void setupWifiAccessPoint(char *ssid, char* password) {
    IPAddress ip_(192, 168, 4, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(ip_, gateway, subnet);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    server.begin();
}

void ClearEeprom() {
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0);
    delay(5);
  }
  EEPROM.commit();
  delay(100);
  DBGT("Eeprom cleared\n");
}

String String2Hash(String HashString){
  String tmp = "";
  uint8_t hash[8];
  sha1(HashString, &hash[0]);
  for (int i = 0; i < 4; i++) {
    if (hash[i] > 15) {
      tmp += String(hash[i], HEX);
    } else {
      tmp += "0" + String(hash[i], HEX);
    }
  }
  return tmp;
}

void storeIP(String StrIP, String StrGateway, String StrSubnet, String ipState) {
    storeIP_tmp(StrIP, ADD_IP_STA_F);
    storeIP_tmp(StrGateway, ADD_GETWAY_F);
    storeIP_tmp(StrSubnet, ADD_SUBNETMARK_F);
    if (ipState == "STATIC") {
        EEPROM.write(ADD_STATUS, 1);
    } else if (ipState == "DHCP") {
        EEPROM.write(ADD_STATUS, 0);
    } else {
        EEPROM.write(ADD_STATUS, 3);
    }
    EEPROM.commit();
    delay(10);
}

void storeIP_tmp(String ip, int offset) {
    int arr[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            arr[i] = ip.toInt();
        } else {
            int mid = (int)(ip.indexOf('.')); //dau "." dau tien
            int tmp = (ip.substring(0, ip.indexOf('.'))).toInt();
            arr[i] = tmp;
            ip = ip.substring(mid + 1);
        }
    }
    EEPROM.write(offset, arr[0]);
    EEPROM.write(offset + 1, arr[1]);
    EEPROM.write(offset + 2, arr[2]);
    EEPROM.write(offset + 3, arr[3]);
    EEPROM.commit();
    delay(5);
}

void accessUpdate(char *ver){
    if (flagUpdateFw){
        flagUpdateFw = false;        
        if (WiFi.status() == WL_CONNECTED) {
          DBG("Updating...");
            char *path;
            path = (char *)calloc(strlen(HOST_UPDATE) + strlen(ver) + 4, sizeof(char));
            sprintf(path, "%sFW%s.bin", HOST_UPDATE, ver);
            DBG(path);
            t_httpUpdate_return ret = ESPhttpUpdate.update(path);
            //t_httpUpdate_return  ret = ESPhttpUpdate.update("https://server/file.bin", "", "fingerprint");
            free(path);
            switch (ret) {
                case HTTP_UPDATE_FAILED:
                DBG("\nHTTP_UPDATE_FAILD Error :"); DBG(ESPhttpUpdate.getLastError()); DBG(ESPhttpUpdate.getLastErrorString());
                break;

                case HTTP_UPDATE_NO_UPDATES:
                DBG("HTTP_UPDATE_NO_UPDATES");
                break;

                case HTTP_UPDATE_OK:
                DBG("HTTP_UPDATE_OK");
                break;
            }
        }
    }
}

String splitStringJson(String strSignal, String scr){
    String result = "";
    if (scr.indexOf("{") >=0 && scr.indexOf("}") > 1 && scr.indexOf(strSignal)>0) {
        String json = scr.substring(scr.indexOf("{"), scr.indexOf("}") + 1);
        String temp = scr.substring(scr.indexOf(strSignal) + strSignal.length() + 2, scr.indexOf("}") + 1);
        // DBGT("temp: "); DBG(temp);
        String temp2 = temp.substring(temp.indexOf("\"") + 1, temp.indexOf("}") + 1 );
        // DBGT("temp2: "); DBG(temp2);
        result = temp2.substring(0, temp2.indexOf("\""));
        // DBGT("result: "); DBG(result);
    }
    return result;
}

void check_req_mqtt(char *req){
  if (strchr(req, '{') != NULL && strchr(req, '}') != NULL && strchr(req,':') !=NULL) {
      uint8_t len = 0;
      len = strchr(req,':') - strchr(req, '{') ;
      char *cmd;
      cmd = (char *)calloc((len + 1), sizeof(char));
      strncpy(cmd, strchr(req, '{') + 1, len-1);
      DBG(cmd);
      len = strchr(req,'}') - strchr(req, ':') ;
      if(strcmp(cmd, "UPDATEFW")== 0){
        memset(newVerFW, '\0', sizeof(newVerFW));
        strncpy(newVerFW, strchr(req,':')+1, len-1);
        if(strcmp(newVerFW, VERSION) == 0 ){
          DBG("This lasted version");
          DBG(newVerFW);
        } else {
          flagUpdateFw=true;
          DBG(newVerFW);
        }
      } else if(strcmp(cmd, "CONTROL")== 0){
        char *temp;
        temp = (char *)calloc(len + 1,sizeof(char));
        strncpy(temp, strchr(req,':')+1, len-1);
        if (strcmp(temp, "ON")== 0) {
            relayControl(device1, RELAYON);
        } else if (strcmp(temp, "OFF")== 0) {
            relayControl(device1, RELAYOFF);
        }
        free(temp);
      } else if(strcmp(cmd, "VIEW")== 0){
        char *temp;
        temp = (char *)calloc(len + 1,sizeof(char));
        strncpy(temp, strchr(req,':')+1, len-1);
        if (strcmp(temp, "TIME")== 0) {
          flagTimeView = true;
        }
        free(temp);
      } else if(strcmp(cmd, "DBG")== 0){
        char *temp;
        temp = (char *)calloc(len + 1,sizeof(char));
        strncpy(temp, strchr(req,':')+1, len-1);
        if (strcmp(temp, "EN")== 0) {
          flagDebug = true;
          EEPROM.write(ADD_DBG, 1);
          EEPROM.commit();
          delay(10);
        } else if (strcmp(temp, "DIS")== 0) {
          EEPROM.write(ADD_DBG, 0);
          EEPROM.commit();
          delay(10);
          flagDebug = false;
        }
        free(temp);
      } else if(strcmp(cmd, "ALARM") == 0){ //{ALARM:ID_ALARM, TIME, DAY, HOUR, MINUTE, REPEAT}
        char *temp;
        temp = (char *)calloc(len + 1,sizeof(char));
        strncpy(temp, strchr(req,':')+1, len-1);
        char *strtoken;
        strtoken = strtok(temp, ",");
        int offsetAdd = 0;
        for (int i = 0; i<6; i++){
          if (strtoken != NULL){
            if ( i==0) {
              if (strstr(strtoken,"0") !=  NULL){
                offsetAdd = 0;
              } else if (strstr(strtoken,"1") !=  NULL){
                offsetAdd = ADD_ALARM_STEP;
              } 
            } else if(i == 1) {
              int dayint = 0;
              dayint = atoi(strtoken);
              if(dayint > 0){
                EEPROM.write(ADD_ALARM_0 + offsetAdd/ADD_ALARM_STEP, 1);
              } else EEPROM.write(ADD_ALARM_0 + offsetAdd/ADD_ALARM_STEP, 0);
              EEPROM.write(ADD_ALARM_TIME + offsetAdd, dayint);
              EEPROM.commit();
              delay(5);
              DBG(dayint);
            } else if(i == 2) {
              int dayint = 0;
              dayint = atoi(strtoken);
              EEPROM.write(ADD_ALARM_DAY + offsetAdd, dayint);
              EEPROM.commit();
              delay(5);
              DBG(dayint);
            } else if(i == 3) {
              int dayint = 0;
              dayint = atoi(strtoken);
              EEPROM.write(ADD_ALARM_HOUR + offsetAdd, dayint);
              EEPROM.commit();
              delay(5);
              DBG(dayint);
            } else if(i == 4) {
              int dayint = 0;
              dayint = atoi(strtoken);
              EEPROM.write(ADD_ALARM_MINUTE + offsetAdd, dayint);
              EEPROM.commit();
              delay(5);
              DBG(dayint);
            } else if(i == 5) {
              int dayint = 0;
              dayint = atoi(strtoken);
              EEPROM.write(ADD_ALARM_REPEAT + offsetAdd, dayint);
              EEPROM.commit();
              delay(5);
              DBG(dayint);
            }
            strtoken = strtok(NULL, ",");
          } else return;
        }
        
        if (strcmp(temp, "DIS")== 0) {
          EEPROM.write(ADD_DBG, 0);
          EEPROM.commit();
          delay(10);
          flagDebug = false;
        }
        free(temp);
      }
      free(cmd);
  } else {
    DBGT("msg rec: ");
    DBG(req);
  }
}

void timecallback(uint32_t current) {
  // adjust to local time zone
  current += (timeZone * 60 * 60);

  // calculate current time
  sec = current % 60;
  current /= 60;
  minu = current % 60;
  current /= 60;
  hour = current % 24;

  // print hour
  if(hour == 0 || hour == 12)
    Serial.print("12");
  if(hour < 12)
    Serial.print(hour);
  else
    Serial.print(hour - 12);

  // print mins
  Serial.print(":");
  if(minu < 10) Serial.print("0");
  Serial.print(minu);

  // print seconds
  Serial.print(":");
  if(sec < 10) Serial.print("0");
  Serial.print(sec);

  if(hour < 12)
    Serial.println(" am");
  else
    Serial.println(" pm");

}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  char *msgpayload;
  msgpayload = (char *)calloc(128, sizeof(char));
  for(int i=0; i<length; i++){
    msgpayload[i] =  payload[i];
  }
  check_req_mqtt(msgpayload);
  Serial.println(length);
  Serial.println(topic);
  free(msgpayload);
}

void onoffcallback(char *data, uint16_t len) {
  if(len>0){
    check_req_mqtt(data);
    DBG(data);
  }
}

void time_is_set(void) {
  gettimeofday(&cbtime, NULL);
  cbtime_set = true;
  DBG("-- settimeofday() was called --");
  flagTimeView = true;
}
