/*
This firmware V02 use for series of manual gang from 1 to 3 with ESP8255 chip of SONOFF Cop. may be comlict with another device.
***Added function update OTA: server:192.168.1.128/*.bin
***Note: build for esp8266ex 1MB(sonoff wifi smart switch, sonoff remote control driver dimming)
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
#include <ESP8266Ping.h>
#include <stdio.h>
#include "configure.h"
#include "Timer.h"
#include <string.h>
// #include <ArduinoJson.h> // this lib may be get error quen using high frequency
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#define VERSION_INT     1.96
Timer timeDelay(SYS_100HZ);  // khoi tao timer delay 0.1s
#if defined(RGB)
String DEVICE_TYPE = "LED_DIMMER_RGB_WF";
#elif defined(CW)
String DEVICE_TYPE = "LED_DIMMER_CW_WF";
#endif
boolean Mode_Station = false;
int rst_val=0;
uint8_t brn_current_val = 0;
uint8_t color_current_val = 0;
char *Ssid = "";
char *Password = "";
char *Ssid_old = "";
char *Password_old = "";
String HOST_UPDATE = "http://192.168.1.128";
String RashomeIP = "";
int SyncNumber = 0;
String seriesNumber="";
String ip = ""; //ip when connect with modem
String default_gateway = "";    //when connect with modem static
String subnet_mark = "";    //when connect with modem static
String req_fr_server = "";  //string of request from server
String res_of_device = "";  //string of response of device to sent to server
String versionNewFw ="";
// other informations of Gang1
bool flagUpdateFw = false;
bool flagEnableCheckNumRst = true;
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
unsigned long timeStartNewRst = 0;
unsigned long indexOfBristh = 0;
unsigned long indexOfTurning = 0;
int bristh = 20;
bool bristh_state = true;
// const size_t bufferSize = JSON_OBJECT_SIZE(4) + 100; //define for decode string json
// StaticJsonBuffer<200> jsonBuffer;
// DynamicJsonBuffer jsonBuffer;
WiFiUDP Udp;
WiFiServer server(TCP_PORT_SERVER);
WiFiClient client;

void checkNumRst(void);
void button1ProcessCb(void);//callback for interrupt button 1 defined in congigure.h file
void button1Process(void); //process button 1(some mode)
String splitStringJson(String strSignal, String scr);
void controlLed(void); 
void brightness(String clr, String percent);
void processBrn(void);
void feedbackStatus(device_struct device);
void storeIP_tmp(String ip, int offset);
void storeIP(String StrIP, String StrGateway, String StrSubnet);
void RHPing(String request);
void RHchangestartup(String request);
void RHchangeWifi(String request);
void RhChangeIP(String request);
void SyncRh2Esp(int gang);
void syncEsp2Rh(String Id);
String String2Hash(String HashString);
void ClearEeprom();
void iotDiscovery();
void ReadEpprom();
void setupWifiAccessPoint(char *ssid, char* password);
boolean sendDataServer(String data);
void processDataFromServer(void);
void startupStatusProcess();
void WiFiReconnect();
boolean ResetFactory();
void checkConnectToRasHome();
void checkConnectWifi(void);
void checkCmd(void);
void accessUpdate(String ver);
// void setupSPIFFS();
#define MAX_BUF_REC_SERIAL   128
char sbuf[MAX_BUF_REC_SERIAL];

void setup() {
    ESP.wdtDisable();
    ESP.wdtEnable(TIME_WTD);
    Serial.begin(115200);
    delay(10);
    // Serial.readBytes(sbuf, 50);
    // Serial.setDebugOutput(true); //enable printf
    EEPROM.begin(512);
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
    String VERSION;
    VERSION = "V" + String(VERSION_INT);
    DBGT("\n>>>>> This test "); DBG(DEVICE_TYPE);
    DBGT(">>>>> Version: "); DBG(VERSION);
    DBGT(">>>>> Series: "); DBG(seriesNumber);

    pinMode(Button, INPUT);
    attachInterrupt(Button, button1ProcessCb, CHANGE);

#if defined(CW)
    pinMode(WARM_PIN, OUTPUT);
    pinMode(COLD_PIN, OUTPUT);
    analogWriteFreq(500);
#elif defined(RGB)
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
#endif
    brightness("0", "0");
    /******************************************/
    pinMode(Led, OUTPUT);
    digitalWrite(Led, LED_ST_OFF);
    /*******************************************/
    ReadEpprom();
    // setupSPIFFS();
    if (Mode_Station && (device1.state == INACTIVE_PAIRING || device1.state == WORKING)) {
        WiFi.begin(Ssid, Password);
        WiFi.mode(WIFI_STA);
        DBGT("Connected to wifi : ");  DBG(Ssid);
        DBGT("My IP : "); DBG(WiFi.localIP());
        server.begin();
        delay(500);
        if (device1.state == WORKING) {
            if (WiFi.status() == WL_CONNECTED) {
                String strPing = "{\"req\":\"PING\"}";
                if (sendDataServer(strPing)) {
                    isRasHomeAlive = true;
                    DBG("Rashome Working");
                } else {
                    DBG("RasHome DIE");
                    isRasHomeAlive = false;
                }
            }
        }
    } else
        WiFi.mode(WIFI_STA);
    startupStatusProcess();
    DBG("Setup DONE");
    ESP.wdtFeed();
}

void loop() {
    // checkConnectToRasHome();
    checkNumRst();
    ESP.wdtFeed();
    checkCmd();
    ESP.wdtFeed();
    checkConnectWifi();
    ESP.wdtFeed();
    // checkConnectToRasHome();
    controlLed();
    ESP.wdtFeed();
    button1Process();
    ESP.wdtFeed();
    processDataFromServer();
    ESP.wdtFeed();
    processBrn();
    ESP.wdtFeed();
}

void checkNumRst(void){
    if (flagEnableCheckNumRst){
        if (millis() - timeStartNewRst > TIME_WAIT_CLEAR_NUM_RST){
            rst_val = 0;
            EEPROM.write(ADD_NUM_SHORT_RST, rst_val);
            EEPROM.commit();
            flagEnableCheckNumRst = false;
        }
    }
}

void button1ProcessCb() {
    if(digitalRead(Button) == PRESS) {
        bt1.pressed = PRESS;
        bt1.timestart = millis();
        bt1.change = true;
    } else {
        bt1.pressed = RELEASE;
        bt1.timestop = millis();
        bt1.change = true;
    }
}

void button1Process(){
    if (device1.state == UNINSTALL) {
        if (bt1.change) {
            if (bt1.pressed == PRESS){
                bt1.change = false;
            } else {
                bt1.change = false;
                unsigned long time_t = bt1.timestop - bt1.timestart;
                DBG(time_t);
                if (time_t < TIME_TO_LEARN_WF){
                    // relayControl(RELAY);
                    bt1.led = true;
                    DBG("UNINSTALL: BUTTON PRESSED");
                } else if (time_t< TIME_TO_RST_FAC) {
                    device1.preState = device1.state;
                    device1.state = LEARN_WF;
                    timeStartLearnWF = millis();
                    DBG("LEARN WIFI ...");
                    setupWifiAccessPoint(AP_SSID, AP_PW);
                } else {
                    DBG("Wait for RST Factory ...");
                    timeStartRSTFac = millis();
                    bt1.counter = 0;
                    device1.preState = device1.state;
                    device1.state = RST_FAC;
                }
            }
        }
    } else if (device1.state == LEARN_WF) {
        if (millis() - timeStartLearnWF > TIME_WAIT_LEARN_WF){
            device1.state = device1.preState;
            timeCounterLed = 0;
            WiFi.softAPdisconnect();
            WiFi.disconnect();
            DBG("Overtime to learn WIFI -> Exit mode learn WIFI goto previous mode");
        } else {
            if (bt1.change) {
                if (bt1.pressed == PRESS){
                    bt1.change = false;
                } else {
                    bt1.change = false;
                    device1.state = device1.preState;
                    timeCounterLed = 0;
                    WiFi.softAPdisconnect();
                    delay(5);
                    WiFi.disconnect();
                    DBG("Break change mode to previous mode");
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
                    unsigned long time_t = bt1.timestop - bt1.timestart;
                    DBG(time_t);
                    if (time_t < TIME_ENABLE_RST_FAC){
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
    } else if (device1.state == INACTIVE_PAIRING) {
        if (bt1.change) {
            if (bt1.pressed == PRESS){
                bt1.change = false;
            } else {
                bt1.change = false;
                unsigned long time_t = bt1.timestop - bt1.timestart;
                DBG(time_t);
                if (time_t < TIME_TO_ACTIVE_PAIRING){
                    bt1.led = true;
                    DBG("INACTIVE: BUTTON PRESSED");
                } else if (time_t< TIME_TO_RST_FAC) {
                    Udp.begin(UDP_PORT);   //  khoi tao udp
                    DBG("Bat dau UDP port");
                    device1.preState = device1.state;
                    device1.state = ACTIVE_PAIRING;
                    timeStartActivePairing = millis();
                    DBG("Mode: ACTIVE_PAIRING");
                } else {
                    DBG("Wait for RST Factory ...");
                    timeStartRSTFac = millis();
                    bt1.counter = 0;
                    device1.preState = device1.state;
                    device1.state = RST_FAC;
                }
            }
        }
    } else if (device1.state == ACTIVE_PAIRING) {
        if (millis() - timeStartActivePairing > TIME_WAIT_ACTIVE_PAIRING){
            device1.state = device1.preState;
            timeCounterLed = 0;
            DBG("Overtime active pairing -> Change mode to previous mode");
        } else {
            if (bt1.change) {
                if (bt1.pressed == PRESS){
                    bt1.change = false;
                } else {
                    bt1.change = false;
                    device1.state = device1.preState;
                    timeCounterLed = 0;
                    DBG("Break change mode to previous mode");
                }
            }
            iotDiscovery();
        }
    } else if (device1.state == WORKING) {
        if (bt1.change) {
            if (bt1.pressed == PRESS){
                bt1.change = false;
            } else {
                bt1.change = false;
                unsigned long time_t = bt1.timestop - bt1.timestart;
                DBG(time_t);
                if (time_t < TIME_TO_RST_FAC) { 
                    bt1.led = true;
                    DBG("WORKING: Channel 1 ON");
                    // feedbackStatus(device1.id);
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

void checkCmd(void) {
    if (Serial.available()){
        memset(sbuf, '\0', MAX_BUF_REC_SERIAL);
        Serial.readBytes(sbuf, MAX_BUF_REC_SERIAL);
        String cmdbk = "";
        cmdbk = sbuf;
        DBGT("Received cmd: "); DBG(cmdbk);
        if(cmdbk == "endbg"){
            flagDebug = true;
            DBG("Start debug");
        } else if(cmdbk == "disdbg"){
            flagDebug = false;
            DBG("Stop debug");
        } else if(cmdbk == "myip"){
            DBGT("\tIP : "); DBG(ip);
        } else if(cmdbk == "rship"){
            DBGT("\tRashomeIP : "); DBG(RashomeIP);
        } else if(cmdbk == "ssid"){
            DBGT("\tSSID : "); DBG(Ssid);
            DBGT("\tPassword : "); DBG(Password);
        } else if(cmdbk == "restart"){
            ESP.restart();
        } else if(cmdbk == "reset"){
            EEPROM.write(ADD_NUM_SHORT_RST, 3);
            EEPROM.commit();
            ESP.restart();
        } else if(cmdbk == "led"){
            DBGT("\color : "); DBG(device1.color_val);
            DBGT("\brightness : "); DBG(device1.brightness_val);
            DBGT("\status : "); DBG(device1.status_val);
        } else {
            DBGT("NOT FOUND\n");
        }
    }
}

void controlLed(void){
#ifdef  CW
    if (device1.state == LEARN_WF){
        if (millis() - indexOfBristh > SPEED_BRISTH_LEARN_WF) {
            indexOfBristh = millis();
            if (bristh_state) {
                bristh+=1;
                if (bristh>80){
                    bristh_state = false;
                }
            } else {
                bristh-=1;
                if (bristh < 10){
                    bristh_state = true;
                }
            }
            brightness("50", String(bristh));
        }
    } else if (device1.state == ACTIVE_PAIRING){
        if (millis() - indexOfBristh > SPEED_BRISTH_PAIR) {
            indexOfBristh = millis();
            if (bristh_state) {
                bristh+=1;
                if (bristh>80){
                    bristh_state = false;
                }
            } else {
                bristh-=1;
                if (bristh < 10){
                    bristh_state = true;
                }
            }
            brightness("50", String(bristh));
        }
    }
#endif
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
    } else if ( device1.state == INACTIVE_PAIRING) {
        if (bt1.led == true){
            timeCounterLed = 3;
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
    } else if ( device1.state == ACTIVE_PAIRING || ((millis() - bt1.timestart > TIME_TO_ACTIVE_PAIRING)&&( device1.state == INACTIVE_PAIRING)&&(bt1.timestop < bt1.timestart))){
        if ( timeCounterLed <= 0) {
            digitalWrite(Led, LED_ST_OFF);
            if (millis() - timePreLedOff > TIME_DELAY_LED_SOFF) {
                timeCounterLed = 3;
            }   
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
    } else if ( device1.state == LEARN_WF || ((millis() - bt1.timestart > TIME_TO_LEARN_WF)&&(bt1.timestop < bt1.timestart)&&( device1.state == UNINSTALL))){
        if ( timeCounterLed <= 0) {
            digitalWrite(Led, LED_ST_OFF);
            if (millis() - timePreLedOff > TIME_DELAY_LED_SOFF) {
                timeCounterLed = 2;
            }   
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

void brightness(String clr, String percent){
    long brn_val = percent.toInt();
    device1.brightness_val = brn_val;
    brn_val = brn_val*1024/100;
    long color_val = clr.toInt();
    device1.color_val = color_val;
    color_val = color_val*brn_val/100;
// #ifdef  CW
    // analogWrite(WARM_PIN, color_val);
    // analogWrite(COLD_PIN, brn_val - color_val);
// #endif
}

void processBrn(void){
    long brn_t, color_t;
    if (millis() - indexOfTurning > TURNING_LED){
        indexOfTurning = millis();
        if ((brn_current_val != device1.brightness_val) || (color_current_val != device1.color_val)){
            if (brn_current_val > device1.brightness_val){
                brn_current_val -=1;
                brn_t = brn_current_val*1024/100;
                color_t = color_current_val*brn_t/100;
            } else if (brn_current_val < device1.brightness_val){
                brn_current_val +=1;
                brn_t = brn_current_val*1024/100;
                color_t = color_current_val*brn_t/100;
            } 
            if (color_current_val > device1.color_val){
                color_current_val -=1;
                brn_t = brn_current_val*1024/100;
                color_t = color_current_val*brn_t/100;
            } else if (color_current_val < device1.color_val){
                color_current_val +=1;
                brn_t = brn_current_val*1024/100;
                color_t = color_current_val*brn_t/100;
            }
            analogWrite(WARM_PIN, color_t);
            analogWrite(COLD_PIN, brn_t - color_t);
        }
    }
    if (flagUpdateFw && (brn_current_val == 0) ) {
        accessUpdate(versionNewFw);
    }
}
//*************READ EEPROM**************
void ReadEpprom() {
    DBG("Read Epprom");
    String tempStr = "";
    Mode_Station = false;
    ip = "";
    subnet_mark = "";
    default_gateway = "";
    for (int i = ADD_ID1_F; i <= ADD_ID1_E; i++) {
        device1.id += String(EEPROM.read(i) - 0x30);
    }
    DBGT("\tID device: \t"); DBG(device1.id);
    for (int i = ADD_KEY1_F; i <= ADD_KEY1_E; i++) {
        device1.secretKey += String(EEPROM.read(i) - 0x30);
    }
    DBGT("\tSecretkey:  \t"); DBG(device1.secretKey);
    if (EEPROM.read(ADD_STARTUP1) == 0) {
        device1.startup = "ON";
    } else if (EEPROM.read(ADD_STARTUP1) == 1) {
        device1.startup = "OFF";
    } else if (EEPROM.read(ADD_STARTUP1) == 2) {
        device1.startup = "SYN";
    }
    DBGT("\tStartup: \t"); DBG(device1.startup);
    if (EEPROM.read(ADD_STATE1) == 3) {
        device1.state = WORKING;
        tempStr = "WORKING";
    } else if (EEPROM.read(ADD_STATE1) == 2) {
        device1.state = INACTIVE_PAIRING;
        tempStr = "INACTIVE_PAIRING";
    } else {
        device1.state = UNINSTALL;
        tempStr = "UNINSTALL";
    }
    DBGT("\tState: \t"); DBG(tempStr);
#ifdef CW
    device1.warm_val = EEPROM.read(ADD_VALUE_WARM);
    device1.cold_val = EEPROM.read(ADD_VALUE_COLD);
#endif
#ifdef RGB
    device1.red_val = EEPROM.read(ADD_VALUE_RED);
    device1.green_val = EEPROM.read(ADD_VALUE_GREEN);
    device1.blue_val = EEPROM.read(ADD_VALUE_BLUE);
#endif
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
        ip = String(EEPROM.read(ADD_IP_STA_F)) + '.' + String(EEPROM.read((ADD_IP_STA_F + 1))) + '.' + String(EEPROM.read((ADD_IP_STA_F + 2))) + '.' + String(EEPROM.read(ADD_IP_STA_E));
        DBGT("\tIP : "); DBG(ip);
        IPAddress gateway(EEPROM.read(ADD_GETWAY_F), EEPROM.read((ADD_GETWAY_F + 1)), EEPROM.read((ADD_GETWAY_F + 2)), EEPROM.read(ADD_GETWAY_E));
        default_gateway = String(EEPROM.read(ADD_GETWAY_F)) + '.' + String(EEPROM.read(ADD_GETWAY_F + 1)) + '.' + String(EEPROM.read(ADD_GETWAY_F + 2)) + '.' + String(EEPROM.read(ADD_GETWAY_E));
        DBGT("\tDEfault Gateway : "); DBG(default_gateway);
        IPAddress subnet(EEPROM.read(ADD_SUBNETMARK_F), EEPROM.read(ADD_SUBNETMARK_F + 1), EEPROM.read(ADD_SUBNETMARK_F + 2), EEPROM.read(ADD_SUBNETMARK_E));
        subnet_mark = String(EEPROM.read(ADD_SUBNETMARK_F)) + '.' + String(EEPROM.read(ADD_SUBNETMARK_F + 1)) + '.' + String(EEPROM.read(ADD_SUBNETMARK_F + 2)) + '.' + String(EEPROM.read(ADD_SUBNETMARK_E));
        DBGT("\tSubnet mark : "); DBG(subnet_mark);
        WiFi.config(ip_, gateway, subnet);
    }
  // READ SSID AND PASSWORD
    if (device1.state == INACTIVE_PAIRING || device1.state == WORKING) {
        if (EEPROM.read(ADD_LEN_SSID) > 0) {
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
            DBG("Lose SSID and PW");
            DBG("state = UNINSTALL");
        }
        // IPAddress rashomeip_(EEPROM.read(ADD_RSH_IP_F), EEPROM.read((ADD_RSH_IP_F + 1)), EEPROM.read((ADD_RSH_IP_F + 2)), EEPROM.read(ADD_RSH_IP_E));
        RashomeIP = String(EEPROM.read(ADD_RSH_IP_F)) + '.' + String(EEPROM.read(ADD_RSH_IP_F + 1)) + '.' + String(EEPROM.read(ADD_RSH_IP_F + 2)) + '.' + String(EEPROM.read(ADD_RSH_IP_E));
        DBGT("\tRashomeIP : "); DBG(RashomeIP);
       
    }
    device1.color_val = EEPROM.read(ADD_VALUE_COLOR);
    device1.brightness_val = EEPROM.read(ADD_VALUE_BRN);
    rst_val = EEPROM.read(ADD_NUM_SHORT_RST);
    DBGT("rst num read: "); DBG(rst_val);
    if (rst_val < 2){
        timeStartNewRst = millis();
        rst_val +=1;
        EEPROM.write(ADD_NUM_SHORT_RST, rst_val);
        DBGT("rst num write: "); DBG(rst_val);
        EEPROM.commit();
    } else {
        rst_val = 0;
        EEPROM.write(ADD_NUM_SHORT_RST, rst_val);
        EEPROM.commit();
        device1.state = UNINSTALL;
        ResetFactory();
    }
    if (device1.state == UNINSTALL){
        bt1.change = true;
        bt1.pressed = RELEASE;
        bt1.timestop = bt1.timestart + TIME_TO_LEARN_WF + 10;
    } else if (device1.state == INACTIVE_PAIRING) {
        bt1.change = true;
        bt1.pressed = RELEASE;
        bt1.timestop = bt1.timestart + TIME_TO_ACTIVE_PAIRING + 10;
    } 
    DBGT("State: "); DBG(device1.state);
}

/*
void setupSPIFFS() {
    DBG("Start SPIFFS\n");
    if (!SPIFFS.begin()) {
        DBG("Failed to mount file system\n");
        return;
    } else {
        DBG("Mount file system success\n");
        File ca = SPIFFS.open("/hlsoft.cer", "r"); //replace ca.crt eith your uploaded file name
        if (!ca) {
            DBG("Failed to open ca file\n");
        } else {
            DBG("Success to open ca file\n");
            bool res = clientS.loadCertificate(ca);
            if (!res) {
                DBG("Failed to load root CA certificate!\n");
                while (true) {
                    yield();
                }
            } else {
                DBG("Load root CA certificate success\n");
            } 
        }
    }
    DBG("DONE\n");
}
*/

void checkConnectToRasHome() {
    if (isRasHomeAlive == false) {
        if (millis() - timeStartCheckConnect > TIME_TO_CHECK_CONNECT) {    
            timeStartCheckConnect = millis();
            if (device1.state == WORKING) {
                if (WiFi.status() == WL_CONNECTED) {
                    String strPing = "{\"req\":\"PING\"}";
                    if (sendDataServer(strPing)) {
                        isRasHomeAlive = true;
                        DBG("Rashome Working");
                        if (flagSyncToRsh) {
                            SyncRh2Esp(1);
                            flagSyncToRsh = false;
                        }
                    } else {
                        DBG("RasHome DIE");
                        isRasHomeAlive = false;
                    }
                }
            }
        }
    }
}

void checkConnectWifi(void) {
    if (millis() - timeStartCheckConnectWifi > TIME_TO_CHECK_CONNECT_WIFI) {    
        timeStartCheckConnectWifi = millis();
        if (device1.state == WORKING || device1.state == INACTIVE_PAIRING) {
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
        if (device1.state == WORKING){
            syncEsp2Rh(device1.id);
        }
    }
}

boolean ResetFactory() { 
    device1.state = UNINSTALL;
    Mode_Station = false;
    ClearEeprom();
    ReadEpprom();
    brightness("0", "0");
    server.close();
    WiFi.softAPdisconnect();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
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
    String SHA1 = String2Hash("FEED_BACK" + device.id + device.color_val + device.status_val + device.brightness_val + String(SyncNumber + 1) + device1.secretKey)  + String(SyncNumber + 1);
    String strFb = "{\"req\":\"FEED_BACK\", \"id\":\"" + device.id + "\", \"color\":\"" + device.color_val + "\", \"brn\":\"" + device.brightness_val + "\", \"status\":\"" + device.status_val + "\", \"checksum\": \"" + SHA1 + "\"}";
    if (sendDataServer(strFb)) {
        DBG(strFb);
    } else {
        DBG("Send FB error");
    }
}

void startupStatusProcess() {
    // if (device1.state == LEARN_WF){
        // setupWifiAccessPoint(AP_SSID, AP_PW);
    // } else if (device1.state == ACTIVE_PAIRING) {
        // Udp.begin(UDP_PORT);
    // } 
    if (device1.state == WORKING) {
        if (device1.startup == "SYN") {
            if (isRasHomeAlive) {
                SyncRh2Esp(1);
            } else {
                flagSyncToRsh = true;
            }
        } else if (device1.startup == "ON") {
            device1.status_val = "ON";
            brightness(String(device1.color_val), String(device1.brightness_val));
            feedbackStatus(device1);
        } else if (device1.startup == "OFF") {
            brightness("0", String("0"));
            device1.status_val = "OFF";
            feedbackStatus(device1);
        }
    } else {
        brightness(String("0"), String("0"));
        device1.status_val = "OFF";
        // relayControl(RELAYOFF);
    }
}

boolean sendDataServer(String data){
    if (WiFi.status() == WL_CONNECTED) {
        // if (isRasHomeAlive) {
            WiFiClient client2;
            if (client2.connect(RashomeIP, TCP_PORT_CLIENT)) {
                client2.println(data);
                client2.stop();
                return true;
            } else {
                client2.stop();
                isRasHomeAlive = false;
                DBGT("IP Rashome: "); DBG(RashomeIP);
                DBGT("Port: "); DBG(TCP_PORT_CLIENT);
                DBG("Connect TCP to feedback failed");
                return false;
            }
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
    DBGT(" IP client: "); DBG(clientConnect.remoteIP());
    unsigned long timeout = millis() + 3000;
    while(!clientConnect.available() && millis() < timeout){
        delay(1);
    }
    if (millis() > timeout) {
        DBG("timeout");
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
        if (device1.state == LEARN_WF) {
            String req = splitStringJson("req", req_fr_server); // "DEVICE"
            if (req == "DEVICE") {
                res_of_device = "{\"res\":\"DEVICE\", \"type\":\"" + DEVICE_TYPE + "\"}";
                clientConnect.print(res_of_device);
                DBGT("res_of_device: "); DBG(res_of_device);
            } else if ( req == "NETWORK") {
                String ssid_ser = splitStringJson("ssid", req_fr_server);
                String password_ser = splitStringJson("password", req_fr_server);
                String ipState_ser = splitStringJson("ip", req_fr_server);
                String ipAdd_ser = splitStringJson("ip_static", req_fr_server);
                String network_prefix_len_ser = splitStringJson("network_prefix_len", req_fr_server);
                String gateway_ser = splitStringJson("gateway", req_fr_server);
                clientConnect.print("{\"res\":\"NETWORK\"}");
                DBG("{\"res\":\"NETWORK\"}");
                Ssid = (char*)malloc((ssid_ser.length()) * sizeof(char));
                Password = (char*)malloc((password_ser.length()) * sizeof(char));
                ssid_ser.toCharArray(Ssid, ssid_ser.length() + 1);
                password_ser.toCharArray(Password, password_ser.length() + 1);
                if ( ipState_ser == "STATIC") {
                    ip = ipAdd_ser;
                    if (network_prefix_len_ser == "8") {
                        subnet_mark = "255.0.0.0";
                    } else if (network_prefix_len_ser == "16") {
                        subnet_mark = "255.255.0.0";
                    } else if (network_prefix_len_ser == "24") {
                        subnet_mark = "255.255.255.0";
                    }
                    default_gateway = gateway_ser;
                    storeIP(ip, default_gateway, subnet_mark, ipState_ser);
                }
                DBGT("ssid: "); DBG(Ssid);
                DBGT("password: "); DBG(Password);
                DBGT("ip: "); DBG(ip);
                DBGT("subnet: "); DBG(subnet_mark);
                DBGT("gateway: "); DBG(default_gateway);
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
                IPAddress rasHomeIpRemote = clientConnect.remoteIP();
                String rshIpStr = String(rasHomeIpRemote[0]) + String(rasHomeIpRemote[1]) + String(rasHomeIpRemote[2]) + String(rasHomeIpRemote[3]);
                storeIP_tmp(rshIpStr, ADD_RSH_IP_F);
                DBGT("IP RSH: ");DBG(rshIpStr);
                DBG("Close server");
                server.close();
                WiFi.softAPdisconnect();
                DBG("disconnect Wifi");
                WiFi.disconnect();
                DBG("Setup wifi and server");
                WiFi.mode(WIFI_STA);
                IPAddress ip_(EEPROM.read(ADD_IP_STA_F), EEPROM.read(ADD_IP_STA_F + 1), EEPROM.read(ADD_IP_STA_F + 2), EEPROM.read(ADD_IP_STA_E));
                IPAddress gateway(EEPROM.read(ADD_GETWAY_F), EEPROM.read(ADD_GETWAY_F + 1), EEPROM.read(ADD_GETWAY_F + 2), EEPROM.read(ADD_GETWAY_E));
                IPAddress subnet(EEPROM.read(ADD_SUBNETMARK_F), EEPROM.read(ADD_SUBNETMARK_F + 1), EEPROM.read(ADD_SUBNETMARK_F + 2), EEPROM.read(ADD_SUBNETMARK_E));
                WiFi.config(ip_, gateway, subnet);
                WiFi.begin(Ssid, Password);
                server.begin();
                // state = "PAIRING"; me
                device1.state = INACTIVE_PAIRING;  // Chuyen trang thai cho nut nut nhan1 va nut nhan 2
                DBG("device1.state = INACTIVE PAIRING");
                EEPROM.write(ADD_STATE1, 2);
                EEPROM.commit();
                delay(5);
                DBG("Configure DONE");
                bt1.change = true;
                bt1.timestop = bt1.timestart + 1 + TIME_TO_ACTIVE_PAIRING;
                return;
            }
        } else {
            String SHA1 = "";
            // String RandomInt;
            String req = splitStringJson("req", req_fr_server);
            DBG(req); 
            if ( req == "PAIR") {
                String type = splitStringJson("type", req_fr_server); // "FEED_BACK"
                String id = splitStringJson("id", req_fr_server); // "FEED_BACK"
                String secretkey = splitStringJson("secretkey", req_fr_server); // "FEED_BACK"
                String startup_t = splitStringJson("startup", req_fr_server); // "FEED_BACK"
                DBGT("Type: "); DBG(type);
                DBGT("NewID:"); DBG(id);
                DBGT("secret key: "); DBG(secretkey);
                DBGT("Startup :"); DBG(startup_t);
                
                String tempStr = "";
                IPAddress tempip = clientConnect.remoteIP();
                tempStr = String(tempip[0]) + '.' + String(tempip[1]) + '.' + String(tempip[2]) + '.' + String(tempip[3]);
                if (RashomeIP != tempStr) {
                    DBGT("\tRashome IP old: "); DBG(RashomeIP);
                    DBGT("\tRashome IP new: "); DBG(tempStr);
                    storeIP_tmp(tempStr, ADD_RSH_IP_F);
                    RashomeIP = tempStr;
                    DBGT("\tRashome IP updated: "); DBG(RashomeIP);
                } else {
                    DBGT("\tRashome IP same: "); DBG(tempStr);
                }
                EEPROM.write(ADD_ID1_F, id[0]);
                EEPROM.write(ADD_ID1_F + 1, id[1]);
                EEPROM.write(ADD_ID1_E, id[2]);
                device1.id = id;

                EEPROM.write(ADD_KEY1_F, secretkey[0]);
                EEPROM.write(ADD_KEY1_F + 1, secretkey[1]);
                EEPROM.write(ADD_KEY1_F + 2, secretkey[2]);
                EEPROM.write(ADD_KEY1_E, secretkey[3]);
                device1.secretKey = secretkey;

                EEPROM.write(ADD_STATE1, 3);  // BIEN TRANG THAI
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
                String color_t = splitStringJson("color", req_fr_server);
                String brn_t = splitStringJson("brn", req_fr_server);
                EEPROM.write(ADD_VALUE_COLOR, color_t.toInt());
                EEPROM.write(ADD_VALUE_BRN, brn_t.toInt());
                
                EEPROM.commit();
                delay(5);
                device1.state = WORKING;
                DBG("-----------DEVICE WORKING --------");
                feedbackStatus(device1);
                SyncNumber = 1;
                Udp.stop();
                clientConnect.println("{\"res\":\"PAIR\"}");
                DBG("{\"res\":\"PAIR\"}");
            } else if (req == "UPDATE_FW") {
                versionNewFw = splitStringJson("version", req_fr_server);
                if (versionNewFw != ""){
                    flagUpdateFw = true;
                    brightness("0", "0");
                }
            } else if (req == "CONTROL") {
                boolean keepFb = false;
                String Id = splitStringJson("id", req_fr_server); // "306"
                String Status = splitStringJson("status", req_fr_server); // "OFF"
                // const char* Checksum = root["checksum"]; // "9ac2138rhjgdfhkjhgkgjjhn12"
                // RandomInt = Checksum.substring(8, Checksum.length());
                // Checksum.remove(8, Checksum.length());
                String idOfDevice_t = "";
                String secretKey_t = "";
                String color = splitStringJson("color", req_fr_server);
                String brn = splitStringJson("brn", req_fr_server);
                if ( Id == device1.id) {
                    // SHA1 = String2Hash("CONTROL" + idOfDevice_t + Status + RandomInt + secretKey_t);
                    // DBGT("SHA1 =   "); DBG(SHA1);
                    // DBGT(" Check Sum = "); DBG(Checksum);
                    // if (RandomInt.toInt() > SyncNumber) DBG(" yesss");
                    // if ((SHA1 == Checksum) && (RandomInt.toInt() > SyncNumber)) {
                    // SyncNumber = RandomInt.toInt();
                    device1.status_val = Status;
                    if (Status == "ON") {
                        brightness(color, brn);
                    } else if (Status == "OFF") {
                        brightness(color, "0");
                    } 
/*
#ifdef  RGB
                    String red = root["red"];
                    String green = root["green"];
                    String blue = root["blue"];
                    if (Status == "ON") {
                        brightness(RED_PIN, red);
                        brightness(GREEN_PIN, green);
                        brightness(BLUE_PIN, blue);
                    } else if (Status == "OFF") {
                        brightness(RED_PIN, "0");
                        brightness(GREEN_PIN, "0");
                        brightness(BLUE_PIN, "0");
                    } 
#endif
*/
                    else if (Status == "INACTIVE_PAIRING") {
                        keepFb = true;
                        if ( Id == device1.id) {
                            device1.state = INACTIVE_PAIRING;
                            DBG("State 1 = INACTIVE_PAIRING");
                            EEPROM.write(ADD_STATE1, 2);
                        } 
                        EEPROM.commit();
                        delay(5);
                    }
                    clientConnect.println("{\"res\":\"" + device1.status_val + "\"}");
                    // if (!keepFb) {
                        // feedbackStatus(device1);
                    // }
                }
            } else if (req == "CHANGE_IP") {
                String ip_address_new = splitStringJson("ip_address", req_fr_server);
                String netmask_new = splitStringJson("net_mask", req_fr_server);
                String gateway_new = splitStringJson("gateway", req_fr_server);
                DBGT("ip_address : "); DBG(ip_address_new);
                DBGT("netmask : "); DBG(netmask_new);
                DBGT("gateway : "); DBG(gateway_new);
                ip = ip_address_new;
                default_gateway = gateway_new;
                subnet_mark = netmask_new;
                String ipState_ser = "STATIC";
                storeIP(ip, default_gateway, subnet_mark, ipState_ser);
                DBG("Close server");
                server.close();
                WiFi.softAPdisconnect();
                DBG("disconnect Wifi");
                WiFi.disconnect();
                DBG("Setup wifi and server");
                IPAddress ip_(EEPROM.read(ADD_IP_STA_F), EEPROM.read(ADD_IP_STA_F + 1), EEPROM.read(ADD_IP_STA_F + 2), EEPROM.read(ADD_IP_STA_E));
                IPAddress gateway(EEPROM.read(ADD_GETWAY_F), EEPROM.read(ADD_GETWAY_F + 1), EEPROM.read(ADD_GETWAY_F + 2), EEPROM.read(ADD_GETWAY_E));
                IPAddress subnet(EEPROM.read(ADD_SUBNETMARK_F), EEPROM.read(ADD_SUBNETMARK_F + 1), EEPROM.read(ADD_SUBNETMARK_F + 2), EEPROM.read(ADD_SUBNETMARK_E));
                WiFi.config(ip_, gateway, subnet);
                WiFi.begin(Ssid, Password);
                server.begin();
                DBG("Setup DONE");
            } else if (req == "SYN") {
                String Id = splitStringJson("id", req_fr_server);
                syncEsp2Rh(Id);
            } else if (req == "CHANGE_RASHOME_IP") {
                String Id = splitStringJson("id", req_fr_server);
                String IP = splitStringJson("ip", req_fr_server);
                String CheckSum = splitStringJson("checksum", req_fr_server);
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                String idOfDevice_t = "";
                String secretKey_t = "";
                if (device1.id == Id) {
                    idOfDevice_t = device1.id;
                    secretKey_t = device1.secretKey;
                } 
                if (idOfDevice_t != "") {
                    String SHA1 = String2Hash("CHANGE_RASHOME_IP" + idOfDevice_t + IP + RandomInt + secretKey_t);
                    DBGT("New RasHome IP = ");
                    DBG(IP);
                    // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                        SyncNumber = RandomInt;
                        storeIP_tmp(IP, ADD_RSH_IP_F);
                        DBG("Changed RasHome IP");
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
                String idOfDevice_t = "";
                String secretKey_t = "";
                if (device1.id == Id) {
                    idOfDevice_t = device1.id;
                    secretKey_t = device1.secretKey;
                } 
                if (idOfDevice_t != "") {
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
                        delay(1000);
                        DBGT("New SSID: "); DBG(ssid_ser);
                        DBGT("New Password: ");DBG(password_ser);
                        server.begin();
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
                    DBG("scan start...");
                    // WiFi.scanNetworks will return the number of networks found
                    int n = WiFi.scanNetworks();
                    DBG("scan done");
                    if (n == 0) {
                        DBG("no networks found");
                    } else {
                        DBGT(n);
                        DBG(" networks found");
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
                                DBG((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
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
                String idOfDevice_t = "";
                String secretKey_t = "";
                int startup_add = 0;
                if (device1.id == Id) {
                    idOfDevice_t = device1.id;
                    secretKey_t = device1.secretKey;
                    startup_add = ADD_STARTUP1;
                } 
                if (idOfDevice_t != "") {
                    String SHA1 = String2Hash("startup" + idOfDevice_t + Status + RandomInt + secretKey_t);
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
                        delay(5);
                        DBG("Changed startup status DONE");
                    // } else {
                        // DBG("Wrong security");
                    // }
                }
            } else if (req == "PING") {
                String Id = splitStringJson("id", req_fr_server);
                String CheckSum = splitStringJson("checksum", req_fr_server);
                int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
                CheckSum.remove(8, CheckSum.length());
                String idOfDevice_t = "";
                String secretKey_t = "";
                if (device1.id == Id) {
                    idOfDevice_t = device1.id;
                    secretKey_t = device1.secretKey;
                } 
                if (idOfDevice_t != "") {
                    String SHA1 = String2Hash("PING" + idOfDevice_t + RandomInt + secretKey_t);
                    // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                        SyncNumber = RandomInt;
                        SHA1 = String2Hash("PING_FEED_BACK" + idOfDevice_t + String(SyncNumber + 1) + secretKey_t)  + String(SyncNumber + 1);
                        String strFb = "{\"req\":\"PING_FEED_BACK\", \"id\":\"" + idOfDevice_t + "\", \"checksum\":\"" + SHA1 + "\"}";
                        if (sendDataServer(strFb)) {
                            DBG(strFb);
                        } else {
                            DBG("Send FB error");
                        }
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

void iotDiscovery() {
    char incomingPacket[255];  // buffer for incoming packets
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        // printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
        int len = Udp.read(incomingPacket, 255);
        if (len > 0) incomingPacket[len] = 0;
        DBG(incomingPacket);
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        char* type_ = (char*)malloc((DEVICE_TYPE.length()) * sizeof(char));
        DEVICE_TYPE.toCharArray(type_, DEVICE_TYPE.length() + 1);
        // if (strstr(incomingPacket, "IOT_DISCOVERY") != NULL && strstr(incomingPacket, type_) != NULL) {
        if (strstr(incomingPacket, "IOT_DISCOVERY") != NULL && strstr(incomingPacket, "WF") != NULL) {
            String ss = Ssid;
            DBG("Detected IOT_DISCOVERY");
            if (device1.state == ACTIVE_PAIRING) {
                String res = "{\"res\":\"IOT_DISCOVERY\", \"type\":\"" + DEVICE_TYPE + "\",\"seriesNum\":\"" + seriesNumber + "\", \"ssid\":\"" + ss + "\"}";
                for (int i = 0; i < 5; i++) {
                    Udp.print(res);
                    Udp.endPacket();
                }
                DBG(res);
            }
        }
    }
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

void syncEsp2Rh(String Id) {
    String idOfDevice_t ="";
    String secretKey_t = "";
    String Status = "";
    int Relay_t = -1;
    boolean enable = true;
    if ( Id == device1.id && device1.state == WORKING) {
        idOfDevice_t = device1.id;
        secretKey_t = device1.secretKey;
        if (WiFi.status() == WL_CONNECTED) {
            enable = true;
        }else {
            enable = false;
        }
        if (enable == true) {
            feedbackStatus(device1);
            /*
            String wvalue = String(warm_brn_current);
            String cvalue = String(cold_brn_current);
            String strSyn = "{\"res\":\"SYN\", \"id\":\"" + idOfDevice_t + "\", \"wvalue\":\"" + wvalue + "\",\"cvalue\":\"" + cvalue + "\", \"checksum\":\"" + CHKSUM + String(SyncNumber) + "\"}";
                // char strFb[128];
                // memset(strFb, '0', 128);
                // sprintf(strFb,"{\"res\":\"SYN\", \"id\":\"%s\", \"status\":\"%s\", \"checksum\":\"%s\"}","257", "ON", "123456");
            if (sendDataServer(strSyn)){
                DBG(strSyn);
                DBG("Device SYN OK");
            } else {
                DBG("Device SYN FAIL");
            }
            */
        }
    }
}

void SyncRh2Esp(int gang) {
    String response = "";
    String idOfDevice_t = "";
    String secretKey_t = "";
    idOfDevice_t = device1.id;
    secretKey_t = device1.secretKey;
    if (client.connect(RashomeIP, TCP_PORT_CLIENT)) {
        String CHKSUM = String2Hash("SYN" + idOfDevice_t + "OBJECT");
        client.println("{\"req\":\"SYN\", \"id\":\"" + idOfDevice_t + "\", \"type\":\"OBJECT\", \"checksum\":\"" + CHKSUM + "\"}");
        DBG("{\"req\":\"SYN\", \"id\":\"" + idOfDevice_t + "\", \"type\":\"OBJECT\", \"checksum\":\"" + CHKSUM + "\"}");

        response = client.readStringUntil('\r');

        client.stop();
        DBG(response);
        if (response != "") {
            String id = splitStringJson("req", response);
            String type = splitStringJson("type", response);
            String Status = splitStringJson("status", response); 
            String CheckSum = splitStringJson("checksum", response);
            
            int RandomInt = CheckSum.substring(8, CheckSum.length()).toInt();
            CheckSum.remove(8, CheckSum.length());
            String SHA1 = String2Hash("SYN" + idOfDevice_t + type + Status + RandomInt + secretKey_t);
            if (type == DEVICE_TYPE){
            // if ((SHA1 == CheckSum) && (RandomInt > SyncNumber)) {
                SyncNumber = RandomInt;
                device1.status_val = Status;
                if (Status == "ON") {
                    String color = splitStringJson("color", response);
                    String brn = splitStringJson("brn", response);
                    brightness(color, brn);
                } else if (Status == "OFF") {
                    brightness("0", "0");
                }
            // } else {
                // DBG("Wrong security");
            // }
            }
        }
    }
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

void accessUpdate(String ver){
    if (flagUpdateFw){
        flagUpdateFw = false;
        if (WiFi.status() == WL_CONNECTED) {
            String path = HOST_UPDATE + "/" + ver + ".bin";
            DBG(path);
            t_httpUpdate_return ret = ESPhttpUpdate.update(path);
            //t_httpUpdate_return  ret = ESPhttpUpdate.update("https://server/file.bin", "", "fingerprint");

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
