/*use for file 123SWSONOFFV02
 */
#ifndef CONFIGURE_H
#define CONFIGURE_H

#define VERSION     "V029"

#define SW1G //SW1G, SW2G, SW3G
uint8_t logFlag = 1;
uint8_t logFlagDebug = 1;
//uint8_t logFlagInfo = 1;
//uint8_t logFlagError = 1;
//#define logdebug(x) do { if(logFlag) Serial.printf("%s():\t%d: ", __func__, __LINE__);printf (x);} while (0)
//#define logDebug(fmt, ...) do { if(logFlag) Serial.printf("[DEBUG] %s(): \t%d: \t" fmt, __func__, __LINE__, ##__VA_ARGS__);} while (0)
//#define logInfo(fmt, ...) do { if(logFlagInfo) printf("[INFO] %s(): \t%d: \t" fmt, __func__, __LINE__, ##__VA_ARGS__);} while (0)
//#define logError(fmt, ...) do { if(logFlagError) printf("[ERROR] %s(): \t%d: \t" fmt, __func__, __LINE__, ##__VA_ARGS__);} while (0)
#define logDebug(fmt, ...) do {if (logFlagDebug) printf("[DEBUG] %s(): \t%d: \t" fmt, __func__, __LINE__, ##__VA_ARGS__ );} while (0)
//#define DEBUG
boolean flagDebug = true;
#define SIZE_OF_LOG     256
//#ifdef DEBUG
#define DBG(x)    if (flagDebug) {Serial.print(__LINE__); Serial.print(": "); Serial.println(x);}//Serial.println(__LINE__)
#define DBGT(y) if (flagDebug) {Serial.print(y);}
//#define mylog(fmt, ...)   mylogFc(fmt, ##__VA_ARGS__)

//#else
//#define DBG(x)
//#endif
/*Address for save data to eeprom*/
#define EEPROM_SIZE       1024
#define ADD_DBG           1

#define ADD_ID1_F         20
#define ADD_ID1_E         22
#define ADD_KEY1_F        23
#define ADD_KEY1_E        26
#define ADD_STARTUP1      27  //[0=ON, 1=OFF, 2=SYN]
#define ADD_STATE1        28  //[UNINSTALL=0, PAIRING=2, WORKING=3]

#define ADD_VALUE_WARM    29
#define ADD_VALUE_COLD    30

#define ADD_ID2_F         40
#define ADD_ID2_E         42
#define ADD_KEY2_F        43
#define ADD_KEY2_E        46
#define ADD_STARTUP2      47
#define ADD_STATE2        48 

#define ADD_ID3_F         60
#define ADD_ID3_E         62
#define ADD_KEY3_F        63
#define ADD_KEY3_E        66
#define ADD_STARTUP3      67
#define ADD_STATE3        68 

#define ADD_RSH_IP_F      80
#define ADD_RSH_IP_E      83
#define ADD_MODE          84  //0=AP, 1=STA
#define ADD_STATUS        85  //0=DHCP, 1=static
#define ADD_IP_STA_F      86
#define ADD_IP_STA_E      89
#define ADD_GETWAY_F      90
#define ADD_GETWAY_E      93
#define ADD_SUBNETMARK_F  94
#define ADD_SUBNETMARK_E  97

#define ADD_LEN_SSID      98
#define ADD_SSID_F        99    //sta ssid
#define ADD_SSID_E        129   
#define ADD_LEN_PW        130
#define ADD_PW_F          131   //sta pw
#define ADD_PW_E          159

#define ADD_ALARM_0       200 // 0001 alarm 1 is set, 0000 no alarm set
#define ADD_ALARM_1       201  //num of current id alarm is set
#define ADD_ALARM_2       202
#define ADD_ALARM_3       203
#define ADD_ALARM_4       204

#define ADD_ALARM_TIME    210  //time for on motor
#define ADD_ALARM_MINUTE  211 
#define ADD_ALARM_HOUR    212
#define ADD_ALARM_DAY     213
#define ADD_ALARM_REPEAT  214  // 1 is repeat every day

#define ALARM_MAX         4
#define ADD_ALARM_STEP        5  //max address for alarm is 235

#define AP_SSID   "MyHomeIoT"    //default SSID when is AP mode
#define AP_PW     "12345678"      //default password for AP mode
#define UDP_PORT  9091            //udp port for run protocol udp
#define TCP_PORT_SERVER  8091     //tcp port device listen when make device is server
#define TCP_PORT_CLIENT  8092     //tcp port device send data to server (RSH)
#define TIME_WTD  2000
#define TIME_WAIT_SYS_SYNC      65000
#define TIME_TO_CHECK_CONNECT   60000          //after this time system will be ping to RSHome
#define TIME_TO_RECONNECT_WIFI  10000            //time wait for connect to modem after command connect is true
#define TIME_TO_CHECK_CONNECT_WIFI  1000        //after this time system auto check connect to modem wifi
#define TIME_TO_LEARN_WF  3000                  //wait for button hold-pressed to change to mode learn wifi 
//#define TIME_TO_UNWORKING 4500
#define TIME_TO_RST_FAC   8500                  //wait for button hold-pressed to change to mode reset factory 
#define TIME_ENABLE_RST_FAC 3000                //time wait for gang final submit to enable resetfactory
//#define TIME_TO_ACTIVE_PAIRING  4500            //wait for button hold-pressed to change to mode pairing
#define TIME_WAIT_LEARN_WF  300000              //mode learn wifi of device possible
//#define TIME_WAIT_ACTIVE_PAIRING  300000         //mode pairing of device possible
#define TIME_RSTART_SER_MODE_PAIRING   5000     //tiem ti restart server to pairing
#define TIME_WAIT_RST_FAC   200000               //time wait for submit to reset factory
#define TIME_DELAY_LED_SIGNAL_ON  50               //time led signal on in a duty
#define TIME_DELAY_LED_SIGNAL_OFF 280 //time delay led signal off in a duty
#define TIME_DELAY_LED_SOFF   1000              //time delay led off when blink end of duty
#define TIME_DELAY_LED_ERROR  60  //time delay led when blink reset factory mode
#define BOUNDTIME_BUTTON        200

#define SUNDAY        1
#define MONDAY        2
#define TUEDAY        3
#define WEDDAY        4
#define THUDAY        5
#define FRIDAY        6
#define SATDAY        7

#define Relay 12 // tai 1
#define Button 0
#define Relay2 5  // TAI 3
#define Button2 9
#define Relay3 4  // TAI2
#define Button3 10

#define Led 13   // LED SIGNAL
#define PRESS   false
#define RELEASE true
#define RELAY_ST_ON   HIGH
#define RELAY_ST_OFF  LOW
#define LED_ST_ON   HIGH
#define LED_ST_OFF  LOW

//typedef enum {UNINSTALL=0,NOIP, PARING, INACTIVE, ACTIVE, W_INSTATLL, ERROR }state_t;
typedef enum {BTN_PRESSED, BTN_RELEASED}  btnState_t;    // trang thai nut nhan
typedef enum {UNINSTALL, LEARN_WF, RST_FAC, INACTIVE_PAIRING, 
              INACTIVE_PAIRING_W,  ACTIVE_PAIRING, ACTIVE_PAIRING_W, WORKING
              } btn_working_state_t;
typedef enum {RELAY, RELAYON, RELAYOFF} relay_state_t;
typedef enum {NONCMD, UPDATEFW, CONTROL} cmd_t;
typedef struct buttonAccess { boolean led; 
                              volatile boolean pressed ; 
                              volatile unsigned long timestart; 
                              unsigned long timestop; 
                              uint8_t counter; 
                              volatile boolean change;
                              };
typedef struct device_struct { uint8_t state = UNINSTALL; \
                              char *startup=""; \
                              String secretKey=""; \
                              String id = ""; \
                              uint8_t preState = UNINSTALL; \
                              uint8_t warm_val; \
                              uint8_t cold_val;\
                              uint8_t relay;\
                              uint8_t button;\
                              };

btn_working_state_t state1 = UNINSTALL;
btn_working_state_t state2 = UNINSTALL;
btn_working_state_t state3 = UNINSTALL;
btn_working_state_t preState1 = UNINSTALL;
btn_working_state_t preState2 = UNINSTALL;
btn_working_state_t preState3 = UNINSTALL;

//relay_state_t relay1_state = RELAYOFF;
//relay_state_t relay2_state = RELAYOFF;
//relay_state_t relay3_state = RELAYOFF;
device_struct device1;

device_struct device2;

device_struct device3;

buttonAccess bt1;
buttonAccess bt2;
buttonAccess bt3;

#endif
