#ifndef Timer_h
#define Timer_h
#include "Arduino.h"

#define sysclk_500Hz 2000
#define sysclk_200Hz 5000
#define sysclk_100Hz 10000
#define sysclk_80Hz  12500
#define sysclk_50Hz  20000
#define sysclk_40Hz  25000
#define sysclk_25Hz  40000
#define sysclk_20Hz  50000
#define sysclk_10Hz  100000
#define sysclk_1Hz   1000000
#define SYS_1HZ 'j'
#define SYS_10HZ 'i'
#define SYS_20HZ 'a'
#define SYS_25HZ 'b'
#define SYS_40HZ 'c'
#define SYS_50HZ 'd'
#define SYS_80HZ 'e'
#define SYS_100HZ 'f'
#define SYS_200HZ 'g'
#define SYS_500HZ 'h'
//Cach khai bao: Timer class(SYS_100HZ);
class Timer {
  private:
    unsigned long duration;
    unsigned long currTimer;
    unsigned long lastTimer;
  public:
    Timer(char index) {
      this->duration=0;
      this->currTimer=0;
      this->lastTimer=0;
      switch (index) {
  case SYS_1HZ:
    this->duration= sysclk_1Hz;
    break;
        case SYS_10HZ:
          this->duration = sysclk_10Hz;
          break;
        case SYS_25HZ:
          this->duration = sysclk_25Hz;
          break;
        case SYS_50HZ:
          this->duration = sysclk_50Hz;
          break;
        case SYS_100HZ:
          this->duration = sysclk_100Hz;
          break;
        case SYS_200HZ:
          this->duration = sysclk_200Hz;
          break;
        case SYS_500HZ:
          this->duration = sysclk_500Hz;
          break;
      }
    }
    int getDuration() {
      return this->duration;
    }
    bool task() {
      this->currTimer = micros();
      if (this->currTimer - this->lastTimer >= this->duration) {
             //   Serial.println(this->currTimer-this->lastTimer);
        this->lastTimer = this->currTimer;
        return 1;
      }
      else return 0;
      
    }
};
#endif
