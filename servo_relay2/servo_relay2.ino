#include <Arduino.h>
#include "PinChangeInterrupt.h"

#define RELAY_ON 1
#define RELAY_OFF 0

/* パルス幅とリレーオン・オフの関係 */
typedef struct {
  int pmin;
  int pmax;
  int digit;
} p2d_t;


p2d_t p2d[] =
{
  {0, 595, RELAY_OFF},
  {596, 698, RELAY_OFF},
  {699, 801, RELAY_OFF},
  {802, 904, RELAY_OFF},
  {905, 1008, RELAY_OFF},
  {1009, 1111, RELAY_OFF},
  {1112, 1214, RELAY_OFF},
  {1215, 1317, RELAY_OFF},
  {1318, 1420, RELAY_OFF},
  {1421, 1523, RELAY_OFF},
  {1524, 1626, RELAY_OFF},
  {1627, 1729, RELAY_OFF},
  {1730, 1832, RELAY_OFF},
  {1833, 1936, RELAY_OFF},    //0度～135度
  {1937, 2039, RELAY_ON},     //136度～180度
  {2040, 2142, RELAY_ON},
  {2143, 2245, RELAY_ON},
  {2246, 2348, RELAY_ON},
  {2349, 2400, RELAY_ON},
  {2401, 20000, RELAY_ON},  //180度以上もON扱いとする
};


typedef void(*VoidFuncPtr)(void);

class PulseMeasure {
  public:
    PulseMeasure(byte _meas_pin, VoidFuncPtr change_dispatch) {
      meas_pin = _meas_pin;
      pinMode(meas_pin, INPUT);

      level = LOW;
      level_old = LOW;
      start_time = 0;
      width = 0;

      attachPinChangeInterrupt(digitalPinToPCINT(meas_pin), change_dispatch, CHANGE);
    }

    unsigned long GetWidth() {
      return  width;
    }

    void change() {
      level = digitalRead(meas_pin);

      if (level_old != level) {
        if (level == HIGH) {
          start_time = micros();
        } else {
          width = micros() - start_time;
        }
      }
      level_old = level;
    }

  private:
    byte meas_pin;
    bool level;
    bool level_old;
    unsigned long  start_time;
    unsigned long  width;
};

class Pulse2Relay {
#define CHECK_NUM 5
  public:
    Pulse2Relay(byte _relay_pin, PulseMeasure &_pulse_meas) {
      relay_pin = _relay_pin;
      pinMode(relay_pin, OUTPUT);
      pulse_meas = &_pulse_meas;
      num_old = 0xff;
      initialized = false;
      for (int i = 0; i < CHECK_NUM; i++) {
        nums[i];
      }
      idx = 0;
    }

    void RelayOn () {
      int num = CheckNum(pulse_meas->GetWidth());
      if (num != 0xff && num_old != num) {
        if (num == RELAY_ON) {
          digitalWrite(relay_pin, LOW);
          Serial.print(relay_pin);
          Serial.println("pin RelayON");
        } else {
          digitalWrite(relay_pin, HIGH);
          Serial.print(relay_pin);
          Serial.println("pin RelayOFF");
        }
      }
      num_old = num;
    }

  private:
    byte relay_pin;
    int num_old;
    PulseMeasure *pulse_meas;
    bool initialized;
    int nums[CHECK_NUM];
    int idx;

    int  CheckNum(int pulse) {
      if (initialized == false) {
        for (int i = 0; i < CHECK_NUM; i++) {
          nums[i] = i;
        }
        initialized = true;
      }

      nums[idx] = Pulse2Digit(pulse);
      idx++;
      if (idx > CHECK_NUM - 1) idx = 0;

      int i;
      for (i = 1; i < CHECK_NUM; i++) {
        if (nums[i] != nums[0]) break;
      }
      if (i == CHECK_NUM) {
        return nums[0];
      } else {
        return 0xff;
      }
    }

    byte Pulse2Digit(int pulse)
    {
      byte digit = 0xff;
      for (int i = 0; i < sizeof(p2d) / sizeof(p2d[0]); i++) {
        if ( p2d[i].pmin <= pulse && pulse <= p2d[i].pmax) {
          digit = p2d[i].digit;
          break;
        }
      }
      return digit;
    }
};


void change0(void);
void change1(void);
void change2(void);
void change3(void);
void change4(void);
void change5(void);
void change6(void);
void change7(void);

PulseMeasure meas[] = {{4, change0}, {5, change1}, {6, change2}, {7, change3}, {8, change4}, {9, change5}, {10, change6}, {11, change7}};
Pulse2Relay relay[] = {{12, meas[0]}, {13, meas[1]}, {A5, meas[2]}, {A4, meas[3]}, {A3, meas[4]}, {A2, meas[5]}, {A1, meas[6]}, {A0, meas[7]}};

void change0(void)
{
  meas[0].change();
}

void change1(void)
{
  meas[1].change();
}

void change2(void)
{
  meas[2].change();
}

void change3(void)
{
  meas[3].change();
}

void change4(void)
{
  meas[4].change();
}

void change5(void)
{
  meas[5].change();
}

void change6(void)
{
  meas[6].change();
}

void change7(void)
{
  meas[7].change();
}
void setup() {
  Serial.begin (9600);
}

void loop() {

  for (int i = 0; i < sizeof(relay) / sizeof(relay[0]); i++) {
    relay[i].RelayOn();
    delay(10);
  }
}





