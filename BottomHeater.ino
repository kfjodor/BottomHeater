#include <HardwareTimer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>

#define MODE_TEMP 1
#define MODE_POWER 2

#define MAXCLK PA10
#define MAXCS PA11
#define MAXDO PA12

#define ENCODER_S1 PB15  //PB13
#define ENCODER_S2 PA8   //PB14
#define ENCODER_KEY PA9  //PB15

#define TRIAC_ZERO_CROSS PB10
#define TRIAC_PWM PB11

#define HEAT_EFF_MIN 0.2  // Required head efficiency 0.2'C per sec
#define HEAT_EFF_MAX 0.4  // Required head efficiency 0.4'C per sec

MAX6675 thermocouple(MAXCLK, MAXCS, MAXDO);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
HardwareTimer outTimer(TIM3);

int mode = MODE_TEMP;
float temp = 0;
float prevTemp = 0;
float prevTemp10sec = 0;
unsigned long prevMillis = 0;

int count = 0;
float heat_eff = 0;
int desiredTemp = 0;
bool tempChanged = false;
int power = 0;  // 0 - 100
bool powerChanged = false;
int powerDelay = 0;  // 1-9999 for 50Hz

void setup() {
  lcd.init();
  lcd.init();
  lcd.backlight();

  desiredTemp = thermocouple.readCelsius();

  pinMode(ENCODER_S1, INPUT_PULLUP);
  pinMode(ENCODER_S2, INPUT_PULLUP);
  pinMode(ENCODER_KEY, INPUT_PULLUP);
  pinMode(TRIAC_ZERO_CROSS, INPUT);
  pinMode(TRIAC_PWM, OUTPUT);

  // Button pressed on start
  if (digitalRead(ENCODER_KEY) == LOW) {
    mode = MODE_POWER;
  }

  attachInterrupt(digitalPinToInterrupt(TRIAC_ZERO_CROSS), ZeroCross, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_S1), EncoderRotate, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_KEY), EncoderKey, FALLING);

  printPower(power, heat_eff);
}

void printTemp(float temp1, int temp2) {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  if (mode == MODE_TEMP) {
    lcd.print("*");
  }
  lcd.print(temp1);

  if (int(temp1) == temp2) {
    lcd.print(" == ");
  } else {
    lcd.print(" -> ");
  }

  lcd.print(temp2);
}

void printPower(int power, float heat_eff) {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  if (mode == MODE_POWER) {
    lcd.print("*");
  }
  lcd.print("P:" + String(power) + "% ");
  lcd.print("E:" + String(heat_eff, 1));
}

void timerTriac() {
  digitalWrite(TRIAC_PWM, HIGH);
}

void ZeroCross() {
  digitalWrite(TRIAC_PWM, LOW);

  outTimer.pause();
  outTimer.attachInterrupt(timerTriac);
  outTimer.setOverflow(powerDelay, MICROSEC_FORMAT);
  outTimer.refresh();
  outTimer.resume();
}

void EncoderRotate() {
  if (mode == MODE_TEMP) {
    EncoderRotateTemp();
  } else {
    EncoderRotatePower();
  }
}

void EncoderKey() {
  if (mode == MODE_TEMP) {
    EncoderKeyTemp();
  } else {
    EncoderKeyPower();
  }
}

void EncoderRotateTemp() {
  if (digitalRead(ENCODER_S2) == LOW) {
    desiredTemp++;
  } else {
    desiredTemp--;
  }

  if (desiredTemp < 0) {
    desiredTemp = 0;
  }

  tempChanged = true;
}

void EncoderKeyTemp() {
  desiredTemp += 10;
  tempChanged = true;
}

void EncoderRotatePower() {
  if (digitalRead(ENCODER_S2) == LOW) {
    power++;
  } else {
    power--;
  }

  // noInterrupts failed on stm32 on main loop so checking here
  if (power > 100) {
    power = 100;
  } else if (power < 0) {
    power = 0;
  } else {
    powerChanged = true;
  }
}

void EncoderKeyPower() {
  power += 10;

  // noInterrupts failed on stm32 on main loop so checking here
  if (power > 100) {
    power = 100;
  } else if (power < 0) {
    power = 0;
  }

  powerChanged = true;
}

void setPowerDelay(int power) {
  switch (power) {
    case 0:
      powerDelay = 10000;
      break;
    case 100:
      powerDelay = 0;
      break;
    default:
      powerDelay = (100 - power) * 100;
  }
}

void loopPower() {
  unsigned long nowMillis = millis();

  if (prevMillis + 1000 < nowMillis) {
    temp = thermocouple.readCelsius();
    if (temp != prevTemp) {
      tempChanged = true;
      prevTemp = temp;
      printTemp(temp, 0);
    }

    prevMillis = nowMillis;
  }

  if (powerChanged) {
    setPowerDelay(power);
    printPower(power, 0);
    powerChanged = false;
  }
}

void loopTemp() {
  unsigned long nowMillis = millis();

  // каждые 1 сек обновляем температуру
  if (prevMillis + 1000 < nowMillis) {
    temp = thermocouple.readCelsius();
    if (temp != prevTemp) {
      tempChanged = true;
      prevTemp = temp;
    }

    prevMillis = nowMillis;
    count++;

    // 10 секундный счетчик
    if (count > 10) {
      heat_eff = temp - prevTemp10sec;  // скорость нагрева в 10 секунд
      prevTemp10sec = temp;

      // нагрев
      if (desiredTemp > temp) {
        // Скорость нагрева меньше 0.2'C - увеличиваем
        if (heat_eff < HEAT_EFF_MIN) {
          power++;
        }
        // Скорость нагрева больше 0.4'C - уменьшаем
        if (heat_eff > HEAT_EFF_MAX) {
          power--;
        }
      } else if (desiredTemp < temp) {       // охлаждение
        if (heat_eff > -1 * HEAT_EFF_MIN) {  // Скорость охлаждения меньше 0.2
          power--;
        }

        if (heat_eff < -1 * HEAT_EFF_MAX) {  // Скорость охлаждения больше 0.4
          power++;
        }
      } else {  // Required temp acquired
        if (heat_eff > 0.0) {
          power--;
        } else if (heat_eff < 0.0) {
          power++;
        }
      }

      if (power > 100) {
        power = 100;
      } else if (power < 0) {
        power = 0;
      }

      setPowerDelay(power);
      printPower(power, heat_eff);
      count = 0;
    }  // 10 sec loop
  }    // 1 sec loop

  // покрутили ручку или произошел нагрев
  if (tempChanged) {
    printTemp(temp, desiredTemp);
    tempChanged = false;
  }
}

void loop() {
  if (mode == MODE_TEMP) {
    loopTemp();
  } else if (mode = MODE_POWER) {
    loopPower();
  }
}
