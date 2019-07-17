/*
  Скетч к проекту "Простой руль для ПК"
  Страница проекта (схемы, описания): https://alexgyver.ru/GyverWheel/
  Исходники на GitHub: https://github.com/AlexGyver/GyverWheel/
  Нравится, как написан код? Поддержи автора! https://alexgyver.ru/support_alex/
  Автор: AlexGyver, AlexGyver Technologies, 2019
  https://AlexGyver.ru/
*/

// стандартный тестер геймпада в Win10: поиск/joy.cpl

// =========== НАСТРОЙКИ ===========
#define DEBUG 0       // режим отладки
#define ENC_TYPE 1    // тип энкодера, 0 или 1
#define INV_WHEEL 1   // инверсия руля

// пины
#define ENC_A 5       // пин энкодера
#define ENC_B 6       // пин энкодера
#define POT_THR A0    // педаль газа
#define POT_BR A1     // педаль тормоза
#define BUTT_BR 4     // кнопка ручника
#define BUTT_CAL 3    // кнопка калибровки

volatile int encCounter = 0;
volatile boolean state0, lastState, turnFlag;
int throttleMin, throttleMax, brakeMin, brakeMax, wheelMax;
uint32_t timer;

#include <EEPROM.h>
#include "HID-Project.h"

void setup() {
  pinMode(BUTT_BR, INPUT_PULLUP);
  pinMode(BUTT_CAL, INPUT_PULLUP);
  setupTmr();
  debug();
  calibration();
  EEPROM.get(0, throttleMin);
  EEPROM.get(2, brakeMin);
  EEPROM.get(4, throttleMax);
  EEPROM.get(6, brakeMax);
  EEPROM.get(8, wheelMax);
  Gamepad.begin();
  //Serial.begin(9600);
}

void loop() {
  gamepadTick();

  // при нажатии кнопки калибровки скидываем позицию руля в 0
  if (!digitalRead(BUTT_CAL)) {
    encCounter = 0;
  }
}

void encTick() {
  state0 = digitalRead(ENC_A);
  if (state0 != lastState) {
#if (ENC_TYPE == 1)
    turnFlag = !turnFlag;
    if (turnFlag)
      encCounter += (digitalRead(ENC_B) != lastState) ? -1 : 1;
#else
    encCounter += (digitalRead(ENC_B) != lastState) ? -1 : 1;
#endif
    lastState = state0;
  }
}

void gamepadTick() {
  if (millis() - timer > 10) {
    timer = millis();
    int wheel;
    if (INV_WHEEL) wheel = constrain(-encCounter, -wheelMax, wheelMax);
    else wheel = constrain(encCounter, -wheelMax, wheelMax);

    wheel = map(wheel, -wheelMax, wheelMax, -32768, 32767);
    Gamepad.xAxis(wheel);

    int thr, br;
    thr = map(analogRead(POT_THR), throttleMin, throttleMax, -128, 127);
    thr = constrain(thr, -128, 127);
    Gamepad.zAxis(thr);

    br = map(analogRead(POT_BR), brakeMin, brakeMax, -128, 127);
    br = constrain(br, -128, 127);
    Gamepad.rzAxis(br);

    if (!digitalRead(BUTT_BR)) Gamepad.press(1);
    else Gamepad.release(1);
    Gamepad.write();
  }
}

// калибровка
void calibration() {
  if (!digitalRead(BUTT_CAL)) {     // нажата кнопка
    while (!digitalRead(BUTT_CAL)); // пока кнопка удерживается
    Serial.begin(9600);
    delay(100);
    Serial.print(F("Calibration start"));
    encCounter = 0;
    int zeroTHR = analogRead(POT_THR);
    int zeroBR = analogRead(POT_BR);
    int maxTHR, maxBR, maxWHEEL;

    EEPROM.put(0, zeroTHR);
    EEPROM.put(2, zeroBR);
    delay(100);                     // дебаунс
    while (true) {                  // крутимся
      if (!digitalRead(BUTT_CAL)) break;
      maxTHR = analogRead(POT_THR);
      maxBR = analogRead(POT_BR);
      Serial.println(encCounter);
    }
    EEPROM.put(4, maxTHR);
    EEPROM.put(6, maxBR);
    EEPROM.put(8, abs(encCounter));

    Serial.println(F("Calibration end"));
    Serial.print(F("Wheel: "));
    Serial.println(abs(encCounter));
    Serial.print(F("Throat: "));
    Serial.print(zeroTHR);
    Serial.print(" - ");
    Serial.println(maxTHR);
    Serial.print(F("Brake: "));
    Serial.print(zeroBR);
    Serial.print(" - ");
    Serial.println(maxBR);
    Serial.println();
  }
  Serial.end();
  delay(3000);  // задержка чтобы кнопку отпустить
}

// дебаг
void debug() {
#if (DEBUG == 1)
  Serial.begin(9600);
  uint32_t timer;
  while (true) {
    encTick();
    if (millis() - timer > 100) {
      timer = millis();
      Serial.print(encCounter);
      Serial.print("\t");
      Serial.print(analogRead(POT_THR));
      Serial.print("\t");
      Serial.print(analogRead(POT_BR));
      Serial.print("\t");
      Serial.print(!digitalRead(BUTT_BR));
      Serial.print("\t");
      Serial.println(!digitalRead(BUTT_CAL));
    }
  }
  Serial.end();
#endif
}

// опрос энкодера в прерывании
ISR(TIMER3_COMPA_vect) {
  encTick();
}

// ставим таймер 3, канал А, период 0.5 мс. Для опроса энкодера
void setupTmr() {
  TCCR3B = 0b00001001;
  TIMSK3 = 0b00000010;
  OCR3AH = highByte(15999 / 2);
  OCR3AL = lowByte(15999 / 2);
}
