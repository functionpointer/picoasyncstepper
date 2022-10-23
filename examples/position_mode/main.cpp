#include <Arduino.h>
#include "picoasyncstepper.h"

PicoAsyncStepper stepper;

#define PIN_STEPPER_EN 0

#define PIN_STEPPER_STEP 3
#define PIN_STEPPER_DIR 2

#define PIN_ENDSTOP 15

void setup() {
  stepper.setEnablePin(PIN_STEPPER_EN);
  stepper.setPinsInverted(true, false, false);
  stepper.begin(PIN_STEPPER_STEP, PIN_STEPPER_DIR);
  stepper.setAcceleration(2000);
  stepper.enableOutputs();
  stepper.setMaxSpeed(700);
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);
  //delay(2000);
}

enum STATUS {
  HOMING, WAITING, OPERATING
};
STATUS status = HOMING;

void loop() {
  static long operating_lastswap = 0;
  switch(status) {
    case HOMING:
      stepper.setSpeed(-100);
      if(digitalRead(PIN_ENDSTOP)==HIGH) {
        status = WAITING;
        stepper.estop();
        stepper.setPosition(0);
      }
    break;
    case WAITING:
      delay(500);
      status=OPERATING;
      Serial.println("moveto 150");
      stepper.moveTo(150);
      operating_lastswap=millis();
      break;
    case OPERATING:
      static bool current = true;
      if(millis()>operating_lastswap+2000) {
        operating_lastswap=millis();
        current^=true;
        Serial.print("swap to ");
        Serial.println(current);
      }
      long tgt = current?600:0;
      stepper.moveTo(tgt);
    break;
  }
  stepper.run();
  static unsigned long lastprint = 0;
  if(millis()>lastprint+250) {
    Serial.print(stepper.speed());
    Serial.print(" status=");
    Serial.print(status);
    Serial.print("posn=");
    Serial.println(stepper.currentPosition());
    lastprint = millis();
  }
}