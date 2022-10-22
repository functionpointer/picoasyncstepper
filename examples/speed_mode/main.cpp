#include <Arduino.h>
#include "picoasyncstepper.h"

PicoAsyncStepper stepper;

#define PIN_STEPPER_EN 0

#define PIN_STEPPER_STEP 3
#define PIN_STEPPER_DIR 2


void setup() {
  Serial.begin(115200);
  stepper.setEnablePin(PIN_STEPPER_EN);
  stepper.begin(PIN_STEPPER_STEP, PIN_STEPPER_DIR);
  stepper.setAcceleration(100);
  stepper.enableOutputs();
  Serial.println("setup done");
  /*pinMode(PIN_STEPPER_EN, OUTPUT);
  pinMode(PIN_STEPPER_DIR, OUTPUT);
  pinMode(PIN_STEPPER_DIR, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, LOW);
  digitalWrite(PIN_STEPPER_DIR, LOW);
  digitalWrite(PIN_STEPPER_STEP, LOW);*/
}


void loop() {
  static unsigned long lastprint = 0;
  if(millis()>lastprint+1000) {
    Serial.println(stepper.speed());
    lastprint = millis();
  }
  if((millis()/5000)%2==0) {
    stepper.setSpeed(200);
  } else {
    stepper.setSpeed(-150);
  }
  stepper.run();
  
}