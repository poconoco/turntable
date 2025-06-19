#include <Arduino.h>

#include <EEPROM.h>
#include <AccelStepper.h>
#include <Encoder.h>

const int dirPin = 2;
const int stepPin = 3;
const int encoderPinA = 5;
const int encoderPinB = 6;
const int buttonPin = 4;

const int eepromSpeedAddress = 0;
const int speedSaveTimeout = 2000;
const int speedStep = 10;
const int maxSpeed = 1000;

void setup() {
  int16_t currentSpeed = 0;
  int16_t lastSpeed = 0; 
  int16_t savedSpeed = 0;
  bool buttonDown = false;
  bool lastButtonDown = false;
  bool motorRunning = false;
  long lastSpeedChange = 0;

  AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
  Encoder encoder(encoderPinA, encoderPinB);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  EEPROM.get(eepromSpeedAddress, savedSpeed);
  // If saved speed is out of bounds, reset to a reasonable value
  if (abs(savedSpeed) > maxSpeed) savedSpeed = 500;
  lastSpeedChange = millis();

  currentSpeed = savedSpeed;

  stepper.setMaxSpeed(maxSpeed);
  stepper.setSpeed(0);

  while(true) {
    long now = millis();

    if (motorRunning) {
      currentSpeed = encoder.read() * speedStep; // Scale encoder position to speed step
      currentSpeed = constrain(currentSpeed, -maxSpeed, maxSpeed);
      encoder.write(currentSpeed / speedStep); // Update encoder position with constrained value
    }
    
    // Read button state
    buttonDown = digitalRead(buttonPin) == LOW;
    if (! buttonDown && lastButtonDown) {
      // Toggle motor state on button up event
      motorRunning = !motorRunning; 
      if (motorRunning) {
        stepper.setSpeed(currentSpeed);
      } else {
        stepper.setSpeed(0);
      }

      lastSpeedChange = now;
    } else if (currentSpeed != lastSpeed && motorRunning) {
      stepper.setSpeed(currentSpeed);
      lastSpeed = currentSpeed;

      lastSpeedChange = now;
    } 

    // Save speed to EEPROM after 2 seconds of inactivity
    if (now - lastSpeedChange > speedSaveTimeout) {
      if (currentSpeed != savedSpeed) {
        EEPROM.put(eepromSpeedAddress, currentSpeed);
        savedSpeed = currentSpeed;
      }
    } 

    lastButtonDown = buttonDown;
    stepper.runSpeed();

    delay(1);
  }
}
