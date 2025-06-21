#include <Arduino.h>

#include <EEPROM.h>
#include <AccelStepper.h>
#include <Encoder.h>

//#define DEBUG

const int dirPin = 2;
const int stepPin = 3;
const int encoderPinA = 5;
const int encoderPinB = 6;
const int buttonPin = 4;

const int eepromSpeedAddress = 0;
const int speedSaveTimeout = 2000;
const int buttonDebounceTimeout = 250;

const int encoderMax = 100;  // Number of encoder steps after which we ignore futher increase, 
                             // goes both sides of 0, e.g. 0encoderMax to encoderMax

const int stepperMaxSpeed = 2000;  // Max stepper speed in its steps per seconds

int getProgressiveSpeed(int speed) {
  const int sign = (speed < 0) ? -1 : 1;
  return sign * (speed * speed) / ((encoderMax * encoderMax) / stepperMaxSpeed);
}

void setup() {
  int16_t encoderInput = 0;
  int16_t lastEncoderInput = 0;
  int16_t savedEncoderInput = 0;
  bool buttonDown = false;
  bool lastButtonDown = false;
  bool stepperRunning = true;
  long lastSpeedChange = 0;
  long lastButtonChange = 0;
  int stepperSpeed;

  // Initialize Serial for debugging
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
  Encoder encoder(encoderPinA, encoderPinB);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  EEPROM.get(eepromSpeedAddress, savedEncoderInput);

  // If saved speed is out of bounds, reset to a reasonable value
  if (abs(savedEncoderInput) > encoderMax)
    encoderInput = encoderMax / 20;
  else
    encoderInput = savedEncoderInput;

  stepper.setMaxSpeed(stepperMaxSpeed);
  encoder.write(encoderInput);

  while(true) {
  long now = millis();

    if (stepperRunning) {
      encoderInput = constrain(encoder.read(), -encoderMax, encoderMax);
      stepperSpeed = getProgressiveSpeed(encoderInput);
      encoder.write(encoderInput); // Update encoder position with constrained value
    }
    
    // Read button state
    buttonDown = digitalRead(buttonPin) == LOW;
    if (! buttonDown && lastButtonDown) {
      // Ignore quick toggles
      if (now - lastButtonChange >= buttonDebounceTimeout) {
        // Toggle motor state on button up event
        stepperRunning = !stepperRunning; 
        if (stepperRunning) {
          #ifdef DEBUG
            Serial.println("Motor on");
          #endif
          stepper.setSpeed(stepperSpeed);
        } else {
          #ifdef DEBUG
            Serial.println("Motor off");
          #endif
          stepper.setSpeed(0);
        }

        lastSpeedChange = now;
        lastButtonChange = now;
      }
    } else if (encoderInput != lastEncoderInput && stepperRunning) {
      stepper.setSpeed(stepperSpeed);
      lastEncoderInput = encoderInput;

      lastSpeedChange = now;
      #ifdef DEBUG
        Serial.print("Speed: ");
        Serial.println(progressiveSpeed);
      #endif
    }

    // Save speed to EEPROM after 2 seconds of inactivity
    if (now - lastSpeedChange > speedSaveTimeout) {
      if (encoderInput != savedEncoderInput) {
        EEPROM.put(eepromSpeedAddress, encoderInput);
        savedEncoderInput = encoderInput;
      }
    } 

    lastButtonDown = buttonDown;
    stepper.runSpeed();

    delay(0.25);
  }
}
