#include <Wire.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <string.h>
#include <Arduino.h>
#include "hardware/pwm.h"

// Pin definitions
const int ledPin = LED_BUILTIN;
const int buttonPin = 16; // Adjust based on your wiring
const int encoderPinA = 18; // Adjust based on your wiring
const int encoderPinB = 22; // Adjust based on your wiring

// Camera connections
const int siocPin = 21; // I2C SCL
const int siodPin = 20; // I2C SDA
const int vsyncPin = 27;
const int hrefPin = 15;
const int pclkPin = 14;
const int xclkPin = 28;
const int d2Pin = 6;
const int d3Pin = 7;
const int d4Pin = 8;
const int d5Pin = 9;
const int d6Pin = 10;
const int d7Pin = 11;
const int d8Pin = 12;
const int d9Pin = 13;
const int resetPin = 26;
const int pwonPin = 19;

// Variables
volatile bool snappingPhoto = false;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastMSB = 0;
volatile int lastLSB = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
bool cameraInitialized = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH); // turn on pullup resistor
  digitalWrite(encoderPinB, HIGH); // turn on pullup resistor

  pinMode(vsyncPin, INPUT);
  pinMode(hrefPin, INPUT);
  pinMode(pclkPin, INPUT);
  pinMode(xclkPin, OUTPUT);
  pinMode(d2Pin, INPUT);
  pinMode(d3Pin, INPUT);
  pinMode(d4Pin, INPUT);
  pinMode(d5Pin, INPUT);
  pinMode(d6Pin, INPUT);
  pinMode(d7Pin, INPUT);
  pinMode(d8Pin, INPUT);
  pinMode(d9Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  Serial.begin(115200);

  // Initialize I2C
  Wire.setSCL(siocPin);
  Wire.setSDA(siodPin);
  Wire.begin();

  // Power on the camera
  pinMode(pwonPin, OUTPUT);
  digitalWrite(pwonPin, HIGH);

  // Start the XCLK
  startXCLK();

  // Initialize camera
  cameraInitialized = initCamera();
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x30); // OV2640 I2C address
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    Serial.print("Failed to write register 0x");
    Serial.print(reg, HEX);
    Serial.print(" with value 0x");
    Serial.println(value, HEX);
    return false;
  }
  return true;
}

bool initCamera() {
  Serial.println("Initializing camera...");

  // Reset the camera
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  delay(100);

  // Write basic initialization registers
  // OV2640 initialization sequence (simplified)
  if (!writeRegister(0xFF, 0x01)) return false; // Select register bank
  delay(10);
  if (!writeRegister(0x12, 0x80)) return false; // Reset all registers
  delay(100);

  // Additional configuration (adjust as needed for your use case)
  if (!writeRegister(0xFF, 0x00)) return false; // Select register bank
  if (!writeRegister(0x2C, 0xFF)) return false;
  if (!writeRegister(0x2E, 0xDF)) return false;
  if (!writeRegister(0xFF, 0x01)) return false;
  if (!writeRegister(0x3C, 0x32)) return false;

  Serial.println("Camera initialized successfully");
  return true;
}

void startXCLK() {
  // Initialize the PWM for the XCLK
  gpio_set_function(xclkPin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(xclkPin);

  // Set the frequency and duty cycle
  pwm_set_wrap(slice_num, 3); // 4 counts (to generate a 10 MHz signal with 125 MHz system clock)
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 2); // 50% duty cycle
  pwm_set_clkdiv(slice_num, 1.25); // Adjust the clock divider as needed

  // Enable the PWM output
  pwm_set_enabled(slice_num, true);

  Serial.println("XCLK started");
}

void snapPhoto() {
  if (cameraInitialized) {
    Serial.println("Snap Photo: Camera is working");
    captureFrame();
  } else {
    Serial.println("Snap Photo: Camera initialization failed");
  }
}

void captureFrame() {
  Serial.println("Capturing frame...");
  String frameData = "";  // Initialize an empty string to hold the frame data

  // Wait for VSYNC (frame start)
  unsigned long start = millis();
  while (digitalRead(vsyncPin) == LOW) {
    if (millis() - start > 1000) { // 1-second timeout
      Serial.println("Timeout waiting for VSYNC LOW to HIGH");
      return;
    }
  }
  Serial.println("VSYNC LOW detected");

  start = millis();
  while (digitalRead(vsyncPin) == HIGH) {
    if (millis() - start > 1000) { // 1-second timeout
      Serial.println("Timeout waiting for VSYNC HIGH to LOW");
      return;
    }
  }
  Serial.println("VSYNC HIGH detected");

  // Capture one frame
  for (int row = 0; row < 240; row++) { // Assuming 240 rows
    // Wait for HREF (line start)
    while (digitalRead(hrefPin) == LOW);
    for (int col = 0; col < 320; col++) { // Assuming 320 columns
      // Wait for PCLK (pixel clock)
      while (digitalRead(pclkPin) == LOW);
      byte pixelData = readPixelData();
      frameData += String(pixelData, HEX) + " ";
      while (digitalRead(pclkPin) == HIGH);
    }
    frameData += "\n";
  }

  // Print the entire frame data at once
  Serial.println(frameData);
  Serial.println("Frame capture complete");
}


byte readPixelData() {
  byte data = 0;
  data |= (digitalRead(d2Pin) << 0);
  data |= (digitalRead(d3Pin) << 1);
  data |= (digitalRead(d4Pin) << 2);
  data |= (digitalRead(d5Pin) << 3);
  data |= (digitalRead(d6Pin) << 4);
  data |= (digitalRead(d7Pin) << 5);
  data |= (digitalRead(d8Pin) << 6);
  data |= (digitalRead(d9Pin) << 7);
  return data;
}

void savePhoto() {
  Serial.println("Save Photo to SD Card");
  // Add actual photo saving code here
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    digitalWrite(ledPin, HIGH);

    if (!snappingPhoto) {
      snappingPhoto = true;
      snapPhoto();
      savePhoto();
    }
  } else {
    snappingPhoto = false;
    digitalWrite(ledPin, LOW);
  }

  delay(50);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin values to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    Serial.println("Rotating Right");
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    Serial.println("Rotating Left");
  }

  lastEncoded = encoded; // store this value for next time
}