#include "./config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <AceButton.h>
#include <Adafruit_NeoPixel.h>
#include "esp_now_midi.h"

using namespace ace_button;
Adafruit_NeoPixel _leds(NUMBER_OF_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


// Timing variables for MIDI message sending
unsigned long lastSendTime = 0;


esp_now_midi ESP_NOW_MIDI;
void customOnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Serial.print("Custom Callback - Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

Adafruit_MPU6050 mpu;

// Tracking variables
float yaw = 0;            // Current angle (0-359°)
float rotationSpeed = 0;  // Current speed (°/sec)
float gyroZoffset = 0;    // Calibration offset
unsigned long lastMicros = 0;
bool isCalibrating = true;

AceButton _buttons[NUMBER_OF_BUTTONS];
void handleEvent(AceButton*, uint8_t, uint8_t);


void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    _buttons[i].init(buttonPins[i], HIGH, i);
  }
  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);

  _leds.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  _leds.show();   // Turn OFF all pixels ASAP
  _leds.setBrightness(50);

  WiFi.mode(WIFI_STA);
  ESP_NOW_MIDI.setup(broadcastAddress, customOnDataSent);

  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    // while (1) delay(10);
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("Calibrating... keep sensor still for 3 seconds");
  calibrateZaxis();
  Serial.println("Ready - Rotate to measure");
  Serial.println("Angle(°)\tSpeed(°/s)");
  lastMicros = micros();
  lastSendTime = millis();


  _leds.setPixelColor(0, _leds.Color(255, 0, 0));
  _leds.setPixelColor(1, _leds.Color(0, 255, 0));
  _leds.setPixelColor(2, _leds.Color(0, 0, 255));
  _leds.setPixelColor(3, _leds.Color(255, 0, 255));

  _leds.show();
}

void loop() {
  for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
    _buttons[i].check();
  }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate time difference in seconds
  unsigned long now = micros();
  float deltaTime = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  // Get raw rotation speed (in rad/s) and apply offset
  float rawGyroZ = g.gyro.z - gyroZoffset;

  // Convert from rad/s to deg/s
  rotationSpeed = rawGyroZ * RAD_TO_DEG;

  // Integrate to get angle
  yaw += rotationSpeed * deltaTime;

  // Convert angle to 0-359° range
  yaw = fmod(yaw, 360);
  if (yaw < 0) yaw += 360;

  // Handle micros() overflow
  if (deltaTime > 0.1 || deltaTime < 0) {
    // Unusually large time difference detected, likely an overflow
    deltaTime = 0.05;  // Use a reasonable default
  }

  // Only perform continuous calibration during first 3 seconds if still calibrating
  if (isCalibrating && millis() < 3000) {
    gyroZoffset = gyroZoffset * 0.99 + g.gyro.z * 0.01;
  } else if (isCalibrating) {
    isCalibrating = false;
    Serial.println("Calibration complete");
  }

  // Apply complementary filter for drift compensation if device is relatively still
  if (abs(rotationSpeed) < 1.0) {
    // Apply small correction to reduce drift when not moving
    rotationSpeed = 0;
  }

  // Check if it's time to send MIDI data
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;

    // Send MIDI data every 10ms
    auto result = ESP_NOW_MIDI.sendControlChange(1, map(yaw, 0, 359, 0, 127), MIDI_CHANNEL);

    // Send speed on second CC channel with configurable sensitivity
    // ESP_NOW_MIDI.sendControlChange(2, map(constrain(abs(rotationSpeed), 0, speedSensitivity), 0, speedSensitivity, 0, 127), MIDI_CHANNEL);

    // Print both angle and speed
    // Serial.print(yaw);
    // Serial.print("\t\t");
    // Serial.println(rotationSpeed);
  }
}

void calibrateZaxis() {
  // Average 200 readings for initial calibration
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(10);
  }
  gyroZoffset = sum / 200;
  yaw = 0;
  rotationSpeed = 0;
}

void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(AceButton::eventName(eventType));
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);


  switch (eventType) {
    case AceButton::kEventPressed:

      break;
    case AceButton::kEventReleased:

      break;
  }
}