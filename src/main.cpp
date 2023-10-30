#include <Wire.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include "Adafruit_MPRLS.h"

#define RELAY_PIN 11
#define RESET_PIN 10
#define MPRLS_POWER_PIN 9
#define MPRLS_GND_PIN 5

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);

float MPRLS_reading_buffer[6];
int buffer_index = 0;
float pressure_setpoint_psi = -1;
float pressure_psi = 0.0;
int led = LED_BUILTIN;
String logMessage = "";
SemaphoreHandle_t mutex;

void printJSON() {
  DynamicJsonDocument data(128);
  data["0"] = pressure_psi;
  serializeJson(data, Serial);
  Serial.print("~~~");
  DynamicJsonDocument setpoints(128);
  setpoints["Pressure"] = pressure_setpoint_psi;
  serializeJson(setpoints, Serial);
  Serial.print("~~~");
  Serial.println(logMessage);
}

void ResetSensor() {
  while (true) {
    digitalWrite(RESET_PIN, LOW);
    delay(100);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);

    if (mpr.begin()) {
      break;
    } else {
      digitalWrite(led, HIGH);
      delay(500);
      digitalWrite(led, LOW);
      delay(100);
      logMessage = "Pressure sensor initialization failure";
      pressure_psi = 0.0;
      printJSON();
      delay(400);
    }
  }
}

void Core0Code(void * pvParameters) {
  unsigned long start_time = 0;
  unsigned long current_time = 0;
  bool waiting = false;

  const unsigned long min_high_duration = 25;  // in milliseconds
  const unsigned long high_wait_time = 50;  // in milliseconds
  const unsigned long low_wait_time = 50;  // in milliseconds

  for (;;) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    float pressure_hPa = mpr.readPressure();
    MPRLS_reading_buffer[buffer_index] = 14.7 - pressure_hPa / 68.947572932;
    buffer_index++;
    xSemaphoreGive(mutex);

    if (buffer_index >= 5) {
      std::sort(MPRLS_reading_buffer, MPRLS_reading_buffer + 5);
      xSemaphoreTake(mutex, portMAX_DELAY);
      pressure_psi = MPRLS_reading_buffer[2];
      xSemaphoreGive(mutex);

      current_time = millis();

      if (pressure_psi < pressure_setpoint_psi * 0.8) {
        digitalWrite(RELAY_PIN, HIGH);
        waiting = false;
      } 
      else if (pressure_psi < pressure_setpoint_psi*0.985) {
        if (!waiting) {
          start_time = current_time;
          digitalWrite(RELAY_PIN, HIGH);
          waiting = true;
        } 
        else {
          if (current_time - start_time > min_high_duration) {
            digitalWrite(RELAY_PIN, LOW);
            if (current_time - start_time > min_high_duration + low_wait_time) {
              waiting = false;
            }
          }
        }
      } 
      else if (pressure_psi > pressure_setpoint_psi * 1.15) {
        digitalWrite(RELAY_PIN, LOW);
        waiting = false;
      } 
      else {
        if (!waiting) {
          start_time = current_time;
          digitalWrite(RELAY_PIN, LOW);
          waiting = true;
        } 
        else {
          if (current_time - start_time > high_wait_time) {
            waiting = false;
          }
        }
      }

      buffer_index = 0;
    }

    delay(5);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(MPRLS_POWER_PIN, OUTPUT);
  pinMode(MPRLS_GND_PIN, OUTPUT);

  digitalWrite(MPRLS_POWER_PIN, HIGH);
  digitalWrite(MPRLS_GND_PIN, LOW);
  logMessage = "Pins established.";
  printJSON();

  mutex = xSemaphoreCreateMutex();
  ResetSensor();
  logMessage = "Sensor reset during initial setup.";
  printJSON();

  xTaskCreatePinnedToCore(
    Core0Code,
    "PressureControl",
    10000,
    NULL,
    1,
    NULL,
    0
  );
  logMessage = "Core0 task (pressure control) initiated.";
  printJSON();
}

void loop() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (isnan(pressure_psi)) {
    ResetSensor();
  } else {
    logMessage = "";
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, input);
    if (!error) {
      if (doc.containsKey("Pressure")) {
        pressure_setpoint_psi = doc["Pressure"];
      }
    } else {
      logMessage += "JSON parse error";
    }
  }

  printJSON();
  xSemaphoreGive(mutex);

  delay(500);
}