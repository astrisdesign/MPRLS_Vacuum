#include <Wire.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include "Adafruit_MPRLS.h"

#define RELAY_PIN A1
#define RESET_PIN A5

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);

float MPRLS_reading_buffer[6];
int buffer_index = 0;
float pressure_setpoint_psi = 3;
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
  setpoints["Temperature"] = 25;
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

      if (pressure_psi < pressure_setpoint_psi) {
        digitalWrite(RELAY_PIN, HIGH);
      } else {
        digitalWrite(RELAY_PIN, LOW);
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

  mutex = xSemaphoreCreateMutex();
  ResetSensor();

  xTaskCreatePinnedToCore(
    Core0Code,
    "PressureControl",
    10000,
    NULL,
    1,
    NULL,
    0
  );
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