#include <Wire.h>
#include <ArduinoJson.h>
#include "Adafruit_MPRLS.h"

#define RELAY_PIN A1
#define RESET_PIN A5

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);

float MPRLS_reading_buffer[6];
int buffer_index = 0;
float pressure_setpoint_psi = 11.7;
float pressure_psi = 0.0;
int led = LED_BUILTIN;
String logMessage = "";

void printJSON() {
  DynamicJsonDocument doc(128);
  doc["0"] = pressure_psi;
  serializeJson(doc, Serial);
  Serial.print("~~~");
  Serial.println(logMessage);
}

void ResetSensor() {
  while (true) {  // Keep trying to initialize the sensor
    digitalWrite(RESET_PIN, LOW);  // Reset the sensor
    delay(100);  // Wait for 100ms
    digitalWrite(RESET_PIN, HIGH);  // Release the reset
    delay(100);  // Wait for 100ms

    if (mpr.begin()) {  // Try to initialize the sensor
      break;  // Break out of the loop if successful
    } else {
      digitalWrite(led, HIGH);  // Turn on the built-in LED
      delay(500);  // Wait for 500ms
      digitalWrite(led, LOW);  // Turn off the built-in LED
      delay(100);  // Wait for 100ms
      logMessage = "Pressure Sensor Initialization Failure";  // Update log message
      pressure_psi = 0.0;  // Set pressure to zero
      printJSON();  // Print JSON using the function for consistent formatting
      delay(400);  // Wait for .4 seconds to make the total delay 1 second
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  ResetSensor();
}

void loop() {
  float pressure_hPa = mpr.readPressure();
  MPRLS_reading_buffer[buffer_index] = pressure_hPa / 68.947572932;
  buffer_index++;

  if (buffer_index >= 5) {
    std::sort(MPRLS_reading_buffer, MPRLS_reading_buffer + 5);
    pressure_psi = MPRLS_reading_buffer[2];
    logMessage = "";

    if (isnan(pressure_psi)) {
      ResetSensor();
    }

    printJSON();
    buffer_index = 0;
  }

  if (pressure_psi > pressure_setpoint_psi) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}
