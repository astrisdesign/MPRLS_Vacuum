#include <Wire.h>
#include "Adafruit_MPRLS.h"

#define RELAY_PIN A1
#define RESET_PIN A5

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);

float MPRLS_reading_buffer[25];
int buffer_index = 0;
float pressure_setpoint_psi = 11.7;
float pressure_psi = 0.0;
int led = LED_BUILTIN;

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  while (1) {  // Keep trying to initialize the sensor
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
      Serial.println("Pressure Sensor Initialization Failure");  // Print error message
      delay(1400);  // Wait for 1.4 seconds to make the total delay 2 seconds
    }
  }
}

void loop() {
  float pressure_hPa = mpr.readPressure();
  MPRLS_reading_buffer[buffer_index] = pressure_hPa / 68.947572932;
  buffer_index++;

  if (buffer_index >= 25) {
    std::sort(MPRLS_reading_buffer, MPRLS_reading_buffer + 25);
    pressure_psi = MPRLS_reading_buffer[12];  // median index is 12
    Serial.println(pressure_psi);
    buffer_index = 0;
  }

  if (pressure_psi > pressure_setpoint_psi) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}
