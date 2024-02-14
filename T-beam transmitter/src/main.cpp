#include <Arduino.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <boards.h>

// Instance of the TinyGPS++ object
TinyGPSPlus gps;

// LoRa frequency in Hz (specifically for regions that use 915MHz, like North America)
const long frequency = 915E6;

// Flag to switch between test mode and normal GPS transmission mode
const bool test = true;

void setup() {
  // Short delay to ensure system stability after reset
  delay(1000);

  // Initialize board-specific configurations
  initBoard();  

  // Set LoRa module pins
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);

  // Initialize LoRa with the specified frequency
  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed for Transmitter!");
  } else {
    Serial.println("Setup Successful for Transmitter");
  }

  // Initialize the display if available and show a success message
  #ifdef HAS_DISPLAY
  if (u8g2) {
    u8g2->clearBuffer();
    u8g2->drawStr(0, 12, " Success  ");
    u8g2->sendBuffer();
  }
  #endif
}

void loop() {
  // Check if we are not in test mode
  if (!test) {
    // Read data from the GPS module
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      if (gps.encode(c)) {

        // Check if there is an updated location
        if (gps.location.isUpdated()) {
          // Transmit location via LoRa
          LoRa.beginPacket();
          LoRa.print("Latitude: ");
          LoRa.print(gps.location.lat(), 6); // 6 decimal places for precision
          LoRa.print(" Longitude: ");
          LoRa.print(gps.location.lng(), 6);
          LoRa.endPacket();

          // Also, print the location to the serial monitor for debugging
          Serial.print("Latitude: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print(" Longitude: ");
          Serial.println(gps.location.lng(), 6);
          Serial.flush(); // Ensure all data is sent before proceeding
        }
      }
    }
  } else {
    // Test mode - send a simple message via LoRa
    Serial.println("Getting here");
    LoRa.beginPacket();
    LoRa.print("Test");
    LoRa.endPacket();
    
    Serial.println("Sent");
    Serial.flush(); // Ensure the message is sent before proceeding
    delay(1000); // Wait a second before sending the next message
  }

  // Short delay to prevent spamming and allow for serial buffer processing
  delay(10);
}
