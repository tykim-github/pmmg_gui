#include <Arduino.h>
#include <ESP32Time.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Updated struct_message to match the updated sender's structure
typedef struct struct_message {
  uint8_t msg;
  unsigned long time;
  int16_t d_imu1[4];  // IMU1 quaternion data in fixed-point format
  int16_t d_imu2[4];  // IMU2 quaternion data in fixed-point format
  int16_t d_imu3[4];  // IMU3 quaternion data in fixed-point format
  float d_pmmg;       // PMMG sensor data
} struct_message;

struct_message myData;
volatile bool newDataReceived = false;
struct_message receivedData;

const float scaleFactor = 1000.0;

ESP32Time rtc(3600);

void onDataReceiver(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  // Copy the received data into the global buffer
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  // Set the flag to indicate new data has been received
  newDataReceived = true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  uint8_t newMACAddress[] = {0xAC, 0x67, 0xB2, 0x40, 0x13, 0x8C}; //AC:67:B2:40:13:8C
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error during ESP-NOW initialization");
    return;
  }

  // Register the callback function for receiving data
  esp_now_register_recv_cb(onDataReceiver);
}

void loop() {
  if (newDataReceived) {
    // Reset the flag
    newDataReceived = false;

    // Convert the received int16_t values back to float
    float imu1[4], imu2[4], imu3[4];

    for (int i = 0; i < 4; i++) {
      imu1[i] = receivedData.d_imu1[i] / scaleFactor;
      imu2[i] = receivedData.d_imu2[i] / scaleFactor;
      imu3[i] = receivedData.d_imu3[i] / scaleFactor;
    }

    unsigned long t = receivedData.time;

    // Printing the received data to the Serial monitor
    Serial.print(receivedData.msg);
    Serial.print(",");
    Serial.print(t);
    Serial.print(",");

    // Print IMU1 data with 3 decimal places
    for(int i = 0; i < 4; i++) {
      Serial.print(imu1[i], 3);
      Serial.print(",");
    }

    // Print IMU2 data with 3 decimal places
    for(int i = 0; i < 4; i++) {
      Serial.print(imu2[i], 3);
      Serial.print(",");
    }

    // Print IMU3 data with 3 decimal places
    for(int i = 0; i < 4; i++) {
      Serial.print(imu3[i], 3);
      Serial.print(",");
    }

    // Print PMMG sensor data with 3 decimal places
    Serial.println(receivedData.d_pmmg, 3);
  }

  // Add a small delay to prevent loop from hogging CPU
  delay(1);
}
