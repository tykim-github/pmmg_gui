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

const float scaleFactor = 1000.0;

ESP32Time rtc(3600);

void onDataReceiver(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  // Your existing code to handle the incoming data remains unchanged
  memcpy(&myData, incomingData, sizeof(myData));

  // Convert the received int16_t values back to float
  float imu1[4], imu2[4], imu3[4];

  for (int i = 0; i < 4; i++) {
    imu1[i] = myData.d_imu1[i] / scaleFactor;
    imu2[i] = myData.d_imu2[i] / scaleFactor;
    imu3[i] = myData.d_imu3[i] / scaleFactor;
  }

  unsigned long t = myData.time;  // Time in milliseconds since the board was last reset

  // Printing the received data to the Serial monitor
  Serial.print(myData.msg);
  Serial.print(",");
  Serial.print(t);
  Serial.print(",");
  for(int i = 0; i < 4; i++) {
    Serial.print(imu1[i]);
    Serial.print(",");
  }
  for(int i = 0; i < 4; i++) {
    Serial.print(imu2[i]);
    Serial.print(",");
  }
  for(int i = 0; i < 4; i++) {
    Serial.print(imu3[i]);
    Serial.print(",");
  }
  Serial.println(myData.d_pmmg);  // PMMG sensor data
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
  // Keep the loop empty if no periodic tasks are needed.
}
