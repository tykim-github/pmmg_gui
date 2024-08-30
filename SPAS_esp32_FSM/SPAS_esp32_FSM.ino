#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include "MS5607.h"
#include <esp_timer.h>

// Pin setting
#define IMU1_RX 22
#define IMU1_TX 27
#define IMU2_RX 21
#define IMU2_TX 32
#define IMU3_RX 16
#define IMU3_TX 17
#define PMMG_CS 5
#define BUTTON 26
#define LED_PIN 33
#define BATTERY 36

// FSM states
enum State {
  IDLE,
  LOW_VOLTAGE,
  PMMG_ERROR,
  IMUS_ERROR,
  STANDBY,
  LEG_ZEROING,
  LEG_ZEROING_END,
  READ_SENSORS,
  READ_SENSORS_END,
  MAG_CALIBRATION
};
volatile State currentState = IDLE;
volatile State nextState = IDLE;

#define MSG_STATE_IS_STANDBY 101
#define MSG_ZEROING_STARTED  102
#define MSG_ZEROING_STOPPED  103
#define MSG_READING_STARTED  104
#define MSG_READING_STOPPED  105
#define MSG_STATE_IS_MAG_CAL 106
#define ERROR_LOW_VOLTAGE    201
#define ERROR_PMMG_MALFUNC   202
#define ERROR_IMUS_MALFUNC   203

uint8_t stateMessage = 0;

struct Quaternion {
  float qw, qx, qy, qz;
  bool valid;  // Indicator for data validity
};
Quaternion quat_imu1, quat_imu2, quat_imu3;

HardwareSerial IMUSerial1(0);
HardwareSerial IMUSerial2(1);
HardwareSerial IMUSerial3(2);

MS5607 PMMG(PMMG_CS, 0);
volatile bool processDataFlag = false;  // Flag to trigger process data

volatile bool button_now = 1, button_old = 1, button_is_pushed = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 20;
const uint16_t battery_threshold = 4000;

#define LONG_PRESS_TIME 1000
unsigned long stateEntryTime = 0;  // Stores the time when a specific state is entered


// Declare a handle for the timer
void IRAM_ATTR onTimer(void* arg);
esp_timer_handle_t myTimer = NULL;

// WI-FI
uint8_t broadcastAddress[] = { 0xAC, 0x67, 0xB2, 0x40, 0x13, 0x8C };
esp_now_peer_info_t peerInfo;

struct struct_message {
  uint8_t msg;
  unsigned long time;
  int16_t d_imu1[4];  // IMU1 quaternion data in fixed-point format
  int16_t d_imu2[4];  // IMU2 quaternion data in fixed-point format
  int16_t d_imu3[4];  // IMU3 quaternion data in fixed-point format
  float d_pmmg;
} myData;

const float SCALE_FACTOR = 1000.0;


// ################################################################# //
void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY, INPUT);
  pinMode(PMMG_CS, OUTPUT);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return;
  }

  const esp_timer_create_args_t timerArgs = {
    .callback = &onTimer,
    .arg = NULL,
    .name = "periodicTimer"
  };

  esp_timer_create(&timerArgs, &myTimer);
  esp_timer_start_periodic(myTimer, 5000);
}
// ################################################################# //

// ################################################################# //
//
//                   ███████╗ ██████╗███╗   ███╗
//                   ██╔════╝██╔════╝████╗ ████║
//                   █████╗  ╚█████╗ ██╔████╔██║
//                   ██╔══╝   ╚═══██╗██║╚██╔╝██║
//                   ██║     ██████╔╝██║ ╚═╝ ██║
//                   ╚═╝     ╚═════╝ ╚═╝     ╚═╝

void loop() {
  // Step 1: Button Checking
  button_is_pushed = button_check();

  // Step 2: State Handling
  switch (currentState) {
    case IDLE:              handleIdle(); break;
    case LOW_VOLTAGE:       handleLowVoltage(); break;
    case PMMG_ERROR:        handlepmmgError(); break;
    case IMUS_ERROR:        handleimusError(); break;
    case STANDBY:           handleStandby(); break;
    case LEG_ZEROING:       handleLegZeroing(); break;
    case LEG_ZEROING_END:   handleLegZeroingEnd(); break;
    case READ_SENSORS:      handleReadSensors(); break;
    case READ_SENSORS_END:  handleReadSensorsEnd(); break;
    case MAG_CALIBRATION:   handleMagCalibration(); break;
  }

  // Step 3: Sensor Data Processing (conditional on processDataFlag)
  if (processDataFlag) {
    processDataFlag = false;

    bool newData = true;
    bool pmmgSuccess = (PMMG.read(8) == 0);
    bool imu1Success = updateQuaternion(IMUSerial1, quat_imu1);
    bool imu2Success = updateQuaternion(IMUSerial2, quat_imu2);
    bool imu3Success = updateQuaternion(IMUSerial3, quat_imu3);

    newData &= pmmgSuccess;c:\Users\RSCLAB\Documents\spas_python\pmmg_gui\dist\pmmg_receiver_gui_v2.exe
    newData &= imu1Success;
    newData &= imu2Success;
    newData &= imu3Success;

    // Handle different outcomes based on sensor data
    if (!pmmgSuccess || PMMG.getPressure() * 0.001 > 150) {
        nextState = PMMG_ERROR;
        storeMsg(ERROR_PMMG_MALFUNC);
    } else if (!imu1Success || !imu2Success || !imu3Success) {
        nextState = IMUS_ERROR;
        storeMsg(ERROR_IMUS_MALFUNC);
    } else if (newData) {
        myData.d_pmmg = PMMG.getPressure() * 0.001;
        // if (currentState == LEG_ZEROING || currentState == READ_SENSORS) {
        storeData();
        sendData_espnow();
        //}
    }
  }

  // Step 4: State Transition
  if (nextState != currentState) {
    currentState = nextState;
    if (nextState == LEG_ZEROING_END || nextState == READ_SENSORS_END) {
      // Optional delay to ensure data is sent before state transition
      delay(200);
    }
    sendData_espnow();  // Send state change message
  }
}

void handleIdle() {
  esp_timer_stop(myTimer);

  IMUSerial1.begin(115200, SERIAL_8N1, IMU1_RX, IMU1_TX);
  IMUSerial2.begin(115200, SERIAL_8N1, IMU2_RX, IMU2_TX);
  IMUSerial3.begin(115200, SERIAL_8N1, IMU3_RX, IMU3_TX);
  SPI.begin();
  PMMG.init();

  esp_timer_start_periodic(myTimer, 5000);

  nextState = (analogRead(BATTERY) < battery_threshold) ? LOW_VOLTAGE : STANDBY;
  storeMsg(nextState == LOW_VOLTAGE ? ERROR_LOW_VOLTAGE : MSG_STATE_IS_STANDBY);
}

void handleLowVoltage() {}

void handlepmmgError() {
  if (button_is_pushed) {
    nextState = IDLE;
  }  
}

void handleimusError() {
  if (button_is_pushed) {
    nextState = IDLE;
  }
}

void handleStandby() {
  static unsigned long buttonPressTime = 0;

  if (button_is_pushed) {
    buttonPressTime = millis();
  } else if (buttonPressTime > 0 && (millis() - buttonPressTime) > LONG_PRESS_TIME) {
    nextState = MAG_CALIBRATION;
    storeMsg(MSG_STATE_IS_MAG_CAL);
    buttonPressTime = 0;
  } else if (buttonPressTime > 0 && (millis() - buttonPressTime) < LONG_PRESS_TIME) {
    if (digitalRead(BUTTON) == HIGH) {
      nextState = LEG_ZEROING;
      stateEntryTime = millis();
      storeMsg(MSG_ZEROING_STARTED);
      buttonPressTime = 0;
    }
  }
}

void handleLegZeroing() {
  if (button_is_pushed) {
    nextState = LEG_ZEROING_END;
    storeMsg(MSG_ZEROING_STOPPED);
  }
}

void handleLegZeroingEnd() {
  if (button_is_pushed) {
    nextState = READ_SENSORS;
    stateEntryTime = millis();
    storeMsg(MSG_READING_STARTED);
  }
}

void handleReadSensors() {
  if (button_is_pushed) {
    nextState = READ_SENSORS_END;
    storeMsg(MSG_READING_STOPPED);
  }
}

void handleReadSensorsEnd() {
  static unsigned long buttonPressTime = 0;

  if (button_is_pushed) {
    buttonPressTime = millis();
  } else if (buttonPressTime > 0 && (millis() - buttonPressTime) > LONG_PRESS_TIME) {
    nextState = STANDBY;
    storeMsg(MSG_STATE_IS_STANDBY);
    buttonPressTime = 0;
  } else if (buttonPressTime > 0 && (millis() - buttonPressTime) < LONG_PRESS_TIME) {
    if (digitalRead(BUTTON) == HIGH) {
      nextState = READ_SENSORS;
      stateEntryTime = millis();
      storeMsg(MSG_READING_STARTED);
      buttonPressTime = 0;
    }
  }
}

void handleMagCalibration() {
  IMUSerial1.println("<cmf>");
  IMUSerial2.println("<cmf>");
  IMUSerial3.println("<cmf>");

  while (!button_check()) { delay(1); }

  bool magcal_allok = true;

  IMUSerial1.println(">");
  magcal_allok &= waitForCalibrationResponse(IMUSerial1);
  IMUSerial2.println(">");
  magcal_allok &= waitForCalibrationResponse(IMUSerial2);
  IMUSerial3.println(">");
  magcal_allok &= waitForCalibrationResponse(IMUSerial3);

  if (magcal_allok) {
    nextState = STANDBY;
    storeMsg(MSG_STATE_IS_STANDBY);
  } else {
    nextState = IMUS_ERROR;
    storeMsg(ERROR_IMUS_MALFUNC);
  }
}

bool waitForCalibrationResponse(HardwareSerial& serial) {
  unsigned long timeout = millis() + 3000;
  while (millis() < timeout) {
    if (serial.available()) {
      String response = serial.readStringUntil('\n');
      if (response == "<ok>") {
        return true;
      }
    }
  }
  return false;
}

// ################################################################# //

// ################################################################# //
//
//               ██╗       ██╗██╗      ███████╗██╗
//               ██║  ██╗  ██║██║      ██╔════╝██║
//               ╚██╗████╗██╔╝██║█████╗█████╗  ██║
//                ████╔═████║ ██║╚════╝██╔══╝  ██║
//                ╚██╔╝ ╚██╔╝ ██║      ██║     ██║
//                 ╚═╝   ╚═╝  ╚═╝      ╚═╝     ╚═╝

void sendData_espnow() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
  if (result != ESP_OK) {
    // Handle error if needed
  }
}

void storeMsg(uint8_t messageCode) {
  myData.msg = messageCode;
  myData.time = millis();
  memset(myData.d_imu1, 0, sizeof(myData.d_imu1));
  memset(myData.d_imu2, 0, sizeof(myData.d_imu2));
  memset(myData.d_imu3, 0, sizeof(myData.d_imu3));
  myData.d_pmmg = 0;
}

void storeData() {
  unsigned long elapsedTime = millis() - stateEntryTime;  // Calculate elapsed time
  myData.msg = 0;
  myData.time = elapsedTime;
  myData.d_imu1[0] = (int16_t)(quat_imu1.qw * SCALE_FACTOR);
  myData.d_imu1[1] = (int16_t)(quat_imu1.qx * SCALE_FACTOR);
  myData.d_imu1[2] = (int16_t)(quat_imu1.qy * SCALE_FACTOR);
  myData.d_imu1[3] = (int16_t)(quat_imu1.qz * SCALE_FACTOR);

  myData.d_imu2[0] = (int16_t)(quat_imu2.qw * SCALE_FACTOR);
  myData.d_imu2[1] = (int16_t)(quat_imu2.qx * SCALE_FACTOR);
  myData.d_imu2[2] = (int16_t)(quat_imu2.qy * SCALE_FACTOR);
  myData.d_imu2[3] = (int16_t)(quat_imu2.qz * SCALE_FACTOR);

  myData.d_imu3[0] = (int16_t)(quat_imu3.qw * SCALE_FACTOR);
  myData.d_imu3[1] = (int16_t)(quat_imu3.qx * SCALE_FACTOR);
  myData.d_imu3[2] = (int16_t)(quat_imu3.qy * SCALE_FACTOR);
  myData.d_imu3[3] = (int16_t)(quat_imu3.qz * SCALE_FACTOR);
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Add any specific actions you need on successful or failed transmission
}
// ################################################################# //

// ################################################################# //
//          ██████╗███████╗███╗  ██╗ ██████╗ █████╗ ██████╗
//         ██╔════╝██╔════╝████╗ ██║██╔════╝██╔══██╗██╔══██╗
//         ╚█████╗ █████╗  ██╔██╗██║╚█████╗ ██║  ██║██████╔╝
//          ╚═══██╗██╔══╝  ██║╚████║ ╚═══██╗██║  ██║██╔══██╗
//         ██████╔╝███████╗██║ ╚███║██████╔╝╚█████╔╝██║  ██║
//         ╚═════╝ ╚══════╝╚═╝  ╚══╝╚═════╝  ╚════╝ ╚═╝  ╚═╝

void IRAM_ATTR onTimer(void* arg) {
  if (currentState == LEG_ZEROING || currentState == READ_SENSORS) {
    processDataFlag = true;
  }
}

bool updateQuaternion(HardwareSerial& serial, Quaternion& quat) {
  if (serial.available()) {
    Quaternion newQuat = readIMUSerial(serial);
    if (newQuat.valid) {
      quat = newQuat;
      return true;
    }
  }
  return true;
}

Quaternion readIMUSerial(HardwareSerial& serial) {
  static char buffer[64];
  static size_t bufPos = 0;
  Quaternion q = { 0.0, 0.0, 0.0, 0.0, false };

  while (serial.available()) {
    char c = serial.read();
    if (c == '\n' && bufPos > 0) {
      buffer[bufPos] = '\0';
      if (buffer[0] == '*' && bufPos > 1) {
        char* p = &buffer[1];
        float values[4];
        int count = 0;
        for (int i = 0; i < 4 && *p; i++) {
          values[i] = strtod(p, &p);
          if (p == &buffer[1] || (*p != ',' && *p != '\0')) {
            bufPos = 0;
            return q;
          }
          count++;
          if (*p == ',') p++;
        }
        if (count == 4) {
          q.qz = values[0];
          q.qy = values[1];
          q.qx = values[2];
          q.qw = values[3];
          q.valid = true;
        }
      }
      bufPos = 0;
      return q;
    } else if (c != '\r') {
      if (bufPos < sizeof(buffer) - 1) {
        buffer[bufPos++] = c;
      }
    }
  }
  return q;
}

bool button_check() {
  bool result = false;
  int reading = digitalRead(BUTTON);

  if (reading != button_old) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != button_now) {
      button_now = reading;
      if (button_now == LOW) {
        result = true;
      }
    }
  }

  button_old = reading;
  return result;
}
