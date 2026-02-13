#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

#include "config.h"
#include "communication.h"
#include "sensors.h"

// Define receiver MAC address (declared as extern in config.h)
uint8_t receiverAddress[6] = { 0x14, 0x33, 0x5C, 0x03, 0xF2, 0xB0 };

// Sensor objects
MPU9250_asukiaaa mySensorMPU6500(MPU9250_ADDR);
MPU6050 mySensorMPU6050(MPU6050_ADDR);

// Orientation variables
float yaw, pitch;
float yawGyro = 0;
float gyroZOffset = 0;
unsigned long lastTime = 0;

// Filter variables
float gyroZBuffer[FILTER_SIZE] = {0};
int filterIndex = 0;

// Sensor status flags
bool mpu6500Ok = false;
bool mpu6050Ok = false;

// Data packet
OrientationData dataToSend;

// Transmission statistics
unsigned long transmissionFailures = 0;

// ESP-NOW initialization flag
bool espNowReady = false;

// ===== Logging Functions =====
void logInfo(String msg) {
  Serial.print("[INFO] ");
  Serial.println(msg);
}

void logError(String msg) {
  Serial.print("[ERROR] ");
  Serial.println(msg);
}

void logWarning(String msg) {
  Serial.print("[WARN] ");
  Serial.println(msg);
}

// ===== ESP-NOW Callback =====
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    transmissionFailures++;
    logError("Transmission failed! Total failures: " + String(transmissionFailures));
  }
}

// ===== Setup Function =====
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial
  
  logInfo("Head Tracker Transmitter v" + String(FIRMWARE_VERSION));
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  logInfo("Transmitter MAC: " + WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    logError("ESP-NOW init failed");
    espNowReady = false;
    while (1) {
      delay(1000);
      logError("System halted due to ESP-NOW initialization failure");
    }
  }
  espNowReady = true;
  logInfo("ESP-NOW initialized");

  // Register send callback for transmission monitoring
  esp_now_register_send_cb(onDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    logError("Failed to add peer");
    return;
  }
  logInfo("Peer added successfully");

  // Initialize sensors
  mpu6500Ok = initializeMPU6500();
  mpu6050Ok = initializeMPU6050();

  // Calibrate gyroscope if MPU6500 is available
  if (mpu6500Ok) {
    calibrateGyroscope();
    lastTime = micros();
  }

  logInfo("Setup complete!");
  delay(100);
}

// ===== Main Loop =====
void loop() {
  // Check ESP-NOW initialization
  if (!espNowReady) {
    delay(1000);
    return;
  }
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "reset") {
      resetYaw();
    } else if (command == "status") {
      printSystemStatus();
    }
  }

  // Read sensor data
  yaw = readYawFromGyro();
  pitch = readPitchFromAccel();

  // Debug output
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Pitch: ");
  Serial.println(pitch);

  // Prepare and send data
  dataToSend.yaw = yaw;
  dataToSend.pitch = pitch;
  dataToSend.timestamp = millis();
  
  // Send data and check immediate return value
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
  if (result != ESP_OK) {
    transmissionFailures++;
    if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      logError("Send failed: ESP-NOW not initialized");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      logError("Send failed: Invalid argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      logError("Send failed: Internal error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      logError("Send failed: Out of memory");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      logError("Send failed: Peer not found");
    } else {
      logError("Send failed: Error code " + String(result));
    }
  }

  delay(LOOP_DELAY_MS);
}

// ===== Sensor Functions =====
bool initializeMPU6500() {
  logInfo("Initializing MPU6500...");
  mySensorMPU6500.setWire(&Wire);
  mySensorMPU6500.beginAccel();
  mySensorMPU6500.beginGyro();
  delay(100);

  // Test connection by reading WHO_AM_I register
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU9250_ADDR, 1) == 1) {
    uint8_t whoAmI = Wire.read();
    Serial.print("MPU6500 WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    
    if (whoAmI == 0x70) {
      logInfo("MPU6500 connected successfully");
      return true;
    } else if (whoAmI == 0x71 || whoAmI == 0x73) {
      logWarning("Detected MPU9250/MPU9255 (0x" + String(whoAmI, HEX) + "), magnetometer may be unavailable");
      return true;
    } else {
      logError("MPU6500 invalid WHO_AM_I (0x" + String(whoAmI, HEX) + ")");
      return false;
    }
  } else {
    logError("MPU6500 I2C communication failed");
    return false;
  }
}

bool initializeMPU6050() {
  logInfo("Initializing MPU6050...");
  mySensorMPU6050.initialize();
  
  if (!mySensorMPU6050.testConnection()) {
    logError("MPU6050 not connected! Pitch will be set to 0");
    return false;
  } else {
    logInfo("MPU6050 connected successfully");
    return true;
  }
}

void calibrateGyroscope() {
  logInfo("Calibrating gyroscope... Keep sensor stationary");
  float sumGyroZ = 0;
  
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    mySensorMPU6500.gyroUpdate();
    sumGyroZ += mySensorMPU6500.gyroZ();
    delay(CALIBRATION_DELAY_MS);
  }
  
  gyroZOffset = sumGyroZ / GYRO_CALIBRATION_SAMPLES;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset);
  
  if (abs(gyroZOffset) > MAX_GYRO_OFFSET) {
    logWarning("Large gyroscope offset detected: " + String(gyroZOffset) + " deg/s");
    logWarning("Recalibrate in a stable environment");
  } else {
    logInfo("Calibration complete");
  }
}

float readYawFromGyro() {
  if (!mpu6500Ok) {
    return 0;
  }

  mySensorMPU6500.gyroUpdate();
  float gyroZ = mySensorMPU6500.gyroZ() - gyroZOffset;

  // Validate gyroscope data before processing
  if (abs(gyroZ) > MAX_VALID_ANGULAR_VELOCITY) {
    logError("Invalid gyroscope data: |" + String(gyroZ) + "| > " + String(MAX_VALID_ANGULAR_VELOCITY));
    return yawGyro;  // Return current value without updating
  }

  // Apply moving average filter
  gyroZBuffer[filterIndex] = gyroZ;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  float gyroZFiltered = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    gyroZFiltered += gyroZBuffer[i];
  }
  gyroZFiltered /= FILTER_SIZE;

  // Calculate delta time
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Integrate filtered gyroZ for yaw
  yawGyro += gyroZFiltered * deltaTime;
  if (yawGyro >= YAW_MAX) yawGyro -= YAW_MAX;
  if (yawGyro < YAW_MIN) yawGyro += YAW_MAX;

  return yawGyro;
}

float readPitchFromAccel() {
  if (!mpu6050Ok) {
    return 0;
  }

  int16_t ax, ay, az;
  mySensorMPU6050.getAcceleration(&ax, &ay, &az);

  // Convert to g
  float fax = ax / ACCEL_SENSITIVITY;
  float fay = ay / ACCEL_SENSITIVITY;
  float faz = az / ACCEL_SENSITIVITY;

  // Validate accelerometer data
  float magnitude = sqrt(fax * fax + fay * fay + faz * faz);
  if (magnitude < ACCEL_MIN_G || magnitude > ACCEL_MAX_G) {
    logError("Invalid accelerometer data: magnitude = " + String(magnitude));
    return 0;
  }

  // Calculate pitch
  return atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / PI;
}

void resetYaw() {
  yawGyro = 0;
  logInfo("Yaw reset to 0");
}

void printSystemStatus() {
  Serial.println("\n===== System Status =====");
  Serial.println("Firmware: v" + String(FIRMWARE_VERSION));
  Serial.println("MPU6500: " + String(mpu6500Ok ? "OK" : "FAILED"));
  Serial.println("MPU6050: " + String(mpu6050Ok ? "OK" : "FAILED"));
  Serial.println("Gyro Offset: " + String(gyroZOffset) + " deg/s");
  Serial.println("Transmission Failures: " + String(transmissionFailures));
  Serial.println("Current Yaw: " + String(yaw) + "°");
  Serial.println("Current Pitch: " + String(pitch) + "°");
  Serial.println("========================\n");
}