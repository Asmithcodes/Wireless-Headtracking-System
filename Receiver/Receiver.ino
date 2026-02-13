#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#include "config.h"
#include "communication.h"

// Data structure
OrientationData incomingData;

// Servo objects
Servo servoYaw;
Servo servoPitch;

// Smoothing variables
float smoothedYaw = SERVO_CENTER_ANGLE;
float smoothedPitch = SERVO_CENTER_ANGLE;

// Timeout detection
unsigned long lastReceiveTime = 0;
bool connectionActive = false;

// Statistics
unsigned long packetCount = 0;
unsigned long prevPacketCount = 0;
unsigned long lastStatsTime = 0;

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

// ===== ESP-NOW Receive Callback =====
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(OrientationData)) {
    memcpy(&incomingData, data, sizeof(incomingData));
    lastReceiveTime = millis();
    packetCount++;

    if (!connectionActive) {
      connectionActive = true;
      logInfo("Connection established!");
    }

    Serial.print("Received -> Yaw: ");
    Serial.print(incomingData.yaw);
    Serial.print(" | Pitch: ");
    Serial.print(incomingData.pitch);
    Serial.print(" | Timestamp: ");
    Serial.println(incomingData.timestamp);

    // Update servo positions with smoothing
    updateServos(incomingData.yaw, incomingData.pitch);
  } else {
    logError("Received invalid data size: " + String(len) + " bytes");
  }
}

// ===== Servo Control with Smoothing =====
void updateServos(float yaw, float pitch) {
  // Map yaw (0–180°) to servo range (180 to 0) - inverted
  int yawVal = constrain(map((int)yaw, YAW_INPUT_MIN, YAW_INPUT_MAX, 
                              SERVO_OUTPUT_MAX, SERVO_OUTPUT_MIN), 
                         SERVO_OUTPUT_MIN, SERVO_OUTPUT_MAX);
  
  // Map pitch (–90 to 90°) to servo range (180 to 0)
  int pitchVal = constrain(map((int)pitch, PITCH_INPUT_MIN, PITCH_INPUT_MAX, 
                                SERVO_OUTPUT_MAX, SERVO_OUTPUT_MIN), 
                           SERVO_OUTPUT_MIN, SERVO_OUTPUT_MAX);

  // Apply exponential smoothing
  smoothedYaw = smoothedYaw + SMOOTHING_FACTOR * (yawVal - smoothedYaw);
  smoothedPitch = smoothedPitch + SMOOTHING_FACTOR * (pitchVal - smoothedPitch);

  // Write to servos
  servoYaw.write((int)smoothedYaw);
  servoPitch.write((int)smoothedPitch);

  Serial.print("Mapped -> Yaw: ");
  Serial.print((int)smoothedYaw);
  Serial.print(" | Pitch: ");
  Serial.println((int)smoothedPitch);
}

// ===== Timeout Check =====
void checkTimeout() {
  if (connectionActive && (millis() - lastReceiveTime > TIMEOUT_MS)) {
    connectionActive = false;
    logWarning("Connection timeout! No data for " + String(TIMEOUT_MS) + "ms");
    
    // Return servos to center position
    returnToCenter();
  }
}

void returnToCenter() {
  logInfo("Returning servos to center position");
  smoothedYaw = SERVO_CENTER_ANGLE;
  smoothedPitch = SERVO_CENTER_ANGLE;
  servoYaw.write(SERVO_CENTER_ANGLE);
  servoPitch.write(SERVO_CENTER_ANGLE);
}

// ===== Statistics Reporting =====
void printStatistics() {
  if (millis() - lastStatsTime > STATS_INTERVAL_MS) {
    unsigned long delta = packetCount - prevPacketCount;
    float dataRate = delta * 1000.0 / STATS_INTERVAL_MS;
    
    Serial.println("\n===== Statistics =====");
    Serial.println("Firmware: v" + String(FIRMWARE_VERSION));
    Serial.println("Connection: " + String(connectionActive ? "Active" : "Inactive"));
    Serial.print("Data Rate: ");
    Serial.print(dataRate);
    Serial.println(" Hz");
    Serial.println("Total Packets: " + String(packetCount));
    Serial.println("=====================\n");
    
    prevPacketCount = packetCount;
    lastStatsTime = millis();
  }
}

// ===== Setup Function =====
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500); // Wait for Serial Monitor
  
  logInfo("Head Tracker Receiver v" + String(FIRMWARE_VERSION));

  // Attach servos with pulse width range
  servoYaw.attach(SERVO_YAW_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoPitch.attach(SERVO_PITCH_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // Verify servo attachment
  if (!servoYaw.attached()) {
    logError("Yaw servo failed to attach!");
  } else {
    logInfo("Yaw servo attached to pin " + String(SERVO_YAW_PIN));
  }
  
  if (!servoPitch.attached()) {
    logError("Pitch servo failed to attach!");
  } else {
    logInfo("Pitch servo attached to pin " + String(SERVO_PITCH_PIN));
  }

  // Center servos on startup
  servoYaw.write(SERVO_CENTER_ANGLE);
  servoPitch.write(SERVO_CENTER_ANGLE);
  logInfo("Servos centered at " + String(SERVO_CENTER_ANGLE) + "°");
  delay(1000);

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  logInfo("Receiver MAC: " + WiFi.macAddress());
  logInfo("Copy this MAC address to the transmitter code!");

  if (esp_now_init() != ESP_OK) {
    logError("ESP-NOW init failed");
    return;
  }
  logInfo("ESP-NOW initialized");

  // Register receive callback
  esp_now_register_recv_cb(onDataRecv);
  
  logInfo("Waiting for data from transmitter...");
  lastReceiveTime = millis();
  lastStatsTime = millis();
}

// ===== Main Loop =====
void loop() {
  // Check for connection timeout
  checkTimeout();
  
  // Print periodic statistics
  printStatistics();
  
  delay(TIMEOUT_CHECK_INTERVAL);
}