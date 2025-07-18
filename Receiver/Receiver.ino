#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

typedef struct {
  float yaw;
  float pitch;
} OrientationData;

OrientationData incomingData;

#define SERVO_YAW_PIN 18    // Base servo (left/right)
#define SERVO_PITCH_PIN 19  // Upper servo (up/down)

Servo servoYaw;
Servo servoPitch;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(OrientationData)) {
    memcpy(&incomingData, data, sizeof(incomingData));

    Serial.print("Received -> Yaw: ");
    Serial.print(incomingData.yaw);
    Serial.print(" | Pitch: ");
    Serial.println(incomingData.pitch);

    float yaw = incomingData.yaw;
    float pitch = incomingData.pitch;

    // Map yaw (0–180°) to servo range (0 to 180)
    int yawVal = constrain(map((int)yaw, 0, 180, 180, 0), 0, 180);
    // Map pitch (–90 to 90°) to servo range (0 to 180)
    int pitchVal = constrain(map((int)pitch, -90, 90, 180, 0), 0, 180);

    servoYaw.write(yawVal);
    servoPitch.write(pitchVal);

    Serial.print("Mapped Yaw: ");
    Serial.print(yawVal);
    Serial.print(" | Mapped Pitch: ");
    Serial.println(pitchVal);
  } else {
    Serial.println("Received invalid data size");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500); // Wait for Serial Monitor

  // Attach servos with pulse width range (1000-2000 microseconds)
  servoYaw.attach(SERVO_YAW_PIN, 1000, 2000);
  servoPitch.attach(SERVO_PITCH_PIN, 1000, 2000);

  servoYaw.write(90);
  servoPitch.write(90);
  delay(1000);

  WiFi.mode(WIFI_STA);
  Serial.println("Receiver MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return; // Avoid infinite loop
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Nothing here
}