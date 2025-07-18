#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22

// Explicitly define I2C addresses
#define MPU6050_ADDR 0x68
#define MPU9250_ADDR 0x69 // MPU6500 at 0x69 (AD0 high)

// MPU6500 (yaw via gyroscope)
MPU9250_asukiaaa mySensorMPU6500(MPU9250_ADDR);

// MPU6050 (pitch)
MPU6050 mySensorMPU6050(MPU6050_ADDR);

float yaw, pitch;
float yawGyro = 0;
float gyroZOffset = 0;
unsigned long lastTime = 0;

// For moving average filter
#define FILTER_SIZE 5
float gyroZBuffer[FILTER_SIZE] = {0};
int filterIndex = 0;

// MAC address of receiver ESP32
uint8_t receiverAddress[] = { 0x14, 0x33, 0x5C, 0x03, 0xF2, 0xB0 };

// Struct to send orientation data
typedef struct {
  float yaw;
  float pitch;
} OrientationData;

OrientationData dataToSend;

bool mpu6500Ok = false; // Track MPU6500 status
bool mpu6050Ok = false; // Track MPU6050 status

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize MPU6500 (accelerometer and gyroscope)
  mySensorMPU6500.setWire(&Wire);
  mySensorMPU6500.beginAccel();
  mySensorMPU6500.beginGyro();
  delay(100);

  // Test MPU6500 connection by reading WHO_AM_I register
  Wire.beginTransmission(MPU9250_ADDR); // MPU9250_ADDR = 0x69 (AD0 high)
  Wire.write(0x75); // WHO_AM_I register
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU9250_ADDR, 1) == 1) {
    uint8_t whoAmI = Wire.read();
    Serial.print("MPU6500 WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    // Expect 0x70 (MPU6500), 0x71 (MPU9250), or 0x73 (MPU9255)
    if (whoAmI == 0x70) {
      mpu6500Ok = true;
      Serial.println("MPU6500 connected successfully. No magnetometer available, using gyroscope for yaw.");
    } else if (whoAmI == 0x71 || whoAmI == 0x73) {
      Serial.println("Warning: Detected MPU9250/MPU9255 (0x" + String(whoAmI, HEX) + "), but module labeled MPU6500. Magnetometer may be unavailable.");
      mpu6500Ok = true;
    } else {
      Serial.println("MPU6500 not connected properly: invalid WHO_AM_I (0x" + String(whoAmI, HEX) + ")!");
      mpu6500Ok = false;
    }
  } else {
    Serial.println("MPU6500 not connected properly: I2C communication failed!");
    mpu6500Ok = false;
  }

  // Calibrate gyroscope offset if connected
  if (mpu6500Ok) {
    Serial.println("Calibrating gyroscope... Keep sensor stationary.");
    float sumGyroZ = 0;
    int samples = 500; // Increased samples for better accuracy
    for (int i = 0; i < samples; i++) {
      mySensorMPU6500.gyroUpdate();
      sumGyroZ += mySensorMPU6500.gyroZ();
      delay(2); // Reduced delay for faster calibration
    }
    gyroZOffset = sumGyroZ / samples;
    Serial.print("Gyro Z offset: ");
    Serial.println(gyroZOffset);
    if (abs(gyroZOffset) > 10) {
      Serial.println("Warning: Large gyroscope offset detected. Recalibrate in a stable environment.");
    }
    lastTime = micros();
  }

  // Initialize MPU6050 (pitch)
  mySensorMPU6050.initialize();
  if (!mySensorMPU6050.testConnection()) {
    Serial.println("MPU6050 not connected properly!");
    mpu6050Ok = false;
    while (1); // Halt if MPU6050 fails
  } else {
    mpu6050Ok = true;
    Serial.println("MPU6050 connected successfully.");
  }

  delay(100);
}

void loop() {
  // Check for yaw reset command
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "reset") {
      yawGyro = 0;
      Serial.println("Yaw reset to 0.");
    }
  }

  // Yaw from MPU6500 gyroscope
  if (mpu6500Ok) {
    mySensorMPU6500.gyroUpdate();
    float gyroZ = mySensorMPU6500.gyroZ() - gyroZOffset;

    // Apply moving average filter to gyroZ
    gyroZBuffer[filterIndex] = gyroZ;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    float gyroZFiltered = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      gyroZFiltered += gyroZBuffer[i];
    }
    gyroZFiltered /= FILTER_SIZE;

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0; // Seconds
    lastTime = currentTime;

    // Integrate filtered gyroZ for yaw
    yawGyro += gyroZFiltered * deltaTime;
    if (yawGyro >= 360) yawGyro -= 360;
    if (yawGyro < 0) yawGyro += 360;
    yaw = yawGyro;

    // Check for valid gyroscope data
    if (abs(gyroZ) > 1000) { // Unrealistic angular velocity
      Serial.println("MPU6500 gyroscope data invalid: angular velocity too large!");
      yaw = 0;
    }
  } else {
    Serial.println("MPU6500 not available, yaw set to 0.");
    yaw = 0;
  }

  // Pitch from MPU6050
  if (mpu6050Ok) {
    int16_t ax6050, ay6050, az6050;
    mySensorMPU6050.getAcceleration(&ax6050, &ay6050, &az6050);

    // Convert to g (assuming ±2g setting, 16384 LSB/g)
    float fax = ax6050 / 16384.0;
    float fay = ay6050 / 16384.0;
    float faz = az6050 / 16384.0;

    // Check for valid accelerometer data
    float magnitude = sqrt(fax * fax + fay * fay + faz * faz);
    if (magnitude < 0.5 || magnitude > 1.5) {
      Serial.println("MPU6050 accelerometer data invalid: magnitude out of range!");
      pitch = 0;
    } else {
      pitch = atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / PI;
    }
  } else {
    Serial.println("MPU6050 not available, pitch set to 0.");
    pitch = 0;
  }

  // Debug output
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Pitch: ");
  Serial.println(pitch);

  // Send data
  dataToSend.yaw = yaw;
  dataToSend.pitch = pitch;
  esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

  delay(100);
}