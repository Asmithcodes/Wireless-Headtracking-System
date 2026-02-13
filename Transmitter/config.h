#ifndef CONFIG_H
#define CONFIG_H

// ===== Serial Configuration =====
#define SERIAL_BAUD 115200

// ===== I2C Configuration =====
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68
#define MPU9250_ADDR 0x69  // MPU6500 at 0x69 (AD0 high)

// ===== Sensor Configuration =====
#define FILTER_SIZE 5                     // Moving average filter size
#define GYRO_CALIBRATION_SAMPLES 500      // Samples for gyroscope calibration
#define MAX_GYRO_OFFSET 10.0              // Maximum acceptable gyro offset (deg/s)
#define MAX_VALID_ANGULAR_VELOCITY 1000.0 // Maximum valid gyro reading (deg/s)
#define ACCEL_SENSITIVITY 16384.0         // LSB/g for ±2g range
#define ACCEL_MIN_G 0.5                   // Minimum valid acceleration magnitude
#define ACCEL_MAX_G 1.5                   // Maximum valid acceleration magnitude

// ===== Orientation Limits =====
#define YAW_MIN 0
#define YAW_MAX 360
#define PITCH_MIN -90
#define PITCH_MAX 90

// ===== Timing Configuration =====
#define LOOP_DELAY_MS 100
#define UPDATE_RATE_HZ 10
#define CALIBRATION_DELAY_MS 2

// ===== Receiver MAC Address =====
// Update this with your receiver's MAC address
extern uint8_t receiverAddress[6];

// ===== Firmware Version =====
#define FIRMWARE_VERSION "1.1.0"

#endif
