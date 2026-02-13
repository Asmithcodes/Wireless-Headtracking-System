#ifndef CONFIG_H
#define CONFIG_H

// ===== Serial Configuration =====
#define SERIAL_BAUD 115200

// ===== Servo Configuration =====
#define SERVO_YAW_PIN 18         // Base servo (left/right)
#define SERVO_PITCH_PIN 19       // Upper servo (up/down)
#define SERVO_MIN_PULSE 1000     // Microseconds
#define SERVO_MAX_PULSE 2000     // Microseconds
#define SERVO_CENTER_ANGLE 90    // Degrees

// ===== Input Range Configuration =====
#define YAW_INPUT_MIN 0
#define YAW_INPUT_MAX 180
#define PITCH_INPUT_MIN -90
#define PITCH_INPUT_MAX 90

// ===== Output Range Configuration =====
#define SERVO_OUTPUT_MIN 0
#define SERVO_OUTPUT_MAX 180

// ===== Smoothing Configuration =====
#define SMOOTHING_FACTOR 0.3     // 0.0 = maximum smoothing (ignores new data), 1.0 = no smoothing (instant response)

// ===== Timeout Configuration =====
#define TIMEOUT_MS 1000          // Milliseconds without data before timeout
#define TIMEOUT_CHECK_INTERVAL 50 // Check timeout every 50ms

// ===== Statistics Configuration =====
#define STATS_INTERVAL_MS 5000   // Print statistics every 5 seconds

// ===== Firmware Version =====
#define FIRMWARE_VERSION "1.1.0"

#endif
