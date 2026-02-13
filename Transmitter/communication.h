#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// Struct to send orientation data wirelessly
typedef struct {
  float yaw;
  float pitch;
  unsigned long timestamp;  // Added for stale data detection
} OrientationData;

#endif
