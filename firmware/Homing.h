#pragma once
#include <Arduino.h>
#include <HLSCL.h>

struct ServoData {
  uint16_t grasp_count;
  uint16_t extend_count;
  int8_t   servo_direction;
};

// Active (mutable) working copy used by the firmware:
extern ServoData sd[15];

// Immutable baselines (compile-time constants):
extern const ServoData sd_base_left[15];
extern const ServoData sd_base_right[15];

// Homing API
bool HOMING_isBusy();
void HOMING_start();

// Utility
void resetSdToBaseline();                 // copy correct baseline -> sd
void applyTrim(uint8_t ch, int16_t d);    // optional helper used by TRIM command

// Provided elsewhere:
extern HLSCL hlscl;

// Use the fixed, 7-element servo ID list from your main sketch:
extern const uint8_t SERVO_IDS[7];     // e.g., {0,1,2,5,8,11,14}
