#include "Homing.h"

// ----------------------- Immutable baselines -----------------------
// Always define both symbols so linker can resolve them
const ServoData sd_base_left[15] = {
  {3145,2048,1},{2048,786,-1},{0,2820,1},{0,0,0},{0,0,0},
  {4095,815,-1},{0,0,0},{0,0,0},{4095,815,-1},{0,0,0},
  {0,0,0},{4095,815,-1},{0,0,0},{0,0,0},{4095,815,-1},
};

const ServoData sd_base_right[15] = {
  {910,2048,-1},{2048,3232,1},{4095,1275,-1},{0,0,0},{0,0,0},
  {0,3280,1},{0,0,0},{0,0,0},{0,3280,1},{0,0,0},
  {0,0,0},{0,3280,1},{0,0,0},{0,0,0},{0,3280,1},
};

// ----------------------- Mutable working copy -----------------------
ServoData sd[15];  // gets initialized from baseline at boot / before homing

// ----------------------- Utilities -----------------------
void resetSdToBaseline() {
#if defined(RIGHT_HAND)
  const ServoData* src = sd_base_right;
#elif defined(LEFT_HAND)
  const ServoData* src = sd_base_left;
#else
  #warning "No hand macro defined; defaulting to RIGHT_HAND baseline"
  const ServoData* src = sd_base_right;
#endif
  for (int i = 0; i < 15; ++i) sd[i] = src[i];
}

void applyTrim(uint8_t ch, int16_t delta) {
  if (ch > 14) return;
  int32_t v = (int32_t)sd[ch].extend_count + (int32_t)delta;
  if (v < 0) v = 0;
  if (v > 4095) v = 4095;
  sd[ch].extend_count = (uint16_t)v;
  // (Optional) persist to NVS here
}

// ----------------------- Busy flag & homing core -----------------------
static volatile bool s_busy = false;
bool HOMING_isBusy() { return s_busy; }

static void zero_with_current(uint8_t servoID, int channel, int direction, int current_limit) {
  int current = 0;
  int position = 0;

  hlscl.ServoMode(servoID);
  hlscl.FeedBack(servoID);

  uint32_t t0 = millis();
  if (direction == 1) {
    while (current < current_limit) {
      hlscl.WritePosEx(servoID, 50000 * direction, 10, 10, current_limit + 100);
      current  = hlscl.ReadCurrent(servoID);
      position = hlscl.ReadPos(servoID);
      if (millis() - t0 > 25000) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  } else {
    while (current > -current_limit) {
      hlscl.WritePosEx(servoID, 50000 * direction, 10, 10, current_limit + 100);
      current  = hlscl.ReadCurrent(servoID);
      position = hlscl.ReadPos(servoID);
      if (millis() - t0 > 25000) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  // Primary calibration at contact
  hlscl.WritePosEx(servoID, position, 60, 50, 500);
  delay(30);
  hlscl.CalibrationOfs(servoID);
  delay(50);
  position = hlscl.ReadPos(servoID);

  if (channel == 0) {
    // Thumb abduction: hold grasp posture for a moment
    hlscl.WritePosEx(servoID, sd[channel].grasp_count, 60, 50, 500);
    delay(2000);
  } else if (channel == 1) {
    // Thumb flexion: go to extend
    hlscl.WritePosEx(servoID, sd[channel].extend_count, 60, 50, 500);
    delay(1000);
  } else if (channel == 2) {
    // Thumb tendon: nudge and recalibrate, then extend
    hlscl.WritePosEx(servoID, position + (direction * 2048), 60, 50, 500);
    delay(1500);
    hlscl.CalibrationOfs(servoID);
    delay(500);
    hlscl.WritePosEx(servoID, sd[channel].extend_count, 60, 50, 500);
    delay(1000);
  } else {
    // Fingers: nudge and recalibrate, then extend
    hlscl.WritePosEx(servoID, position + (direction * 2048), 60, 50, 500);
    delay(1500);
    hlscl.CalibrationOfs(servoID);
    delay(500);
    hlscl.WritePosEx(servoID, sd[channel].extend_count, 60, 50, 500);
    delay(1000);
  }
}

void zero_all_motors() {
  resetSdToBaseline();

  zero_with_current(SERVO_IDS[0], 0,  sd[0].servo_direction, 850);   // Thumb Abduction
  zero_with_current(SERVO_IDS[1], 1,  sd[1].servo_direction, 850);   // Thumb Flex
  zero_with_current(SERVO_IDS[2], 2,  sd[2].servo_direction, 850);   // Thumb Tendon

  zero_with_current(SERVO_IDS[3], 5,  sd[5].servo_direction, 850);   // Index
  zero_with_current(SERVO_IDS[4], 8,  sd[8].servo_direction, 850);   // Middle
  zero_with_current(SERVO_IDS[5], 11, sd[11].servo_direction, 850);  // Ring
  zero_with_current(SERVO_IDS[6], 14, sd[14].servo_direction, 850);  // Pinky

  // Post-homing settling moves
  hlscl.WritePosEx(SERVO_IDS[0], sd[0].extend_count, 60, 50, 500);   // Thumb Abduction to extend
  hlscl.WritePosEx(SERVO_IDS[2], sd[2].extend_count, 60, 50, 500);   // Thumb Tendon to extend
}

void HOMING_start() {
  s_busy = true;
  zero_all_motors();
  s_busy = false;
}
