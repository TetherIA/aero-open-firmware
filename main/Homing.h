// Copyright 2025 TetherIA, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <Arduino.h>
#include <HLSCL.h>

struct ServoData {
  uint16_t grasp_count;
  uint16_t extend_count;
  int8_t   servo_direction;
};

// Active (mutable) working copy used by the firmware:
extern ServoData sd[7];

// Immutable baselines (compile-time constants):
extern const ServoData sd_base_left[7];
extern const ServoData sd_base_right[7];

// Homing API
bool HOMING_isBusy();
void HOMING_start();

// Utility
void resetSdToBaseline();                 // copy correct baseline -> sd

// Provided elsewhere:
extern HLSCL hlscl;
extern SemaphoreHandle_t gBusMux;

// Use the fixed, 7-element servo ID list from your main sketch:
extern const uint8_t SERVO_IDS[7];     // e.g., {0,1,2,3,4,5,6}
