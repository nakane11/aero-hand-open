#pragma once
#include <Arduino.h>
#include <HLSCL.h>

struct ServoData {
  uint16_t grasp_count;
  uint16_t extend_count;
  int8_t   servo_direction;
};

// 自作ハンド用の設定（ファイルに書くイメージでここを調整してください）
const int MY_DIRECTIONS[7] = { -1, 1,  1,  -1, 1,  -1, 1}; // 1:正転, -1:逆転
const int MY_LIMIT_PULSES[7] = {720, 1500, 1350, 1800, 1800, 1750, 1800}; // 可動範囲（パルス幅。最大4095）

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
