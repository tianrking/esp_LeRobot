#pragma once

#include "ConfigManager.h" 

// --- 修改：定義舵機數量和新的數據結構 ---
const int NUM_SERVOS = 7; // 控制 0-6 號，共 7 個舵機

// 這個結構體將在 Leader 和 Follower 之間傳遞
struct ArmStateData {
  float angles[NUM_SERVOS]; // 一個浮點數數組，存儲所有舵機的角度
};
// --- 修改結束 ---

// 啟動 ESP-NOW 任務的函數 (不變)
void startEspNowTask(ConfigManager& configMgr);