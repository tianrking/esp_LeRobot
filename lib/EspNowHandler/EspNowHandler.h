#pragma once

#include "ConfigManager.h" // 引用配置管理器

// 用來在 Leader 和 Follower 之間傳遞的數據結構
struct MotionData {
  int motor1_angle;
  int motor2_angle;
  bool gripper_closed;
};

// 啟動 ESP-NOW 任務的函數
void startEspNowTask(ConfigManager& configMgr);