#pragma once

#include "ConfigManager.h" 

const int NUM_SERVOS = 7;

struct ArmStateData {
  float angles[NUM_SERVOS];
};

void startEspNowTask(ConfigManager& configMgr);