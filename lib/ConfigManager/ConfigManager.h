#pragma once

#include <Arduino.h>
#include <Preferences.h>

// 用來存儲我們的配置信息
struct Config {
  char role[10];
  char peer_macs_str[256];
  int channel;
  bool configured;
};

class ConfigManager {
public:
  ConfigManager();
  bool load();
  void save();
  bool isConfigured();
  Config& getConfig();

private:
  Preferences preferences;
  Config currentConfig;
};