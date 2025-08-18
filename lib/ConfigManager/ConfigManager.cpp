#include "ConfigManager.h"

const char* PREF_NAMESPACE = "robo_config";
const char* PREF_KEY_CONFIG = "config_data";

ConfigManager::ConfigManager() {
  // 構造函數中設定初始默認值
  strcpy(currentConfig.role, "Follower");
  strcpy(currentConfig.peer_macs_str, "");
  currentConfig.channel = 1;
  currentConfig.configured = false;
}

bool ConfigManager::load() {
  preferences.begin(PREF_NAMESPACE, true); // true = 只讀模式
  if (preferences.getBytesLength(PREF_KEY_CONFIG) == sizeof(Config)) {
    preferences.getBytes(PREF_KEY_CONFIG, &currentConfig, sizeof(Config));
    preferences.end();
    Serial.println("成功從 NVS 載入配置。");
    return true;
  }
  
  preferences.end();
  Serial.println("NVS 中未找到有效配置，使用默認值。");
  return false;
}

void ConfigManager::save() {
  preferences.begin(PREF_NAMESPACE, false); // false = 讀寫模式
  preferences.putBytes(PREF_KEY_CONFIG, &currentConfig, sizeof(Config));
  preferences.end();
  Serial.println("配置已成功保存到 NVS。");
}

bool ConfigManager::isConfigured() {
  return currentConfig.configured;
}

Config& ConfigManager::getConfig() {
  return currentConfig;
}