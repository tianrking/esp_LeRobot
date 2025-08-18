#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

// 內部使用的變數和函數
static ConfigManager* pConfigManager = NULL;
static MotionData currentMotion;

// MAC地址 <-> 字符串 轉換工具
String macToString(const uint8_t* mac) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void stringToMac(String macStr, uint8_t* mac) {
  sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW 發送狀態: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失敗");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&currentMotion, incomingData, sizeof(currentMotion));
  Serial.print("收到來自 ");
  Serial.print(macToString(mac));
  Serial.println(" 的數據");
  Serial.printf("Motor1: %d, Motor2: %d, Gripper: %s\n", 
                currentMotion.motor1_angle, currentMotion.motor2_angle, currentMotion.gripper_closed ? "閉合" : "張開");
}

void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  peerInfo.channel = currentConfig.channel;
  peerInfo.encrypt = false;

  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("角色: Leader - 準備廣播");
    char* macs_copy = strdup(currentConfig.peer_macs_str);
    char* mac_ptr = strtok(macs_copy, "\r\n");
    while (mac_ptr != NULL) {
      String mac_str = String(mac_ptr);
      mac_str.trim();
      if (mac_str.length() == 17) {
        Serial.print("添加 Follower: "); Serial.println(mac_str);
        stringToMac(mac_str, peerInfo.peer_addr);
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          Serial.println("添加夥伴失敗");
        }
      }
      mac_ptr = strtok(NULL, "\r\n");
    }
    free(macs_copy);
  } else {
    Serial.println("角色: Follower - 準備接收");
    String mac_str = String(currentConfig.peer_macs_str);
    mac_str.trim();
    if (mac_str.length() == 17) {
        Serial.print("添加 Leader: "); Serial.println(mac_str);
        stringToMac(mac_str, peerInfo.peer_addr);
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          Serial.println("添加夥伴失敗");
        }
    }
  }

  for (;;) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      currentMotion.motor1_angle = random(0, 181);
      currentMotion.motor2_angle = random(0, 181);
      currentMotion.gripper_closed = !currentMotion.gripper_closed;

      esp_now_send(0, (uint8_t *)&currentMotion, sizeof(currentMotion));
      Serial.printf("Leader 廣播數據: M1=%d, M2=%d\n", currentMotion.motor1_angle, currentMotion.motor2_angle);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(
      espNowTask, "ESP-NOW Task", 4096, NULL, 1, NULL
  );
}