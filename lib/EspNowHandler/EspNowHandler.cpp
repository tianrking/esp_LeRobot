#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

// 內部使用的變數和函數
static ConfigManager* pConfigManager = NULL;
static MotionData currentMotion;
// 定義廣播地址，所有 Leader 都將向這個地址發送數據
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/**
 * @brief 將 uint8_t 類型的 MAC 地址數組轉換為易於閱讀的 String
 * @param mac 指向 MAC 地址數組的指針
 * @return 格式化的 MAC 地址字符串
 */
String macToString(const uint8_t* mac) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

/**
 * @brief ESP-NOW 數據發送完成後的回調函數
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW 發送狀態: ");
  Serial.print(macToString(mac_addr));
  Serial.print(" -> ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失敗");
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // 檢查數據長度是否匹配，增加健壯性
  if (len == sizeof(MotionData)) {
    memcpy(&currentMotion, incomingData, sizeof(currentMotion));
    Serial.print("收到來自 ");
    Serial.print(macToString(mac));
    Serial.println(" 的廣播數據");
    Serial.printf("  -> Motor1: %d, Motor2: %d, Gripper: %s\n", 
                  currentMotion.motor1_angle, 
                  currentMotion.motor2_angle, 
                  currentMotion.gripper_closed ? "閉合" : "張開");
    
    // =======================================================
    // 在這裡添加驅動 Follower 機械臂舵機運動的真實程式碼
    // 例如:
    // servo1.write(currentMotion.motor1_angle);
    // servo2.write(currentMotion.motor2_angle);
    // =======================================================
  } else {
    Serial.printf("收到長度不符的數據: %d bytes\n", len);
  }
}

/**
 * @brief ESP-NOW 工作任務的主體函數
 * @param parameter 指向 ConfigManager 實例的指針
 */
void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();

  // 1. 設置 Wi-Fi 為 STA 模式，這是 ESP-NOW 的最佳實踐
  WiFi.mode(WIFI_STA);
  // 設置 Wi-Fi 頻道，Leader 和 Follower 必須在同一頻道
  if (esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("設置 Wi-Fi 頻道失敗");
      vTaskDelete(NULL);
  }

  // 2. 初始化 ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    ESP.restart(); // 如果核心功能初始化失敗，重啟可能是最佳選擇
  }

  // 3. 註冊回調函數
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // 4. 根據角色配置 ESP-NOW
  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("角色: Leader - 設置為廣播模式");
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = currentConfig.channel;
    peerInfo.encrypt = false; // 廣播模式不能加密

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("添加廣播夥伴失敗");
    } else {
      Serial.println("廣播夥伴已添加。");
    }
  } else {
    Serial.println("角色: Follower - 設置為無差別監聽模式 (訂閱廣播)");
    // Follower 不需要添加任何 peer，即可接收廣播
  }

  // 5. 進入任務主循環
  for (;;) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      // Leader 的邏輯：定期生成並廣播運動數據
      // 實際應用中，這裡的數據應來自 BLE 或其他控制源
      currentMotion.motor1_angle = random(0, 181);
      currentMotion.motor2_angle = random(0, 181);
      currentMotion.gripper_closed = !currentMotion.gripper_closed;

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &currentMotion, sizeof(currentMotion));
      
      if (result == ESP_OK) {
        Serial.printf("Leader 廣播數據: M1=%d, M2=%d\n", currentMotion.motor1_angle, currentMotion.motor2_angle);
      } else {
        Serial.println("廣播發送出錯");
      }
    }
    
    // Follower 在 loop 中無需做事，完全由 OnDataRecv 回調驅動
    // Leader 也可在這裡執行其他任務
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * @brief 啟動 ESP-NOW 任務的外部接口
 * @param configMgr 對 ConfigManager 實例的引用
 */
void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(
      espNowTask,       // 任務函數
      "ESP-NOW Task",   // 任務名稱
      4096,             // 堆疊大小 (Stack size)
      NULL,             // 傳遞給任務的參數
      2,                // 任務優先級 (設為 2，高於預設的 1)
      NULL              // 任務句柄
  );
}