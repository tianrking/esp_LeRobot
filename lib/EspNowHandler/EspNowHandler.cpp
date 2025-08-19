#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

// --- 新增：引入舵機驅動庫 ---
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// --- 新增：舵機硬體配置 ---
#define SERVO_BAUDRATE 1000000 
#define SERVO_TX_PIN D2
#define SERVO_RX_PIN D3
HardwareSerial servoSerial(1);
FSUS_Protocol protocol(&servoSerial, SERVO_BAUDRATE);

// 內部使用的變數和函數
static ConfigManager* pConfigManager = NULL;
static ArmStateData currentArmState; // <-- 修改：使用新的數據結構
static FSUS_Servo* servos[NUM_SERVOS]; // <-- 新增：舵機對象指針數組
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

String macToString(const uint8_t* mac) { /* ... (不變) ... */ }

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("ESP-NOW 發送失敗，狀態碼: "); Serial.println(status);
  }
}

// --- 修改：OnDataRecv 現在會解析新的 ArmStateData 結構體 ---
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    memcpy(&currentArmState, incomingData, sizeof(currentArmState));
    Serial.print("收到來自 Leader 的廣播數據: ");
    
    // 遍歷打印所有收到的角度
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.printf("[ID%d: %.1f] ", i, currentArmState.angles[i]);
    }
    Serial.println();
    
    // =======================================================
    // Follower 在這裡根據收到的 currentArmState.angles 數組
    // 來驅動自己的舵機，實現跟隨動作
    // =======================================================
  }
}

void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();

  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_max_tx_power(80) == ESP_OK) {
    Serial.println("最大發射功率已設定為 +20dBm。");
  }

  if (esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("設置 Wi-Fi 頻道失敗"); vTaskDelete(NULL);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗"); ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("角色: Leader - 初始化舵機...");
    
    // --- 新增：在 Leader 模式下初始化所有舵機 ---
    servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i] = new FSUS_Servo(i, &protocol);
      servos[i]->init();
      if (!servos[i]->ping()) {
         Serial.printf("警告：舵機 #%d 未響應！\n", i);
      }
    }
    Serial.println("舵機初始化完成。");

    // 添加廣播夥伴
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = currentConfig.channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("添加廣播夥伴失敗");
    }
  } else {
    Serial.println("角色: Follower - 準備接收廣播");
  }

  // 任務主循環
  for (;;) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      // --- 修改：從真實舵機讀取角度並廣播 ---
      Serial.print("Leader 正在讀取並廣播角度: ");
      for (int i = 0; i < NUM_SERVOS; i++) {
        currentArmState.angles[i] = servos[i]->queryAngle();
        Serial.printf("[ID%d: %.1f] ", i, currentArmState.angles[i]);
      }
      Serial.print("\r"); // 回到行首刷新

      esp_now_send(broadcastAddress, (uint8_t *)&currentArmState, sizeof(currentArmState));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 以 10Hz 的頻率更新
  }
}

void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(espNowTask, "ESP-NOW Task", 4096, NULL, 2, NULL);
}