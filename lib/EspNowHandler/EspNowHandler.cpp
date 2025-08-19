#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

// 引入舵機驅動庫
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// 舵機硬體配置
#define SERVO_BAUDRATE 1000000 
#define SERVO_TX_PIN 2
#define SERVO_RX_PIN 3
HardwareSerial servoSerial(1);
FSUS_Protocol protocol(&servoSerial, SERVO_BAUDRATE);

// 內部使用的變數
static ConfigManager* pConfigManager = NULL;
static ArmStateData currentArmState;
static FSUS_Servo* servos[NUM_SERVOS];
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


/**
 * @brief 構建並通過 USB 串口發送標準的 11 字節數據幀
 * @param armState 包含當前機械臂所有角度的數據結構
 * @param role 當前設備的角色 ("Leader" 或 "Follower")
 */
void sendSerialData(const ArmStateData& armState, const char* role) {
  uint8_t serialPacket[11]; // 11 字節的數據包

  // Byte 0: 設置幀頭
  serialPacket[0] = 0xAA;

  // Byte 1: 設置身份 (0x00 = Leader, 0x01 = Follower)
  serialPacket[1] = (strcmp(role, "Leader") == 0) ? 0x00 : 0x01;

  // Byte 2-8: 填充 7 個舵機的角度數據
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 將 float 角度轉換為 uint8_t (0-180)，並防止數據溢出
    serialPacket[i + 2] = (uint8_t)constrain(round(armState.angles[i]), 0, 180);
  }

  // Byte 9: 計算校驗和 (對字節 1 到 8 進行累加)
  uint8_t checksum = 0;
  for (int i = 1; i <= 8; i++) {
    checksum += serialPacket[i];
  }
  serialPacket[9] = checksum;

  // Byte 10: 設置幀尾
  serialPacket[10] = 0xBB;

  // 通過串口以二進制形式寫出整個數據包
  Serial.write(serialPacket, 11);
}

/**
 * @brief ESP-NOW 數據發送完成後的回調函數
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 為了避免在高頻發送時串口輸出過多訊息，只在失敗時打印
  if (status != ESP_NOW_SEND_SUCCESS) {
    // 身份標籤只在需要時添加，保持常規運行時的串口乾淨
    Serial.println("[Leader] ESP-NOW 發送失敗!");
  }
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    memcpy(&currentArmState, incomingData, sizeof(currentArmState));
    
    // Follower 收到數據後，通過串口發送符合協議的數據幀
    sendSerialData(currentArmState, "Follower");

    // =======================================================
    // 在這裡添加驅動 Follower 機械臂舵機運動的真實程式碼
    // =======================================================
  }
}

/**
 * @brief ESP-NOW 工作任務的主體函數
 */
void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();

  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_max_tx_power(80) != ESP_OK) {
    Serial.println("設定最大發射功率失敗！");
  }

  if (esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("設置 Wi-Fi 頻道失敗");
      vTaskDelete(NULL);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("[Leader] 初始化舵機...");
    servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i] = new FSUS_Servo(i, &protocol);
      servos[i]->init();
      if (!servos[i]->ping()) {
         Serial.printf("警告：舵機 #%d 未響應！\n", i);
      }
    }
    Serial.println("[Leader] 舵機初始化完成。");

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = currentConfig.channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[Leader] 添加廣播夥伴失敗");
    }
  } else {
    Serial.println("[Follower] 設置為監聽模式。");
  }

  // 任務主循環
  for (;;) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      // 1. 從真實舵機讀取角度
      for (int i = 0; i < NUM_SERVOS; i++) {
        currentArmState.angles[i] = servos[i]->queryAngle();
      }
      
      // 2. 通過 ESP-NOW 廣播
      esp_now_send(broadcastAddress, (uint8_t *)&currentArmState, sizeof(currentArmState));

      // 3. Leader 也通過串口發送符合協議的數據幀
      sendSerialData(currentArmState, "Leader");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 以 10Hz 的頻率更新
  }
}

/**
 * @brief 啟動 ESP-NOW 任務的外部接口
 */
void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(
      espNowTask,       // 任務函數
      "ESP-NOW Task",   // 任務名稱
      4096,             // 堆疊大小 (Stack size)
      NULL,             // 傳遞給任務的參數
      2,                // 任務優先級
      NULL              // 任務句柄
  );
}