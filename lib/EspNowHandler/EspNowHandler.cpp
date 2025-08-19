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
 * @brief ESP-NOW 數據發送完成後的回調函數
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 在這個版本中，我們讓主循環處理所有串口輸出，所以這裡可以留空或只在出錯時打印
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("E,SEND_FAIL"); // 使用簡短的錯誤碼
  }
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    memcpy(&currentArmState, incomingData, sizeof(currentArmState));
    
    // --- 修改：Follower 收到數據後，打印 CSV 格式 ---
    Serial.print("1"); // 身份ID: 1 代表 Follower
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.print(",");
      Serial.print(currentArmState.angles[i], 1); // 打印角度，保留一位小數
    }
    Serial.println(); // 換行表示一條數據結束

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
    Serial.println("E,SET_POWER_FAIL");
  }

  if (esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("E,SET_CHANNEL_FAIL");
      vTaskDelete(NULL);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("E,ESPNOW_INIT_FAIL");
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("I,ROLE,LEADER"); // 使用簡短的訊息碼表示狀態
    servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i] = new FSUS_Servo(i, &protocol);
      servos[i]->init();
      if (!servos[i]->ping()) {
         Serial.printf("W,SERVO_PING_FAIL,%d\n", i); // W for Warning
      }
    }
    Serial.println("I,SERVOS_INIT_DONE");

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = currentConfig.channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("E,ADD_PEER_FAIL");
    }
  } else {
    Serial.println("I,ROLE,FOLLOWER");
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

      // --- 修改：Leader 也通過串口打印 CSV 格式 ---
      Serial.print("0"); // 身份ID: 0 代表 Leader
      for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print(",");
        Serial.print(currentArmState.angles[i], 1); // 打印角度，保留一位小數
      }
      Serial.println(); // 換行
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
      espNowTask, "ESP-NOW Task", 4096, NULL, 2, NULL
  );
}