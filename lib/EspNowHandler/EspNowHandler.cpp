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
#define SERVO_TX_PIN D2
#define SERVO_RX_PIN D3
HardwareSerial servoSerial(1);
FSUS_Protocol protocol(&servoSerial, SERVO_BAUDRATE);

// 內部使用的變數
static ConfigManager* pConfigManager = NULL;
static ArmStateData armStateBuffer;
static FSUS_Servo* servos[NUM_SERVOS];
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// FreeRTOS 相關句柄
static QueueHandle_t espNowQueue;
static SemaphoreHandle_t serialMutex;

/**
 * @brief 構建並通過串口發送帶有校驗的數據幀
 */
void sendSerialData(const ArmStateData& armState, const char* role) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    String payload = (strcmp(role, "Leader") == 0) ? "0" : "1";
    for (int i = 0; i < NUM_SERVOS; i++) {
      payload += ",";
      payload += String(armState.angles[i], 1);
    }
    uint8_t checksum = 0;
    for (int i = 0; i < payload.length(); i++) {
      checksum ^= payload[i];
    }
    Serial.print(">");
    Serial.print(payload);
    Serial.print("*");
    Serial.printf("%02X\n", checksum);
    xSemaphoreGive(serialMutex);
  }
}

/**
 * @brief ESP-NOW 數據發送完成後的回調函數 (ISR 安全)
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 為了避免在高頻發送時串口輸出過多訊息，通常保持此處簡潔
  if (status != ESP_NOW_SEND_SUCCESS) {
    // 可以在此處設置一個全域錯誤標誌位，由主任務讀取
  }
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數 (ISR 安全)
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    // 將數據發送到隊列中，讓主任務去處理，避免在回調中做過多工作
    xQueueSend(espNowQueue, incomingData, 0);
  }
}

/**
 * @brief ESP-NOW 工作任務的主體函數
 */
void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();
  
  // =======================================================
  // --- 新增：恢復被遺漏的 Wi-Fi 和 ESP-NOW 初始化 ---
  // =======================================================
  // 1. 設置 Wi-Fi 模式和發射功率
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_max_tx_power(80) != ESP_OK) {
      Serial.println("E,SET_POWER_FAIL");
  }

  // 2. 設置 Wi-Fi 頻道
  if (esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("E,SET_CHANNEL_FAIL");
      vTaskDelete(NULL);
  }

  // 3. (最關鍵) 初始化 ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("E,ESPNOW_INIT_FAIL");
    ESP.restart();
    return; // 雖然會重啟，但加上 return 避免後續程式碼執行
  }

  // 4. 註冊回調函數
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  // =======================================================
  // --- 初始化結束 ---
  // =======================================================


  // 根據角色配置 ESP-NOW 和硬件
  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("I,ROLE,LEADER");
    servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i] = new FSUS_Servo(i, &protocol);
        servos[i]->init();
        if(!servos[i]->ping()) { Serial.printf("W,SERVO_PING_FAIL,%d\n", i); }
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
      // Leader: 讀取舵機 -> 廣播 -> 打印到串口
      for (int i = 0; i < NUM_SERVOS; i++) {
        armStateBuffer.angles[i] = servos[i]->queryAngle();
      }
      esp_now_send(broadcastAddress, (uint8_t *)&armStateBuffer, sizeof(armStateBuffer));
      sendSerialData(armStateBuffer, "Leader");
      vTaskDelay(pdMS_TO_TICKS(100));

    } else { // Follower
      // Follower: 等待從隊列接收數據，然後打印和執行動作
      if (xQueueReceive(espNowQueue, &armStateBuffer, portMAX_DELAY) == pdPASS) {
        sendSerialData(armStateBuffer, "Follower");
        
        // =======================================================
        // 在這裡添加驅動 Follower 機械臂舵機運動的真實程式碼
        // =======================================================
      }
    }
  }
}

/**
 * @brief 啟動 ESP-NOW 任務的外部接口
 */
void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;

  // 創建隊列和互斥鎖
  espNowQueue = xQueueCreate(10, sizeof(ArmStateData));
  serialMutex = xSemaphoreCreateMutex();

  if (espNowQueue == NULL || serialMutex == NULL) {
    Serial.println("E,RTOS_CREATE_FAIL");
    return;
  }
  
  xTaskCreate(espNowTask, "ESP-NOW Task", 4096, NULL, 2, NULL);
}