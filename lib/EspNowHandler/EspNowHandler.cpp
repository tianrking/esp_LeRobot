#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

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
static ArmStateData armStateBuffer; // 用於在任務和回調間傳遞數據
static FSUS_Servo* servos[NUM_SERVOS];
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- 新增: FreeRTOS 相關句柄 ---
static QueueHandle_t espNowQueue; // 用於從回調接收數據的隊列
static SemaphoreHandle_t serialMutex; // 用於保護串口訪問的互斥鎖

/**
 * @brief 構建並通過串口發送帶有校驗的數據幀
 */
void sendSerialData(const ArmStateData& armState, const char* role) {
  // 1. 嘗試獲取串口鎖，最多等待 10ms
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // 2. 構建核心數據字符串
    String payload = (strcmp(role, "Leader") == 0) ? "0" : "1";
    for (int i = 0; i < NUM_SERVOS; i++) {
      payload += ",";
      payload += String(armState.angles[i], 1);
    }

    // 3. 計算校驗和 (簡單的異或校驗)
    uint8_t checksum = 0;
    for (int i = 0; i < payload.length(); i++) {
      checksum ^= payload[i];
    }

    // 4. 打印完整數據幀
    Serial.print(">");
    Serial.print(payload);
    Serial.print("*");
    Serial.printf("%02X\n", checksum); // 以兩位十六進制打印校驗和

    // 5. 釋放串口鎖
    xSemaphoreGive(serialMutex);
  }
}

/**
 * @brief ESP-NOW 數據發送完成後的回調函數 (ISR 安全)
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    // 這裡盡量不要做 Serial 操作，但在調試時可以打開
    // Serial.println("E,SEND_FAIL");
  }
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數 (ISR 安全)
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    // 將數據發送到隊列中，讓主任務去處理。
    // xQueueSendFromISR 是中斷安全版本，但這裡 Wi-Fi 回調不是嚴格的中斷，用 xQueueSend 也可以
    xQueueSend(espNowQueue, incomingData, 0);
  }
}

/**
 * @brief ESP-NOW 工作任務的主體函數
 */
void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();
  // ... (Wi-Fi, ESP-NOW 初始化代碼不變) ...
  // ...
  
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
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("E,ADD_PEER_FAIL"); }
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
      // Follower: 等待從隊列接收數據，然後打印
      if (xQueueReceive(espNowQueue, &armStateBuffer, portMAX_DELAY) == pdPASS) {
        sendSerialData(armStateBuffer, "Follower");
        // 在這裡添加驅動 Follower 機械臂的真實程式碼
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
  espNowQueue = xQueueCreate(10, sizeof(ArmStateData)); // 創建一個能緩衝 10 個數據包的隊列
  serialMutex = xSemaphoreCreateMutex(); // 創建串口互斥鎖

  if (espNowQueue == NULL || serialMutex == NULL) {
    Serial.println("E,RTOS_CREATE_FAIL");
    return;
  }
  
  xTaskCreate(espNowTask, "ESP-NOW Task", 4096, NULL, 2, NULL);
}