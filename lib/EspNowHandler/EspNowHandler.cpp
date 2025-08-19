#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "EspNowHandler.h"

// 引入舵機驅動庫
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// --- 舵機硬體配置 (依照您的要求修改) ---
#define SERVO_BAUDRATE 1000000      // <-- 修改: 波特率設定為 1Mbps
#define SERVO_TX_PIN 18             // <-- 修改: 使用 Arduino 板級引腳名 D2
#define SERVO_RX_PIN 19             // <-- 修改: 使用 Arduino 板級引腳名 D3
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
    for (size_t i = 0; i < payload.length(); i++) {
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
 * @brief ESP-NOW 數據發送完成後的回調函數
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    if (xSemaphoreTake(serialMutex, 0) == pdTRUE) { // Try to take mutex without waiting
        Serial.println("E,SEND_FAIL");
        xSemaphoreGive(serialMutex);
    }
  }
}

/**
 * @brief ESP-NOW 接收到數據時的回調函數
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ArmStateData)) {
    xQueueSend(espNowQueue, incomingData, 0);
  }
}

/**
 * @brief ESP-NOW 工作任務的主體函數
 */
void espNowTask(void *parameter) {
  Config currentConfig = pConfigManager->getConfig();
  
  // 初始化 Wi-Fi 和 ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(80);
  esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("E,ESPNOW_INIT_FAIL");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // 初始化所有舵機，無論是 Leader 還是 Follower
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.printf("I,ROLE,%s\n", currentConfig.role);
    servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i] = new FSUS_Servo(i, &protocol);
        servos[i]->init();
        if(strcmp(currentConfig.role, "Leader") == 0 && !servos[i]->ping()) { 
          Serial.printf("W,SERVO_PING_FAIL,%d\n", i); 
        }
    }
    Serial.println("I,SERVOS_INIT_DONE");
    xSemaphoreGive(serialMutex);
  }

  // Leader 需要添加廣播夥伴
  if (strcmp(currentConfig.role, "Leader") == 0) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = currentConfig.channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { 
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.println("E,ADD_PEER_FAIL");
            xSemaphoreGive(serialMutex);
        }
    }
  }

  // 任務主循環
  for (;;) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      // Leader: 讀取舵機 -> 廣播 -> 打印到串口
      for (int i = 0; i < NUM_SERVOS; i++) {
        float angle = servos[i]->queryAngle();
        if (angle < -185.0 || angle > 185.0) {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("W,SERVO_READ_ERR,ID=%d,Value=%.1f\n", i, angle);
                xSemaphoreGive(serialMutex);
            }
        } else {
             armStateBuffer.angles[i] = angle;
        }
      }
      esp_now_send(broadcastAddress, (uint8_t *)&armStateBuffer, sizeof(armStateBuffer));
      sendSerialData(armStateBuffer, "Leader");
      vTaskDelay(pdMS_TO_TICKS(100));

    } else { // Follower
      // Follower: 等待從隊列接收數據，然後驅動舵機和打印
      if (xQueueReceive(espNowQueue, &armStateBuffer, portMAX_DELAY) == pdPASS) {
        // 先將指令下發給舵機，確保動作優先
        for (int i = 0; i < NUM_SERVOS; i++) {
          servos[i]->setRawAngle(armStateBuffer.angles[i]);
        }
        // 然後再打印日誌
        sendSerialData(armStateBuffer, "Follower");
      }
    }
  }
}

/**
 * @brief 啟動 ESP-NOW 任務的外部接口
 */
void startEspNowTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  espNowQueue = xQueueCreate(10, sizeof(ArmStateData));
  serialMutex = xSemaphoreCreateMutex();
  if (espNowQueue == NULL || serialMutex == NULL) {
    Serial.println("E,RTOS_CREATE_FAIL");
    return;
  }
  xTaskCreate(espNowTask, "ESP-NOW Task", 4096, NULL, 2, NULL);
}