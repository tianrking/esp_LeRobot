// #include <Arduino.h>
// #include "ConfigManager.h"
// #include "Provisioning.h"
// #include "EspNowHandler.h"
// #include <WiFi.h>

// const char* CONFIG_HTML PROGMEM = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
//   <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
//   <title>機械臂配置</title>
//   <style>
//     body { font-family: Arial, sans-serif; background: #f4f4f4; margin: 40px; }
//     .container { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }
//     h2 { text-align: center; color: #333; } .form-group { margin-bottom: 15px; }
//     label { display: block; margin-bottom: 5px; font-weight: bold; }
//     input[type="number"] { width: 95%; padding: 10px; border-radius: 4px; border: 1px solid #ccc; }
//     input[type="radio"] { margin-right: 5px; }
//     .btn { background: #007bff; color: #fff; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-size: 16px; }
//     .btn:hover { background: #0056b3; } #status { margin-top: 15px; text-align: center; font-weight: bold; }
//   </style>
// </head>
// <body>
//   <div class="container">
//     <h2>機械臂配置</h2>
//     <p>本機 MAC 地址: <strong id="my_mac"></strong></p>
//     <form id="configForm">
//       <div class="form-group">
//         <label>設定角色:</label>
//         <input type="radio" id="leader" name="role" value="Leader"> <label for="leader">Leader (主機)</label>
//         <input type="radio" id="follower" name="role" value="Follower" checked> <label for="follower">Follower (從機)</label>
//       </div>
//       <div class="form-group">
//         <label for="channel">通訊頻道 (1-13):</label>
//         <input type="number" id="channel" name="channel" min="1" max="13" value="1" required>
//       </div>
//       <button type="submit" class="btn">保存並重啟</button>
//     </form>
//     <div id="status"></div>
//   </div>
//   <script>
//     document.getElementById('my_mac').textContent = '%MY_MAC%';
//     document.getElementById('configForm').addEventListener('submit', function(e) {
//       e.preventDefault(); const formData = new FormData(this); const statusDiv = document.getElementById('status');
//       statusDiv.textContent = '正在保存...';
//       fetch('/save', { method: 'POST', body: formData })
//       .then(response => response.text()).then(data => {
//         statusDiv.textContent = data; setTimeout(() => { statusDiv.textContent = '設備正在重啟...'; }, 2000);
//       }).catch(error => { statusDiv.textContent = '保存失敗: ' + error; });
//     });
//   </script>
// </body>
// </html>
// )rawliteral";

// ConfigManager configManager;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("\n=================================");
//   Serial.println("  機械臂控制系統啟動... v2.0 (RTOS)");
//   Serial.println("=================================");
  
//   configManager.load();
  
//   Serial.print("本機 MAC 地址: ");
//   Serial.println(WiFi.macAddress());

//   // 統一啟動配網任務，它內部會決定下一步做什麼
//   startProvisioningTask(configManager);

//   Serial.println("Setup 完成，控制權交給 FreeRTOS 任務。");
//   // 主 setup-loop 任務可以結束了
//   vTaskDelete(NULL);
// }

// void loop() {
//   // 此處完全留空
// }

#include <Arduino.h>

// 引入舵機驅動庫
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// =========== 舵機硬體配置 ===========
#define SERVO_BAUDRATE 1000000
const int NUM_SERVOS = 2;
uint8_t servoIDs[] = {0, 1};

// --- 總線1: Leader (輸入) ---
#define LEADER_SERVO_TX_PIN 19
#define LEADER_SERVO_RX_PIN 18

// --- 總線2: Follower (輸出) ---
#define FOLLOWER_SERVO_TX_PIN 17
#define FOLLOWER_SERVO_RX_PIN 16

// --- 為每個總線創建獨立的協議和舵機對象 ---
FSUS_Protocol leaderProtocol(&Serial1, SERVO_BAUDRATE);
FSUS_Servo* leaderServos[NUM_SERVOS];

FSUS_Protocol followerProtocol(&Serial2, SERVO_BAUDRATE);
FSUS_Servo* followerServos[NUM_SERVOS];

// =========== FreeRTOS 與共享數據 ===========
static float latest_angles[NUM_SERVOS];
static SemaphoreHandle_t angleMutex;

// --- 任務函數聲明 ---
void readerTask(void *parameter);
void writerTask(void *parameter);


void setup() {
  // 1. 初始化主串口 (UART0)，用於日誌
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32 DevKit 雙總線並行轉發測試 (最終優化版) ---");

  // 2. 初始化 Leader (輸入) 總線 (UART1)
  Serial1.begin(SERVO_BAUDRATE, SERIAL_8N1, LEADER_SERVO_RX_PIN, LEADER_SERVO_TX_PIN);
  for (int i = 0; i < NUM_SERVOS; i++) {
      leaderServos[i] = new FSUS_Servo(servoIDs[i], &leaderProtocol);
      leaderServos[i]->init();
  }

  // 3. 初始化 Follower (輸出) 總線 (UART2)
  Serial2.begin(SERVO_BAUDRATE, SERIAL_8N1, FOLLOWER_SERVO_RX_PIN, FOLLOWER_SERVO_TX_PIN);
  for (int i = 0; i < NUM_SERVOS; i++) {
      followerServos[i] = new FSUS_Servo(servoIDs[i], &followerProtocol);
      followerServos[i]->init();
  }

  // 4. 創建互斥鎖
  angleMutex = xSemaphoreCreateMutex();
  if (angleMutex == NULL) {
    Serial.println("創建 Mutex 失敗! 系統暫停。");
    while(1);
  }

  // 5. 創建並啟動兩個任務
  xTaskCreatePinnedToCore(
      readerTask, "Reader Task", 8192, NULL, 2, NULL, 0);

  xTaskCreatePinnedToCore(
      writerTask, "Writer Task", 8192, NULL, 1, NULL, 1);

  Serial.println("初始化完成，讀寫任務已在雙核上並行啟動...");
  Serial.println("------------------------------------------");
}

/**
 * @brief 任務一：高頻讀取 Leader 舵機角度 (增量更新邏輯)
 */
void readerTask(void *parameter) {
  const int DELAY_BETWEEN_SERVO_QUERIES_MS = 2;
  
  for (;;) {
    // 從總線1逐個讀取舵機的角度
    for (int i = 0; i < NUM_SERVOS; i++) {
      // 1. 查詢單個舵機
      float angle = leaderServos[i]->queryAngle();
      
      // 2. 驗證數據
      if (angle > -185.0 && angle < 185.0) {
        // 3. (關鍵修改) 獲取鎖，並立刻更新共享緩衝區中的這一個角度值
        if (xSemaphoreTake(angleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          latest_angles[i] = angle;
          xSemaphoreGive(angleMutex);
        }
      }
      
      // 4. 為總線穩定而延遲
      vTaskDelay(pdMS_TO_TICKS(DELAY_BETWEEN_SERVO_QUERIES_MS));
    }
  }
}

/**
 * @brief 任務二：高頻將最新角度寫入 Follower 舵機
 */
void writerTask(void *parameter) {
  const int CONTROL_UPDATE_INTERVAL_MS = 20; // 以 50Hz 的頻率更新 Follower

  for (;;) {
    float angles_to_write[NUM_SERVOS];

    // 獲取鎖，從共享緩衝區複製一份最新的、可能被增量更新過的數據
    if (xSemaphoreTake(angleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(angles_to_write, latest_angles, sizeof(angles_to_write));
      xSemaphoreGive(angleMutex);
    }
    
    // 將角度高速下發到總線2
    for (int i = 0; i < NUM_SERVOS; i++) {
      followerServos[i]->setRawAngle(angles_to_write[i]);
    }
    
    // (可選) 降低頻率打印日誌
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 100) {
        lastPrintTime = millis();
        String output_line = "Angles: ";
        for(int i=0; i<NUM_SERVOS; ++i){
            output_line += "[ID" + String(servoIDs[i]) + ":" + String(angles_to_write[i], 1) + "] ";
        }
        Serial.println(output_line);
    }

    // 控制寫入的整體頻率
    vTaskDelay(pdMS_TO_TICKS(CONTROL_UPDATE_INTERVAL_MS));
  }
}


void loop() {
  vTaskDelete(NULL);
}