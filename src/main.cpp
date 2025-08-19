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

// =========== 硬體配置 ===========
// 為 Serial1 和 Serial2 分配 GPIO 引腳
// 這些是 ESP32-WROOM-32 DevKitC 上常用的安全引腳
#define RX1_PIN 18
#define TX1_PIN 19
#define RX2_PIN 16
#define TX2_PIN 17

void setup() {
  // 1. 初始化主串口 (UART0)，用於日誌和控制
  Serial.begin(115200);
  delay(1000); // 等待串口穩定
  Serial.println("\n--- ESP32 DevKit 三硬體串口並發測試 ---");
  
  // 2. 初始化串口1 (UART1)
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  Serial.println("  - Serial (UART0) on GPIO 1(TX)/3(RX) -> Ready (for Logging)");
  Serial.printf("  - Serial1 (UART1) on GPIO %d(TX)/%d(RX) -> Ready\n", TX1_PIN, RX1_PIN);

  // 3. 初始化串口2 (UART2)
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.printf("  - Serial2 (UART2) on GPIO %d(TX)/%d(RX) -> Ready\n", TX2_PIN, RX2_PIN);

  Serial.println("\n測試方法:");
  Serial.println("  1. 使用 USB-to-Serial 適配器連接到 Serial1 或 Serial2 的引腳。");
  Serial.println("  2. 在對應的串口助手中發送任何字符，設備會將其回顯。");
  Serial.println("  3. 在本窗口輸入 '1' 或 '2'，可以讓設備主動從 Serial1 或 Serial2 發送測試訊息。");
  Serial.println("----------------------------------------------------");
}

void loop() {
  static unsigned long lastHeartbeatTime = 0;

  // 任務1: 處理來自 USB 主串口的控制指令
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      String msg = "Hello from Serial1!";
      Serial1.println(msg);
      Serial.printf("[Control] Sent '%s' via Serial1\n", msg.c_str());
    } else if (c == '2') {
      String msg = "Hello from Serial2!";
      Serial2.println(msg);
      Serial.printf("[Control] Sent '%s' via Serial2\n", msg.c_str());
    }
  }

  // 任務2: 回顯來自 Serial1 的所有數據
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial1.write(c); // Echo back
    Serial.printf("[Serial1 RX] Received: %c\n", c);
  }

  // 任務3: 回顯來自 Serial2 的所有數據
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial2.write(c); // Echo back
    Serial.printf("[Serial2 RX] Received: %c\n", c);
  }

  // 任務4: 在主串口打印心跳，證明系統沒有崩潰
  if (millis() - lastHeartbeatTime > 2000) {
    lastHeartbeatTime = millis();
    Serial.println("[Heartbeat] System is running...");
  }
}