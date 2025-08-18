#include <Arduino.h>
#include "ConfigManager.h"
#include "Provisioning.h"
#include "EspNowHandler.h"
#include <WiFi.h>

const char* CONFIG_HTML PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>機械臂配置</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f4f4f4; margin: 40px; }
    .container { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }
    h2 { text-align: center; color: #333; }
    .form-group { margin-bottom: 15px; }
    label { display: block; margin-bottom: 5px; font-weight: bold; }
    input[type="text"], input[type="number"], textarea { width: 95%; padding: 10px; border-radius: 4px; border: 1px solid #ccc; }
    input[type="radio"] { margin-right: 5px; }
    .btn { background: #007bff; color: #fff; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-size: 16px; }
    .btn:hover { background: #0056b3; }
    #status { margin-top: 15px; text-align: center; font-weight: bold; }
  </style>
</head>
<body>
  <div class="container">
    <h2>機械臂配置</h2>
    <p>本機 MAC 地址: <strong id="my_mac"></strong></p>
    <form id="configForm">
      <div class="form-group">
        <label>設定角色:</label>
        <input type="radio" id="leader" name="role" value="Leader" checked>
        <label for="leader">Leader (主機)</label>
        <input type="radio" id="follower" name="role" value="Follower">
        <label for="follower">Follower (從機)</label>
      </div>
      <div class="form-group">
        <label for="peer_macs">夥伴 MAC 地址:</label>
        <textarea id="peer_macs" name="peer_macs" rows="4" placeholder="如果是 Leader，填入所有 Follower 的 MAC，每行一個。\n如果是 Follower，填入 Leader 的 MAC。"></textarea>
      </div>
      <div class="form-group">
        <label for="channel">通訊頻道 (1-13):</label>
        <input type="number" id="channel" name="channel" min="1" max="13" value="1" required>
      </div>
      <button type="submit" class="btn">保存並重啟</button>
    </form>
    <div id="status"></div>
  </div>
  <script>
    document.getElementById('my_mac').textContent = '%MY_MAC%';
    document.getElementById('configForm').addEventListener('submit', function(e) {
      e.preventDefault();
      const formData = new FormData(this);
      const statusDiv = document.getElementById('status');
      statusDiv.textContent = '正在保存...';
      
      fetch('/save', {
        method: 'POST',
        body: formData
      })
      .then(response => response.text())
      .then(data => {
        statusDiv.textContent = data;
        setTimeout(() => {
          statusDiv.textContent = '設備正在重啟，請手動重新連接您的 Wi-Fi。';
        }, 2000);
      })
      .catch(error => {
        statusDiv.textContent = '保存失敗: ' + error;
      });
    });
  </script>
</body>
</html>
)rawliteral";

ConfigManager configManager;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=================================");
  Serial.println("  機械臂控制系統啟動... v2.0 (RTOS)");
  Serial.println("=================================");
  
  configManager.load();
  
  Serial.print("本機 MAC 地址: ");
  Serial.println(WiFi.macAddress());

  // 統一啟動配網任務，它內部會決定下一步做什麼
  startProvisioningTask(configManager);

  Serial.println("Setup 完成，控制權交給 FreeRTOS 任務。");
  // 主 setup-loop 任務可以結束了
  vTaskDelete(NULL);
}

void loop() {
  // 此處完全留空
}