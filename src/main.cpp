#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h" // 為了使用 esp_wifi_set_channel()
#include <esp_now.h>
#include <WebServer.h>
#include <Preferences.h>
#include <vector>

// =========== 全域設定 ===========
// --- AP 配網模式設定 ---
const char* AP_SSID_PREFIX = "RoboArm_Setup_"; // Wi-Fi 名稱的前綴
const char* ap_password = NULL; // 不設密碼，方便連接
String full_ssid = ""; // <-- 新增: 用於存儲完整的 SSID

// --- NVS (非揮發性存儲) 設定 ---
Preferences preferences;
const char* PREF_NAMESPACE = "robo_config";
const char* PREF_KEY_CONFIG = "config_data";

// --- 程式狀態管理 ---
enum State {
  STATE_PROVISIONING, // 配網模式
  STATE_OPERATION     // ESP-NOW 工作模式
};
State currentState = STATE_PROVISIONING;

// --- 計時器相關 ---
unsigned long provisioningStartTime = 0;
const unsigned long PROVISIONING_TIMEOUT = 30000; // 30秒
bool clientConnected = false;

unsigned long lastStatusUpdateTime = 0; // <-- 新增: 上次狀態更新的時間
const unsigned long STATUS_UPDATE_INTERVAL = 3000; // <-- 新增: 狀態更新間隔 (3秒)

// --- ESP-NOW 相關 ---
esp_now_peer_info_t peerInfo;

// =========== 數據結構 ===========
struct Config {
  char role[10];
  char peer_macs_str[256];
  int channel;
  bool configured;
};
Config currentConfig;

struct MotionData {
  int motor1_angle;
  int motor2_angle;
  bool gripper_closed;
};
MotionData currentMotion;


// =========== Web 伺服器實例 ===========
WebServer server(80);


// =========== 函數聲明 ===========
void startProvisioningMode();
void startOperationMode();
void loadConfiguration();
void saveConfiguration();
String macToString(const uint8_t* mac);
void stringToMac(String macStr, uint8_t* mac);


// =========== HTML 網頁程式碼 ===========
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


// =========== ESP-NOW 回調函數 ===========
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW 發送狀態: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失敗");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&currentMotion, incomingData, sizeof(currentMotion));
  Serial.print("收到來自 ");
  Serial.print(macToString(mac));
  Serial.println(" 的數據");
  Serial.printf("Motor1: %d, Motor2: %d, Gripper: %s\n", 
                currentMotion.motor1_angle, 
                currentMotion.motor2_angle, 
                currentMotion.gripper_closed ? "閉合" : "張開");
}


// =========== 主要邏輯函數 ===========
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("設備啟動...");
  String myMac = WiFi.macAddress();
  Serial.print("本機 MAC 地址: ");
  Serial.println(myMac);
  loadConfiguration();
  provisioningStartTime = millis();
  startProvisioningMode();
  Serial.println("已進入配網模式，30 秒後若無連接將進入工作模式...");
}

void loop() {
  if (currentState == STATE_PROVISIONING) {
    server.handleClient();

    // --- 新增的狀態提示邏輯 ---
    if (millis() - lastStatusUpdateTime > STATUS_UPDATE_INTERVAL) {
      lastStatusUpdateTime = millis(); // 重置計時器
      if (clientConnected) {
        // 如果有客戶端連接了，就提示等待配置
        Serial.println("客戶端已連接，正在等待網頁配置...");
      } else {
        // 如果還沒有客戶端連接，就打印倒數計時
        unsigned long elapsed = millis() - provisioningStartTime;
        if (elapsed < PROVISIONING_TIMEOUT) {
          int remaining_seconds = (PROVISIONING_TIMEOUT - elapsed) / 1000;
          Serial.printf("配網模式中... 請連接 Wi-Fi: %s. %d 秒後將進入工作模式。\n", full_ssid.c_str(), remaining_seconds);
        }
      }
    }
    // --- 狀態提示邏輯結束 ---

    // 檢查倒數計時是否結束
    if (!clientConnected && (millis() - provisioningStartTime > PROVISIONING_TIMEOUT)) {
      Serial.println("30 秒倒數計時結束，無客戶端連接。");
      if (currentConfig.configured) {
        Serial.println("檢測到已保存的配置，切換到工作模式...");
        startOperationMode();
      } else {
        Serial.println("未檢測到有效配置，將保持在配網模式。");
        // 我們讓它停留在配網模式，並通過將 clientConnected 設為 true 來停止倒計時和提示
        clientConnected = true; 
      }
    }
  } 
  else if (currentState == STATE_OPERATION) {
    if (strcmp(currentConfig.role, "Leader") == 0) {
      currentMotion.motor1_angle = random(0, 181);
      currentMotion.motor2_angle = random(0, 181);
      currentMotion.gripper_closed = !currentMotion.gripper_closed;

      esp_now_send(0, (uint8_t *) &currentMotion, sizeof(currentMotion));
      
      Serial.printf("Leader 廣播數據: M1=%d, M2=%d\n", currentMotion.motor1_angle, currentMotion.motor2_angle);
      
      delay(2000);
    }
  }
}

// =========== 輔助功能函數實現 ===========

void startProvisioningMode() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String unique_id = mac.substring(6);
  full_ssid = String(AP_SSID_PREFIX) + unique_id; // <-- 修改: 賦值給全域變數

  Serial.print("正在創建 Wi-Fi 熱點，名稱為: ");
  Serial.println(full_ssid);

  WiFi.softAP(full_ssid.c_str(), ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP 地址: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, []() {
    clientConnected = true;
    Serial.println("客戶端已連接，倒數計時取消。");
    String html = CONFIG_HTML;
    html.replace("%MY_MAC%", WiFi.macAddress());
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, []() {
    strcpy(currentConfig.role, server.arg("role").c_str());
    strcpy(currentConfig.peer_macs_str, server.arg("peer_macs").c_str());
    currentConfig.channel = server.arg("channel").toInt();
    currentConfig.configured = true;

    saveConfiguration();
    
    server.send(200, "text/plain", "配置已保存成功！設備將在 3 秒後重啟。");
    Serial.println("配置已保存，準備重啟...");
    delay(3000);
    ESP.restart();
  });

  server.begin();
}

void startOperationMode() {
  currentState = STATE_OPERATION;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(currentConfig.channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失敗");
    ESP.restart();
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  if (strcmp(currentConfig.role, "Leader") == 0) {
    Serial.println("角色: Leader - 準備廣播");
    char* macs_copy = strdup(currentConfig.peer_macs_str);
    char* mac_ptr = strtok(macs_copy, "\r\n");
    while(mac_ptr != NULL) {
      String mac_str = String(mac_ptr);
      mac_str.trim();
      if(mac_str.length() == 17) {
        Serial.print("添加 Follower: ");
        Serial.println(mac_str);
        stringToMac(mac_str, peerInfo.peer_addr);
        peerInfo.channel = currentConfig.channel;  
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
          Serial.println("添加夥伴失敗");
        }
      }
      mac_ptr = strtok(NULL, "\r\n");
    }
    free(macs_copy);
  } else {
    Serial.println("角色: Follower - 準備接收");
    String mac_str = String(currentConfig.peer_macs_str);
    mac_str.trim();
    if(mac_str.length() == 17) {
      Serial.print("添加 Leader: ");
      Serial.println(mac_str);
      stringToMac(mac_str, peerInfo.peer_addr);
      peerInfo.channel = currentConfig.channel;  
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("添加夥伴失敗");
      }
    }
  }
}

void loadConfiguration() {
  preferences.begin(PREF_NAMESPACE, true);
  if (preferences.getBytesLength(PREF_KEY_CONFIG) == sizeof(Config)) {
    preferences.getBytes(PREF_KEY_CONFIG, &currentConfig, sizeof(Config));
    Serial.println("成功從 NVS 載入配置:");
    Serial.printf("  角色: %s\n", currentConfig.role);
    Serial.printf("  頻道: %d\n", currentConfig.channel);
    Serial.printf("  已配置: %s\n", currentConfig.configured ? "是" : "否");
  } else {
    Serial.println("NVS 中未找到有效配置，使用默認值。");
    strcpy(currentConfig.role, "Follower");
    strcpy(currentConfig.peer_macs_str, "");
    currentConfig.channel = 1;
    currentConfig.configured = false;
  }
  preferences.end();
}

void saveConfiguration() {
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putBytes(PREF_KEY_CONFIG, &currentConfig, sizeof(Config));
  preferences.end();
  Serial.println("配置已成功保存到 NVS。");
}

String macToString(const uint8_t* mac) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void stringToMac(String macStr, uint8_t* mac) {
  sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
}