#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "DNSServer.h"
#include "Provisioning.h"
#include "EspNowHandler.h"

// 內部使用的變數和函數
static ConfigManager* pConfigManager = NULL;
static WebServer server(80);
static DNSServer dnsServer;

// HTML 網頁字串
extern const char* CONFIG_HTML PROGMEM;

// FreeRTOS 任務函數
void provisioningTask(void *parameter) {
  const byte DNS_PORT = 53;
  const unsigned long PROVISIONING_TIMEOUT = 10000;
  const unsigned long STATUS_UPDATE_INTERVAL = 3000;
  unsigned long provisioningStartTime = millis();
  unsigned long lastStatusUpdateTime = 0;
  bool clientConnected = false;

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String unique_id = mac.substring(6);
  String full_ssid = "RoboArm_Setup_" + unique_id;

  Serial.print("正在創建 Wi-Fi 熱點: "); Serial.println(full_ssid);
  int channel = 9;
  WiFi.softAP(full_ssid.c_str(), NULL, channel);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP 地址: "); Serial.println(IP);
  dnsServer.start(DNS_PORT, "*", IP);
  Serial.println("DNS 伺服器已啟動。");

  server.on("/", HTTP_GET, [&]() {
    if (!clientConnected) {
      clientConnected = true;
      Serial.println("客戶端已連接，倒數計時取消。");
    }
    String html = CONFIG_HTML;
    html.replace("%MY_MAC%", WiFi.macAddress());
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [&]() {
    Serial.println("收到配置保存請求，準備立即切換模式...");
    Config& cfg = pConfigManager->getConfig();
    strcpy(cfg.role, server.arg("role").c_str());
    // strcpy(cfg.peer_macs_str, server.arg("peer_macs").c_str());
    cfg.channel = server.arg("channel").toInt();
    cfg.configured = true;
    pConfigManager->save();
    server.send(200, "text/plain", "配置成功！設備正在立即切換到工作模式...");
    delay(100);
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    Serial.println("AP 和服務器已關閉。");
    Serial.println("啟動 ESP-NOW 任務...");
    startEspNowTask(*pConfigManager);
    Serial.println("配網任務結束，正在終止自身...");
    vTaskDelete(NULL); 
  });
  
  server.onNotFound([]() {
    server.sendHeader("Location", "http://" + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  for (;;) {
    server.handleClient();
    dnsServer.processNextRequest();

    if (!clientConnected) {
      if (WiFi.softAPgetStationNum() > 0) {
        clientConnected = true;
        Serial.println("Wi-Fi 客戶端已連接，倒數計時取消。請打開瀏覽器訪問配置頁面。");
      }
    }

    // --- 修改後的狀態提示邏輯 ---
    if (millis() - lastStatusUpdateTime > STATUS_UPDATE_INTERVAL) {
      lastStatusUpdateTime = millis();
      if (clientConnected) {
        Serial.println("客戶端已連接，正在等待網頁配置...");
      } else {
        // 只有在設備已被配置過的情況下，才顯示倒數計時
        if (pConfigManager->isConfigured()) {
          unsigned long elapsed = millis() - provisioningStartTime;
          if (elapsed < PROVISIONING_TIMEOUT) {
            int remaining_seconds = (PROVISIONING_TIMEOUT - elapsed) / 1000;
            Serial.printf("配網模式中... %d 秒後將載入上次配置。請連接 Wi-Fi: %s 進行修改。\n", remaining_seconds, full_ssid.c_str());
          }
        } else {
          // 如果從未配置過，則顯示永久等待的訊息
          Serial.printf("設備未配置，請連接 Wi-Fi: %s 進行初次設置。\n", full_ssid.c_str());
        }
      }
    }

    // --- 修改後的超時邏輯 ---
    // 只有在設備已被配置過，且無人連接的情況下，超時機制才生效
    if (pConfigManager->isConfigured() && !clientConnected && (millis() - provisioningStartTime > PROVISIONING_TIMEOUT)) {
      Serial.println("10 秒倒數計時結束，無客戶端連接。");
      server.stop();
      dnsServer.stop();
      WiFi.softAPdisconnect(true);
      
      Serial.println("載入上次配置，啟動 ESP-NOW 任務...");
      startEspNowTask(*pConfigManager);
      
      Serial.println("配網任務結束。");
      vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void startProvisioningTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(
      provisioningTask, "Provisioning Task", 4096, NULL, 1, NULL
  );
}