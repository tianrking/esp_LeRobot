#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Provisioning.h"
#include "EspNowHandler.h" // 需要調用 EspNow 任務

// 內部使用的變數和函數
static ConfigManager* pConfigManager = NULL;
static WebServer server(80);

// 從本檔案外部引入 HTML 網頁字串
extern const char* CONFIG_HTML;

// FreeRTOS 任務函數
void provisioningTask(void *parameter) {
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
  WiFi.softAP(full_ssid.c_str(), NULL);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP 地址: "); Serial.println(IP);

  // 根目錄請求：提供配置頁面
  server.on("/", HTTP_GET, [&]() {
    clientConnected = true;
    Serial.println("客戶端已連接，倒數計時取消。");
    String html = CONFIG_HTML;
    html.replace("%MY_MAC%", WiFi.macAddress());
    server.send(200, "text/html", html);
  });

  // 保存配置請求：執行模式切換
  server.on("/save", HTTP_POST, [&]() {
    Serial.println("收到配置保存請求，準備立即切換模式...");

    // 1. 更新並保存配置
    Config& cfg = pConfigManager->getConfig();
    strcpy(cfg.role, server.arg("role").c_str());
    strcpy(cfg.peer_macs_str, server.arg("peer_macs").c_str());
    cfg.channel = server.arg("channel").toInt();
    cfg.configured = true;
    pConfigManager->save();
    
    // 2. 向網頁回傳成功訊息
    server.send(200, "text/plain", "配置成功！設備正在立即切換到工作模式...");
    delay(100); // 確保訊息有足夠時間發送出去

    // 3. 關閉 AP 和 WebServer
    server.stop();
    WiFi.softAPdisconnect(true);
    Serial.println("AP 和 WebServer 已關閉。");

    // 4. 啟動 ESP-NOW 工作任務
    Serial.println("啟動 ESP-NOW 任務...");
    startEspNowTask(*pConfigManager);
    
    // 5. 配網任務完成，刪除自身
    Serial.println("配網任務結束，正在終止自身...");
    vTaskDelete(NULL); 
  });

  server.begin();

  // 這個循環是配網模式的主體，處理網頁請求和30秒超時邏輯
  for (;;) {
    server.handleClient();

    // 定期打印狀態 (僅在無人連接時)
    if (!clientConnected && (millis() - lastStatusUpdateTime > STATUS_UPDATE_INTERVAL)) {
      lastStatusUpdateTime = millis();
      unsigned long elapsed = millis() - provisioningStartTime;
      if (elapsed < PROVISIONING_TIMEOUT) {
        int remaining_seconds = (PROVISIONING_TIMEOUT - elapsed) / 1000;
        Serial.printf("配網模式中... 請連接 Wi-Fi: %s. %d 秒後將進入工作模式。\n", full_ssid.c_str(), remaining_seconds);
      }
    }

    // 檢查30秒超時
    if (!clientConnected && (millis() - provisioningStartTime > PROVISIONING_TIMEOUT)) {
      Serial.println("30 秒倒數計時結束，無客戶端連接。");
      server.stop();
      WiFi.softAPdisconnect(true);
      
      if (pConfigManager->isConfigured()) {
        Serial.println("檢測到已保存的配置，啟動 ESP-NOW 任務...");
        startEspNowTask(*pConfigManager);
      } else {
        Serial.println("未檢測到有效配置，請重啟設備重新配置。");
        // 可以在此處讓 LED 閃爍報警
      }
      
      Serial.println("配網任務結束。");
      vTaskDelete(NULL); // 刪除自身任務
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 讓出 CPU
  }
}

void startProvisioningTask(ConfigManager& configMgr) {
  pConfigManager = &configMgr;
  xTaskCreate(
      provisioningTask, "Provisioning Task", 4096, NULL, 1, NULL
  );
}