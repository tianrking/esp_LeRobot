# ESP32 機械臂同步控制系統固件

## 項目簡介

本項目旨在為一組基於 ESP32-S3 的機械臂提供一個靈活、可靠的無線同步控制方案。固件採用 Leader-Follower 架構，通過 ESP-NOW 協議實現低延遲、高同步性的動作指令廣播。

該固件最大的特點是集成了**智能配網模式**和**工作模式**，允許用戶通過 Wi-Fi 熱點和網頁，輕鬆配置每一台設備的角色（Leader 或 Follower）及其通訊參數，無需為不同角色的設備燒錄不同的程式碼。

## ✨ 現有功能

  * **單一固件**：一套程式碼適用於所有設備，可在運行時被配置為 Leader 或 Follower。
  * **斷電保存 (NVS)**：設備配置（角色、夥伴MAC地址、頻道）會被保存在非揮發性存儲中，斷電不丟失。
  * **智能配網模式**：
      * 設備上電後自動進入 AP 配網模式，並提供一個網頁配置界面。
      * 為防止混淆，每個設備創建的 Wi-Fi 熱點名稱 (SSID) 都帶有其 MAC 地址後六位的唯一後綴 (例如 `RoboArm_Setup_8C4B82`)。
      * 提供 30 秒的配置窗口期，若無人連接，設備會自動載入上次的配置並進入工作模式。
      * 若有用戶連接，則會取消倒數計時，無限等待用戶完成配置。
  * **ESP-NOW 工作模式**：配置完成後，設備進入高性能的 ESP-NOW 通訊模式，用於實時的遙控操作。
  * **串口狀態回饋**：在啟動和配網階段，通過串口實時打印狀態訊息，方便開發者調試。

## 硬件要求

  * 若干塊 Seeed Studio XIAO ESP32S3 或其他 ESP32-S3 開發板。
  * 舵機、機械臂結構等外圍設備。

## 🛠️ 安裝與使用

1.  **環境準備**：確保您已安裝 VS Code 及 PlatformIO 擴展。
2.  **燒錄固件**：將本倉庫的程式碼通過 PlatformIO 上傳到**所有**的機械臂主控板上。
3.  **配置流程**：
    a.  **配置 Leader**：
    \* 選擇一台設備作為 Leader，單獨給它上電。
    \* 在電腦或手機的 Wi-Fi 列表中，找到名為 `RoboArm_Setup_XXXXXX` 的網絡並連接。
    \* 打開瀏覽器訪問 `192.168.4.1`。
    \* 在網頁中，將角色設定為 **Leader**，夥伴 MAC 地址暫時留空，設定頻道（例如 `1`），點擊「保存並重啟」。
    \* 通過串口監視器記錄下這台 Leader 的 MAC 地址。
    b.  **配置 Follower(s)**：
    \* 逐個為所有 Follower 設備上電。
    \* 每台設備上電後，都連接其創建的 `RoboArm_Setup_XXXXXX` 網絡。
    \* 訪問 `192.168.4.1`，將角色設定為 **Follower**，在「夥伴 MAC 地址」欄中填入**第 a 步記錄的 Leader 的 MAC 地址**，頻道設定為與 Leader 相同的頻道（例如 `1`），點擊「保存並重啟」。
    \* 同時，記錄下每一台 Follower 的 MAC 地址。
    c.  **最終配置 Leader**：
    \* 再次讓 Leader 進入配置模式（重新上電並在 30 秒內連接）。
    \* 在配置頁面，角色保持 **Leader**，在「夥伴 MAC 地址」欄中，填入**所有 Follower 的 MAC 地址**，每行一個。
    \* 點擊「保存並重啟」。
4.  **完成**：現在，所有設備上電 30 秒後，都會自動組成 ESP-NOW 網絡並開始工作。

## 🚀 下一步開發計劃 (Roadmap)

基於當前穩定的框架，下一步的重點是將模擬數據替換為真實的硬件控制和雙向通訊。

### 1\. 實現真實的舵機控制 (Follower)

  * **目標**：讓 Follower 在收到 ESP-NOW 數據後，不再是打印訊息，而是實際驅動舵機運動。
  * **實施路徑**：
    1.  在程式碼中引入 `ESP32Servo.h` 庫。
    2.  在全域範圍內創建舵機對象，例如 `Servo servo1;` `Servo servo2;`。
    3.  在 `setup()` 或 `startOperationMode()` 中，使用 `servo1.attach(PIN_NUM)` 將舵機對象與實際的 GPIO 引腳綁定。
    4.  修改 `OnDataRecv()` 回調函數，將 `Serial.printf(...)` 的部分替換為舵機控制指令：
        ```cpp
        void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
          memcpy(&currentMotion, incomingData, sizeof(currentMotion));
          
          // 原來的 Serial.printf(...) 可以保留用於調試
          
          // 新增的舵機控制程式碼
          servo1.write(currentMotion.motor1_angle);
          servo2.write(currentMotion.motor2_angle);
          // ... 根據 gripper_closed 控制爪子舵機 ...
        }
        ```

### 2\. 建立反饋機制 (Follower -\> Leader)

  * **目標**：讓 Follower 能夠將自己的狀態（例如：動作是否完成、是否遇到障礙物）反饋給 Leader。
  * **實施路徑**：
    1.  定義一個新的數據結構 `FeedbackData`，用於承載反饋信息。
        ```cpp
        struct FeedbackData {
          uint8_t follower_mac[6]; // 自己的MAC地址
          bool task_completed;
          int error_code;
        };
        ```
    2.  在 Follower 的 `loop()` 函式或 `OnDataRecv()` 函式完成動作後，填充 `FeedbackData` 結構體。
    3.  調用 `esp_now_send()` 將此結構體發送給 Leader（Leader 的 MAC 地址已在配置中保存）。
    4.  修改 Leader 的 `OnDataRecv()` 回調函數，使其能夠接收並處理來自 Follower 的 `FeedbackData`。通過判斷數據的來源 MAC 地址，Leader 就能知道是哪個 Follower 發來的反饋。

### 3\. Leader 模式的真實控制邏輯

  * **目標**：將 Leader 在 `loop()` 中隨機生成數據的邏輯，替換為接收真實外部指令（例如來自手機 App 的 BLE 指令）的邏輯。
  * **實施路徑**：
    1.  將我們之前驗證過的 **BLE 伺服器程式碼**整合進來。
    2.  在 `startOperationMode()` 中，如果角色是 Leader，則同時初始化 ESP-NOW 和 BLE 服務。
    3.  在 BLE 的 `onWrite()` 回調函數中，解析來自手機 App 的指令。
    4.  根據指令填充 `MotionData` 結構體。
    5.  **直接在 `onWrite()` 回調函數中**調用 `esp_now_send()` 將 `MotionData` 廣播出去。
    6.  這樣，Leader 的 `loop()` 函式就可以保持空閒，整個控制流程變為由 BLE 事件驅動，響應更及時。

### 4\. 引入指令路由和回調系統 (程式碼重構)

  * **目標**：當指令類型變多時，避免在 `OnDataRecv` 中使用大量的 `if/else`，使程式碼更清晰、易於擴展。
  * **實施路徑**：
    1.  在 `MotionData` 和 `FeedbackData` 結構體中增加一個 `command_id` 字段。
    2.  在 `OnDataRecv` 函式中，使用 `switch (receivedData.command_id)` 來代替 `if/else`。
    3.  每一個 `case` 調用一個專門的處理函式，例如 `case CMD_MOVE: handleMoveCommand(data); break;`。這就是一個簡單的指令路由。
    4.  這種架構將數據的「接收」和「處理」解耦，是大型專案的標準做法。