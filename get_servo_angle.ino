#include <Arduino.h>

// 引入舵機驅動庫
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// =========== 舵機硬體配置 ===========
#define SERVO_BAUDRATE 1000000  // 舵機通信波特率

#define SERVO_TX_PIN D2         // 對應 XIAO ESP32S3 的 D2 引腳
#define SERVO_RX_PIN D3         // 對應 XIAO ESP32S3 的 D3 引腳

// =========== 舵機對象管理 ===========
const int NUM_SERVOS = 7; // 我們要監控的舵機數量 (ID 0 到 6)
FSUS_Servo* servos[NUM_SERVOS]; // 創建一個舵機對象的指針數組

// =========== 創建通信實例 ===========
HardwareSerial servoSerial(1); // 使用 UART1
FSUS_Protocol protocol(&servoSerial, SERVO_BAUDRATE);


void setup() {
  // 啟動用於日誌輸出的主串口 (USB)
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("\n--- 舵機實時角度監控程式 ---");

  // 啟動用於控制舵機的 UART1
  servoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);

  // 初始化所有舵機對象
  Serial.println("正在初始化 0-6 號舵機對象...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 為數組中的每個指針分配一個舵機對象實例
    // 舵機的 ID 就是循環的索引 i
    servos[i] = new FSUS_Servo(i, &protocol);
    servos[i]->init();
  }

  // 開機時先 Ping 一次所有舵機，確認在線狀態
  Serial.println("正在檢測舵機在線狀態...");
  bool all_online = true;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i]->ping()) {
      Serial.printf("  - 舵機 #%d: 在線 (Online)\n", i);
    } else {
      Serial.printf("  - 舵機 #%d: 離線 (Offline)\n", i);
      all_online = false;
    }
  }

  if (all_online) {
    Serial.println("所有目標舵機均在線，開始實時監控！");
  } else {
    Serial.println("警告：有部分舵機離線，讀取數據可能會失敗。");
  }
  Serial.println("---------------------------------");
}

void loop() {
  String output_line = "實時角度: ";

  // 遍歷所有舵機，查詢並拼接角度信息
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 調用 queryAngle() 會向舵機發送指令並等待回覆
    // 函數會返回角度值，同時也會更新 servo.curAngle 屬性
    float angle = servos[i]->queryAngle();

    // 拼接每個舵機的信息
    output_line += "[ID ";
    output_line += i;
    output_line += ": ";
    output_line += String(angle, 1); // 顯示到小數點後一位
    output_line += "] ";
  }

  // 打印整行數據
  Serial.print(output_line);
  
  // 使用 '\r' (回車符) 將光標移回行首，而不是 '\n' (換行符)
  // 這樣就能實現 "在同一行刷新數據" 的效果
  Serial.print("\r");

  // 設定刷新頻率，例如每 200 毫秒 (5Hz)
  delay(200);
}