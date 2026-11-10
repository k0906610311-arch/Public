#include <Arduino.h>
#include "avr/boot.h"
#include <SoftwareSerial.h>
#include "eeprom_anything.h"
#include "CustomDelay.h"
#include <WiFiEspAT.h>
#include "PubSubClient.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>  // 加入MLX90614感測器的庫
#include <SPI.h>
#include <Unistep2.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <avr/wdt.h>
#include <U8g2_for_Adafruit_GFX.h>
#include "my_images.h"

// ==========================================
// 參數集中管理區 (優化：統一集中設定)
// ==========================================
#define WIFI_SSID     "114514"
#define WIFI_PWD      "00000000"
const char* RENDER_HOST = "kimiimaniu.onrender.com";  // Render 雲端後台網址[cite: 1]
const int   RENDER_PORT = 443;                        // HTTPS 預設連接埠

#define DEBOUNCE_DELAY 50    // 訊號穩定時間
#define esp01Serial Serial1  //主板線路有問題

//WiFi
struct wifi_config {                                       
  bool state;  // 確認 eeprom 裡是否存有 wifi 資訊
  char name[20];
  char pwd[20];
  char uuid[20];  // 配對手機之 uuid，取得自 android app
  char morningconfig[12]; 
  char noonconfig[12];
  char nightconfig[12];
} wifiConfig;

// 初始化MLX90614感測器
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//伺服馬達
Servo extruderServo;  // 分藥機頂部推藥 servo
Servo catcherServo;   // 分藥機底部接藥 servo
Servo leftservo;      // 包藥模組左 servo
Servo rightservo;     // 包藥模組右 servo
Servo upservo;        // 包藥模組分辨藥物 servo

// 心率血氧模組
MAX30105 particleSensor;

//包藥模組步進馬達
Unistep2 stepper(30, 31, 32, 33, 4096, 900);  // IN1, IN2, IN3, IN4, 總step數, 每步的延遲(in micros)  如果遇到包藥的馬達反方向就用33 32 31 30

//WiFi及MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);  // MQTT object
int8_t wifiState = WL_IDLE_STATUS;   // WL_IDLE_STATUS => the Wifi radio's status
String BUID = "SMD-1";
char mqttClientId[20];  // mqtt client id，若與其他設備重複名稱會互相踢出訂閱狀態
char mqttServer[] = "broker.emqx.io";                       //能用就好
char pubTopic[] = "medicine/resultInfo/SMD-1ff1193";        //跟sub沒什麼差 看序列埠SM-1ffxxxx跟著改就行
char subTopic[60];
char mqttBuffer[64];  // buffer for storing mqtt message from android app

//分藥機PIN腳及定義
const uint8_t slide_switch_pin = 9;
const uint8_t step_pin = 12;    // A4988 馬達驅動板 step
const uint8_t dir_pin = 11;     // A4988 馬達驅動板 dir
const uint8_t buzzer_pin = 10;  // 蜂鳴器
const uint8_t lm393_pin = 2;            // 落藥偵測模組(改OC3B)
const uint8_t extruderServo_pin = 14;  // PH0
const uint8_t catcherServo_pin = 46;   // PH1

const uint8_t leftservo_pin = 44;   // PH0
const uint8_t rightservo_pin = 45;  // PH1
const uint8_t upservo_pin = 46;     // PA0

const uint8_t tubeNum = 8;  // 藥管數量
#define TFT_CS A8   //這些define都是觸控面板的
#define TFT_DC A9
#define TFT_RST A10
#define TOUCH_CS A11
#define TOUCH_IRQ_PIN 3
#define FRAME_COUNT (sizeof(frames) / sizeof(frames[0]))

#define FRAME_W 64           // 修改為 64
#define FRAME_H 64           // 修改為 64
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ_PIN);   //以上兩個是面板的函式
U8G2_FOR_ADAFRUIT_GFX u8g2_gfx;
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
int16_t init_extruderServo_angle = 70;
int16_t init_catcherServo_angle = 90;
int16_t maxDepth[tubeNum] = { 150, 150, 150, 150, 150, 150, 150, 150 };  // 紀錄藥管推進深度
bool isConfirm[tubeNum] = { 0, 0, 0, 0, 0, 0, 0, 0 };                     // 是否已確認足以順利推藥的角度
bool isDosedThisMinute = false;
bool pauseRequested = false;
int manualTakenScheduleIdx = -1;
bool timeInitialized = false;
unsigned long nextTickAt = 0;           // 移至全域，讓 loop 和 syncNetworkTime 都能存取
bool timeSyncDoneThisHour = false;
uint8_t retryCount = 0;
int16_t prevDetectedMeds = 0;
int16_t stepperAngles[tubeNum] = { 12, 38, 63, 87, -12, -38, -63, -87 };  // 步進馬達從 switch slide 到各藥管需轉動的角度
uint8_t pills[tubeNum] = { 0, 0, 0, 0, 0, 0, 0, 0 };                     // 對應 mqttBuffer 3組數字中後8個數 -> 一格藥盒(早 or 中 or 晚)中每個藥管需要幾顆藥
uint8_t dailyBoxUsed = 0;                                           // 一日式藥盒使用幾格(在 mqtt callback 中由手機發送的訊息得知)
bool dispensingError = false;
bool chosmod=false;
int currentHour = 0;
int currentMinute = 0;
int currentSecond = 0;
unsigned long lastTimeSync = 0;

bool tubeFinished[8] = { false };
bool requiredTubes[8] = { false };
char cloudSchedules[6][14]; 
uint8_t totalSchedules = 0; // 記錄目前抓到幾筆
long startTime = 0;
long waitTime = 0;
long maxWaitTime = 30;  // 秒
int32_t beatAvg = 0;
float beatsPerMinute;
const uint8_t RATE_SIZE = 4;  // Increase this for more averaging. 4 is good.
uint8_t rates[RATE_SIZE];      // Array of heart rates
uint32_t irBuffer[100];        // infrared LED sensor data
uint32_t redBuffer[100];      // red LED sensor data
int32_t bufferLength;          // data length
int32_t spo2 = 0;             // SPO2 value
int8_t validSPO2 = 0;         // indicator to show if the SPO2 calculation is valid

bool isActive = false;    // 是否執行分藥操作
bool isSchedule = false;  // 是否為按時取藥
bool isTimeout = false;   // 按時取藥後，是否直到超過時限都沒有量測心率血氧
bool isActiveDailyBox = false;
bool isActivePublish = false;
volatile uint8_t medDropCount = 0;  // 偵測到的藥物掉落數量
volatile bool lm393_detectState = LOW;
volatile bool lm393_detectTag = LOW;

int16_t getExtruderServoAngle(uint8_t, int16_t, bool);

//包藥模組PIN腳及定義
const int heaterPin1 = 7;  // 加熱片控制引腳，必須是PWM引腳
const int heaterPin2 = 8;  // 加熱片控制引腳，必須是PWM引腳

bool errorDetected = false;
unsigned long tempPreviousMillis = 0;  // 記錄上一次讀取溫度的時間
const long tempInterval = 2000;        // 設置讀取溫度的間隔時間為2秒

int pwmValue = 0;  // PWM值，範圍從0到255
int setnode=0;
uint8_t medTube;

//模組判定
const int dispensingDetectPin = 26;  // 偵測包藥模組的腳位
const int packagingDetectPin = 27;   // 偵測包藥模組的腳位
bool isDispensingMode = false;
bool isPackagingMode = false;
bool isProcessing = false;
bool isProcessingDaily = false;
unsigned long lastDebounceTime = 0;
float currentTemperature = 0.0;  // 初始化currentTemperature變數
int birdFrame = 0;
unsigned long lastAnimUpdate = 0;
int dotCount = 0; // 供 initWifi 使用
String lastLCDMsg = "";
enum MenuState { HIDDEN, SHOW_SETTING_BTN, MAIN_MENU };
MenuState currentMenu = SHOW_SETTING_BTN;

//各項函式宣告
void playSimpleFadeLogo();
void syncAllFromCloud();
void updateOnlyTime();
void checkCloudAlarm();
int syncNetworkTime();
void refreshMenuDisplay();
void loopDispensing();
void turnStepperMotor(int16_t);
void medCounterByISR();
void setMedBoxPosition(uint8_t, bool);
void dispensingOperation(int32_t, int32_t);
void dispensingOpForDailyBox();
void startUpBuzzer(uint8_t);
void calibration();
void initWifi();
void reconnect();
void enableApMode();
void disableApMode();
void setupWifiFromAP();
void userResponse();
void callback(char*, uint8_t*, unsigned int);
bool publishResult();
void displayIdleScreen();
void drawimg(const unsigned char* bitmap, int w, int h, uint16_t color);
void moveImageLocal(const unsigned char* bitmap, int x, int y, int oldX, int oldY, int w, int h, uint16_t color);
void displayHealthData(int32_t hr, int32_t o2);
void lcdsetup();
void drawCustomBtn(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bgColor, uint16_t textColor, const char* label);
void updateTFT(const char* line1, const char* line2 = "");
void loopPackaging();
void dispensingOperation_packaging(int32_t, int32_t);
void setPackagingactivity(uint8_t step);
void stepperMove(int steps);
void controlServo1();
void controlServo2();
void controlErrorServo1();
void controlErrorServo2();
void readTemperature();
void controlHeater(float temperature);
void heating();
void turnOffHeating();
void beepBuzzer();
void checkAllTubesFinished();

void setup() {
  Wire.begin();
  Serial.begin(9600);  // 初始化串口通訊
  esp01Serial.begin(9600);
  lcdsetup();
  playSimpleFadeLogo();

  pinMode(slide_switch_pin, INPUT);
  pinMode(lm393_pin, INPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(dispensingDetectPin, INPUT_PULLUP);

  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);
  pinMode(packagingDetectPin, INPUT_PULLUP);

  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup(60, 4, 2, 400, 411, 16384);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  bool packagingModuleInserted = (digitalRead(packagingDetectPin) == LOW);
  if (packagingModuleInserted) {
    if (!mlx.begin()) {
      Serial.println("Error connecting to MLX90614 sensor. Check wiring.");
    }
  } else {
    Serial.println("Packaging module not inserted. Skipping its initialization.");
  }

  attachInterrupt(digitalPinToInterrupt(lm393_pin), medCounterByISR, CHANGE);

  EEPROM_readAnything(0, wifiConfig);

  for (int32_t i = 20; i < 24; i++) {
    String tmp = "";
    tmp = String(boot_signature_byte_get(i), HEX);
    BUID = BUID + tmp;
  }
  strcpy(mqttClientId, BUID.c_str());
  Serial.println(BUID);
  String dynamicTopic = "medicine/actionInfo/" + BUID;
  dynamicTopic.toCharArray(subTopic, 60);

  calibration();
  setupWifiFromAP();
  initWifi();
  if (WiFi.status() == WL_CONNECTED) {
    syncAllFromCloud();
  }
  mqttClient.setKeepAlive(120);
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    wifiState = WiFi.status();
    if (wifiState != WL_CONNECTED) {
      Serial.println("Wifi disconnected");
      drawimg(lookingforwifi, 108, 34, ILI9341_WHITE);
      
      while (wifiState != WL_CONNECTED) {
        wifiState = WiFi.begin(wifiConfig.name, wifiConfig.pwd);
        milisDelay(500);
      }
      drawimg(susswifi, 162, 34, ILI9341_WHITE);
      currentMenu = SHOW_SETTING_BTN;
      refreshMenuDisplay();
      milisDelay(1000);
    }
    reconnect();
  } else {
    if (isActivePublish) {
      if (publishResult()) {
        isActivePublish = false;
        isSchedule = false;
      }
    }
  }

  if (!timeInitialized && WiFi.status() == WL_CONNECTED) {
    int s = syncNetworkTime();
    if (s != -1) {
      nextTickAt = millis() + (60 - s) * 1000;
      timeInitialized = true;
      Serial.print("相位對齊！將在 ");
      Serial.print(60 - s);
      Serial.println(" 秒後進入下一分鐘。");
    }
  }

  if (timeInitialized && millis() >= nextTickAt) {
    nextTickAt += 60000;
    isDosedThisMinute = false;
    if (currentMenu == SHOW_SETTING_BTN && !isActive && !isProcessing) {
        currentMinute++;
        if (currentMinute >= 60) {
          currentMinute = 0;
          currentHour++;
          if (currentHour >= 24) currentHour = 0;
        }
        updateOnlyTime();
    } else {
        currentMinute++;
        if (currentMinute >= 60) {
          currentMinute = 0;
          currentHour++;
          if (currentHour >= 24) currentHour = 0;
        }
    }
    
    if (currentMinute == 32 && !timeSyncDoneThisHour) { 
      timeInitialized = false; 
      timeSyncDoneThisHour = true;
    }
    if (currentMinute == 33) timeSyncDoneThisHour = false;
  }
  
  char nowStr[5];
  sprintf(nowStr, "%02d%02d", currentHour, currentMinute);

  for (int i = 0; i < totalSchedules; i++) {
    String sched = String(cloudSchedules[i]);
    String schedTimeStr = sched.substring(9);
    if (manualTakenScheduleIdx == i && atoi(nowStr) > schedTimeStr.toInt()) {
      manualTakenScheduleIdx = -1;
    }
    if (schedTimeStr == String(nowStr) && !isDosedThisMinute) {
      if (manualTakenScheduleIdx == i) {
        isDosedThisMinute = true;
        break;
      }
      memset(mqttBuffer, 0, sizeof(mqttBuffer));
      mqttBuffer[0] = 's';
      memcpy(mqttBuffer + 1, cloudSchedules[i], 13);
      setnode = 1;
      isActive = true; 
      isSchedule = true;
      isDosedThisMinute = true;
      break; 
    }
  }

  mqttClient.loop();

  if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    bool stableDispensing = (digitalRead(dispensingDetectPin) == LOW);
    bool stablePackaging = (digitalRead(packagingDetectPin) == LOW);
    String newMsg = "";

    if (stableDispensing && !stablePackaging) {
      isDispensingMode = true;
      isPackagingMode = false;
      newMsg = "Dispensing Mode";
    } else if (!stableDispensing && stablePackaging) {
      isDispensingMode = false;
      isPackagingMode = true;
      newMsg = "Packaging Mode";
    } else {
      isDispensingMode = false;
      isPackagingMode = false;
      newMsg = "ERROR:Check Mod!";
    }
    lastDebounceTime = millis();
    if (newMsg != lastLCDMsg) {
      if (newMsg.indexOf("ERROR") >= 0) {
        tft.fillScreen(ILI9341_RED);
        updateTFT("System error", newMsg.c_str());
      }
      lastLCDMsg = newMsg;
    }
  }

  if (isDispensingMode) {
    chosmod = true;
    loopDispensing();
  } else if (isPackagingMode) {
    chosmod = false;
    loopPackaging();
  }
  updateMenuLogic();
}

void loopDispensing() {
  static bool firstEntry = true;
  if (firstEntry) {
    currentMenu = SHOW_SETTING_BTN;
    refreshMenuDisplay();
    firstEntry = false;
  }
  if (isActive) {
    if (isActiveDailyBox)
      dispensingOpForDailyBox();
    else
      dispensingOperation(4, setnode);
    if (isSchedule) {
      userResponse();
      startUpBuzzer(2);
    } else {
      startUpBuzzer(2);
      milisDelay(800);
      startUpBuzzer(2);
    }
    
    if (isTimeout == true) {
      drawimg(timeerror, 177, 35, ILI9341_RED); 
    } else {
      drawimg(enderror, 56, 35, ILI9341_GREEN);
    }
    if (isSchedule) {
      displayHealthData(beatAvg, spo2);
    }
    milisDelay(5000);
    if (chosmod == false) displayIdleScreen(); 
    else displayDispensingIdle();
    
    currentMenu = SHOW_SETTING_BTN; 
    refreshMenuDisplay();
    if (isSchedule) isActivePublish = true;
    isActive = false;
    isActiveDailyBox = false;
    isTimeout = false;
    validSPO2 = 0;
    currentMenu = SHOW_SETTING_BTN;
    refreshMenuDisplay();
    return;
  }
  updateMenuLogic();
}

void loopPackaging() {
  static bool hasHeated = false;
  static bool hasSlept = false;  
  static unsigned long lastActivityTime = 0;
  const unsigned long HEATING_TIMEOUT = 5UL * 60UL * 1000UL;  
      
  static bool step1_Heating = false;      
  static bool step2_Balancing = false;     
  static unsigned long balanceStartTime = 0;
  const unsigned long BALANCE_DURATION = 10000;

  if (digitalRead(packagingDetectPin) == LOW && !hasSlept) {
    readTemperature(); 
    controlHeater(currentTemperature);
  } else if (hasSlept || hasHeated){
    turnOffHeating();
  }
  if (!isProcessing && !hasHeated && !hasSlept) {
    isProcessing = true;
    step1_Heating = true;
    tft.fillScreen(ILI9341_BLACK);
    tft.drawRect(20, 180, 280, 25, ILI9341_WHITE);
    tft.drawBitmap(131, 80, readying, 57, 24, ILI9341_WHITE);
  }

  if (isProcessing) {
    int totalProgress = 0;
    readTemperature(); 
    if (step1_Heating) {
      float tempPercent = (currentTemperature - 25.0) / (120.0 - 25.0);
      tempPercent = constrain(tempPercent, 0.0, 1.0);
      totalProgress = (int)(tempPercent * 50);

      if (currentTemperature >= 120.0) {
        step1_Heating = false;
        step2_Balancing = true;
        balanceStartTime = millis();
        startUpBuzzer(1); 
      }
    } 
    else if (step2_Balancing) {
      unsigned long elapsed = millis() - balanceStartTime;
      float timePercent = (float)elapsed / BALANCE_DURATION;
      timePercent = constrain(timePercent, 0.0, 1.0);
      totalProgress = 50 + (int)(timePercent * 50); 

      if (elapsed >= BALANCE_DURATION) {
        tft.fillRect(22, 182, 276, 21, ILI9341_GREEN); 
        startUpBuzzer(2);
        delay(500);
        controlErrorServo1();
        delay(500);
        controlErrorServo2();
        delay(500);
        controlServo1();
        delay(200);
        stepperMove(3100);
        delay(200);
        isProcessing = false;
        step2_Balancing = false;
        hasHeated = true;
        startUpBuzzer(2);
        displayIdleScreen();
        currentMenu = SHOW_SETTING_BTN;
        refreshMenuDisplay();
        lastActivityTime = millis();
      }
    }

    if (isProcessing) {
      int barWidth = map(totalProgress, 0, 100, 0, 276);
      tft.fillRect(22, 182, barWidth, 21, ILI9341_GREEN);
    }
  }

  if (isActive && !isProcessing) {
    if (isActiveDailyBox)
      dispensingOpForDailyBox();
    else
      dispensingOperation_packaging(4, setnode);

    if (isSchedule) {
      userResponse();
      startUpBuzzer(2);
    } else {
      startUpBuzzer(2);
      milisDelay(800);
      startUpBuzzer(2);
    }
    
    if (isTimeout == true) {
      drawimg(timeerror, 177, 35, ILI9341_RED); 
    } else {
      drawimg(enderror, 56, 35, ILI9341_GREEN); 
    }
    if (isSchedule) {
      displayHealthData(beatAvg, spo2);
    }
    milisDelay(5000);
    if (isSchedule) isActivePublish = true;
    isActive = false;
    isActiveDailyBox = false;
    isTimeout = false;
    currentMenu = SHOW_SETTING_BTN; 
    refreshMenuDisplay();
    lastActivityTime = millis();
  }

  if (hasHeated && !hasSlept && (millis() - lastActivityTime > HEATING_TIMEOUT)) {
    displaySleepMode();
    turnOffHeating();
    hasHeated = false;
    hasSlept = true;
    isProcessing = false;
  }
  if (isActive && hasSlept) {
    hasSlept = false;
    hasHeated = false;
    isProcessing = false;
  }
}

int16_t getExtruderServoAngle(uint8_t tubeNum, int16_t currentAngle, bool isFirst) {
  if (prevDetectedMeds > 0 || isFirst) {
    if (isConfirm[tubeNum]) {
      return maxDepth[tubeNum];
    } else {
      return init_extruderServo_angle;
    }
  } else {
    if (isConfirm[tubeNum] == 0) {
      return currentAngle += 5;
    } else {
      retryCount++;
      return maxDepth[tubeNum] + 5;
    }
  }
}

void turnStepperMotor(int16_t angle) {
  digitalWrite(dir_pin, angle > 0 ? LOW : HIGH);
  for (int16_t i = 0; i < abs(angle); i++) {
    digitalWrite(step_pin, HIGH);
    milisDelay(10);
    digitalWrite(step_pin, LOW);
    milisDelay(10);
  }
}

void displayDispensingIdle() {
  tft.fillScreen(ILI9341_BLACK);
  int16_t x1, y1;
  uint16_t w, h;

  tft.setTextSize(7); 
  tft.setTextColor(ILI9341_WHITE);
  char timeBuf[10];
  sprintf(timeBuf, "%02d:%02d", currentHour, currentMinute);
  tft.getTextBounds(timeBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 50);
  tft.print(timeBuf);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  String nextSched = "No Alarm";
  char nowStr[5];
  sprintf(nowStr, "%02d%02d", currentHour, currentMinute);

  if (totalSchedules > 0) {
    bool found = false;
    for (int i = 0; i < totalSchedules; i++) {
        if (String(cloudSchedules[i]).substring(9).toInt() >= String(nowStr).toInt()) {
            nextSched = "Next: " + String(cloudSchedules[i]).substring(9, 11) + ":" + String(cloudSchedules[i]).substring(11, 13);
            found = true;
            break;
        }
    }
    if (!found) {
        nextSched = "All Done Today";
    }
  }
  tft.getTextBounds(nextSched.c_str(), 0, 0, &x1, &y1, &w, &h);
  
  tft.fillRoundRect((320 - w) / 2 - 10, 135, w + 20, 35, 8, ILI9341_DARKGREY); 
  tft.setCursor((320 - w) / 2, 145);
  tft.print(nextSched);

  drawCustomBtn(210, 180, 100, 50, ILI9341_BLUE, ILI9341_WHITE, "SET");
}

void updateMenuLogic() {
  if (currentMenu == HIDDEN) return;

  if (touch.touched()) {
    TS_Point p = touch.getPoint();
    int16_t x = map(p.x, 4000, 130, 0, 320);
    int16_t y = map(p.y, 3800, 150, 0, 240);

    if (currentMenu == SHOW_SETTING_BTN) {
      if (x > 200 && y > 180) {
        playRippleAnimation(x, y, ILI9341_WHITE);
        currentMenu = MAIN_MENU;
        refreshMenuDisplay();
      }
    } 
    else if (currentMenu == MAIN_MENU) {
      if (x > 60 && x < 260 && y > 40 && y < 90) { 
        playRippleAnimation(x, y, ILI9341_YELLOW);
        syncAllFromCloud(); 

        char nowStr[5];
        sprintf(nowStr, "%02d%02d", currentHour, currentMinute);

        bool foundNextMed = false;
        for (int i = 0; i < totalSchedules; i++) {
          String sched = String(cloudSchedules[i]);
          if (sched.substring(9).toInt() >= String(nowStr).toInt()) {
            memset(mqttBuffer, 0, sizeof(mqttBuffer));
            mqttBuffer[0] = 'd';
            memcpy(mqttBuffer + 1, cloudSchedules[i], 13);
            
            setnode = 1;
            isActive = true;
            foundNextMed = true;
            manualTakenScheduleIdx = i; 
            break; 
          }
        }

        if (!foundNextMed && totalSchedules > 0) {
           memset(mqttBuffer, 0, sizeof(mqttBuffer));
           mqttBuffer[0] = 'd';
           memcpy(mqttBuffer + 1, cloudSchedules[totalSchedules - 1], 13);
           setnode = 1;
           isActive = true;
           manualTakenScheduleIdx = totalSchedules - 1;
        }

        currentMenu = SHOW_SETTING_BTN; 
        refreshMenuDisplay();
      }
      else if (x > 60 && x < 260 && y > 75 && y < 120) {
        playRippleAnimation(x, y, ILI9341_GREEN);
        syncAllFromCloud(); 

        if (totalSchedules > 0) {
          memset(mqttBuffer, 0, sizeof(mqttBuffer));
          mqttBuffer[0] = 'd';
          mqttBuffer[1] = 'd'; 
          
          for (int i = 0; i < totalSchedules; i++) {
            memcpy(mqttBuffer + 2 + (i * 9), cloudSchedules[i], 9);
          }

          setnode = 2; 
          isActive = true;
          isActiveDailyBox = true;
          dailyBoxUsed = totalSchedules; 
        } else {
          updateTFT("No Data", "Cloud is empty!");
          delay(2000);
        }

        currentMenu = SHOW_SETTING_BTN;
        refreshMenuDisplay();
      }
      else if (x > 60 && x < 260 && y > 130 && y < 175) {
        wdt_enable(WDTO_15MS);
        while(1);
      } 
      else if (x > 60 && x < 260 && y > 185 && y < 230) {
        playRippleAnimation(x, y, ILI9341_WHITE);
        currentMenu = SHOW_SETTING_BTN;
        refreshMenuDisplay();
      }
    }
    delay(200);
  }
}

void lcdsetup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  touch.begin();
  touch.setRotation(3);
  u8g2_gfx.begin(tft);
}

void drawimg(const unsigned char* bitmap, int w, int h, uint16_t color) {
  tft.fillScreen(ILI9341_BLACK);
  int alignedW = (w + 7) & ~7; 
  int x = (tft.width() - w) / 2;
  int y = (tft.height() - h) / 2;
  tft.drawBitmap(x, y, bitmap, alignedW, h, color);
}

void drawRoundedBtn(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, const char* label, uint8_t textSize, uint16_t colortext) {
  tft.fillRoundRect(x, y, w, h, 15, color);
  tft.setTextColor(colortext);
  tft.setTextSize(textSize);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(x + (w - tw) / 2, y + (h - th) / 2);
  tft.print(label);
}

void playRippleAnimation(int16_t centerX, int16_t centerY, uint16_t color) {
  for (int r = 5; r < 40; r += 5) {
    tft.drawCircle(centerX, centerY, r, color);
    delay(10);
  }
}

void playSimpleFadeLogo() {
  tft.fillScreen(ILI9341_BLACK);
  const char* logoText = "i-Dispenser";
  tft.setTextSize(4);
  
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(logoText, 0, 0, &x1, &y1, &w, &h);
  int x = (320 - w) / 2;
  int y = (240 - h) / 2;

  for (int i = 0; i <= 31; i++) {
    uint16_t color = tft.color565(i * 8, i * 8, i * 8); 
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.print(logoText);
    delay(30); 
  }
  delay(1000);

  for (int i = 31; i >= 0; i--) {
    uint16_t color = tft.color565(i * 8, i * 8, i * 8);
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.print(logoText);
    delay(20);
  }
  tft.fillScreen(ILI9341_BLACK);
}

void displayIdleScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds("i-Dispenser", 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 90);
  tft.print("i-Dispenser");
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  
  char timeBuf[10];
  sprintf(timeBuf, "%02d:%02d", currentHour, currentMinute);
  tft.getTextBounds(timeBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 150); 
  tft.print(timeBuf);
  drawCustomBtn(200, 180, 110, 50, ILI9341_BLUE, ILI9341_WHITE, "SETTING");
}

void displayHealthData(int32_t hr, int32_t o2) {
  tft.fillScreen(ILI9341_BLACK); 
  tft.drawBitmap(118, 20, hrspres, 88, 27, ILI9341_CYAN);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(40, 90);
  tft.print("HR: ");
  tft.print(hr);
  tft.setTextColor(ILI9341_CYAN); 
  tft.setCursor(40, 160);
  tft.print("SpO2: ");
  if (o2 > 100) o2 = 100; 
  if (o2 < 0) o2 = 0;
  tft.print(o2);
  tft.print("%");
}

void displaySleepMode() {
  tft.fillScreen(ILI9341_NAVY); 
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_LIGHTGREY);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds("Sleeping...", 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 100);
  tft.print("Sleeping...");
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(50, 160);
  tft.print("Send New Command");
  tft.setCursor(70, 190);
  tft.print("to Wake Up");
}

void drawCustomBtn(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bgColor, uint16_t textColor, const char* label) {
  tft.fillRoundRect(x, y, w, h, 10, bgColor);
  tft.drawRoundRect(x, y, w, h, 10, ILI9341_WHITE);
  tft.setTextColor(textColor);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(x + (w - tw) / 2, y + (h + th) / 2 - 10);
  tft.print(label);
}

void updateDispensingStatus(int count, int left) {
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(5, 45, 310, 180, ILI9341_YELLOW);
  tft.drawBitmap(116, 55, Dispensing, 88, 35, ILI9341_YELLOW); 
  tft.drawBitmap(30, 105, dropingC, 88, 34, ILI9341_CYAN);
  tft.setCursor(130, 110);
  tft.setTextColor(ILI9341_GREEN); 
  tft.setTextSize(4);
  tft.print(count);
  tft.drawBitmap(30, 160, leftC, 88, 34, ILI9341_CYAN);
  tft.setCursor(130, 165);
  tft.setTextColor(ILI9341_ORANGE); 
  tft.setTextSize(4);
  tft.print(left);

  if (isProcessingDaily || isActiveDailyBox || setnode == 2) {
    if (pauseRequested) {
      drawCustomBtn(190, 165, 110, 50, ILI9341_YELLOW, ILI9341_BLACK, "PAUSING...");
    } else {
      drawCustomBtn(190, 165, 110, 50, ILI9341_YELLOW, ILI9341_BLACK, "PAUSE");
    }

    unsigned long displayStart = millis();
    while (millis() - displayStart < 3000) {
      if (touch.touched() && !pauseRequested) {
        TS_Point p = touch.getPoint();
        int16_t x = map(p.x, 4000, 130, 0, 320);
        int16_t y = map(p.y, 3800, 150, 0, 240);
        
        if (x > 190 && x < 300 && y > 165 && y < 215) {
          playRippleAnimation(x, y, ILI9341_WHITE);
          pauseRequested = true;
          drawCustomBtn(190, 165, 110, 50, ILI9341_YELLOW, ILI9341_BLACK, "PAUSING...");
        }
      }
      delay(20);
    }
  } else {
    delay(1500);
  }
}

void updateTFT(const char* line1, const char* line2 = "") {
  tft.fillScreen(ILI9341_BLACK); 
  u8g2_gfx.setFont(u8g2_font_wqy15_t_chinese1);
  u8g2_gfx.setFontMode(1); 
  u8g2_gfx.setFontDirection(0);
  u8g2_gfx.setForegroundColor(ILI9341_CYAN);
  int16_t w1 = u8g2_gfx.getUTF8Width(line1);
  u8g2_gfx.setCursor((tft.width() - w1) / 2, 60); 
  u8g2_gfx.print(line1);
  if (line2[0] != '\0') {
    u8g2_gfx.setForegroundColor(ILI9341_WHITE);
    int16_t w2 = u8g2_gfx.getUTF8Width(line2); 
    u8g2_gfx.setCursor((tft.width() - w2) / 2, 120);
    u8g2_gfx.print(line2);
  }
}

void refreshMenuDisplay() {
  if (currentMenu == SHOW_SETTING_BTN) {
    displayDispensingIdle(); 
    drawCustomBtn(200, 180, 110, 50, ILI9341_BLUE, ILI9341_WHITE, "SETTING");
  } 
  else if (currentMenu == MAIN_MENU) {
    tft.fillScreen(ILI9341_BLACK);
    drawCustomBtn(60, 20, 200, 45, ILI9341_BLUE, ILI9341_WHITE, "Instant Med");
    drawCustomBtn(60, 75, 200, 45, ILI9341_BLUE, ILI9341_WHITE, "Full Day");
    drawCustomBtn(60, 130, 200, 45, ILI9341_RED, ILI9341_WHITE, "Reset");
    drawCustomBtn(60, 185, 200, 45, ILI9341_DARKGREY, ILI9341_WHITE, "Back");
  }
}

void medCounterByISR() {
  lm393_detectState = digitalRead(lm393_pin);
  if (lm393_detectState == HIGH && lm393_detectTag == LOW) {
    medDropCount++;
    lm393_detectTag = HIGH;
  }
  if (lm393_detectState == LOW && lm393_detectTag == HIGH) {
    lm393_detectTag = LOW;
    for (uint8_t i = 0; i < 4; i++) {
      delayMicroseconds(10000);
    }
  }
}

void setMedBoxPosition(uint8_t position, bool isReturn) {
  if (!isReturn) {
    switch (position) {
      case 1:
        for (uint8_t angle = 90; angle < 180; angle++) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 2:
        for (uint8_t angle = 90; angle < 110; angle++) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 3:
        break;
      case 4:
        for (uint8_t angle = 90; angle > 50; angle--) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 5:
        for (uint8_t angle = 90; angle > 25; angle--) { catcherServo.write(angle); milisDelay(15); }
        break;
    }
  } else {
    milisDelay(500);
    switch (position) {
      case 1:
        for (uint8_t angle = 180; angle > 90; angle--) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 2:
        for (uint8_t angle = 110; angle > 90; angle--) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 3:
        break;
      case 4:
        for (uint8_t angle = 50; angle < 90; angle++) { catcherServo.write(angle); milisDelay(15); }
        break;
      case 5:
        for (uint8_t angle = 25; angle < 90; angle++) { catcherServo.write(angle); milisDelay(15); }
        break;
    }
  }
}

void dispensingOpForDailyBox() {
  pauseRequested = false; 
  isProcessingDaily = true;
  for (uint8_t period = 0; period < dailyBoxUsed; period++) { 
    pauseRequested = false;
    int boxIndex = 2 + (period * 9);
    uint8_t specBox = (mqttBuffer[boxIndex] - '0') + 1;

    prevDetectedMeds = 0; 
    medDropCount = 0;
    retryCount = 0;
    if (isPackagingMode){
      dispensingOperation_packaging(specBox, boxIndex);
    } else{
      dispensingOperation(specBox, boxIndex);
    }
    if (period >= dailyBoxUsed - 1) {
      break; 
    }
    if (pauseRequested) {
      tft.fillScreen(ILI9341_BLACK);
      updateTFT("Meds Paused", "Take meds, then click NEXT");
      drawCustomBtn(80, 150, 160, 60, ILI9341_GREEN, ILI9341_WHITE, "NEXT");

      bool confirmed = false;
      while (!confirmed) { 
        if (touch.touched()) {
          TS_Point p = touch.getPoint();
          int16_t x = map(p.x, 4000, 130, 0, 320);
          int16_t y = map(p.y, 3800, 150, 0, 240);
          if (x > 80 && x < 240 && y > 150 && y < 210){ 
            playRippleAnimation(x, y, ILI9341_WHITE);
            confirmed = true;
        }
        delay(50);
      }
      pauseRequested = false;
    } 
  }
  isProcessingDaily = false;
  delay(2000);
}}

void dispensingOperation(int32_t specBox, int32_t node) {
  for (uint8_t i = 0; i < 8 ;i++) {
    pills[i] = mqttBuffer[node+ i + 1] -'0'; 
    if (pills[i] < 0 || pills[i] > 9) pills[i] = 0;
  }
  bool isFirst = true;
  catcherServo.attach(catcherServo_pin);
  for (uint8_t medTube = 0; medTube < 8; medTube++) {
    if (pills[medTube] == 0) {
      continue;
    }
    catcherServo.detach(); 
    milisDelay(50);
    tft.fillScreen(ILI9341_BLACK);
    tft.drawBitmap(116, 25, Dispensing, 88, 35, ILI9341_YELLOW);
    tft.drawBitmap(40, 90, tube, 176, 34, ILI9341_CYAN);
    tft.setTextColor(ILI9341_ORANGE);
    tft.setCursor(225, 95);
    tft.setTextSize(3);
    tft.print("#");
    tft.print(medTube + 1);
    
    if (isFirst || prevDetectedMeds > 0) {
      turnStepperMotor(stepperAngles[medTube]);
    }
    int16_t extruderServoAngle = init_extruderServo_angle;
    medDropCount = 0;
    extruderServo.attach(extruderServo_pin);
    extruderServo.write(0); 
    milisDelay(150);
    
    extruderServoAngle = getExtruderServoAngle(medTube, extruderServoAngle, isFirst);
    for (int32_t angle = 0; angle <= extruderServoAngle; angle++) {
      extruderServo.write(angle);
      milisDelay(8);
    }
    milisDelay(30);
    for (int32_t angle = extruderServoAngle; angle >= 0; angle--) {
      extruderServo.write(angle);
      milisDelay(8);
    }
    extruderServo.detach();
    catcherServo.attach(catcherServo_pin);
    isFirst = false;
    milisDelay(1000);

    if (medDropCount == 0 && extruderServoAngle < 150) {
      prevDetectedMeds = 0;
      if (isConfirm[medTube] == 1 && (retryCount == 4)) {
        turnStepperMotor(-stepperAngles[medTube]); 
        isFirst = true;
        retryCount = 0;
        dispensingError = true; 
      } else {
        medTube--; 
      }
    }
    else if (medDropCount == 0 && extruderServoAngle >= 150) {
      setMedBoxPosition(5, false);
      turnStepperMotor(-stepperAngles[medTube]);
      setMedBoxPosition(5, true);
      isFirst = true;
      dispensingError = true; 
    }
    else {
      prevDetectedMeds = medDropCount;
      int newPillCount = pills[medTube] - medDropCount;

      if (newPillCount < 0) {
        updateTFT("System Start", "Calibration..");
        setMedBoxPosition(5, false);                
        turnStepperMotor(-stepperAngles[medTube]); 
        setMedBoxPosition(5, true);                  
        medTube--;                                  
        retryCount = 0;
      } else {
        pills[medTube] = newPillCount;
        updateDispensingStatus(medDropCount, pills[medTube]);
        if (maxDepth[medTube] > extruderServoAngle) {
          maxDepth[medTube] = extruderServoAngle;
          isConfirm[medTube] = 1;
        }
        if (pills[medTube] > 0) {
          setMedBoxPosition(specBox, false);          
          turnStepperMotor(-stepperAngles[medTube]); 
          setMedBoxPosition(specBox, true);           
          milisDelay(1500);
          medTube--; 
          retryCount = 0;
        }
        else if (pills[medTube] == 0) {
          setMedBoxPosition(specBox, false);          
          turnStepperMotor(-stepperAngles[medTube]); 
          setMedBoxPosition(specBox, true);           
          isFirst = true;
          retryCount = 0;
        }
      }
    }
    medDropCount = 0; 
  }
  catcherServo.detach();
}

void startUpBuzzer(uint8_t target) {
  uint8_t buzzerCount = 0;
  while (buzzerCount < target) {
    digitalWrite(buzzer_pin, HIGH);
    milisDelay(100);
    digitalWrite(buzzer_pin, LOW);
    milisDelay(100);
    buzzerCount++;
  }
}

void userResponse() {
  updateTFT("Reminding...", "Touch the sensor");
  startTime = millis();

  do {
    startUpBuzzer(2);
    milisDelay(800);
    waitTime = (millis() - startTime) / 1000;
    if (waitTime > maxWaitTime) {
      isTimeout = true;
      return;
    }
  } while (particleSensor.getIR() < 25000);

  if (!isTimeout) {
    updateTFT("Get HR..", "Touch the sensor");
    bool displayChanged = false;
    long lastBeat = 0;
    uint8_t rateSpot = 0;
    startTime = millis();
    long irValue = 0;

    while (beatAvg == 0 && !isTimeout) {
      if (displayChanged == true) {
        updateTFT("Get HR..", "Touch the sensor");
        displayChanged = false;
      }
      irValue = particleSensor.getIR();
      if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat; 
        lastBeat = millis();
        beatsPerMinute = 60 / (delta / 1000.0);

        if (40 < beatsPerMinute && beatsPerMinute < 255) {
          rates[rateSpot++] = (uint8_t)beatsPerMinute;
          startUpBuzzer(1);
        }
        if (rateSpot == 4) {
          for (uint8_t x = 0; x < RATE_SIZE; x++) {
            beatAvg += rates[x];
          }
          beatAvg /= RATE_SIZE;
        }
      }
      if (irValue < 50000) {
        updateTFT("Left Sensor..", "");
        displayChanged = true;
        rateSpot = 0;
        uint8_t delayCount = 5;
        while (delayCount-- > 0) {
          if (particleSensor.getIR() < 50000) {
            delay(1000);
          } else {
            break;
          }
        }
        if (++delayCount == 0) {
          isTimeout = true;
        }
      }
    }
    if (!isTimeout) {
      updateTFT("Get SpO2..", "Touch the sensor");
      int32_t heartRate;
      int8_t validHeartRate;
      while (validSPO2 == 0 || spo2 <= 80) {
        validSPO2 = 0;
        bufferLength = 100; 

        for (uint8_t i = 0; i < bufferLength; i++) {
          while (particleSensor.available() == false) 
            particleSensor.check(); 

          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample(); 
        }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      }
    }
  }
}

void moveImageLocal(const unsigned char* bitmap, int x, int y, int oldX, int oldY, int w, int h, uint16_t color) {
  if (x == oldX && y == oldY) return;
  tft.fillRect(oldX, oldY, w, h, ILI9341_BLACK);
  tft.drawBitmap(x, y, bitmap, w, h, color);
}

void calibration() {
  drawimg(servoC, 190, 34, ILI9341_WHITE);
  digitalWrite(dir_pin, HIGH);
  while (digitalRead(slide_switch_pin) == LOW) {
    digitalWrite(step_pin, HIGH);
    milisDelay(10);
    digitalWrite(step_pin, LOW);
    milisDelay(10);
  }
  digitalWrite(dir_pin, LOW);
  for (uint8_t i = 0; i < 92; i++) {
    digitalWrite(step_pin, HIGH);
    milisDelay(10);
    digitalWrite(step_pin, LOW);
    milisDelay(10);
  }
  startUpBuzzer(2);
}

void initWifi() {
  EEPROM_readAnything(0, wifiConfig);
  if (strlen(wifiConfig.name) == 0 || (uint8_t)wifiConfig.name[0] == 255) {
    updateTFT("no config", "");
    return;
  }
  drawimg(lookingforwifi2, 81, 34, ILI9341_WHITE);
  WiFi.init(esp01Serial);
  unsigned long connectStartTime = millis(); 
  bool isConnecting = true;                 
  bool buttonsVisible = false;                

  wifiState = WiFi.begin(wifiConfig.name, wifiConfig.pwd);

  while (wifiState != WL_CONNECTED) {
    if (touch.touched()) {
      TS_Point p = touch.getPoint();
      int16_t x = map(p.x, 4000, 130, 0, 320);
      int16_t y = map(p.y, 3800, 150, 0, 240);
      x = constrain(x, 0, 319);
      y = constrain(y, 0, 239);
      
      if (!isConnecting) {
        if (x > 230 && y > 180) {
          playRippleAnimation(x, y, ILI9341_WHITE);
          drawCustomBtn(180, 180, 125, 50, ILI9341_LIGHTGREY, ILI9341_CYAN, "Reset");
          wdt_enable(WDTO_15MS);
          while(1); 
        }
        
        if (x < 120 && y > 180) {
          playRippleAnimation(x, y, ILI9341_WHITE);
          drawCustomBtn(15, 180, 125, 50, ILI9341_LIGHTGREY, ILI9341_CYAN, "Retry");
          drawimg(lookingforwifi2, 81, 34, ILI9341_CYAN);
  
          wifiState = WiFi.begin(wifiConfig.name, wifiConfig.pwd);
          isConnecting = true;
          buttonsVisible = false; 
          connectStartTime = millis(); 
        }
      }
    }

    if (isConnecting) {
      static unsigned long lastUpdate = 0;
      if (millis() - lastUpdate > 800) {
        wifiState = WiFi.status();
        lastUpdate = millis();
      }

      if (millis() - connectStartTime > 10000) { 
        isConnecting = false; 
        
        if (!buttonsVisible) {
          drawimg(nolookingforwifi, 112, 34, ILI9341_WHITE);
          drawRoundedBtn(15, 180, 125, 50, ILI9341_BLUE, "Retry", 3, ILI9341_WHITE); 
          drawRoundedBtn(180, 180, 125, 50, ILI9341_RED, "Reset", 3, ILI9341_WHITE);     
          buttonsVisible = true;
        }
      }
    }
    delay(20); 
  }

  tft.fillScreen(ILI9341_BLACK);
  drawimg(susswifi, 162, 34, ILI9341_WHITE);
  delay(1000);
}

void reconnect() {
  if (mqttClient.connected()) {
    return;
  }

  drawimg(ss, 105, 27, ILI9341_WHITE);
  if (mqttClient.connect(mqttClientId)) {
    bool sub_result = mqttClient.subscribe(subTopic);
    if (sub_result) {
      drawimg(subsuss, 84, 27, ILI9341_WHITE);
      tft.drawRect(0, 0, 320, 240, ILI9341_GREEN);
      tft.setTextSize(2);
      tft.setCursor(140, 80);
      tft.print("MQTT");
    } else {
      drawimg(subfail, 84, 27, ILI9341_WHITE);
      tft.drawRect(0, 0, 320, 240, ILI9341_RED);
      tft.setTextSize(2);
      tft.setCursor(140, 80);
      tft.print("MQTT");
    }
    milisDelay(1500);
  } else {
    tft.fillScreen(ILI9341_RED);
    switch (mqttClient.state()) {
      case -4: updateTFT("MQTT Error", "Conn Timeout"); break;
      case -3: updateTFT("MQTT Error", "Conn Lost"); break;
      case -2: updateTFT("MQTT Error", "Conn Failed"); break;
      case -1: updateTFT("MQTT Error", "Disconnected"); break;
      default: updateTFT("MQTT Error", "Other Error"); break;
    }
    delay(5000);
  }
}

void enableApMode() {
  esp01Serial.println("AT+RST");
  delay(450);
  esp01Serial.println("AT+CWMODE=2"); 
  delay(100);
  esp01Serial.println("AT+CWSAP=\"" + BUID + "\",\"\",6,0");
  delay(100);
  esp01Serial.println("AT+CIPMUX=1"); 
  delay(100);
  esp01Serial.println("AT+CIPSERVER=1,5050");
  delay(100);
  Serial.println("AP Mode");
}

void disableApMode() {
  esp01Serial.println("AT+CWMODE=1"); 
  delay(100);
  esp01Serial.println("AT+CIPSERVER=0"); 
  delay(100);
  esp01Serial.println("AT+CIPMUX=0"); 
  delay(100);
}

void updateOnlyTime() {
  tft.fillRect(35, 50, 250, 60, ILI9341_BLACK); 
  tft.setTextSize(7);
  tft.setTextColor(ILI9341_WHITE);
  char timeBuf[10];
  sprintf(timeBuf, "%02d:%02d", currentHour, currentMinute);
  
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(timeBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 50); 
  tft.print(timeBuf);
}

void setupWifiFromAP() {
  enableApMode();
  tft.fillScreen(ILI9341_BLACK);
  drawimg(wifisetup, 136, 34, ILI9341_WHITE);
  
  drawRoundedBtn(15, 180, 125, 50, ILI9341_BLUE, "Connect", 2, ILI9341_WHITE); 
  drawRoundedBtn(180, 180, 125, 50, ILI9341_RED, "Reset", 3, ILI9341_WHITE);     
  
  strcpy(wifiConfig.name, WIFI_SSID);
  strcpy(wifiConfig.pwd, WIFI_PWD);
  wifiConfig.state = 1;
  EEPROM_writeAnything(0, wifiConfig); 
  
  while (true) {
    if (esp01Serial.available()) {
      String message_str = esp01Serial.readString(); 
      const char *message_chr = message_str.c_str();
      char *ptr1 = strchr(message_chr, '[');

      if (ptr1 != NULL) {
        memset(wifiConfig.name, 0, sizeof(wifiConfig.name));
        memset(wifiConfig.pwd, 0, sizeof(wifiConfig.pwd));
        memset(wifiConfig.uuid, 0, sizeof(wifiConfig.uuid));

        ptr1 = ptr1 + 1;
        char *ptr2 = strchr(ptr1, ',');
        if (ptr2 != NULL) {
          for (int32_t i = 0; i < ptr2 - ptr1; i++) { wifiConfig.name[i] = *(ptr1 + i); }
          ptr2 = ptr2 + 1;
          ptr1 = strchr(ptr2, ',');
          if (ptr1 != NULL) {
            for (int32_t i = 0; i < ptr1 - ptr2; i++) { wifiConfig.pwd[i] = *(ptr2 + i); }
            ptr1 = ptr1 + 1;
            char *ptr3 = strchr(ptr1, ']');
            if (ptr3 != NULL) {
              for (int32_t i = 0; i < ptr3 - ptr1; i++) { wifiConfig.uuid[i] = *(ptr1 + i); }
              wifiConfig.state = 1;
              EEPROM_writeAnything(0, wifiConfig);
              updateTFT("WIFI Setup", "Setup Succ!");
              disableApMode();
              return; 
            }
          }
        }
      }
    }

    if (touch.touched()) {
      TS_Point p = touch.getPoint();
      int16_t x = map(p.x, 4000, 130, 0, 320);
      int16_t y = map(p.y, 3800, 150, 0, 240);
      x = constrain(x, 0, 319);
      y = constrain(y, 0, 239);
      
      if (x > 230 && y > 180) { 
        drawCustomBtn(180, 180, 125, 50, ILI9341_LIGHTGREY, ILI9341_CYAN, "Reset");
        playRippleAnimation(x, y, ILI9341_WHITE);
        wdt_enable(WDTO_15MS);
        while(1); 
      }

      if (x < 120 && y > 180) {
        drawCustomBtn(15, 180, 125, 50, ILI9341_LIGHTGREY, ILI9341_CYAN, "connecting");
        playRippleAnimation(x, y, ILI9341_WHITE);
        updateTFT("Bind Device:", wifiConfig.uuid);
        milisDelay(1000);
        disableApMode();
        return; 
      }
    }
    delay(20); 
  }
}

void callback(char* topic, uint8_t* payload, unsigned int length) {
  memset(mqttBuffer, 0, sizeof(mqttBuffer));
  for (int32_t i = 0; i < length; i++) {
    mqttBuffer[i] = (char)payload[i];
  }
  char type = (char)payload[0]; 

  switch (mqttBuffer[0]) {
    case 'd': 
      if (mqttBuffer[1] == 'd') { 
        isActiveDailyBox = true;
        setnode = 0;
        if (totalSchedules > 0) {
            dailyBoxUsed = totalSchedules; 
        } else {
            dailyBoxUsed = length / 9;
        }
      }
      isActive = true;
      break;

    case 's': 
      isSchedule = true;
      isActive = true;
      setnode = 0;
      break;
      
    case 'G':
      syncAllFromCloud(); 
      if (currentMenu == SHOW_SETTING_BTN) {
        refreshMenuDisplay();
      }
      break;
    default:
      break;
  }
}

bool publishResult() {
  String payload = "";
  payload += beatAvg;
  payload += ",";
  payload += spo2;

  char attrs[100];
  payload.toCharArray(attrs, 100);

  beatAvg = 0;
  spo2 = 0;

  return mqttClient.publish(pubTopic, attrs);
}

// ==========================================
// 優化後的雲端同步函式 (加入重試機制與 20 秒逾時保護)
// ==========================================
void syncAllFromCloud() {
  WiFiSSLClient client; 
  int maxRetries = 3;  // 最多重試 3 次，應對 Render 冷啟動延遲

  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    Serial.print(">>> 嘗試連線 Render 雲端 (");
    Serial.print(attempt);
    Serial.print("/");
    Serial.print(maxRetries);
    Serial.println(")...");

    if (client.connect(RENDER_HOST, RENDER_PORT)) {
      String url = "/getdata.php?uid=" + BUID;
      
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: " + RENDER_HOST + "\r\n" +
                   "User-Agent: Arduino-SMD-Device\r\n" + 
                   "Connection: close\r\n\r\n");

      unsigned long timeout = millis();
      while (client.available() == 0) {
        if (millis() - timeout > 20000) { // 延長至 20 秒等待伺服器喚醒
          Serial.println(">>> 伺服器回應超時，斷線重試...");
          client.stop();
          break;
        }
        delay(10);
      }

      if (client.available()) {
        String allData = "";
        while (client.available()) {
          allData += (char)client.read();
        }
        client.stop();

        int bodyStart = allData.indexOf("\r\n\r\n");
        String payload = "";
        if (bodyStart != -1) {
          payload = allData.substring(bodyStart + 4);
        } else {
          payload = allData;
        }
        payload.trim();

        if (payload == "NODATA") {
          Serial.println(">>> 伺服器回傳：找不到此 UID 排程");
          totalSchedules = 0;
          return;
        } else if (payload.length() >= 13) {
          Serial.print(">>> 同步成功！內容: ");
          Serial.println(payload);
          
          totalSchedules = 0;
          int start = 0;
          int comma = payload.indexOf(',');
          while (comma != -1 && totalSchedules < 6) {
            String item = payload.substring(start, comma);
            item.trim();
            strcpy(cloudSchedules[totalSchedules], item.c_str());
            totalSchedules++;
            start = comma + 1;
            comma = payload.indexOf(',', start);
          }
          if (start < payload.length() && totalSchedules < 6) {
            String lastItem = payload.substring(start);
            lastItem.trim();
            strcpy(cloudSchedules[totalSchedules], lastItem.c_str());
            totalSchedules++;
          }
          return; // 成功取得資料，直接結束函式
        }
      }
    } else {
      client.stop();
    }
    
    // 若連線失敗或逾時，等待 3 秒讓 Render 繼續喚醒，然後進行下一次嘗試
    Serial.println(">>> 伺服器可能正在從休眠喚醒中，3秒後重試...");
    delay(3000);
  }

  Serial.println(">>> 錯誤：超過最大重試次數，無法連線至 Render 伺服器");
}

int syncNetworkTime() {
  esp01Serial.println("AT+CIPSNTPCFG=1,8,\"pool.ntp.org\"");
  delay(500);
  esp01Serial.println("AT+CIPSNTPTIME?");
  
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (esp01Serial.available()) {
      String resp = esp01Serial.readString();
      int timePos = resp.indexOf(':');
      
      if (timePos != -1 && resp.length() > 20) {
        currentHour = resp.substring(timePos + 12, timePos + 14).toInt();
        currentMinute = resp.substring(timePos + 15, timePos + 17).toInt();
        int s = resp.substring(timePos + 18, timePos + 20).toInt();
        nextTickAt = millis() + (60 - s) * 1000 - 800;
        lastTimeSync = millis();
        return s;
      }
    }
  }
  return -1;
}

void setPackagingactivity(uint8_t step) {
  switch (step) {
    case 1:
      delay(500);
      controlErrorServo1();
      delay(500);
      controlErrorServo2();
      delay(500);
      stepperMove(300);
      delay(500);
      controlServo2();
      delay(500);
      stepperMove(1500);
      delay(500);
      controlServo1();
      delay(500);
      controlServo2();
      delay(500);
      stepperMove(1300);
      delay(500);
      controlServo2();
      delay(500);
      stepperMove(1300);
      delay(500);
      controlServo2();
      delay(500);
      stepperMove(1300);
      delay(500);
      for (int i = 0; i < tubeNum; i++) { tubeFinished[i] = false; }
      break;

    case 2:
      delay(500);
      controlErrorServo1();
      delay(500);
      controlErrorServo2();
      delay(500);
      break;
  }
}

void dispensingOperation_packaging(int32_t specBox, int32_t node) {
  tft.fillScreen(ILI9341_BLACK);
  for (uint8_t i = 0; i < 8; i++) {
    requiredTubes[i] = false;
    tubeFinished[i] = false;
  }
  dispensingError = false;
  for (uint8_t i = 0; i < 8 ;i++) {
    pills[i] = mqttBuffer[node+ i + 1] -'0';
    if (pills[i] < 0 || pills[i] > 9) pills[i] = 0;
    if (pills[i] > 0) {
      requiredTubes[i] = true;
      tubeFinished[i] = false;
    } else {
      requiredTubes[i] = false;
      tubeFinished[i] = true;
    }
  }
  bool isFirst = true;

  for (uint8_t medTube = 0; medTube < 8; medTube++) {
    if (pills[medTube] == 0) continue;
    int16_t extruderServoAngle = init_extruderServo_angle;
    medDropCount = 0;
    catcherServo.detach(); 
    upservo.detach();
    milisDelay(50);
    
    tft.fillScreen(ILI9341_BLACK);
    tft.drawBitmap(116, 25, Dispensing, 88, 35, ILI9341_YELLOW);
    tft.drawBitmap(40, 90, tube, 176, 34, ILI9341_CYAN);
    tft.fillRect(225, 90, 60, 40, ILI9341_BLACK);
    tft.setTextColor(ILI9341_ORANGE);
    tft.setCursor(225, 95);
    tft.setTextSize(3);
    tft.print("#");
    tft.print(medTube + 1);
    
    if (isFirst || prevDetectedMeds > 0) {
      turnStepperMotor(stepperAngles[medTube]);
    }
    extruderServo.attach(extruderServo_pin);
    extruderServo.write(0);
    milisDelay(100);

    extruderServoAngle = getExtruderServoAngle(medTube, extruderServoAngle, isFirst);
    for (int32_t angle = 0; angle <= extruderServoAngle; angle++) {
      extruderServo.write(angle);
      milisDelay(8);
    }
    milisDelay(30);
    for (int32_t angle = extruderServoAngle; angle >= 0; angle--) {
      extruderServo.write(angle);
      milisDelay(8);
    }
    extruderServo.detach();
    isFirst = false;
    milisDelay(1000);

    if (medDropCount == 0 && extruderServoAngle < 150) {
      prevDetectedMeds = 0;
      if (isConfirm[medTube] == 1 && (retryCount == 4)) {
        turnStepperMotor(-stepperAngles[medTube]);
        drawimg(tryerror, 177, 34, ILI9341_CYAN);
        isFirst = true;
        retryCount = 0;
        dispensingError = true;
      } else {
        medTube--;
      }
    } else if (medDropCount == 0 && extruderServoAngle >= 150) {
      updateTFT("Error", "Tube Empty/Jam");
      setPackagingactivity(3);
      turnStepperMotor(-stepperAngles[medTube]);
      isFirst = true;
      dispensingError = true;
    } else {
      prevDetectedMeds = medDropCount;
      int newPillCount = pills[medTube] - medDropCount;

      if (newPillCount < 0) {
        updateTFT("Error", "Excessive Drops");
        controlErrorServo1();
        delay(500);
        turnStepperMotor(-stepperAngles[medTube]);
        delay(1000);
        controlErrorServo2();
        delay(500);
        medTube--;
        retryCount = 0;
      } else {
        pills[medTube] = newPillCount;
        if (maxDepth[medTube] > extruderServoAngle) {
          maxDepth[medTube] = extruderServoAngle;
          isConfirm[medTube] = 1;
        }
        updateDispensingStatus(medDropCount, pills[medTube]);
        if (pills[medTube] > 0) {
          turnStepperMotor(-stepperAngles[medTube]);
          setPackagingactivity(2);
          milisDelay(1500);
          medTube--;
          retryCount = 0;
        } else if (pills[medTube] == 0) {
          if (requiredTubes[medTube]) { tubeFinished[medTube] = true; }
          turnStepperMotor(-stepperAngles[medTube]);
          checkAllTubesFinished();
          isFirst = true;
          retryCount = 0;
        }
      }
    }
    medDropCount = 0;
  }
  catcherServo.detach();
}

void stepperMove(int steps) {
  stepper.move(steps);
  while (stepper.stepsToGo() > 0) {
    stepper.run();
  }
}

void controlServo1() {
  leftservo.attach(leftservo_pin);
  leftservo.write(51);
  delay(100);
  leftservo.detach();
  delay(3000);
  leftservo.attach(leftservo_pin);
  leftservo.write(225);
  delay(100);
  leftservo.detach();
}

void controlServo2() {
  rightservo.attach(rightservo_pin);
  rightservo.write(51);
  delay(100);
  rightservo.detach();
  delay(3000);
  rightservo.attach(rightservo_pin);
  rightservo.write(225);
  delay(100);
  rightservo.detach();
}

void controlErrorServo1() {
  upservo.attach(upservo_pin);
  upservo.write(135);
  delay(100);
  upservo.detach();
}

void controlErrorServo2() {
  upservo.attach(upservo_pin);
  upservo.write(60);
  delay(500);
  upservo.detach();
}

void readTemperature() {
  currentTemperature = mlx.readObjectTempC();
  controlHeater(currentTemperature);
}

void controlHeater(float temperature) {
  const float targetTemperature = 120;
  const int maxPWM = 225;
  const int minPWM = 190;

  if (temperature < targetTemperature) {
    pwmValue = maxPWM;
  } else if (temperature >= targetTemperature) {
    pwmValue = minPWM;
  }

  if (pwmValue < minPWM) {
    pwmValue = minPWM;
  }

  analogWrite(heaterPin1, pwmValue);
  analogWrite(heaterPin2, pwmValue);
}

void heating() {
  unsigned long currentMillis = millis();

  if (currentMillis - tempPreviousMillis >= tempInterval) {
    tempPreviousMillis = currentMillis;
    readTemperature();
  }

  if (currentTemperature >= 100.0) {
    delay(5000);
    startUpBuzzer(2);
    controlServo1();
    delay(200);
    stepperMove(3100);
    delay(200);
    updateTFT("Packaging already", "");
  }
}

void turnOffHeating() {
  pwmValue = 0;
  analogWrite(heaterPin1, pwmValue);
  analogWrite(heaterPin2, pwmValue);
}

void beepBuzzer() {
  tone(buzzer_pin, 1000, 200);
  delay(300);
  tone(buzzer_pin, 1000, 200);
  delay(1000);
}

void checkAllTubesFinished() {
  bool allFinished = true;
  for (int i = 0; i < tubeNum; i++) {
    if (requiredTubes[i] && !tubeFinished[i]) {
      allFinished = false;
      break;
    }
  }

  if (allFinished) {
    setPackagingactivity(1);
  }
}