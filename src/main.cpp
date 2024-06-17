/**
 * @file main.cpp
 * @author 周晨阳 (2922462750@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-06-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AHT10.h>
#include <ADXL345_WE.h>
#include "EEPROM.h"
#define LENGTH(x) (strlen(x) + 1) // length of char string
#define EEPROM_SIZE 200           // EEPROM size
#define WiFi_rst 0                // WiFi credential reset pin (Boot button on ESP32)
String ssid;                      // string variable to store ssid
String pss;                       // string variable to store password
unsigned long rst_millis;
#define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH
const int int1Pin = 2;
volatile bool tap = false;

// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adapted
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN 2
#define DATA_PIN 3
#define CS_PIN 4

// 配置NTP客户端
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com", 28800, 60000);
// HARDWARE SPI
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
// SOFTWARE SPI
// MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_MAX72XX *pM;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);
ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);
// Display parameters
const uint8_t SCROLL_SPEED = 50; // text speed
const textEffect_t SCROLL_EFFECT = PA_NO_EFFECT;
const textPosition_t SCROLL_ALIGN = PA_CENTER;
const uint16_t SCROLL_PAUSE = 0; // in milliseconds

const uint8_t TEXT_ZONE = 0; // just change this to adjust display
const uint8_t TXT_LOWER = 0;
const uint8_t TXT_UPPER = 3;

char message[] = {"08:00:00:00:00:00"};
void tapISR()
{
  tap = true;
}
void writeStringToFlash(const char *toStore, int startAddr)
{
  int i = 0;
  for (; i < LENGTH(toStore); i++)
  {
    EEPROM.write(startAddr + i, toStore[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}

String readStringFromFlash(int startAddr)
{
  char in[128]; // char array of size 128 for reading the stored data
  int i = 0;
  for (; i < 128; i++)
  {
    in[i] = EEPROM.read(startAddr + i);
  }
  return String(in);
}
void setup(void)
{
  Serial.begin(115200);
  Serial.println("matrixClock");

  P.begin();
  P.setZone(TEXT_ZONE, TXT_LOWER, TXT_UPPER);
  P.setIntensity(15);
  pM = P.getGraphicObject();
  strcpy(message, "Wifi...");

  P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
  P.displayAnimate();

  pinMode(WiFi_rst, INPUT_PULLUP);

  while (!EEPROM.begin(EEPROM_SIZE))
  { // Init EEPROM
    strcpy(message, "ERROR 1");

    P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
    P.displayAnimate();

    Serial.println("failed to init EEPROM");
    delay(1000);
  }

  Serial.println("reading wifi info ");

  ssid = readStringFromFlash(0); // Read SSID stored at address 0
  Serial.print("SSID = ");
  Serial.println(ssid);
  pss = readStringFromFlash(40); // Read Password stored at address 40
  Serial.print("psss = ");
  Serial.println(pss);

  WiFi.begin(ssid.c_str(), pss.c_str());
  delay(3500);
  if (WiFi.status() != WL_CONNECTED) // if WiFi is not connected
  {
    Serial.println("wifi init");

    // Init WiFi as Station, start SmartConfig
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    // Wait for SmartConfig packet from mobile
    Serial.println("Waiting for SmartConfig.");

    while (!WiFi.smartConfigDone())
    {
      strcpy(message, "Wifi.");
      P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
      P.displayAnimate();
      delay(500);
      strcpy(message, "Wifi..");
      P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
      P.displayAnimate();
      delay(500);
      strcpy(message, "Wifi...");
      P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
      P.displayAnimate();
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("SmartConfig received.");

    // Wait for WiFi to connect to AP
    Serial.println("Waiting for WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }

    Serial.println("WiFi Connected.");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // read the connected WiFi SSID and password
    ssid = WiFi.SSID();
    pss = WiFi.psk();
    Serial.print("SSID:");
    Serial.println(ssid);
    Serial.print("PSS:");
    Serial.println(pss);
    Serial.println("Store SSID & PSS in Flash");
    writeStringToFlash(ssid.c_str(), 0); // storing ssid at address 0
    writeStringToFlash(pss.c_str(), 40); // storing pss at address 40
  }
  else
  {
    Serial.println("WiFi Connected");
  }

  pinMode(int1Pin, INPUT);

  delay(100);

  // 初始化NTP客户端
  timeClient.begin();
  // 更新时间
  timeClient.update();

  // 获取当前时间
  String formattedTime = timeClient.getFormattedTime();
  strcpy(message, formattedTime.substring(0, 5).c_str());
  P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);

  // 输出当前时间
  Serial.println(formattedTime);
  P.displayAnimate();

  while (myAHT20.begin() != true)
  {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    strcpy(message, "ERROR 2");

    P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
    P.displayAnimate();

    delay(1000);
  }
  Serial.println(F("AHT20 OK"));

  while (!myAcc.init())
  {
    Serial.println("ADXL345 not connected!");
    strcpy(message, "ERROR 3");

    P.displayZoneText(TEXT_ZONE, message, SCROLL_ALIGN, SCROLL_SPEED, SCROLL_PAUSE, SCROLL_EFFECT, SCROLL_EFFECT);
    P.displayAnimate();
    delay(1000);
  }
  myAcc.setDataRate(ADXL345_DATA_RATE_200);
  Serial.print("Data rate: ");
  Serial.print(myAcc.getDataRateAsString());

  myAcc.setRange(ADXL345_RANGE_2G);
  Serial.print("  /  g-Range: ");
  Serial.println(myAcc.getRangeAsString());
  Serial.println();

  attachInterrupt(digitalPinToInterrupt(int1Pin), tapISR, RISING);
  myAcc.setGeneralTapParameters(ADXL345_XYZ, 3.0, 30, 100.0);
  myAcc.setAdditionalDoubleTapParameters(false, 250);
  myAcc.setInterrupt(ADXL345_DOUBLE_TAP, INT_PIN_2);
  myAcc.readAndClearInterrupts();
}
// 上次更新时间的时间戳
unsigned long lastUpdateTime = 0;
unsigned long lastUpdateTime2 = 0;

// 定义呼吸灯周期和亮暗程度
const int breathePeriod = 2000; // 2 秒钟一个周期
const int minBrightness = 0;
const int maxBrightness = 6;
int displayMode = 0;
float temp, humidity;
String formattedTime;
#define DURATION 3000
void loop(void)
{
  P.displayAnimate();
  // 计算当前时间在呼吸周期内的百分比
  unsigned long currentMillis = millis();
  float percentage = (float)(currentMillis % breathePeriod) / breathePeriod;

  // 根据百分比计算当前亮度
  int brightness;
  if (percentage < 0.5)
  { // 从暗到亮
    brightness = minBrightness + (maxBrightness - minBrightness) * 2 * percentage;
  }
  else
  { // 从亮到暗
    brightness = maxBrightness - (maxBrightness - minBrightness) * 2 * (percentage - 0.5);
  }

  // 设置 LED 亮度
  P.setIntensity(brightness);

  // 延迟 10 毫秒,让呼吸效果更平滑
  delay(10);
  if (displayMode == 1)
  {
    sprintf(message, "%d'%d%%", (int)temp, (int)humidity);
    P.displayReset();
    delay(30);

    if (millis() - lastUpdateTime2 >= DURATION)
    {
      displayMode = 0;
      lastUpdateTime2 = millis();
      strcpy(message, formattedTime.substring(0, 5).c_str());
      P.displayReset();
    }
  }
  else
  {
    lastUpdateTime2 = millis();
  }

  // 时间更新
  if (millis() - lastUpdateTime >= 60000)
  {
    // 更新时间
    timeClient.update();

    // 获取当前时间
    formattedTime = timeClient.getFormattedTime();

    // 输出当前时间
    Serial.println(formattedTime);
    strcpy(message, formattedTime.substring(0, 5).c_str());
    P.displayReset();

    // 更新上次更新时间的时间戳
    lastUpdateTime = millis();
  }
  // 拍击检测
  if (tap == true)
  {
    // byte actTapSource = myAcc.getActTapStatus();
    // Serial.println(actTapSource, BIN);
    String axes = myAcc.getActTapStatusAsString();
    byte intSource = myAcc.readAndClearInterrupts();
    if (myAcc.checkInterrupt(intSource, ADXL345_DOUBLE_TAP))
    {
      Serial.print("DOUBLE TAP at: ");
      Serial.println(axes);
      temp = myAHT20.readTemperature(AHT10_FORCE_READ_DATA);
      humidity = myAHT20.readHumidity(AHT10_USE_READ_DATA);
      displayMode = 1;
    }

    myAcc.readAndClearInterrupts();
    tap = false;
  }

  rst_millis = millis();
  while (digitalRead(WiFi_rst) == LOW)
  {
    // Wait till boot button is pressed

    // check the button press time if it is greater than 3sec clear wifi cred and restart ESP
    if (millis() - rst_millis >= 3000)
    {
      Serial.println("Reseting the WiFi credentials");
      writeStringToFlash("", 0);  // Reset the SSID
      writeStringToFlash("", 40); // Reset the Password
      Serial.println("Wifi credentials erased");
      Serial.println("Restarting the ESP");
      delay(500);
      ESP.restart(); // Restart ESP
    }
  }
}