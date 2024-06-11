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
// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adapted
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN 2
#define DATA_PIN 3
#define CS_PIN 4

// 替换成你的WiFi网络名称和密码
const char *ssid = "TP-LINK_8C30";
const char *password = "z888888889";

// 配置NTP客户端
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com", 28800, 60000);
// HARDWARE SPI
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
// SOFTWARE SPI
// MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_MAX72XX *pM;

// Display parameters
const uint8_t SCROLL_SPEED = 50; // text speed
const textEffect_t SCROLL_EFFECT = PA_NO_EFFECT;
const textPosition_t SCROLL_ALIGN = PA_CENTER;
const uint16_t SCROLL_PAUSE = 0; // in milliseconds

const uint8_t TEXT_ZONE = 0; // just change this to adjust display
const uint8_t TXT_LOWER = 0;
const uint8_t TXT_UPPER = 3;

char message[] = {"08:00:00:00:00:00"};

void setup(void)
{
  Serial.begin(115200);
  P.begin();
  P.setZone(TEXT_ZONE, TXT_LOWER, TXT_UPPER);
  P.setIntensity(15);
  pM = P.getGraphicObject();

  delay(100);

  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

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
}
// 上次更新时间的时间戳
unsigned long lastUpdateTime = 0;
// 定义呼吸灯周期和亮暗程度
const int breathePeriod = 2000; // 2 秒钟一个周期
const int minBrightness = 3;
const int maxBrightness = 15;
void loop(void)
{
  // bounceBall();
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
  if (millis() - lastUpdateTime >= 60000)
  {
    // 更新时间
    timeClient.update();

    // 获取当前时间
    String formattedTime = timeClient.getFormattedTime();

    // 输出当前时间
    Serial.println(formattedTime);
    strcpy(message, formattedTime.substring(0, 5).c_str());
    P.displayReset();

    // 更新上次更新时间的时间戳
    lastUpdateTime = millis();
  }
}