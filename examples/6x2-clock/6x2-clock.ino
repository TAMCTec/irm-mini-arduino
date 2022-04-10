#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <irm_mini.h>
#include <WiFi.h>
#include "time.h"
#include "sntp.h"
#include <HTTPClient.h>

#include "secret.h"
#include "weather_icon.h"

#define PIN 13
#define BRIGHTNESS 2

#define RED   matrix->Color(200,  20,  20)
#define WHITE matrix->Color(255, 255, 255)
#define GREY  matrix->Color(180, 180, 180)
#define WEATHER_CITY_NAME "ShenZhen"
#define WEATHER_UNITS     "metric"

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600 * 8; // GMT +8
const int   daylightOffset_sec = 0;
String weatherUrl = String("https://api.openweathermap.org/data/2.5/weather?q=") + \
  WEATHER_CITY_NAME + "&appid=" + WEATHER_API_KEY + "&units=" + WEATHER_UNITS;

uint16_t millisCounter = 0;

// Define full matrix width and height.
#define mw 48
#define mh 16
IRM_Mini *matrix = new IRM_Mini(
  8, 8, 6, 2, PIN,
  NEO_MATRIX_TOP  + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG + 
  NEO_TILE_TOP    + NEO_TILE_LEFT +
  NEO_TILE_ROWS   + NEO_TILE_ZIGZAG,
  NEO_RGB         + NEO_KHZ800 );

void setup() {
  Serial.begin(115200);
  matrix->begin();
  matrix->setTextWrap(false);
  matrix->setBrightness(BRIGHTNESS);
  
  sntp_servermode_dhcp(1);    // (optional)
  sntp_setservername(0, ntpServer1);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.printf("Connecting to %s ", SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");

}

void loop() {
  // matrix->clear();
  matrix->fillScreen(GREY);
  // drawTime();
  // testIcon();
  testWeather();
  matrix->show();
  delay(5000);
  // if ((millis() - millisCounter) % 1000 == 0) {
  //   drawTime();
  // }
  // if (millis() - millisCounter % 5000 == 0) {
  //   drawWeather();
  //   millisCounter = millis();
  // }
}

void drawTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  String timeStr;
  String hour = String(timeinfo.tm_hour);
  if (hour.length() == 1) hour = "0" + hour;
  String minute = String(timeinfo.tm_min);
  if (minute.length() == 1) minute = "0" + minute;
  timeStr = hour + ":" + minute;
  matrix->drawAscii(48-19, 0, timeStr, WHITE, FONT5);
}

void testIcon() {
  for (uint i=0; i<=16; i++) {
  matrix->fillScreen(GREY);
    matrix->drawRGBBitmap(0, 0, WEATHER_ICONS[i], 16, 16);
    Serial.println(WEATHER_ICON_LABLES[i]);
    delay(1000);
    matrix->show();
  }
}

void testWeather() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  HTTPClient http;

  Serial.print("[HTTP] ");
  Serial.println(weatherUrl);
  // configure traged server and url
  //http.begin("https://www.howsmyssl.com/a/check", ca); //HTTPS
  http.begin(weatherUrl.c_str()); //HTTP

  Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();
  Serial.printf("[HTTP] Done. code: %d\n", httpCode);

  // httpCode will be negative on error
  if(httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      // Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      // file found at server
      if(httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
          Serial.println(payload);
      }
  } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}