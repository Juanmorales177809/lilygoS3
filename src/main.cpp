#include <SensirionI2CScd4x.h>
#include <Adafruit_Sensor.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <WiFiClientSecure.h>
#include <ThingsBoard.h>
#include "Adafruit_SGP40.h"



//==============================================================================
#define THINGSBOARD_MQTT_SERVER       "thingsboard.cloud"
#define THINGSBOARD_MQTT_ACESSTOKEN   "ZV5l5sPY7xWPwTo7VGzL"
#define THINGSBOARD_MQTT_PORT         1883
//==============================================================================