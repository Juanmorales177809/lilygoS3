#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include "Adafruit_SHT4x.h"
#include "sps30.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SleepyDog.h> //Include the watchdog library
#include "TFT_eSPI.h"
#include "iconos_bme280_140x170.h"
#define THI true
#define ANYSPS30 true
#define CES true

#if THI
#include <ThingsBoard.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#endif

#define DEBUG 0

#if ANYSPS30
SensirionI2CScd4x scd4x;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif
//==============================================================================
const uint32_t READ_DELAY = 10000;
static uint32_t msDelay = millis();
const uint32_t SENS_DELAY = 50;
#define WDT_TIMEOUT 70
//==============================================================================
#define SPS30_COMMS SERIALPORT1
#define TX_PIN 17
#define RX_PIN 18
//==============================================================================
#if THI
WiFiClient espClient;
ThingsBoard tb(espClient);
int status = WL_IDLE_STATUS;
//==============================================================================

#define THINGSBOARD_MQTT_SERVER "thingsboard.cloud"
#define THINGSBOARD_MQTT_ACESSTOKEN "DDEGVdhkiNAvoM6LbxL1"
#define THINGSBOARD_MQTT_PORT 1883
#endif
//==============================================================================
TFT_eSPI tft= TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
//==============================================================================
SPS30 sps30;
bool SPS30B;    // Begin out
bool SPS30P;    // Probe out
bool SPS30R;    // Reset out
bool SPS30I;    // Start out
uint8_t SPS30V; // Get values out
uint8_t SPS30G; // Get status
//==============================================================================
int i = 0;
struct sps_values val;
float SPS30_MCR[4];
float SPS30_NPR[4];
float SPS30_PSZ;
//==============================================================================
#if THI
#if CES
#define WIFISSID "UCES_Concesionarios"       // WiFi SSID here
#define PASSWORD "Cafeterias2023*.*" // WiFi password here
#else
#define WIFISSID          "Familia Morales"                                            //WiFi SSID here
#define PASSWORD          "2205631700"                                                    //WiFi password here
#endif
#endif
//==============================================================================

void setup()
{
  Watchdog.enable(360000); //Enable the watchdog with a 6 minute timeout
  Serial.begin(115200);
#if ANYSPS30
  Wire.begin(43, 44);
  sht4.begin();
  scd4x.begin(Wire);
  scd4x.stopPeriodicMeasurement();
#endif
//------------------------------------------------------------------------------
#if THI
#include <WiFiUdp.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WiFiUdp.h>


WiFiUDP ntpUDP;

//==============================================================================
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_WHITE);
  tft.pushImage(180,0,140,170,iconos_bme280_140x170);

  sprite.createSprite(179,150);
 
  sprite.setTextColor(TFT_BLACK,TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(1);
  tft.drawString("Universidad CES",10,160);
  tft.setTextSize(2);
  bool status;


  //==============================================================================
WiFi.begin(WIFISSID, PASSWORD);
//------------------------------------------------------------------------------
while (WiFi.status() != WL_CONNECTED)
{
  Serial.print(".");
  delay(500);
}
Serial.println();
#endif
//================================================================
#if ANYSPS30
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  Serial.println("SYSTM_INI_OK");
#endif
  //================================================================
  sps30.EnableDebugging(DEBUG);
  sps30.SetSerialPin(RX_PIN, TX_PIN);
  if (!sps30.begin(SPS30_COMMS))
    Serial.println("could not connect with SPS30.");
  else
    Serial.println("Detected SPS30.");
  if (!sps30.probe())
    Serial.println("could not probe / connect with SPS30.");
  else
    Serial.println("Detected SPS30.");
  if (sps30.reset())
    Serial.println("SPS30_RES_OK");
  else
    Serial.println("SPS30_RES_ER");
  if (sps30.start())
    Serial.println("SPS30_STA_OK");
  else
    Serial.println("SPS30_STA_ER");

// stop potentially previously started measurement
// Start Measurement
#if ANYSPS30
  scd4x.startPeriodicMeasurement();
  Serial.println("Waiting for first measurement... (5 sec)");
#endif
   status = sps30.begin(SPS30_COMMS);  
  if (!status) {
    tft.drawString("No se encuentra SPS30",10,50);
    while (1);
  }
  delay(READ_DELAY);
  
}

void loop()
{
  struct sps_values val;
  char buff[100];
#if ANYSPS30
  uint16_t co2 = 0;                                  // SCD40
  float temperature = 0.0f;                          // SCD40
  float humidity = 0.0f;                             // SCD40
  bool isDataReady = false;                          // SCD40
  float promedio_temperatura = 0.0f, promedio_humedad = 0.0f; // SCD40
  float promedio_co2 = 0.0f;                         // SCD40

#endif

  //==========================================================================
  sps30.GetValues(&val);
  delay(SENS_DELAY);

//==========================================================================
#if ANYSPS30
  // Read Measurement
  sensors_event_t humedad, temperatura;  // sht40
  sht4.getEvent(&humedad, &temperatura); // sht40
  scd4x.readMeasurement(co2, temperature, humidity); // SCD40
  promedio_temperatura = (temperatura.temperature + temperature) / 2; //promedio de temperatura
  promedio_humedad = (humedad.relative_humidity + humidity) / 2;     //promedio de humedad
  delay(READ_DELAY);
#endif
//=========================================================================
#if THI
  if (!tb.connected())
  {
    if (!tb.connect(THINGSBOARD_MQTT_SERVER, THINGSBOARD_MQTT_ACESSTOKEN, THINGSBOARD_MQTT_PORT))
    {
      Serial.println("Failed to connect");
      return;
    }
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    return;
  }

#endif
  // Mass Concentration P1.0 P2.5 P4.0 P10
  SPS30_MCR[0] = val.MassPM1;
  SPS30_MCR[1] = val.MassPM2;
  SPS30_MCR[2] = val.MassPM4;
  SPS30_MCR[3] = val.MassPM10;
  //------------------------------------------------------------------------------
  sprintf(buff, "[SPS30] PM1: %08.2f, PM2: %08.2f, PM4: %08.2f, P10: %08.2f", SPS30_MCR[0], SPS30_MCR[1], SPS30_MCR[2], SPS30_MCR[3]);
  Serial.println(buff);
  delay(SENS_DELAY);
  //=========================================================================
  // tb.sendTelemetryFloat("T1", SD40[0]);
  // tb.sendTelemetryFloat("T2", BME6[0]);
#if THI
  tb.sendTelemetryFloat("MassPM1", SPS30_MCR[0]);
  tb.sendTelemetryFloat("MassPM2", SPS30_MCR[1]);
  tb.sendTelemetryFloat("MassPM4", SPS30_MCR[2]);
  tb.sendTelemetryFloat("MassPM10", SPS30_MCR[3]);
  tb.sendTelemetryFloat("PartSize", SPS30_PSZ);
  tb.sendTelemetryFloat("CO2_SCD40", co2);
  tb.sendTelemetryFloat("TEMP", promedio_temperatura);
  tb.sendTelemetryFloat("HUMIDITY", promedio_humedad);
  tb.sendTelemetryFloat("NumPM0", SPS30_NPR[0]);
  tb.sendTelemetryFloat("NumPM1", SPS30_NPR[1]);
  tb.sendTelemetryFloat("NumPM2", SPS30_NPR[2]);
  tb.sendTelemetryFloat("NumPM4", SPS30_NPR[3]);
  tb.sendTelemetryFloat("NumPM10", SPS30_NPR[4]);

  tb.loop();
#endif
#if ANYSPS30
  Serial.print("Co2:");         // SCD40
  Serial.print(co2);            // SCD40
  Serial.print("\t");           // SCD40
  Serial.print("Temperature:"); // SCD40
  Serial.print(temperature);    // SCD40
  Serial.print("\t");           // SCD40
  Serial.print("Humidity:");    // SCD40
  Serial.println(humidity);     // SCD40

  Serial.print("Temperatura_SHT40: ");
  Serial.print(temperatura.temperature);
  Serial.println(" degrees C"); // SHT40
  Serial.print("Humedad_SHT40: ");
  Serial.print(humedad.relative_humidity);
  Serial.println("% rH"); // SHT40

#endif
  //==========================================================================
  sprite.fillSprite(TFT_WHITE);
  sprite.setFreeFont(&Orbitron_Light_32);
  sprite.drawString(String(promedio_temperatura),20,10);
  sprite.drawString(String(promedio_humedad),20,60);
  sprite.drawString(String(co2),20,110);
  sprite.pushSprite(0,0);
  //==========================================================================
  delay(READ_DELAY);
  Watchdog.reset(); //Reset the watchdog
}
void reconnect()
{
#if THI
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    WiFi.begin(WIFISSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(READ_DELAY);
      Serial.print(".");
    }
    Serial.println(WiFi.localIP());
    Serial.println("Connected to AP");
  }
#endif
}
