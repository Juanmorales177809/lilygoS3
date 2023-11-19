#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include "Adafruit_SHT4x.h"
#include "sps30.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#define THI false

#if THI
#include <ThingsBoard.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#endif


#define DEBUG 0

SensirionI2CScd4x scd4x;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

//==============================================================================
const uint32_t  READ_DELAY  = 30000;
static uint32_t msDelay     = millis();
const uint32_t  SENS_DELAY  = 50;
#define         WDT_TIMEOUT 70
//==============================================================================
#define SP30_COMMS Wire
//==============================================================================
WiFiClient espClient;
ThingsBoard tb(espClient);
int status = WL_IDLE_STATUS;
//==============================================================================
#define THINGSBOARD_MQTT_SERVER       "thingsboard.cloud"
#define THINGSBOARD_MQTT_ACESSTOKEN   "ZV5l5sPY7xWPwTo7VGzL"
#define THINGSBOARD_MQTT_PORT         1883
//==============================================================================
SPS30     sps30;
bool      SPS30B;                                                               //Begin out
bool      SPS30P;                                                               //Probe out
bool      SPS30R;                                                               //Reset out
bool      SPS30I;                                                               //Start out
uint8_t   SPS30V;                                                               //Get values out
uint8_t   SPS30G;                                                               //Get status
//==============================================================================
int       i = 0;
struct    sps_values val;
float     SPS30_MCR[4];
float     SPS30_NPR[4];
float     SPS30_PSZ;
//==============================================================================
#define WIFISSID          "ces"                                             //WiFi SSID here
#define PASSWORD          "univerces"                                        //WiFi password here
// #define WIFISSID          "Familia Morales"                                            //WiFi SSID here
// #define PASSWORD          "2205631700"                                                    //WiFi password here

//==============================================================================
const uint32_t  READ_DELAY  = 2000;
static uint32_t msDelay     = millis();
const uint32_t  SENS_DELAY  = 50;
#define         WDT_TIMEOUT 70
//==============================================================================



void setup() {

  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Wire.begin(43,44);
  sht4.begin();
  scd4x.begin(Wire);
  //------------------------------------------------------------------------------ 
  WiFi.begin(WIFISSID, PASSWORD);
//------------------------------------------------------------------------------
  while (WiFi.status() != WL_CONNECTED){Serial.print(".");delay(500);}
  Serial.println();
//------------------------------------------------------------------------------
  timeClient.begin();
  timeClient.setUpdateInterval(28800000);                                       //Milliseconds
  timeClient.update();
//------------------------------------------------------------------------------
  //================================================================
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
   Serial.println("SYSTM_INI_OK");
  //================================================================
   SP30_COMMS.begin(43,44);
   sps30.begin(&SP30_COMMS);
  // stop potentially previously started measurement
  // Start Measurement
  scd4x.startPeriodicMeasurement();
  Serial.println("Waiting for first measurement... (5 sec)");
  //------------------------------------------------------------------------------
  sps30.EnableDebugging(DEBUG);
  SP30_COMMS.begin();  
  SPS30B =      sps30.begin(&SP30_COMMS);
  if(SPS30B)    Serial.println("SPS30_INI_OK");
  else          Serial.println("SPS30_INI_ER");
//------------------------------------------------------------------------------  
  SPS30P =      sps30.probe();
  if(SPS30P)    Serial.println("SPS30_PRO_OK");
  else          Serial.println("SPS30_PRO_ER");
//------------------------------------------------------------------------------    
  SPS30R =      sps30.reset();
  if(SPS30R)    Serial.println("SPS30_RES_OK");
  else          Serial.println("SPS30_RES_ER");
//------------------------------------------------------------------------------  
  SPS30I =      sps30.start();
  if(SPS30I)    Serial.println("SPS30_STA_OK");
  else          Serial.println("SPS30_STA_ER");
//------------------------------------------------------------------------------
delay(200);
}

void loop() {
  char buff[100];
  sensors_event_t humedad, temperatura; //sht40
  sht4.getEvent(&humedad, &temperatura);//sht40
//==========================================================================
  if(SPS30B)  SPS30V = sps30.GetValues(&val);
  delay(SENS_DELAY);
//==========================================================================
    // Read Measurement
  uint16_t co2 = 0;//SCD40
  float temperature = 0.0f; //SCD40
  float humidity = 0.0f; //SCD40
  bool isDataReady = false; //SCD40
  scd4x.readMeasurement(co2, temperature, humidity); //SCD40
//=========================================================================
if(WiFi.status() != WL_CONNECTED)
    {
    reconnect();
    return;
    }
// Mass Concentration P1.0 P2.5 P4.0 P10
  SPS30_MCR[0] = val.MassPM1;
  SPS30_MCR[1] = val.MassPM2;
  SPS30_MCR[2] = val.MassPM4;
  SPS30_MCR[3] = val.MassPM10; 
//------------------------------------------------------------------------------
  sprintf(buff,"[SPS30] PM1: %08.2f, PM2: %08.2f, PM4: %08.2f, P10: %08.2f", SPS30_MCR[0], SPS30_MCR[1], SPS30_MCR[2], SPS30_MCR[3]);
  Serial.println(buff);
  delay(SENS_DELAY);
//=========================================================================
//------------------------------------------------------------------------------
  if(!tb.connected())
    {
    if(!tb.connect(THINGSBOARD_MQTT_SERVER, THINGSBOARD_MQTT_ACESSTOKEN, THINGSBOARD_MQTT_PORT)) 
      {
      Serial.println("Failed to connect");
      return;
      }
    }
//------------------------------------------------------------------------------
  // tb.sendTelemetryFloat("T1", SD40[0]);
  // tb.sendTelemetryFloat("T2", BME6[0]); 
  tb.sendTelemetryFloat("T3", SPS30_MCR[0]);
  tb.sendTelemetryFloat("T4", SPS30_MCR[1]);
  tb.sendTelemetryFloat("T5", SPS30_MCR[2]);
  tb.sendTelemetryFloat("T6", SPS30_MCR[3]);
  tb.sendTelemetryFloat("T7", SPS30_PSZ);
  tb.sendTelemetryFloat("T8", co2);
  tb.sendTelemetryFloat("T9", temperature);
  tb.sendTelemetryFloat("T10", humidity);
  tb.sendTelemetryFloat("T11", temperatura.temperature);
  tb.sendTelemetryFloat("T12", humedad.relative_humidity);
  tb.sendTelemetryFloat("T13", SPS30_NPR[0]);
  tb.sendTelemetryFloat("T14", SPS30_NPR[1]);
  tb.sendTelemetryFloat("T15", SPS30_NPR[2]);
  tb.sendTelemetryFloat("T16", SPS30_NPR[3]);
  tb.sendTelemetryFloat("T17", SPS30_NPR[4]);



  tb.loop();
  Serial.print("Co2:"); //SCD40
  Serial.print(co2);//SCD40
  Serial.print("\t");//SCD40
  Serial.print("Temperature:");//SCD40
  Serial.print(temperature);//SCD40
  Serial.print("\t");//SCD40
  Serial.print("Humidity:");//SCD40
  Serial.println(humidity);//SCD40

  Serial.print("Temperatura_SHT40: ");
  Serial.print(temperatura.temperature); 
  Serial.println(" degrees C"); //SHT40
  Serial.print("Humedad_SHT40: "); 
  Serial.print(humedad.relative_humidity); 
  Serial.println("% rH");//SHT40
        
 
  


  delay(10000);
}
void reconnect() 
{                                                                               
//------------------------------------------------------------------------------
  status = WiFi.status();
  if(status != WL_CONNECTED)
    {
    WiFi.begin(WIFISSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) 
      {
      delay(500);
      Serial.print(".");
      }
    Serial.println(WiFi.localIP());
    Serial.println("Connected to AP");
    }
}

