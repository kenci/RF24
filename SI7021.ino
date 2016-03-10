/* Sketch with Si7021 and battery monitoring.
by m26872, 20151109
21 - temp_badezimmer *
22 - temp_schlafzimmer
23 - temp_kindnord *
24 - temp_kindsued *
25 - temp_dachboden
26 - fenster_schlafzimmer
27 - fenster_badezimmer
28 - fenster_kindnord
29 - fenster_kindsued
*/
#include <MySensor.h>  
#include <Wire.h>
#include <SI7021.h>
#include <SPI.h>
#include <RunningAverage.h>
#include <Vcc.h>

#ifdef DEBUG
#define DEBUG_SERIAL(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_SERIAL(x)
#define DEBUG_PRINT(x) 
#define DEBUG_PRINTLN(x) 
#endif

#define NODE_ID 21             // Node ID
#define SKETCH_INFO "Temp BZ"
#define SKETCH_VERSION "2.3 10032016"
#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
// #define SLEEP_TIME 15000 // 15s for DEBUG
#define SLEEP_TIME 300000   // 5 min
#define FORCE_TRANSMIT_CYCLE 6  // half an hour |  5min*12=1/hour, 5min*36=1/3hour 
#define BATTERY_REPORT_CYCLE 2880   // Once per 5min   =>   12*24*7 = 2016 (one report/week)
//#define VMIN 1.70
//#define VMAX 3.24
#define HUMI_TRANSMIT_THRESHOLD 2.0  // THRESHOLD tells how much the value should have changed since last time it was transmitted.
#define TEMP_TRANSMIT_THRESHOLD 0.5
#define AVERAGES 2

const float vccMin   = 1.70;           // Minimum expected Vcc level, in Volts.
const float vccMax   = 3.24;
const float vccCorrection = 3.35/3.31; // Measured Vcc by multimeter divided by reported Vcc
Vcc vcc(vccCorrection);

int batteryReportCounter = BATTERY_REPORT_CYCLE - 1;  // to make it report the first time.
int measureCount = 0;
float lastTemperature = -100;
int lastHumidity = -100;

RunningAverage raHum(AVERAGES);
SI7021 humiditySensor;

MySensor gw;
MyMessage msgTemp(CHILD_ID_TEMP,V_TEMP); // Initialize temperature message
MyMessage msgHum(CHILD_ID_HUM,V_HUM);

void setup() {
  DEBUG_SERIAL(9600);    
  DEBUG_PRINTLN("Serial started");
  
  DEBUG_PRINT("Voltage: ");
  DEBUG_PRINT(vcc.Read_Volts()); 
  DEBUG_PRINT("Percent: ");
  DEBUG_PRINT(vcc.Read_Perc(vccMin, vccMax)); 
  DEBUG_PRINTLN(" %");

  delay(500); // Allow time for radio if power used as reset
  gw.begin(NULL,NODE_ID);
  gw.sendSketchInfo(SKETCH_INFO, SKETCH_VERSION); 
  gw.present(CHILD_ID_TEMP, S_TEMP);   // Present sensor to controller
  gw.present(CHILD_ID_HUM, S_HUM);
  DEBUG_PRINT("Node and "); DEBUG_PRINTLN("2 children presented.");
  
  raHum.clear();
  
}

void loop() { 

  measureCount ++;
  batteryReportCounter ++;
  bool forceTransmit = false;
  
  if (measureCount > FORCE_TRANSMIT_CYCLE) {
    forceTransmit = true; 
  }
  sendTempHumidityMeasurements(forceTransmit);

  // Check battery
  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    float batteryVolt = vcc.Read_Volts();
    DEBUG_PRINT("Battery voltage: "); DEBUG_PRINT(batteryVolt); DEBUG_PRINTLN(" V");
    uint8_t batteryPcnt = vcc.Read_Perc(vccMin, vccMax);   //constrain(map(batteryVolt,VMIN,VMAX,0,100),0,255);
    DEBUG_PRINT("Battery percent: "); DEBUG_PRINT(batteryPcnt); DEBUG_PRINTLN(" %");
    gw.sendBatteryLevel(batteryPcnt);
    batteryReportCounter = 0;
  }
  
  gw.sleep(SLEEP_TIME);
}

/*********************************************
 * * Sends temperature and humidity from Si7021 sensor
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHumidityMeasurements(bool force) {
  bool tx = force;

  si7021_env data = humiditySensor.getHumidityAndTemperature();
  
  float temperature = data.celsiusHundredths / 100.0;
  DEBUG_PRINT("T: ");DEBUG_PRINTLN(temperature);
  float diffTemp = abs(lastTemperature - temperature);
  DEBUG_PRINT(F("TempDiff :"));DEBUG_PRINTLN(diffTemp);
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD || tx) {
    gw.send(msgTemp.set(temperature,1));
    lastTemperature = temperature;
    measureCount = 0;
    DEBUG_PRINTLN("T sent!");
  }
  
  int humidity = data.humidityPercent;
  DEBUG_PRINT("H: ");DEBUG_PRINTLN(humidity);
  raHum.addValue(humidity);
  humidity = raHum.getAverage();  // MA sample imply reasonable fast sample frequency
  float diffHum = abs(lastHumidity - humidity);  
  DEBUG_PRINT(F("HumDiff  :"));DEBUG_PRINTLN(diffHum); 
  if (diffHum > HUMI_TRANSMIT_THRESHOLD || tx) {
    gw.send(msgHum.set(humidity));
    lastHumidity = humidity;
    measureCount = 0;
    DEBUG_PRINTLN("H sent!");
  }
}
