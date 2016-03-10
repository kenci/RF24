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

#define NODE_ID 22             // Node ID
#define SKETCH_INFO "TempReed SZ"
#define SKETCH_VERSION "3.3 10032016"
#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define REPEAT_SENDING 10
//Reed
#define CHILD_ID_REED 2
#define REED_BUTTON_PIN  2  // Arduino Digital I/O pin for button/reed switch
#define SLEEP_TIME 300000                 // Sleep time between reports (in milliseconds)
// 1h = 3600000
// 30m = 1800000
// 15m = 900000
// 5m = 300000
#define FORCE_TRANSMIT_CYCLE 6  // half an hour | 5min*12=1 per hour, 5min*36=1 per 3 hour 
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
// Reed Switch
int oldValue=-1;
int wakeUp = 0;
int reedCount = 0;

RunningAverage raHum(AVERAGES);
SI7021 humiditySensor;

MySensor mys;
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP); // Initialize temperature message
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgReed(CHILD_ID_REED, V_TRIPPED);

void setup() {
  DEBUG_SERIAL(9600);    
  DEBUG_PRINTLN("Serial started");
  
  DEBUG_PRINT("Voltage: ");
  DEBUG_PRINT(vcc.Read_Volts()); 
  DEBUG_PRINT("Percent: ");
  DEBUG_PRINT(vcc.Read_Perc(vccMin, vccMax)); 
  DEBUG_PRINTLN(" %");

  delay(500); // Allow time for radio if power used as reset
  mys.begin(NULL,NODE_ID);
  mys.sendSketchInfo(SKETCH_INFO, SKETCH_VERSION); 
  mys.present(CHILD_ID_TEMP, S_TEMP);   // Present sensor to controller
  mys.present(CHILD_ID_HUM, S_HUM);
  // Reed Switch
  pinMode(REED_BUTTON_PIN, INPUT_PULLUP);
  // Register binary input sensor to mys (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  mys.present(CHILD_ID_REED, S_DOOR);
  
  DEBUG_PRINT("Node and "); DEBUG_PRINTLN("2 children presented.");
  
  raHum.clear();
  
}

void loop() {
  bool forceTransmit = false;
  mys.sleep(20);
  
  measureCount ++;
  batteryReportCounter ++;
  
  if (measureCount > FORCE_TRANSMIT_CYCLE) {
    forceTransmit = true; 
    measureCount = 0;
  }
  
  sendReedChange(forceTransmit);
  sendTempHum(forceTransmit);
  sendBattery();
  
  DEBUG_PRINTLN("Going to sleep...");
  DEBUG_PRINTLN("");
  wakeUp = mys.sleep(REED_BUTTON_PIN-2, CHANGE, SLEEP_TIME);
}

void sendReedChange(bool force) {
  int value = digitalRead(REED_BUTTON_PIN);
 
  if (value != oldValue or force == true) {
     // Send in the new value
     DEBUG_PRINT("Reed Switch #1: "); DEBUG_PRINT(value==HIGH ? 1 : 0); DEBUG_PRINTLN("");
     resend((msgReed.set(value == HIGH ? "0" : "1")), REPEAT_SENDING, force);
     oldValue = value;
  }
}

/*********************************************
 * * Sends temperature and humidity from Si7021 sensor
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHum(bool force) {
  bool tx = force;

  si7021_env data = humiditySensor.getHumidityAndTemperature();
  
  float temperature = data.celsiusHundredths / 100.0;
  DEBUG_PRINT("T: ");DEBUG_PRINTLN(temperature);
  float diffTemp = abs(lastTemperature - temperature);
  DEBUG_PRINT(F("TempDiff :"));DEBUG_PRINTLN(diffTemp);
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD || tx) {
    mys.send(msgTemp.set(temperature,1));
    lastTemperature = temperature;
    measureCount = 0;
    DEBUG_PRINTLN("T sent!");
  }
  
  int humidity = data.humidityPercent;
  DEBUG_PRINT("H: ");DEBUG_PRINTLN(humidity);
  raHum.addValue(humidity);
  humidity = raHum.getAverage();  // MA sample imply reasonable fast sample frequency
  float diffHum = abs(lastHumidity - humidity);  
  DEBUG_PRINT(F("HumDiff  :")); DEBUG_PRINTLN(diffHum); 
  if (diffHum > HUMI_TRANSMIT_THRESHOLD || tx) {
    mys.send(msgHum.set(humidity));
    lastHumidity = humidity;
    measureCount = 0;
    DEBUG_PRINTLN("H sent!");
  }
}
void sendBattery() // Measure battery
{
  // Check battery
  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    float batteryVolt = vcc.Read_Volts();
    DEBUG_PRINT("Battery voltage: "); DEBUG_PRINT(batteryVolt); DEBUG_PRINTLN(" V");
    uint8_t batteryPcnt = vcc.Read_Perc(vccMin, vccMax);   //constrain(map(batteryVolt,VMIN,VMAX,0,100),0,255);
    DEBUG_PRINT("Battery percent: "); DEBUG_PRINT(batteryPcnt); DEBUG_PRINTLN(" %");
    mys.sendBatteryLevel(batteryPcnt);
    batteryReportCounter = 0;
  }
}

void resend(MyMessage &msg, int repeats, bool force)
{
  int repeat = 1;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    DEBUG_PRINT(force ? "F " : "S ");
    if (mys.send(msg)) {
      sendOK = true;
    } else {
      sendOK = false;
      DEBUG_PRINT("Send ERROR ");
      DEBUG_PRINTLN(repeat);
      repeatdelay += 250;
      //led(true,1,5);
    } repeat++; delay(repeatdelay);
  }
}
