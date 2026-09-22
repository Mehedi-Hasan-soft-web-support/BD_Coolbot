/*
  IceSentinel — AC Manipulator Node
  ESP32 firmware

  Hardware:
    - 2x DS18B20  : one in the room (air), one on the AC coil/fins
    - 1x 0.96" OLED (SSD1306, I2C, 128x64)
    - Heater      : 8.2R resistor, driven by IRLZ44N MOSFET on PWM

  Wiring (matches the earlier wiring diagram):
    GPIO4  -> Room DS18B20 data   (+ 4.7k pull-up to 3.3V)
    GPIO5  -> Fins DS18B20 data   (+ 4.7k pull-up to 3.3V)
    GPIO21 -> OLED SDA
    GPIO22 -> OLED SCL
    GPIO25 -> 100R -> MOSFET Gate (heater PWM)

  Libraries (Arduino IDE > Tools > Manage Libraries):
    - OneWire
    - DallasTemperature
    - Adafruit GFX Library
    - Adafruit SSD1306
    - ArduinoJson (v6.x)

  Fill in WIFI_SSID / WIFI_PASSWORD below before uploading.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>

// ================================================================
// CONFIG — edit these
// ================================================================
const char* WIFI_SSID     = "Me";
const char* WIFI_PASSWORD = "mehedi113";

const char* SUPABASE_URL       = "https://lqximixglzjwezdbozrd.supabase.co";
const char* SUPABASE_ANON_KEY  = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImxxeGltaXhnbHpqd2V6ZGJvenJkIiwicm9sZSI6ImFub24iLCJpYXQiOjE3OTAwMzk0MjgsImV4cCI6MjEwNTYxNTQyOH0.rfsF1wwmq8AWdZe9YLp7qiF08SPKk90hs6bdb6KeWQw";
const char* DEVICE_ID           = "00000000-0000-0000-0000-000000000001";

// ================================================================
// PINS
// ================================================================
#define ONE_WIRE_ROOM 4
#define ONE_WIRE_FINS 5
#define HEATER_PIN    25
#define OLED_SDA      21
#define OLED_SCL      22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_I2C_ADDR 0x3C   // try 0x3D if the screen stays blank

// ================================================================
// SENSORS / DISPLAY
// ================================================================
OneWire oneWireRoom(ONE_WIRE_ROOM);
OneWire oneWireFins(ONE_WIRE_FINS);
DallasTemperature roomSensor(&oneWireRoom);
DallasTemperature finsSensor(&oneWireFins);
DeviceAddress roomAddr, finsAddr;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================================================================
// TIMING
// ================================================================
const unsigned long CONVERSION_TIME = 750;    // 12-bit DS18B20 conversion time
const unsigned long SYNC_INTERVAL   = 10000;  // push to Supabase every 10s
const unsigned long SAFETY_TIMEOUT  = 30000;  // force heater off if control loop stalls
const unsigned long WDT_TIMEOUT     = 15;     // seconds, hardware watchdog

bool conversionInProgress = false;
unsigned long conversionStart = 0;
unsigned long lastSync = 0;
unsigned long lastHeaterUpdate = 0;

// ================================================================
// LIVE STATE
// ================================================================
float roomTemp = NAN, finsTemp = NAN;
bool roomFault = true, finsFault = true;
bool firstReadingReceived = false;

int  heaterDuty = 0;
bool heaterOn = false;
bool iceRisk = false;

// Settings — pulled from Supabase; these are safe local defaults
// used until the first successful fetch.
bool  powerOn = true;
float setPoint = 7.0;
float notifyMin = 5.0, notifyMax = 9.0;
bool  antiFreezeEnabled = true;
bool  manualOverride = false;
int   manualOverrideDuty = 0;
const float ICE_RISK_TEMP   = 3.0;   // coil below this -> heater forced off
const float SAFE_RESUME_TEMP = 6.0;  // coil above this -> heater allowed again
const float HYSTERESIS = 0.5;        // deadband around set_point to stop on/off chatter
bool wasHeating = false;             // remembers state while inside the deadband

// Forced rest cycle — caps continuous runtime regardless of how far above
// set_point the room still is, so the coil gets a break before ice can build up.
// Tune these two during calibration testing.
const unsigned long MAX_CONTINUOUS_ON = 20UL * 60UL * 1000UL;  // 20 min max continuous run
const unsigned long FORCED_REST_TIME  = 5UL  * 60UL * 1000UL;  // 5 min forced rest
unsigned long heaterOnSince = 0;     // when the current ON streak started (0 = not running)
bool inForcedRest = false;
unsigned long restStartTime = 0;

// Moving-average filter, one per sensor
const int FILTER_SIZE = 5;
float roomBuf[FILTER_SIZE], finsBuf[FILTER_SIZE];
int roomIdx = 0, finsIdx = 0;
bool roomFilled = false, finsFilled = false;

float pushAvg(float* buf, int &idx, bool &filled, float val){
  buf[idx] = val;
  idx = (idx + 1) % FILTER_SIZE;
  if(idx == 0) filled = true;
  int count = filled ? FILTER_SIZE : idx;
  float sum = 0;
  for(int i = 0; i < count; i++) sum += buf[i];
  return sum / count;
}

// ================================================================
// SETUP
// ================================================================
void setup(){
  Serial.begin(115200);

  pinMode(HEATER_PIN, OUTPUT);
  ledcAttach(HEATER_PIN, 1000, 8);   // pin, freq 1kHz, 8-bit resolution (core v3.x API)
  ledcWrite(HEATER_PIN, 0);          // heater OFF until we have a valid reading

  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)){
    Serial.println("OLED not found — check wiring/address");
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("IceSentinel booting...");
  display.display();

  roomSensor.begin();
  finsSensor.begin();
  roomSensor.setWaitForConversion(false);
  finsSensor.setWaitForConversion(false);

  if(roomSensor.getAddress(roomAddr, 0)){
    roomSensor.setResolution(roomAddr, 12);
  } else {
    Serial.println("Room DS18B20 not found!");
  }
  if(finsSensor.getAddress(finsAddr, 0)){
    finsSensor.setResolution(finsAddr, 12);
  } else {
    Serial.println("Fins DS18B20 not found!");
  }

  connectWiFi();

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);    // core v3.x API (struct-based)
  esp_task_wdt_add(NULL);

  lastHeaterUpdate = millis();
}

void connectWiFi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - start < 15000){
    delay(300);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " failed, will retry");
}

// ================================================================
// SENSOR READING (non-blocking)
// ================================================================
void startConversion(){
  roomSensor.requestTemperaturesByAddress(roomAddr);
  finsSensor.requestTemperaturesByAddress(finsAddr);
  conversionStart = millis();
  conversionInProgress = true;
}

void readSensors(){
  float rawRoom = roomSensor.getTempC(roomAddr);
  float rawFins = finsSensor.getTempC(finsAddr);

  roomFault = (rawRoom == DEVICE_DISCONNECTED_C || rawRoom < -50 || rawRoom > 125);
  finsFault = (rawFins == DEVICE_DISCONNECTED_C || rawFins < -50 || rawFins > 125);

  if(!roomFault){
    roomTemp = pushAvg(roomBuf, roomIdx, roomFilled, rawRoom);
  }
  if(!finsFault){
    finsTemp = pushAvg(finsBuf, finsIdx, finsFilled, rawFins);
    firstReadingReceived = true;
  }
}

// ================================================================
// HEATER CONTROL
// ================================================================
void updateHeater(){
  bool sensorsOk = firstReadingReceived && !finsFault;

  if(!sensorsOk || !powerOn){
    setHeaterPower(0);
    heaterOn = false;
    wasHeating = false;
    heaterOnSince = 0;
    inForcedRest = false;
    return;
  }

  // Frost-safety check — this always wins, even over manual override
  if(antiFreezeEnabled){
    if(finsTemp < ICE_RISK_TEMP) iceRisk = true;
    else if(finsTemp > SAFE_RESUME_TEMP) iceRisk = false;
  } else {
    iceRisk = false;
  }

  int targetDuty = 0;

  if(iceRisk){
    targetDuty = 0;
    wasHeating = false;
    heaterOnSince = 0;
    inForcedRest = false;
  } else if(manualOverride){
    targetDuty = manualOverrideDuty;   // rest cycle doesn't apply during supervised testing
  } else if(roomFault){
    targetDuty = 0;   // no reliable room reading: fail safe, heater off
  } else {
    // Step 1: what does the thermostat want, ignoring runtime limits?
    bool wantsHeat;
    if(roomTemp > setPoint + HYSTERESIS){
      wantsHeat = true;
    } else if(roomTemp < setPoint - HYSTERESIS){
      wantsHeat = false;
    } else {
      wantsHeat = wasHeating;   // inside the deadband: hold previous state
    }

    // Step 2: cap continuous runtime so the coil gets a periodic break
    if(inForcedRest){
      if(millis() - restStartTime >= FORCED_REST_TIME){
        inForcedRest = false;
        heaterOnSince = 0;
      } else {
        wantsHeat = false;   // still resting, overrides the thermostat's wish
      }
    } else if(wantsHeat){
      if(heaterOnSince == 0) heaterOnSince = millis();
      if(millis() - heaterOnSince >= MAX_CONTINUOUS_ON){
        inForcedRest = true;
        restStartTime = millis();
        wantsHeat = false;
      }
    } else {
      heaterOnSince = 0;   // heater not wanted right now: reset the ON-streak timer
    }

    targetDuty = wantsHeat ? 100 : 0;
    wasHeating = wantsHeat;
  }

  setHeaterPower(targetDuty);
  heaterOn = targetDuty > 0;
}

void setHeaterPower(int percent){
  percent = constrain(percent, 0, 100);
  heaterDuty = percent;
  int duty = map(percent, 0, 100, 0, 255);
  ledcWrite(HEATER_PIN, duty);
  lastHeaterUpdate = millis();
}

// ================================================================
// OLED DISPLAY
// ================================================================
void updateDisplay(){
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi --");
  display.setCursor(90, 0);
  display.print(heaterOn ? "HEAT" : "off");

  // Big current (room) temperature
  display.setTextSize(3);
  display.setCursor(0, 16);
  if(roomFault || isnan(roomTemp)){
    display.print("--.-");
  } else {
    display.print(roomTemp, 1);
  }
  display.setTextSize(2);
  display.print("C");

  display.setTextSize(1);
  if(iceRisk){
    display.setCursor(0, 52);
    display.print("FROST RISK - HEATER OFF");
  } else if(inForcedRest){
    display.setCursor(0, 52);
    display.print("RESTING (coil cooldown)");
  } else {
    display.setCursor(0, 52);
    display.print("Coil:");
    display.print(finsFault ? -99.0 : finsTemp, 1);
    display.print("C  ");
    display.print(heaterDuty);
    display.print("%");
  }

  display.display();
}

// ================================================================
// SUPABASE SYNC
// ================================================================
void fetchSettings(){
  if(WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();   // skips cert validation — simplest option for a prototype
  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/settings?device_id=eq." + DEVICE_ID + "&select=*";
  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);

  int code = http.GET();
  if(code == 200){
    StaticJsonDocument<1024> doc;
    if(!deserializeJson(doc, http.getString()) && doc.size() > 0){
      JsonObject s = doc[0];
      powerOn            = s["power_on"] | true;
      setPoint           = s["set_point"] | 7.0;
      notifyMin          = s["notify_min"] | 5.0;
      notifyMax          = s["notify_max"] | 9.0;
      antiFreezeEnabled  = s["anti_freeze_enabled"] | true;
      manualOverride     = s["manual_override"] | false;
      manualOverrideDuty = s["manual_override_duty"] | 0;
    }
  } else {
    Serial.printf("Settings fetch failed: %d\n", code);
  }
  http.end();
}

void pushDeviceState(){
  if(WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/devices?id=eq." + DEVICE_ID;
  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");

  StaticJsonDocument<512> doc;
  if(!roomFault) doc["room_temp"] = roomTemp;
  if(!finsFault) doc["fins_temp"] = finsTemp;
  doc["heater_duty"]      = heaterDuty;
  doc["heater_status"]    = heaterOn;
  doc["ice_risk"]         = iceRisk;
  doc["wifi_strength"]    = wifiPercent();
  doc["mac_address"]      = WiFi.macAddress();
  doc["firmware_version"] = "1.0.0";

  String body;
  serializeJson(doc, body);
  int code = http.PATCH(body);
  if(code < 200 || code >= 300){
    Serial.printf("Device push failed: %d\n", code);
  }
  http.end();
}

void pushReading(){
  if(WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(SUPABASE_URL) + "/rest/v1/readings";
  http.begin(client, url);
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");

  StaticJsonDocument<512> doc;
  doc["device_id"] = DEVICE_ID;
  if(!roomFault) doc["room_temp"] = roomTemp;
  if(!finsFault) doc["fins_temp"] = finsTemp;
  doc["heater_duty"]   = heaterDuty;
  doc["heater_status"] = heaterOn;
  doc["ice_risk"]      = iceRisk;

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  if(code < 200 || code >= 300){
    Serial.printf("Reading push failed: %d\n", code);
  }
  http.end();
}

int wifiPercent(){
  long rssi = WiFi.RSSI();
  return constrain(map(rssi, -100, -50, 0, 100), 0, 100);
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop(){
  esp_task_wdt_reset();

  if(WiFi.status() != WL_CONNECTED){
    connectWiFi();
  }

  if(!conversionInProgress){
    startConversion();
  }
  if(conversionInProgress && millis() - conversionStart >= CONVERSION_TIME){
    readSensors();
    conversionInProgress = false;
    updateHeater();
    updateDisplay();
  }

  if(millis() - lastSync >= SYNC_INTERVAL){
    fetchSettings();
    pushDeviceState();
    pushReading();
    lastSync = millis();
  }

  // Software safety net: if the control loop ever stalls, force the heater off
  if(millis() - lastHeaterUpdate > SAFETY_TIMEOUT){
    setHeaterPower(0);
  }
}
