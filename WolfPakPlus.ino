#define DEVICE_ID "2601017"

String FirmwareVer = { "4.5" };

/* ╔════════════════════════════════════════════════════════════════════════════════════════╗
   ║  WolfPakPlus                                                                           ║
   ║  ************************************************************************************  ║
   ║  Solar-Powered Remote Network Controller                                               ║
   ║  ************************************************************************************  ║
   ║                                                                                        ║
   ║ -ESP32 Remote, Solar-Powered, Network Terminal                                         ║
   ║ -Typically powered by 2 20AH LiFePO4 batteries                                         ║
   ║ -Charged by 100W Renogy Panel with Victron 75/10 charge controller                     ║
   ║ -Monitors input power to a 5V buck PSU                                                 ║
   ║ -Switches 2 MOSFET loads (Load1 and Load2), used to power the network equipment        ║
   ║ -Switches 1 MOSFET Fan load, used for cooling and is managed by reading a SHT4x        ║
   ║ -Keeps track of time and shuts network equipment down at night                         ║
   ║                                                                                        ║
   ║  Developed on Arduino IDE v2.3.4 with esp32 by Espressif Systems v3.3.4                ║
   ║                                                                                        ║
   ║  AUTHOR: Eric Lockman (prymaltech.com)                                                 ║
   ╚════════════════════════════════════════════════════════════════════════════════════════╝
*/

// #define DEV_TEST  1          //uncomment this when developing without hardware
// #define CLOUD_VER 1          //uncomment this when building the binary going into the cloud

/*
 * Revised by: Eric Lockman (608)852-6677 (prymaltech.com)
 * 12/31/2025:  Version 4.3 (updates)
 *              Updated Ethernet drivers
 *              Added serial proxy command interface
 *              Added custom partition table for 8MB Flash
 *              Updated user manual
 * 
 * Revised by: Eric Lockman (608)852-6677 (prymaltech.com)
 * 11/20/2025:  Version 4.2 (updates)
 *              Added override functions and MQTT messages
 * 
 * Revised by: Eric Lockman (608)852-6677 (prymaltech.com)
 * 11/18/2025:  Version 4.1 (updates)
 *              Updated config on web configuration
 * 
 * Revised by: Eric Lockman (608)852-6677 (prymaltech.com)
 * 10/15/2025:  Original code for WolfPakPlus PCB
 */

/*
  Optional Upgrades:
    Serial Port Proxy
    Ethernet Port
    AWS Firmware Updates
*/

//  Development Environment: Arduino IDE v2.3.4 
//  Arduino Board: esp32 by Espressif Systems v3.1.0 / v3.3.4       ║

#include <SPI.h>                    //Core Library 
#include <Wire.h>                   //Core Library
#include <ETH.h>                    //Core Library
#include <HTTPUpdate.h>             //Core Library
#include <WiFi.h>                   //Core Library
//#include <WiFiClient.h>             //Core Library
#include <WiFiClientSecure.h>       //Core Library

#include <esp_crc.h>                //ESP-IDF Library
#include <esp_heap_caps.h>          //ESP-IDF Library
#include <esp_system.h>             //ESP-IDF Library
#include <nvs_flash.h>              //ESP-IDF Library
#include <time.h>                   //ESP-IDF Library

#include <Adafruit_SHT4x.h>         //Adafruit SHT4x Library  by Adafruit               v1.0.5
#include <ArduinoJson.h>            //ArduinoJson             by Benoit Blanchon        v7.4.2
#include <AsyncTCP.h>               //Async TCP               by ESP32Async             v3.4.9
#include <ESPAsyncWebServer.h>      //ESP Async WebServer     by ESP32Async             v3.9.0
#include <ESPmDNS.h>                //mDNS_Generic            by Georg Kaindl           v1.4.2
#include <Ethernet.h>               //Ethernet                by Various                v2.0.2
//#include <EthernetENC.h>          // or Ethernet3.h
#include <HTTPClient.h>             //HTTPClient              by Adrian McEwen          v2.2.0
#include <INA226_WE.h>              //INA226_WE               by Wolfgang Ewald         v1.3.0
#include <DFRobot_ADS1115.h>        //DFRobot_ADS1115         by DFRobot                v1.0.0
#include <Preferences.h>            //Preferences             by Volodymyr Shymanskyy   v2.2.2
#include <PubSubClient.h>           //PubSubClient            by Nick O'Leary           v2.8
#include <RTClib.h>                 //RTCLib by NeiroN        by NeiroN                 v1.6.3

#include "cert.h"                   // Your custom root CA (keep quotes)

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
DFRobot_ADS1115 ads(&Wire);
DS3231 rtc;

uint32_t epoch =        0;

// Global temperature reading
uint16_t therm;

#define  PIN_BOOT       0
#define  PIN_TXD0       1
#define  PIN_NC_02      2
#define  PIN_RXD0       3
#define  PIN_ETH_INT    4
#define  PIN_ETH_CS     5
#define  PIN_NC_12     12
#define  PIN_LEDB      13
#define  PIN_LEDR      14
#define  PIN_ETH_RST   15
#define  PIN_RXD       16
#define  PIN_TXD       17
#define  PIN_SCLK      18
#define  PIN_MISO      19
#define  PIN_SDA       21
#define  PIN_SCL       22
#define  PIN_MOSI      23
#define  PIN_LEDG      25
#define  PIN_FAN_5V    26
#define  PIN_FAN_12V   27
#define  PIN_LOAD2     32
#define  PIN_LOAD1     33
#define  PIN_NC_34     34
#define  PIN_BTN       35

#define ETH_PHY_TYPE ETH_PHY_W5500
#define ETH_PHY_ADDR 1

// Your MAC address (unique!)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

bool FAN_state = 1;
bool LOAD1_state = 1;
bool LOAD2_state = 1;
bool load1_should_be_active = false;
bool load2_should_be_active = false;

//---------------------------------------------------------------------------------------------------- Creds storing (start)
// Create an instance of Preferences
Preferences preferences;
const char* PRIMARY_NS = "Creds";
const char* BACKUP_NS = "backup";
bool config_changed = false;
unsigned long last_save = 0;
const unsigned long SAVE_INTERVAL = 300000; // 5 minutes in ms
#ifdef DEV_TEST
const float SAFE_VOLTAGE = 0.0;             // Min voltage for NVS writes
#else
const float SAFE_VOLTAGE = 0.0;            // Min voltage for NVS writes
#endif
const float USB_VOLTAGE =  5.0;

// Configuration variables
String device_id;             // Default device_id
String ssid;                  // Primary SSID
String password;              // Primary Password
String ssid2;                 // Secondary SSID
String password2;             // Secondary Password
String email;                 // Email (unsure of use)
int16_t hi_temp = 100;        // Fan on at 100°F
int16_t lo_temp = 90;         // Fan off at 90°F
uint16_t off_time = 2300;     // Load 1 off at 11 PM
uint16_t on_time = 700;       // Load 1 on at 7 AM
uint16_t off_time2 = 2300;    // Load 2 off at 11 PM
uint16_t on_time2 = 700;      // Load 2 on at 7 AM
int8_t timezone = -8;         // Pacific Time
uint32_t boots = 0;           // Number of reboots
uint32_t crc = 0;             // CRC checksum
bool daylight_savings = true; // DLS flag

uint32_t overrideSeconds1 = 0;  // load 1 override seconds timer
uint32_t overrideSeconds2 = 0;  // load 2 override  seconds timer

//TIME ZONE OFFSETS
//  Eastern Time (ET)           UTC−5   /   UTC−4 (Daylight Saving)
//  Central Time (CT)           UTC−6   /   UTC−5
//  Mountain Time (MT)          UTC−7   /   UTC−6
//  Pacific Time (PT)           UTC−8   /   UTC−7
//  Alaska Time (AKT)           UTC−9   /   UTC−8
//  Hawaii–Aleutian Time (HAT)  UTC−10

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
float lowSupplyVoltage = 15.0;
float supplyVoltage = 0.0;
float supplyCurrent = 0.0;
float supplyPower = 0.0;

float max_volts = 0.0;
float min_volts = 0.0;
float max_amps = 0.0;
float min_amps = 0.0;
float max_watts = 0.0;
float min_watts = 0.0;

float therm_temp = 0.0;
float th_temp = 0.0;
float th_humid = 0.0;

String reasonStr;

nvs_stats_t nvsStats;
uint8_t nvs_usage = 0;
char timeStr[32];
unsigned long secondsSinceBoot = 0;
unsigned long lastWifiConnectTime = 0;
unsigned long lastWifiDisconnectTime = 0;
unsigned long connectTime = 0;
unsigned long disconnectTime = 0;
uint16_t curr_time = 0;

static bool firmware_checked = false;
static bool eth_connected = false;
static bool wifi_connected = false;
static bool wired_internet_connected = false;
static bool wireless_internet_connected = false;
static bool internet_connected = false;
static bool start_mqtt = false;

void printLocalTimeFromEpoch(time_t epoch) {
  struct tm* timeinfo = localtime(&epoch);  // Converts to local time based on timezone

  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  Serial.print("Local time: ");
  Serial.println(buffer);
}

void writeTimeFromEpoch(time_t epoch) {
  // Convert epoch to DateTime object
  DateTime dt(epoch + timezone*60);
  // Set RTC time
  #ifndef SIM_DATA
  rtc.adjust(dt);
  #endif
}

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("WolfPakPlus");//ethernet hostname
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: 
      Serial.println("ETH Connected"); 
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH Got IP: '%s'\n", esp_netif_get_desc(info.got_ip.esp_netif));
      Serial.println(ETH);
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default: break;
  }
}

#define THERM_VCC         3300
#define THERM_RES_NOMINAL 10000
#define THERM_RES_SERIES  10000
#define THERM_BETA        3950
#define THERM_PULLUP      1

float getResistance(float voltage) {
  float iThermistor;
  float rThermistor;

  if (THERM_PULLUP) {
    // Pull-up resistor calculation
    iThermistor = (THERM_VCC - voltage) / THERM_RES_SERIES;
    rThermistor = voltage / iThermistor;
  } else {
    // Pull-down resistor calculation
    iThermistor = voltage / THERM_RES_SERIES;
    rThermistor = (THERM_VCC - voltage) / iThermistor;
  }

  return rThermistor;
}

float getTemperatureC(float voltage) {
  float nominalTemperature = 25.0;
  float rThermistor = getResistance(voltage);
  float tempC = (1.0 / ( (1.0 / (nominalTemperature + 273.15)) + (1.0 / THERM_BETA) * log(rThermistor / THERM_RES_NOMINAL) )) -273.15;
  return tempC;
}

float getTemperatureF(float voltage) {
  float tempC = getTemperatureC(voltage);
  float tempF = (tempC * 9.0 / 5.0) + 32.0;
  return tempF;
}

// Compute CRC32 for data integrity
uint32_t compute_crc32() {
  String data = device_id + "|" + ssid + "|" + password + "|" + ssid2 + "|" + password2 + "|" + email;
  return esp_crc32_le(0, (uint8_t*)data.c_str(), data.length());
}

bool clear_settings(const char* namespace_name) {
  preferences.begin(namespace_name, false); //false for read/write access
  Serial.println("Factory Reset Settings... ");
  preferences.remove("ssid");
  preferences.remove("password");
  preferences.remove("ssid2");
  preferences.remove("password2");

  preferences.remove("hi_temp");
  preferences.remove("lo_temp");
  preferences.remove("off_time");
  preferences.remove("on_time");
  preferences.remove("off_time2");
  preferences.remove("on_time2");
  preferences.remove("timezone");
  preferences.remove("daylight_savings");
  preferences.end();
}

// Save settings to NVS namespace
bool save_settings(const char* namespace_name) {
  if ((supplyVoltage < SAFE_VOLTAGE) && !((supplyVoltage > (USB_VOLTAGE-1.0)) && (supplyVoltage < (USB_VOLTAGE+1.0)))) {
    Serial.println("Low voltage, skipping NVS write");
    return false;
  }

  preferences.begin(namespace_name, false); //false for read/write access
  crc = compute_crc32();
  preferences.putString("device_id", device_id);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putString("ssid2", ssid2);
  preferences.putString("password2", password2);
  preferences.putString("email", email);
  preferences.putInt("hi_temp", hi_temp);
  preferences.putInt("lo_temp", lo_temp);
  preferences.putUShort("off_time", off_time);
  preferences.putUShort("on_time", on_time);
  preferences.putUShort("off_time2", off_time2);
  preferences.putUShort("on_time2", on_time2);
  preferences.putChar("timezone", timezone);
  preferences.putUInt("boots", boots);
  // preferences.putUInt("daylight_savings", daylight_savings ? 1 : 0);
  preferences.putBool("daylight_savings", daylight_savings);
  preferences.putUInt("crc", crc);
  preferences.end();
  Serial.printf("Saved settings to %s namespace\n", namespace_name);
  return true;
}

// Load settings from NVS namespace
bool load_settings(const char* namespace_name) {
  preferences.begin(namespace_name, true); //true for read-only access
  String loaded_device_id = preferences.getString("device_id", device_id);
  String loaded_ssid = preferences.getString("ssid", "");
  String loaded_password = preferences.getString("password", "");
  String loaded_ssid2 = preferences.getString("ssid2", "");
  String loaded_password2 = preferences.getString("password2", "");
  String loaded_email = preferences.getString("email", "");
  String data = loaded_device_id + "|" + loaded_ssid + "|" + loaded_password + "|" + 
                loaded_ssid2 + "|" + loaded_password2 + "|" + loaded_email;
  uint32_t loaded_crc = preferences.getUInt("crc", 0);
  uint32_t computed_crc = esp_crc32_le(0, (uint8_t*)data.c_str(), data.length());

  if (loaded_crc == computed_crc && loaded_device_id.length() > 0) {
    device_id = loaded_device_id;
    ssid = loaded_ssid;
    password = loaded_password;
    ssid2 = loaded_ssid2;
    password2 = loaded_password2;
    email = loaded_email;
    hi_temp = preferences.getInt("hi_temp", 100);
    lo_temp = preferences.getInt("lo_temp", 90);
    off_time = preferences.getUShort("off_time", 2300);
    on_time = preferences.getUShort("on_time", 0700);
    off_time2 = preferences.getUShort("off_time2", 2300);
    on_time2 = preferences.getUShort("on_time2", 0700);
    timezone = preferences.getChar("timezone", -8);
    boots = preferences.getUInt("boots", 0);
    // daylight_savings = preferences.getUInt("daylight_savings", 0) == 1;
    daylight_savings = preferences.getBool("daylight_savings", true);
    crc = loaded_crc;
    preferences.end();
    Serial.printf("Loaded valid settings from %s namespace\n", namespace_name);
    return true;
  }
  preferences.end();
  Serial.printf("Invalid or missing settings in %s namespace\n", namespace_name);
  Serial.printf("loaded_crc (%UL) != computed_crc (%UL)\n", loaded_crc, computed_crc);
  return false;
}

// Sync primary and backup namespaces
void sync_settings() {

  bool primary_valid = load_settings(PRIMARY_NS);
  Serial.print("daylight_savings: ");
  Serial.println(daylight_savings);
  bool backup_valid = load_settings(BACKUP_NS);
  Serial.print("daylight_savings: ");
  Serial.println(daylight_savings);

  // Increment boot count
  boots++; 
  Serial.print("BOOTS: ");
  Serial.println(boots);

  if ((ssid2.length() < 1 && ssid.length() >= 1) && (password.length() < 8 && password2.length() >= 8)) {
    ssid2 = ssid;
    password2 = password;
  }

#ifndef CLOUD_VER
  //if this is not the cloud version, update the device ID through local FLASHING
  device_id = DEVICE_ID;
  save_settings(PRIMARY_NS);
  save_settings(BACKUP_NS);
#else
  if (primary_valid && !backup_valid) {
    save_settings(BACKUP_NS); // Restore backup from primary
  } else if (!primary_valid && backup_valid) {
    save_settings(PRIMARY_NS); // Restore primary from backup
  } else if (!primary_valid && !backup_valid) {
    Serial.println("No valid settings, using defaults");
    hi_temp = 100;          // Fan on at 100°F
    lo_temp = 90;           // Fan off at 90°F
    off_time = 2300;        // Load 2 off at 11 PM
    on_time = 700;          // Load 2 on at 7 AM
    off_time2 = 2300;       // Load 2 off at 11 PM
    on_time2 = 700;         // Load 2 on at 7 AM
    timezone = -8;          // Pacific Time
    daylight_savings = true;//DLS flag
    save_settings(PRIMARY_NS);
    save_settings(BACKUP_NS);
  } else {
    save_settings(PRIMARY_NS);
    save_settings(BACKUP_NS);
  }
#endif
}
//---------------------------------------------------------------------------------------------------- Creds storing (end)

//---------------------------------------------------------------------------------------------------- WiFi Manager (start)
String local_ssid;
// unsigned long wifiDisconnectedSince = 0;
const unsigned long apTimeout = 60000;//120000;  // 2 minutes = 120,000 ms
bool apModeActive = false;

// Create an instance of AsyncWebServer
AsyncWebServer webServer(80);

// Function to start the WiFi connection
void connectToWiFi(bool blocking, bool backup) {
  if(wired_internet_connected)return;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_MODE_STA);
  // WiFi.mode(WIFI_MODE_AP);
  // WiFi.mode(WIFI_AP_STA);

  // Clear old config so hostname can be set
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // Set custom hostname
  WiFi.setHostname("WolfPakPlus");

  if(backup) WiFi.begin(ssid2, password2);
  else WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  delay(1000);

  if(!blocking)return;

  for (int attempts = 0; attempts < 20; attempts++) {
    if (WiFi.status() == WL_CONNECTED) {
      if(!wifi_connected){
        start_mqtt=true;
      }
      wifi_connected=true;
      Serial.println("\nWiFi connected successfully.");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return;
    }else{
      wifi_connected=false;
    }
    delay(1000);
    Serial.print(".");
  }
}

// Function to start the configuration webpage
void startConfigPage() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_MODE_AP);
  // WiFi.mode(WIFI_STA);
  // WiFi.mode(WIFI_AP_STA);  // Allow AP+STA

  // Clear old config so hostname can be set
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // Set custom hostname
  WiFi.setHostname("WolfPakPlus");

  WiFi.softAP(local_ssid);
  WiFi.setAutoReconnect(false);
  Serial.println("Started Access Point: ESP32_Config");

  // Initialize mDNS
  if (!MDNS.begin("wolfpak")) {
    Serial.println("Error setting up MDNS responder!");
  }

  // Main configuration page
  webServer.on("/", HTTP_GET, [=](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>WiFi Config</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;background:#121212;color:#fff;text-align:center;}label,input{margin:10px;}input{padding:8px;border-radius:5px;border:1px solid #bb86fc;background:#1f1f1f;color:white;}button{padding:10px 20px;background:#bb86fc;color:white;border:none;border-radius:5px;cursor:pointer;}button:hover{background:#3700b3;}</style>";
    html += "</head><body><h1>Configure Device</h1>";
    html += "<form action='/save' method='POST'>";
    html += "<label for='ssid'>WiFi SSID:</label><input type='text' id='ssidInput' name='ssid' required><br>";
    html += "<label for='password'>Password:</label><input type='password' id='password' name='password' required><br>";
    html += "<input type='checkbox' onclick='togglePassword()'> Show Password<br>";
    html += "<label for='email'>Email:</label><input type='email' id='email' name='email' required><br>";
    // html += "<label for='device_id'>Device ID:</label><input type='text' id='device_id' name='device_id' value='" + device_id + "' required><br>";
    html += "<label for='hi_temp'>Fan On Temp:</label><input type='number' id='hi_temp' name='hi_temp' value='100' min='0' max='9999' step='1' required><br>";
    html += "<label for='lo_temp'>Fan Off Temp:</label><input type='number' id='lo_temp' name='lo_temp' value='90' min='0' max='9999' step='1' required><br>";
    html += "<label for='on_time'>L1 On Time:</label><input type='number' id='on_time' name='on_time' value='0700' min='0' max='9999' step='1' required><br>";
    html += "<label for='off_time'>L1 Off Time:</label><input type='number' id='off_time' name='off_time' value='2300' min='0' max='9999' step='1' required><br>";
    html += "<label for='on_time2'>L2 On Time:</label><input type='number' id='on_time2' name='on_time2' value='0700' min='0' max='9999' step='1' required><br>";
    html += "<label for='off_time2'>L2 Off Time:</label><input type='number' id='off_time2' name='off_time2' value='2300' min='0' max='9999' step='1' required><br>";
    html += "<label for='daylight_savings'>Daylight Savings:</label><input type='checkbox' id='daylight_savings'  name='daylight_savings' value='1'><br>";
    html += "<label for='timezone'>Time Zone:</label><select id='timezone' name='timezone' required><option value='-5'>Eastern (UTC-5)</option><option value='-6'>Central (UTC-6)</option><option value='-7'>Mountain (UTC-7)</option><option value='-8'>Pacific (UTC-8)</option><option value='-9'>Alaska (UTC-9)</option><option value='-10'>Hawaii (UTC-10)</option></select><br>";
    html += "<label>Load 1:</label><input type='checkbox' id='load1' onchange='toggleLoad(1)' " + String(LOAD1_state ? "checked" : "") + "><br>";
    html += "<label>Load 2:</label><input type='checkbox' id='load2' onchange='toggleLoad(2)' " + String(LOAD2_state ? "checked" : "") + "><br>";
    html += "<button type='submit'>Save</button></form>";
    html += "<script>";
    html += "function togglePassword(){ var p = document.getElementById('password'); p.type = p.type === 'password' ? 'text' : 'password'; }";
    html += "function toggleLoad(n){ let c = document.getElementById('load'+n); fetch('/toggle_load'+n+'?state='+(c.checked?1:0)); }";
    html += "</script></body></html>";
    request->send(200, "text/html", html);
  });

  // Main configuration page
  webServer.on("/set-id", HTTP_GET, [=](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><title>Set Device ID</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;background:#121212;color:#fff;text-align:center;}label,input{margin:10px;}input{padding:8px;border-radius:5px;border:1px solid #bb86fc;background:#1f1f1f;color:white;}button{padding:10px 20px;background:#bb86fc;color:white;border:none;border-radius:5px;cursor:pointer;}button:hover{background:#3700b3;}</style>";
    html += "</head><body><h1>Set Device ID</h1>";
    html += "<form action='/save_id' method='POST'>";
    html += "<label for='device_id'>Device ID:</label><input type='text' id='device_id' name='device_id' value='" + device_id + "' required><br>";
    html += "<button type='submit'>Save</button></form>";
    html += "<script>";
    html += "</script></body></html>";
    request->send(200, "text/html", html);
  });

  // Save form handler
  webServer.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true) && 
        request->hasParam("password", true) && 
        request->hasParam("email", true) && 
        request->hasParam("hi_temp", true) && 
        request->hasParam("lo_temp", true) && 
        request->hasParam("on_time", true) && 
        request->hasParam("off_time", true) &&
        request->hasParam("on_time2", true) && 
        request->hasParam("off_time2", true) &&
        request->hasParam("timezone", true) ) 
    {
      ssid = request->getParam("ssid", true)->value();
      password = request->getParam("password", true)->value();
      email = request->getParam("email", true)->value();

      hi_temp = request->getParam("hi_temp", true)->value().toInt();
      lo_temp = request->getParam("lo_temp", true)->value().toInt();
      on_time = request->getParam("on_time", true)->value().toInt();
      off_time = request->getParam("off_time", true)->value().toInt();
      on_time2 = request->getParam("on_time2", true)->value().toInt();
      off_time2 = request->getParam("off_time2", true)->value().toInt();
      timezone = request->getParam("timezone", true)->value().toInt();
      daylight_savings = request->hasParam("daylight_savings", true);

      bool LOAD1_state = request->hasParam("load1", true);
      bool LOAD2_state = request->hasParam("load2", true);

      // config_changed = true;
      save_settings(PRIMARY_NS);
      save_settings(BACKUP_NS);

      Serial.println("Saved to preferences:");
      // Serial.println("SSID: " + ssid);
      // Serial.println("Password: " + password);
      // Serial.println("Email: " + email);
      // Serial.println("hi_temp: " + hi_temp);
      // Serial.println("lo_temp: " + lo_temp);
      // Serial.println("on_time: " + on_time);
      // Serial.println("off_time: " + off_time);
      // Serial.println("on_time2: " + on_time2);
      // Serial.println("off_time2: " + off_time2);
      // Serial.println("timezone: " + timezone);
      // Serial.println("daylight_savings: " + daylight_savings);
if(daylight_savings)Serial.println("daylight_savings: ON");
else Serial.println("daylight_savings: OFF");

      request->send(200, "text/plain", "Saved! Device will now try to connect to WiFi.");
      delay(3000);
      // stopConfigPage();
      ESP.restart();
    } else {
      request->send(400, "text/plain", "Invalid Input");
    }
  });


  // Save form handler
  webServer.on("/save_id", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("device_id", true) ) 
    {
      device_id = request->getParam("device_id", true)->value();

      // config_changed = true;
      save_settings(PRIMARY_NS);
      save_settings(BACKUP_NS);

      Serial.println("Saved to preferences:");
      Serial.println("Device ID: " + device_id);

      request->send(200, "text/plain", "Device ID updated! Rebooting...");
      delay(3000);
      ESP.restart();
    } else {
      request->send(400, "text/plain", "Invalid Input");
    }
  });


  // Toggle load endpoints
  webServer.on("/toggle_load1", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("state")) {
      bool state = (request->getParam("state")->value() == "1");
      digitalWrite(PIN_LOAD1, state ? HIGH : LOW);
      Serial.printf("Load 1 (IO4) toggled to: %s\n", state ? "ON" : "OFF");
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing 'state' parameter");
    }
  });

  webServer.on("/toggle_load2", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("state")) {
      bool state = (request->getParam("state")->value() == "1");
      digitalWrite(PIN_LOAD2, state ? HIGH : LOW);
      Serial.printf("Load 2 (IO13) toggled to: %s\n", state ? "ON" : "OFF");
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing 'state' parameter");
    }
  });

  webServer.begin();
  Serial.println("Configuration webpage started.");
  apModeActive = true;
  wifi_connected=false;
}

void stopConfigPage() {
  webServer.end();
  delay(100);
  WiFi.softAPdisconnect(true);  // Turn off AP
  Serial.println("Stopped configuration AP and web server.");
  apModeActive = false;
}
//---------------------------------------------------------------------------------------------------- WiFi Manager (end)

//---------------------------------------------------------------------------------------------------- OTA (start)
#ifdef DEV_TEST
// #define URL_fw_Version "https://raw.githubusercontent.com/elockman/Wolfpak-connect/main/firmware_version.txt"
// #define URL_fw_Bin "https://raw.githubusercontent.com/elockman/Wolfpak-connect/master/firmware.bin"
#define URL_fw_Version "https://raw.githubusercontent.com/petee1525/Wolfpak-connect/main/firmware_version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/petee1525/Wolfpak-connect/master/firmware.bin"
#else
#define URL_fw_Version "https://raw.githubusercontent.com/petee1525/Wolfpak-connect/main/firmware_version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/petee1525/Wolfpak-connect/master/firmware.bin"
#endif


unsigned long previousMillis = 0;

void firmwareUpdate(void) {
  WiFiClientSecure fwClient;
  fwClient.setCACert(CAcert);  // fallback to default hardcoded cert

  // httpUpdate.setLedPin(wifi_led, HIGH);
//  httpUpdate.setLedPin(UNUSED_PIN, HIGH);
  
  t_httpUpdate_return ret = httpUpdate.update(fwClient, URL_fw_Bin);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure *otaClient = new WiFiClientSecure;

  if (otaClient) {
    otaClient->setCACert(CAcert);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
    HTTPClient https;

    if (https.begin(*otaClient, fwurl)) {  // HTTPS
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK)  // if version received
      {
        payload = https.getString();  // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete otaClient;
  }

  if (httpCode == HTTP_CODE_OK)  // if version received
  {
    payload.trim();
    Serial.print("File Version: ");
    Serial.println(payload);
    if (payload.toFloat() <= FirmwareVer.toFloat()) {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    } else {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

void repeatedFirmwareCheck() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck < 900000) return;
  lastCheck = now;  // reset timer

  if (FirmwareVersionCheck()) {
    firmwareUpdate();
  }
}
//---------------------------------------------------------------------------------------------------- OTA (end)

//---------------------------------------------------------------------------------------------------- NTP (start)
struct tm currentTime;
time_t rawTime;
bool timeSynced = false;

void printLocalTime() {
  if (!getLocalTime(&currentTime)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&currentTime, "%A, %B %d %Y %H:%M:%S");
  // Serial.print("Day of week: ");
  // Serial.println(&currentTime, "%A");
  // Serial.print("Month: ");
  // Serial.println(&currentTime, "%B");
  // Serial.print("Day of Month: ");
  // Serial.println(&currentTime, "%d");
  // Serial.print("Year: ");
  // Serial.println(&currentTime, "%Y");
  // Serial.print("Hour: ");
  // Serial.println(&currentTime, "%H");
  // Serial.print("Hour (12 hour format): ");
  // Serial.println(&currentTime, "%I");
  // Serial.print("Minute: ");
  // Serial.println(&currentTime, "%M");
  // Serial.print("Second: ");
  // Serial.println(&currentTime, "%S");
}
//---------------------------------------------------------------------------------------------------- NTP (end)

//---------------------------------------------------------------------------------------------------- Schedule (start)
// #include <esp_timer.h>

// Separate timers for each load
// esp_timer_handle_t timer_load;

// unsigned long lastQuietCheck = 0;
// const unsigned long quietInterval = 5000;

// Quiet hour timer
// bool quietTimerActive = false;
// unsigned long quietTimerStart = 0;
// unsigned long quietTimerDuration = 0;

// // Timer callback for Loads
// void timerCallbackLoad(void *arg) {
//   digitalWrite(PIN_LOAD1, HIGH);  // Turn ON the load when timer expires
//   digitalWrite(PIN_LOAD2, HIGH);  // Turn ON the load when timer expires
//   timer_active = false;
//   Serial.println("Timer expired for Loads");
//   preferences.remove("hours");
//   preferences.remove("start");

//   LOAD1_state = 1;
//   LOAD2_state = 1;

//   preferences.putBool("load1", LOAD1_state);
//   preferences.putBool("load2", LOAD2_state);
// }

// // Start countdown for Load 1
// void startCountdownLoad(double hours) {
//   preferences.putDouble("hours", hours);
//   preferences.putULong64("start", esp_timer_get_time());

//   uint64_t us = static_cast<uint64_t>(hours * 60 * 60 * 1000000);
//   esp_timer_start_once(timer_load, us);
//   digitalWrite(PIN_LOAD1, LOW);  // Turn OFF the load
//   digitalWrite(PIN_LOAD2, LOW);  // Turn OFF the load
//   Serial.println("Countdown started for Loads");
// }

// Restore timer for Load 1 on reboot
// void restoreScheduleLoad() {
//   if (preferences.isKey("hours") && preferences.isKey("start")) {
//     double hours = preferences.getDouble("hours", 0);     // Use getDouble for fractional hours
//     uint64_t start = preferences.getULong64("start", 0);  // Stored using esp_timer_get_time()
//     uint64_t now = esp_timer_get_time();                  // Current time in µs

//     uint64_t total = hours * 60 * 60 * 1000000ULL;  // Convert hours to µs
//     uint64_t elapsed = now - start;
//     uint64_t remaining = (elapsed >= total) ? 0 : total - elapsed;

//     if (remaining > 0) {
//       esp_timer_start_once(timer_load, remaining);
//       digitalWrite(PIN_LOAD1, LOW);  // Resume ON state
//       digitalWrite(PIN_LOAD2, LOW);  // Resume ON state
//       Serial.printf("Restored Loads with %.2f minutes remaining\n", remaining / 60000000.0);
//     } else {
//       digitalWrite(PIN_LOAD1, HIGH);  // Timer already expired
//       digitalWrite(PIN_LOAD2, HIGH);  // Timer already expired
//       Serial.println("Loads timer already expired on restore");
//     }
//   }
// }

// void enforceQuietHours() {
//   struct tm timeinfo;
//   if (!getLocalTime(&timeinfo)) return;

//   int hour = timeinfo.tm_hour;
//   int minute = timeinfo.tm_min;
//   int second = timeinfo.tm_sec;

//   if (hour >= 23 || hour < 7) {
//     if (LOAD1_state || LOAD2_state) {
//       Serial.println("Quiet Hours Active: Forcing loads OFF");

//       int currentSeconds = hour * 3600 + minute * 60 + second;
//       int targetSeconds = (hour < 7) ? 7 * 3600 : (24 + 7) * 3600;
//       int secondsUntil7AM = targetSeconds - currentSeconds;

//       quietTimerStart = millis();
//       quietTimerDuration = secondsUntil7AM * 1000;
//       quietTimerActive = true;

//       preferences.putULong("quiet_start", millis() / 1000);
//       preferences.putUInt("quiet_duration", secondsUntil7AM);

//       uint64_t us = (uint64_t)secondsUntil7AM * 1000000ULL;

//       esp_timer_start_once(timer_load, us);
//       digitalWrite(PIN_LOAD1, LOW);  // Turn OFF the load
//       digitalWrite(PIN_LOAD2, LOW);  // Turn OFF the load

//       LOAD1_state = 0;
//       LOAD2_state = 0;

//       preferences.putBool("load1", LOAD1_state);
//       preferences.putBool("load2", LOAD2_state);

//       Serial.printf("Quiet timer set for %d seconds\n", secondsUntil7AM);
//     }
//   }
// }
//---------------------------------------------------------------------------------------------------- Schedule (end)

//---------------------------------------------------------------------------------------------------- MQTT (start)
#ifdef DEV_TEST
const char *mqttServer = "broker.emqx.io";
#else
const char *mqttServer = "16.171.144.239";
#endif

WiFiClient espClient;
PubSubClient mqttClient(espClient);
long lastMsg = 0;
char msg[1500];
int value = 0;

String l1_topic = String(DEVICE_ID) + "/inverter/state";
String l2_topic = device_id + "/cig/status";
String wifi_topic = device_id + "/stored_wifi";
String new_cred_topic = device_id + "/wifi_creds";
String ota_topic = device_id + "/FOTA";
String fw_topic = device_id + "/fwVer";
String batt_topic = device_id + "/bat/voltage";

String dev_mac_topic = device_id + "/device/mac";
String dev_fw_topic = device_id + "/device/firmware";
String dev_ram_topic = device_id + "/device/total_ram";
String dev_free_ram_topic = device_id + "/device/free_ram";
String dev_used_ram_topic = device_id + "/device/used_ram";
String dev_used_flash_topic = device_id + "/device/used_flash";

String cfg_dls_time_topic = device_id + "/config/dls_time";
String cfg_timezone_topic = device_id + "/config/timezone";
String cfg_lo_temp_topic = device_id + "/config/fan_offtemp";
String cfg_hi_temp_topic = device_id + "/config/fan_ontemp";
String cfg_off_time_topic = device_id + "/config/l1_offtime";
String cfg_on_time_topic = device_id + "/config/l1_ontime";
String cfg_override_topic = device_id + "/config/l1_override";
String cfg_off_time2_topic = device_id + "/config/l2_offtime";
String cfg_on_time2_topic = device_id + "/config/l2_ontime";
String cfg_override2_topic = device_id + "/config/l2_override";

String stat_boots_topic = device_id + "/status/boots";
String stat_reason_topic = device_id + "/status/reboot_reason";
String stat_lastboot_topic = device_id + "/status/last_boot";
String stat_disconnect_topic = device_id + "/status/last_disconnect";
String stat_connect_topic = device_id + "/status/last_connect";
String stat_time_topic = device_id + "/status/time";
String stat_currtime_topic = device_id + "/status/current_time";

String net_ip_topic = device_id + "/network/ip_address";
String net_rssi_topic = device_id + "/network/rssi";
String net_ssid_topic = device_id + "/network/ssid";
String net_pw_topic = device_id + "/network/pw";
String net_ssid2_topic = device_id + "/network/ssid2";
String net_pw2_topic = device_id + "/network/pw2";
String net_enet_topic = device_id + "/network/enet";

String sens_lv_topic = device_id + "/sensor/lowvoltage";
String sens_v_topic = device_id + "/sensor/voltage";
String sens_i_topic = device_id + "/sensor/current";
String sens_p_topic = device_id + "/sensor/power";
String sens_tp_topic = device_id + "/sensor/tempprobe";
String sens_t_topic = device_id + "/sensor/temperature";
String sens_h_topic = device_id + "/sensor/humidity";

String load_fan_topic = device_id + "/load/fan";
String load_1_topic = device_id + "/load/1";
String load_2_topic = device_id + "/load/2";

String sens_vmx_topic = device_id + "/sensor/vmax";
String sens_vmn_topic = device_id + "/sensor/vmin";
String sens_imx_topic = device_id + "/sensor/imax";
String sens_imn_topic = device_id + "/sensor/imin";
String sens_pmx_topic = device_id + "/sensor/pmax";
String sens_pmn_topic = device_id + "/sensor/pmin";
String sens_tmx_topic = device_id + "/sensor/tmax";
String sens_tmn_topic = device_id + "/sensor/tmin";

String set_config_topic = device_id + "/set_config/#";
String set_dls_time_topic = device_id + "/set_config/dls_time";
String set_timezone_topic = device_id + "/set_config/timezone";
String set_lo_temp_topic = device_id + "/set_config/fan_offtemp";
String set_hi_temp_topic = device_id + "/set_config/fan_ontemp";
String set_off_time_topic = device_id + "/set_config/l1_offtime";
String set_on_time_topic = device_id + "/set_config/l1_ontime";
String set_override_topic = device_id + "/set_config/l1_override";
String set_off_time2_topic = device_id + "/set_config/l2_offtime";
String set_on_time2_topic = device_id + "/set_config/l2_ontime";
String set_override2_topic = device_id + "/set_config/l2_override";

void mqttCallback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String messageTemp;

  // Build debug payload: "topic | message"
  String debugPayload = "Topic: ";
  debugPayload += topic;
  debugPayload += " | Message: ";
  debugPayload += messageTemp;

  mqttClient.publish("debug/callback", debugPayload.c_str());

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == l1_topic) {
    Serial.print("Changing output to ");
    if (messageTemp == "1") {
      Serial.println("Load 1 turned on");
      LOAD1_state = 1;
      digitalWrite(PIN_LOAD1, LOAD1_state);
      // preferences.putBool("load1", LOAD1_state);
    } else if (messageTemp == "0") {
      Serial.println("Load 1 turned off");
      LOAD1_state = 0;
      digitalWrite(PIN_LOAD1, LOAD1_state);
      // preferences.putBool("load1", LOAD1_state);
    }
  }

  if (String(topic) == l2_topic) {
    Serial.print("Changing output to ");
    if (messageTemp == "1") {
      Serial.println("Load 2 turned on");
      LOAD2_state = 1;
      digitalWrite(PIN_LOAD2, LOAD2_state);
      // preferences.putBool("load2", LOAD2_state);
    } else if (messageTemp == "0") {
      Serial.println("Load 2 turned off");
      LOAD2_state = 0;
      digitalWrite(PIN_LOAD2, LOAD2_state);
      // preferences.putBool("load2", LOAD2_state);
    }
  }

  if (String(topic) == new_cred_topic) {
    // StaticJsonDocument<64> doc;
    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, messageTemp);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    //TODO:
    //uh-oh...this could totally kill the node by pushing bad credentials
    //must store previous ssid
    //do not add ssid unless it is validated
    const char *SSID = doc["ssid"];  // "ssid"
    const char *PASS = doc["pass"];  // "pass"
    ssid2 = String(SSID);
    password2 = String(PASS);

    connectToWiFi(true, true);

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi-2 connected successfully: ");
      Serial.println(ssid2);
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      if(!wifi_connected){
        start_mqtt=true;
      }
      wifi_connected=true;
      config_changed = true;
      // preferences.putString("ssid2", ssid2);
      // preferences.putString("password2", password2);
    }else{
      wifi_connected=false;
      connectToWiFi(false, false);
    }
  }

  if (String(topic) == ota_topic) {
    Serial.println("FOTA requested via MQTT");

    String firmwareURL = messageTemp;
    Serial.println("Firmware URL: " + firmwareURL);

    WiFiClientSecure secureClient;
    secureClient.setCACert(CAcert);  // use your certificate or NULL if HTTP

    // httpUpdate.setLedPin(wifi_led, HIGH);  // optional: show activity
//    httpUpdate.setLedPin(UNUSED_PIN, HIGH);  // optional: show activity

    t_httpUpdate_return ret = httpUpdate.update(secureClient, firmwareURL);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n",
                      httpUpdate.getLastError(),
                      httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK - Rebooting...");
        break;
    }
  }

  if (String(topic) == set_dls_time_topic){
    daylight_savings = (messageTemp.toInt() != 0);
    Serial.print("DLS set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_timezone_topic){
    timezone = messageTemp.toInt();
    Serial.print("Timezone set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_off_time_topic){
    off_time = messageTemp.toInt();
    Serial.print("Off Time set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_on_time_topic){
    on_time = messageTemp.toInt();
    Serial.print("On Time set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_override_topic){
    overrideSeconds1 = messageTemp.toInt();
    Serial.print("Override Time set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_off_time2_topic){
    off_time2 = messageTemp.toInt();
    Serial.print("Off Time 2 set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_on_time2_topic){
    on_time2 = messageTemp.toInt();
    Serial.print("On Time 2 set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_override2_topic){
    overrideSeconds2 = messageTemp.toInt();
    Serial.print("Override Time 2 set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_lo_temp_topic){
    lo_temp = messageTemp.toInt();
    Serial.print("Lo Temp set to: ");
    Serial.println(messageTemp.toInt());
  }
  if (String(topic) == set_hi_temp_topic){
    hi_temp = messageTemp.toInt();
    Serial.print("Hi Temp set to: ");
    Serial.println(messageTemp.toInt());
  }
}

void reconnect() {
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 10000) return;
  lastRun = now;  // reset timer
  if (!mqttClient.connected() && internet_connected) {
    Serial.println(" ");
    Serial.print("Attempting MQTT connection from ");
    Serial.print(WiFi.localIP());
    Serial.print(" to: ");
    Serial.println(mqttServer);

    mqttClient.disconnect();
    mqttClient.setServer(mqttServer, 1883);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(60);   // seconds

    // Attempt to connect mqtt server
    if (mqttClient.connect(device_id.c_str())) {
      Serial.println("connected");
      // Subscribe
      mqttClient.subscribe(l1_topic.c_str());
      mqttClient.subscribe(l2_topic.c_str());
      mqttClient.subscribe(new_cred_topic.c_str());
      mqttClient.subscribe(ota_topic.c_str());
      mqttClient.subscribe(set_config_topic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 10 seconds");
    }
  }
}
//---------------------------------------------------------------------------------------------------- MQTT (end)

//---------------------------------------------------------------------------------------------------- INA226 (start)
#define INA_ADDR 0x40

INA226_WE ina226 = INA226_WE(INA_ADDR);

// unsigned long previousMillis_reading = 0;     // Tracks the last time the function was called
// const unsigned long interval_reading = 5000;  // Interval in milliseconds (5s delay)

// void measure_battery_consumption(float &current_A, float &voltage_V) {

//   shuntVoltage_mV = 0.0;
//   loadVoltage_V = 0.0;
//   busVoltage_V = 0.0;
//   current_mA = 0.0;
//   power_mW = 0.0;

//   ina226.readAndClearFlags();
//   shuntVoltage_mV = ina226.getShuntVoltage_mV();
//   busVoltage_V = ina226.getBusVoltage_V();
//   current_mA = ina226.getCurrent_mA();
//   power_mW = ina226.getBusPower();
//   loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

//   voltage_V = loadVoltage_V;

//   // Serial.print("Shunt Voltage [mV]: ");
//   // Serial.println(shuntVoltage_mV);
//   // Serial.print("Bus Voltage [V]: ");
//   // Serial.println(busVoltage_V);
//   // Serial.print("Load Voltage [V]: ");
//   // Serial.println(loadVoltage_V);
//   // Serial.print("Current[mA]: ");
//   // Serial.println(current_mA);
//   // Serial.print("Bus Power [mW]: ");
//   // Serial.println(power_mW);
//   // if (!ina226.overflow) {
//   //   Serial.println("Values OK - no overflow");
//   // } else {
//   //   Serial.println("Overflow! Choose higher current range");
//   // }
//   // Serial.println();
// }
//---------------------------------------------------------------------------------------------------- INA226 (end)

// #include <DHT.h>
// #define DHTPIN 3              // RX_PIN: 3, TX_PIN: 1
// #define DHTTYPE    DHT22      // DHT 22 (AM2302)
// DHT dht(DHTPIN, DHTTYPE);

bool power_avail = false;
bool temp_avail = false;
int16_t box_temp = 0;
int16_t max_temp = 0;
int16_t min_temp = 0;
int8_t box_humid = 0;
int8_t max_humid = 0;
int8_t min_humid = 0;

void btn_init(){
  pinMode(PIN_BTN, INPUT);
}

void adc_init(){
//    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1);   // 0x48
    ads.setAddr_ADS1115(0x48);   // 0x48
    ads.setGain(eGAIN_TWOTHIRDS);   // 2/3x gain
    ads.setMode(eMODE_SINGLE);       // single-shot mode
    ads.setRate(eRATE_128);          // 128SPS (default)
    ads.setOSMode(eOSMODE_SINGLE);   // Set to start a single-conversion
    ads.init();
}

void th_init(){
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    temp_avail=false;
  }else{
    Serial.println("Found SHT4x sensor");
    Serial.print("Serial number 0x");
    Serial.println(sht4.readSerial(), HEX);
    temp_avail=true;
  }

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }

  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }

  sensors_event_t humidity, temp;
  
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature*9/5 + 32.0); Serial.println(" degrees F");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  if(temp_avail){
    max_temp = min_temp = temp.temperature*9/5 + 32.0;
    max_humid = min_humid = humidity.relative_humidity;
  }
}

void rtc_init(){
  #ifdef SIM_DATA
  DateTime now = DateTime(__DATE__, __TIME__);
  epoch = now.unixtime();
  return;
  #else
  //sync time from RTC
  DateTime now = rtc.now();
  epoch = now.unixtime();
  epoch -= timezone*60;
  #endif  

  // Initialize I2C and RTC
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    Serial.println("Found RTC at 0x68");
  }else{
    Serial.println("RTC NOT FOUND at 0x68!");
//    ledMode = RGB_MODE_RED;
  }

  rtc.begin();

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
//    ledMode = RGB_MODE_RED;
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
}

void rgb_init(){
  pinMode(PIN_LEDR, OUTPUT);
  pinMode(PIN_LEDG, OUTPUT);
  pinMode(PIN_LEDB, OUTPUT);
  digitalWrite(PIN_LEDR, LOW);         //Blink network LED at boot
  digitalWrite(PIN_LEDG, LOW);         //Blink network LED at boot
  digitalWrite(PIN_LEDB, LOW);         //Blink network LED at boot
}

void fan_init(){
  FAN_state = 1;
  pinMode(PIN_FAN_5V, OUTPUT);
  pinMode(PIN_FAN_12V, OUTPUT);
  digitalWrite(PIN_FAN_12V, FAN_state);     //Force Fan on at boot
  digitalWrite(PIN_FAN_5V, FAN_state);      //Force Fan on at boot
}

void load_init(){
  LOAD1_state = 1;
  LOAD2_state = 1;
  pinMode(PIN_LOAD1, OUTPUT);
  pinMode(PIN_LOAD2, OUTPUT);
  digitalWrite(PIN_LOAD1, LOAD1_state);    //Force Load 1 on at boot
  digitalWrite(PIN_LOAD2, LOAD2_state);    //Force Load 2 on at boot
}

void power_init(){
  Wire.begin();
  if(ina226.init()){
    power_avail = true;
    Serial.println("Power chip (INA226) Found!");
  } else {
    power_avail = false;
    Serial.println("Power chip (INA226) NOT Found!");
  }

  ina226.setAverage(INA226_AVERAGE_64);  // choose mode and uncomment for change of default
  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take:
     number of samples to be averaged x conversion time x 2
       Mode *         * conversion time
     CONV_TIME_140          140 µs
     CONV_TIME_204          204 µs
     CONV_TIME_332          332 µs
     CONV_TIME_588          588 µs
     CONV_TIME_1100         1.1 ms (default)
     CONV_TIME_2116       2.116 ms
     CONV_TIME_4156       4.156 ms
     CONV_TIME_8244       8.244 ms  */
  // ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default
  ina226.setResistorRange(0.001, 1.3);  // shunt, max current
  // ina226.setCorrectionFactor(0.93); //For setting a correction

  // ina226.readAndClearFlags();
  // supplyVoltage = ina226.getBusVoltage_V() + (ina226.getShuntVoltage_mV()/1000.0);
  // lowSupplyVoltage = supplyVoltage;
}

void eth_init(){
    Network.onEvent(onEvent);
    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
    ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, PIN_ETH_CS, PIN_ETH_INT, PIN_ETH_RST, SPI);
}

void ethernet_init(){
  // // Hardware reset the W5500 (critical for reliable startup)
  // pinMode(PIN_ETH_INT, INPUT);
  // if (PIN_ETH_RST != -1) {  // Only if connected
  //   pinMode(PIN_ETH_RST, OUTPUT);
  //   digitalWrite(PIN_ETH_RST, HIGH);  // Normal state
  //   delay(10);                        // Stabilize
  //   digitalWrite(PIN_ETH_RST, LOW);   // Assert reset (active low)
  //   delay(10);                        // Hold low ~10ms (per examples/datasheet)
  //   digitalWrite(PIN_ETH_RST, HIGH);  // Release reset
  //   delay(200);                       // Wait for chip to stabilize (100-400ms common)
  //   Serial.println("W5500 hardware reset complete");
  // }

  // // Initialize SPI and Ethernet
  // //SPI.begin();  // Default pins, or specify if custom: SPI.begin(SCK, MISO, MOSI, CS);
  // SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
  // Ethernet.init(PIN_ETH_CS);  // Critical: Set CS pin for W5500

  // Serial.println("Starting Ethernet...");

  // // Try DHCP first
  // if (Ethernet.begin(mac) == 0) {
  //   Serial.println("DHCP failed. Using static IP.");
  //   // Fallback static IP (adjust to your network)
  //   IPAddress ip(192, 168, 1, 177);
  //   IPAddress gateway(192, 168, 1, 1);
  //   IPAddress subnet(255, 255, 255, 0);
  //   Ethernet.begin(mac, ip, gateway, gateway, subnet);
  // }

  // // Wait for link
  // delay(1000);
  // if (Ethernet.hardwareStatus() == EthernetW5500) {
  //   Serial.println("W5500 detected.");
  // } else {
  //   Serial.println("No W5500 detected!");
  //   return;
  // }

  // if (Ethernet.linkStatus() == LinkON) {
  //   Serial.println("Link ON");
  // } else {
  //   Serial.println("Link OFF");
  //   return;
  // }

  // Serial.print("IP: ");
  // Serial.println(Ethernet.localIP());
}

// DST check function
bool isDST(struct tm *timeinfo) {
  int year = timeinfo->tm_year + 1900;

  // Second Sunday in March
  struct tm startDST = {0, 0, 2, 8, 2, year - 1900}; // March 8, 2:00 AM
  mktime(&startDST);
  int startDay = 7 - startDST.tm_wday;
  startDST.tm_mday += startDay;
  mktime(&startDST);

  // First Sunday in November
  struct tm endDST = {0, 0, 2, 1, 10, year - 1900}; // Nov 1, 2:00 AM
  mktime(&endDST);
  endDST.tm_mday += (7 - endDST.tm_wday) % 7;
  mktime(&endDST);

  time_t current = mktime(timeinfo);
  time_t start = mktime(&startDST);
  time_t end = mktime(&endDST);

  return current >= start && current < end;
}

void rtc_task() {
  static unsigned long last_reading = 0;
  unsigned long now = millis();
  if (now - last_reading < 1000) return;
  last_reading = now;

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
//    ledMode = RGB_MODE_RED;
//    status_reg |= (1<<(ERR_RTC)); 
//    sprintf(status_msg, "REAL TIME CLOCK ERROR");
  }else{
//    status_reg &= ~(1 << ERR_RTC);
  printLocalTimeFromEpoch(epoch + timezone*60);
  }
  epoch++;
}

void timeprint_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 10000) return;
  lastRun = now;  // reset timer
  printLocalTime();
}

bool syncTimeWithNTP() {
  if (!internet_connected)return false;

  configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // Get UTC time

  if (getLocalTime(&currentTime)) {
    rawTime = mktime(&currentTime);
    bool dst = isDST(&currentTime);
    int offset = timezone * 3600 + (dst && daylight_savings ? 3600 : 0);
    rawTime += offset;

    struct tm *adjustedTime = localtime(&rawTime);
    Serial.println("Adjusted local time:");
    Serial.println(adjustedTime, "%A, %B %d %Y %H:%M:%S");
    timeSynced = true;
    return true;
  } else {
    Serial.println("Failed to fetch NTP time.");
    return false;
  }


  // struct tm timeinfo;
  // if (getLocalTime(&timeinfo)) {
  //   bool dst = isDST(&timeinfo);
  //   int offset = dst ? -6 : -7; // MDT = UTC-6, MST = UTC-7

  //   time_t rawTime = mktime(&timeinfo);
  //   rawTime += offset * 3600; // Apply offset
  //   struct tm *localTime = localtime(&rawTime);

  //   Serial.print("Local time: ");
  //   Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  //   Serial.print("DST active: ");
  //   Serial.println(dst ? "Yes (MDT)" : "No (MST)");
  //   Serial.print("Adjusted time: ");
  //   Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");
  // } else {
  //   Serial.println("Failed to obtain time");
  // }



  // //get time without DLS
  // configTime(timezone * 3600, 0, "pool.ntp.org", "time.nist.gov");

  // if (getLocalTime(&currentTime)) {
  //   rawTime = mktime(&currentTime);  // Convert to time_t for easier handling
  //   if(isDST(&currentTime) && daylight_savings){
  //     configTime(timezone * 3600, 3600, "pool.ntp.org", "time.nist.gov");
  //     if (getLocalTime(&currentTime)) {
  //       rawTime = mktime(&currentTime);  // Convert to time_t for easier handling
  //       timeSynced = true;
  //       Serial.println("NTP time synced with DLS.");
  //       return true;
  //     } else {
  //       Serial.println("Failed to fetch NTP time with DLS.");
  //       return false;
  //     }
  //   }else{
  //     timeSynced = true;
  //     Serial.println("NTP time synced.");
  //     return true;
  //   }
  // } else {
  //   Serial.println("Failed to fetch NTP time.");
  //   return false;
  // }
}

void time_task(){
  static unsigned long lastSecond = 0;
  static unsigned long lastNTPSync = 0;
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  // Update every second
  if (now - lastSecond >= 1000UL) {
    lastSecond = now;

    secondsSinceBoot = now / 1000;
    lastWifiConnectTime = secondsSinceBoot - connectTime;
    lastWifiDisconnectTime = secondsSinceBoot - disconnectTime;

    if (timeSynced) {
      rawTime++; // Increment stored time_t value
      localtime_r(&rawTime, &currentTime); // Convert back to tm structure
    } else {
      currentTime.tm_sec++; // Fallback: increment seconds
      rawTime = mktime(&currentTime); // Normalize date and time
      localtime_r(&rawTime, &currentTime);
    }

    // Check if it's time to sync (every hour)
    if ((!timeSynced && !apModeActive) || now - lastNTPSync >= 3600000UL) {
      if (internet_connected) {
        if(syncTimeWithNTP()) lastNTPSync = now;
      } else {
        Serial.println("INTERNET not available. Skipping NTP sync.");
      }
    }
  }

  if (now - lastPrint >= 10000UL) {
    lastPrint = now;
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &currentTime);
    Serial.print("Current Time: ");
    Serial.println(timeStr);
  }
}

void internet_task(){
  static bool prev_connected = false;
  static unsigned long discTimer = 0;
  static unsigned long lastRead = 0;
  unsigned long now = millis();
  if (now - lastRead < 5000) return;
  lastRead = now;  // reset timer
  if (wired_internet_connected || wireless_internet_connected) internet_connected = true;
  else internet_connected = false;

  // if internet reconnected
  if(!prev_connected && internet_connected){
    discTimer = 0;

    // sync time to internet
    syncTimeWithNTP();
    getLocalTime(&currentTime);
    printLocalTime();

    // check for firmware update
    if(!firmware_checked){
      firmware_checked = true;
      if (FirmwareVersionCheck()) {
        firmwareUpdate();
      }
    }

    // Stop config mode once back online
    if (apModeActive) {
      stopConfigPage();
    }

    // Attempt to connect mqtt server
    if (!mqttClient.connected()) {
      mqttClient.disconnect();
      mqttClient.setServer(mqttServer, 1883);
      mqttClient.setCallback(mqttCallback);
      mqttClient.setKeepAlive(60);   // seconds
      if (mqttClient.connect(device_id.c_str())) {
        Serial.println("connected");
        // Subscribe
        mqttClient.subscribe(l1_topic.c_str());
        mqttClient.subscribe(l2_topic.c_str());
        mqttClient.subscribe(new_cred_topic.c_str());
        mqttClient.subscribe(ota_topic.c_str());
        mqttClient.subscribe(set_config_topic.c_str());
      }
    }
  }else if(internet_connected){
    discTimer = 0;
    repeatedFirmwareCheck();  // OTA
  }else if(!internet_connected){
    discTimer+=5000;
    if(!apModeActive && discTimer>apTimeout) startConfigPage();
  }
}

void power_task(){
  // read supply voltage every second
  // record the lowest value in the last 30 seconds for NVS write protection
  static float voltageBuffer[10];
  static uint8_t idx=0;

  static unsigned long lastRead = 0;
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastRead < 1000) return;
  lastRead = now;  // reset timer

  if(power_avail){
    ina226.readAndClearFlags();
    // shuntVoltage_mV = ina226.getShuntVoltage_mV();
    // busVoltage_V = ina226.getBusVoltage_V();
    // current_mA = ina226.getCurrent_mA();
    // power_mW = ina226.getBusPower();
    // loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

    supplyVoltage = ina226.getBusVoltage_V() + (ina226.getShuntVoltage_mV()/1000.0);
    supplyCurrent = ina226.getCurrent_mA()/1000;
    supplyPower = supplyVoltage * supplyCurrent;

    //record last 10 values in a ring buffer
    voltageBuffer[idx]=supplyVoltage;
    idx++;
    if(idx>=10)idx=0;
    //find the lowest voltage reading 
    for (uint8_t i = 0; i < 10; i++) {
      if (voltageBuffer[i] < lowSupplyVoltage) {
        lowSupplyVoltage = voltageBuffer[i];
      }
    }

    if(max_volts < supplyVoltage)max_volts = supplyVoltage;
    if(min_volts > supplyVoltage)min_volts = supplyVoltage;
    if(max_amps < supplyCurrent)max_amps = supplyCurrent;
    if(min_amps > supplyCurrent)min_amps = supplyCurrent;
    if(max_watts < supplyPower)max_watts = supplyPower;
    if(min_watts > supplyPower)min_watts = supplyPower;
  }else{
    // Serial.println("INA226 Power Sensor NOT Available!");
  }

  if (now - lastPrint >= 10000UL) {
    lastPrint = now;
    if(power_avail){
      Serial.print("supplyVoltage ");
      Serial.println(supplyVoltage);
      Serial.print("supplyCurrent ");
      Serial.println(supplyCurrent);
      Serial.print("supplyPower ");
      Serial.println(supplyPower);
    }
  }
}

void eth_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 5000) return;
  lastRun = now;  // reset timer

  if(ETH.linkUp() && ETH.localIP()!=IPAddress(0,0,0,0)){
    Serial.print("IP: ");
    Serial.println(ETH.localIP());

    Serial.print("Gateway: ");
    Serial.println(ETH.gatewayIP());

    Serial.print("DNS: ");
    Serial.println(ETH.dnsIP());

    // NetworkClient ethClient;
    WiFiClient ethClient;       //use WiFiClient instead of NetworkClient or EthernetClient, even for an ethernet connection
    if (ethClient.connect("www.google.com", 80)) {
      Serial.println("Ethernet Connection to the INTERNET!");
      wired_internet_connected=true;
      ethClient.stop();
    } else {
      Serial.println("Ethernet INTERNET Connection failed.");
      wired_internet_connected=false;
    }  
  }else{
    Serial.println("Ethernet not connected (no link or no IP)");
    wired_internet_connected=false;
  }
}
  // if (eth_connected) {
  //   NetworkClient ethClient;
  //   if (ethClient.connect("www.google.com", 80)) {
  //     Serial.println("Ethernet Connection to the INTERNET!");
  //     wired_internet_connected=true;
  //     ethClient.stop();
  //   } else {
  //     Serial.println("Ethernet INTERNET Connection failed.");
  //     wired_internet_connected=false;
  //   }  
  // }else{
  //   wired_internet_connected=false;
  // }
// }

void ethernet_task() {
  // static unsigned long lastRun = 0;
  // unsigned long now = millis();

  // // Run only every 5 seconds
  // if (now - lastRun < 5000) return;
  // lastRun = now;

  // // First check if Ethernet is physically connected and has an IP
  // if (Ethernet.linkStatus() == LinkON && Ethernet.localIP() != IPAddress(0,0,0,0)) {
  //   // We have link and a valid IP → test actual internet reachability
  //   EthernetClient ethClient;  // Note: EthernetClient, not NetworkClient

  //   if (ethClient.connect("www.google.com", 80)) {
  //     Serial.println("Ethernet Connection to the INTERNET!");
  //     wired_internet_connected = true;
  //     ethClient.stop();
  //   } else {
  //     Serial.println("Ethernet INTERNET Connection failed (no response from google.com)");
  //     wired_internet_connected = false;
  //   }
  // } else {
  //   // No link or no IP assigned
  //   Serial.println("Ethernet not connected (no link or no IP)");
  //   wired_internet_connected = false;
  // }

  // // Optional: maintain DHCP lease (recommended for long-running devices)
  // Ethernet.maintain();
}

int16_t vBatt = 0;
int16_t v5Bus = 0;
int16_t v3Bus = 0;

void btn_task(){
  if(!digitalRead(PIN_BTN)){
      digitalWrite(PIN_LEDR, LOW);
    }else{
      digitalWrite(PIN_LEDR, HIGH);
  }
}

void adc_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 1000) return;
  lastRun = now;  // reset timer

  if (ads.checkADS1115())
  {
      int16_t adc3;
      v5Bus = 2 * ads.readVoltage(0);
      Serial.print("5V Bus:");
      Serial.print(v5Bus);
      Serial.print("mV,  ");
      vBatt = 2 * ads.readVoltage(1);
      Serial.print("BATT:");
      Serial.print(vBatt);
      Serial.print("mV,  ");
      v3Bus = 2 * ads.readVoltage(2);
      Serial.print("3v3 Bus:");
      Serial.print(v3Bus);
      Serial.print("mV,  ");
      adc3 = ads.readVoltage(3);
      Serial.print("Therm:");
      Serial.print(adc3);
      Serial.println("mV");
      therm_temp=getTemperatureF(float(adc3));
      Serial.print("therm_temp: ");
      Serial.println(therm_temp);
  } else {
      Serial.println("ADS1115 Disconnected!");
  }
}

void th_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 1000) return;
  lastRun = now;  // reset timer

  sensors_event_t humidity, temp;
  
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature*9/5 + 32.0); Serial.println(" degrees F");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  if(temp_avail){
    // record relative humidity if read ok
    if( !isnan(temp.temperature) ){
      th_temp = temp.temperature*9/5 + 32;
    }
    // record relative humidity if read ok
    if( !isnan(humidity.relative_humidity) ){
      th_humid = humidity.relative_humidity;
      if(max_humid<box_humid)max_humid=box_humid;
      if(min_humid>box_humid)min_humid=box_humid;
    } else {
      th_humid = 0;
    }
    Serial.print("Temp: ");
    Serial.println(th_temp);
    Serial.print("Humidity: ");
    Serial.println(th_humid);
  } else {
    Serial.println("SHT40 Sensor NOT Available!");
    //sensor not available, so use the schedule
    FAN_state = LOAD1_state;
  }
}

void temp_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 1000) return;
  lastRun = now;  // reset timer
}

void fan_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 5000) return;
  lastRun = now;  // reset timer

    // record temperature if read ok
    // (disconnected temp is about -100F)
    if( therm_temp>0 ) {
      box_temp = therm_temp;
      if(max_temp<box_temp)max_temp=box_temp;
      if(min_temp>box_temp)min_temp=box_temp;
      if(box_temp>hi_temp){
        FAN_state = 1;
      }else if(box_temp<lo_temp){
        FAN_state = 0;
      }
    } else if( !isnan(th_temp) ) {
      box_temp = th_temp;
      if(max_temp<box_temp)max_temp=box_temp;
      if(min_temp>box_temp)min_temp=box_temp;
      if(box_temp>hi_temp){
        FAN_state = 1;
      }else if(box_temp<lo_temp){
        FAN_state = 0;
      }
    } else {
      //show invalid reading, but keep last output state
      box_temp = 0;
    }

  Serial.print("FAN_state: ");
  Serial.println(FAN_state);
  digitalWrite(PIN_FAN_12V, FAN_state);
  digitalWrite(PIN_FAN_5V, FAN_state);
}

void load_task(){
  static int8_t prev_timezone = 0;  
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 1000) return;
  lastRun = now;  // reset timer

  //detect timezone change
  if(prev_timezone != timezone){
    syncTimeWithNTP();
    //configTime(timezone * 3600, 3600, "pool.ntp.org", "time.nist.gov");
    prev_timezone = timezone;
  }

  curr_time = (100*currentTime.tm_hour) + currentTime.tm_min;

  if(overrideSeconds1){
    overrideSeconds1--;
    LOAD1_state=1;
  }else if(!timeSynced){
    LOAD1_state=1;
  }else if(off_time>on_time){
    bool shouldBeActive = (curr_time >= on_time && curr_time < off_time);
    if(shouldBeActive && !load1_should_be_active)      { load1_should_be_active = true; LOAD1_state=1; }
    else if(!shouldBeActive && load1_should_be_active) { load1_should_be_active = false; LOAD1_state=0; }
  }else if(off_time<on_time){
    bool shouldBeActive = (curr_time >= on_time || curr_time < off_time);
    if(shouldBeActive && !load1_should_be_active)      { load1_should_be_active = true; LOAD1_state=1; }
    else if(!shouldBeActive && load1_should_be_active) { load1_should_be_active = false; LOAD1_state=0; }
  }else{
    LOAD1_state=0;    
  }

  if(overrideSeconds2){
    overrideSeconds2--;
    LOAD2_state=1;
  }else if(!timeSynced){
    LOAD2_state=1;
  }else if(off_time2>on_time2){
    bool shouldBeActive2 = (curr_time >= on_time2 && curr_time < off_time2);
    if(shouldBeActive2 && !load2_should_be_active)      { load2_should_be_active = true; LOAD2_state=1; }
    else if(!shouldBeActive2 && load2_should_be_active) { load2_should_be_active = false; LOAD2_state=0; }
  }else if(off_time2<on_time2){
    bool shouldBeActive2 = (curr_time >= on_time2 || curr_time < off_time2);
    if(shouldBeActive2 && !load2_should_be_active)      { load2_should_be_active = true; LOAD2_state=1; }
    else if(!shouldBeActive2 && load2_should_be_active) { load2_should_be_active = false; LOAD2_state=0; }
  }else{
    LOAD2_state=0;
  }

//Reset lowest voltage of the day here
  if(curr_time == off_time){
    lowSupplyVoltage = 15.0;
  }
  // Serial.print("L2 CurrTime: ");
  // Serial.println(curr_time);
  // Serial.print("L2 OnTime: ");
  // Serial.println(on_time);
  // Serial.print("L2 OffTime: ");
  // Serial.println(off_time);
  // Serial.print("L2 State: ");
  // Serial.println(LOAD2_state);

  Serial.print("LOAD1_state: ");
  Serial.println(LOAD1_state);
  Serial.print("LOAD2_state: ");
  Serial.println(LOAD2_state);
  digitalWrite(PIN_LOAD1, LOAD1_state);
  digitalWrite(PIN_LOAD2, LOAD2_state);
}

void wifi_task(){
  static bool prevConnectState = false;
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 5000) return;
  lastRun = now;  // reset timer

  if(apModeActive)return;
  if(wired_internet_connected){
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if(!wifi_connected){
      start_mqtt=true;
    }
    wifi_connected=true;

    Serial.println("WiFi connected.");
    // digitalWrite(wifi_led, HIGH);

    if(!prevConnectState){
      connectTime = millis()/1000;
      prevConnectState=true;
      Serial.print("connectTime: ");
      Serial.println(connectTime);
    }

    WiFiClient wfClient;
    if (wfClient.connect("www.google.com", 80)) {
      Serial.println("Wireless Connection to the INTERNET!");
      wireless_internet_connected=true;
      wfClient.stop();
    } else {
      Serial.println("Wireless INTERNET Connectionfailed.");
      wireless_internet_connected=false;
    }  

    // if (wifiDisconnectedSince > 0) {
    //   Serial.println("WiFi reconnected!");
    //   wifiDisconnectedSince = 0;

      // if (apModeActive) {
      //   stopConfigPage();  // Stop config mode once back online
      // }

      // Start services only once after reconnect
      // if(start_mqtt){
      //   start_mqtt=false;
      //   mqttClient.setServer(mqttServer, 1883);
      //   mqttClient.setCallback(mqttCallback);
      //   if (!mqttClient.connected()) reconnect();
      // }

      // syncTimeWithNTP();
      // //configTime(timezone * 3600, 3600, "pool.ntp.org", "time.nist.gov");
      // getLocalTime(&currentTime);
      // printLocalTime();

      // if(!firmware_checked){
      //   firmware_checked = true;
      //   if (FirmwareVersionCheck()) {
      //     firmwareUpdate();
      //   }
      // }
    // }
    // repeatedFirmwareCheck();  // OTA

  }else{
    Serial.println("WiFi disconnected.");
    // digitalWrite(wifi_led, LOW);
    wifi_connected=false;

    if(prevConnectState){
      disconnectTime = millis()/1000;
      prevConnectState=false;
    }

    // if (wifiDisconnectedSince == 0) {
    //   wifiDisconnectedSince = millis();  // Start timer
    //   Serial.println("WiFi starting reconnect attempts...");
    // }

    // Keep retrying every 10 seconds (non-blocking)
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry >= 10000) {
      lastRetry = millis();
      Serial.println("Retrying WiFi...");
      connectToWiFi(false, false);
    }

    // If timeout passed and AP not active
    // if (!apModeActive && (millis() - wifiDisconnectedSince >= apTimeout)) {
    //   // startConfigPage();
    // }else{
    //   Serial.print("wifiDisconnectedSince: ");
    //   Serial.println((millis() - wifiDisconnectedSince)/1000);
    // }
  }
}

void server_task() {
  static unsigned long tick = 0;
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 10000) return;
  lastRun = now;  // reset timer

  if(!apModeActive) return;

  int connectedClients = WiFi.softAPgetStationNum();
  Serial.print("Connected Clients: ");
  Serial.println(connectedClients);

  //If stuck in config mode
  if(tick>=60 && connectedClients==0){
    //reboot the device
    ESP.restart();
  }

  tick++;
}

void update_topics() {
  //Ensure correct device ID is in the topic
  l1_topic = String(DEVICE_ID) + "/inverter/state";
  l2_topic = device_id + "/cig/state";
  wifi_topic = device_id + "/stored_wifi";
  new_cred_topic = device_id + "/wifi_creds";
  ota_topic = device_id + "/FOTA";
  fw_topic = device_id + "/fwVer";
  batt_topic = device_id + "/bat/voltage";

  dev_mac_topic = device_id + "/device/mac";
  dev_fw_topic = device_id + "/device/firmware";
  dev_ram_topic = device_id + "/device/total_ram";
  dev_free_ram_topic = device_id + "/device/free_ram";
  dev_used_ram_topic = device_id + "/device/used_ram";
  dev_used_flash_topic = device_id + "/device/used_flash";

  cfg_dls_time_topic = device_id + "/config/dls_time";
  cfg_timezone_topic = device_id + "/config/timezone";
  cfg_lo_temp_topic = device_id + "/config/fan_offtemp";
  cfg_hi_temp_topic = device_id + "/config/fan_ontemp";
  cfg_off_time_topic = device_id + "/config/l1_offtime";
  cfg_on_time_topic = device_id + "/config/l1_ontime";
  cfg_override_topic = device_id + "/config/l1_override";
  cfg_off_time2_topic = device_id + "/config/l2_offtime";
  cfg_on_time2_topic = device_id + "/config/l2_ontime";
  cfg_override2_topic = device_id + "/config/l2_override";

  stat_boots_topic = device_id + "/status/boots";
  stat_reason_topic = device_id + "/status/reboot_reason";
  stat_lastboot_topic = device_id + "/status/last_boot";
  stat_disconnect_topic = device_id + "/status/last_disconnect";
  stat_connect_topic = device_id + "/status/last_connect";
  stat_time_topic = device_id + "/status/time";
  stat_currtime_topic = device_id + "/status/current_time";

  net_ip_topic = device_id + "/network/ip_address";
  net_rssi_topic = device_id + "/network/rssi";
  net_ssid_topic = device_id + "/network/ssid";
  net_pw_topic = device_id + "/network/pw";
  net_ssid2_topic = device_id + "/network/ssid2";
  net_pw2_topic = device_id + "/network/pw2";
  net_enet_topic = device_id + "/network/enet";

  sens_lv_topic = device_id + "/sensor/lowvoltage";
  sens_v_topic = device_id + "/sensor/voltage";
  sens_i_topic = device_id + "/sensor/current";
  sens_p_topic = device_id + "/sensor/power";
  sens_tp_topic = device_id + "/sensor/tempprobe";
  sens_t_topic = device_id + "/sensor/temperature";
  sens_h_topic = device_id + "/sensor/humidity";

  load_fan_topic = device_id + "/load/fan";
  load_1_topic = device_id + "/load/1";
  load_2_topic = device_id + "/load/2";

  sens_vmx_topic = device_id + "/sensor/vmax";
  sens_vmn_topic = device_id + "/sensor/vmin";
  sens_imx_topic = device_id + "/sensor/imax";
  sens_imn_topic = device_id + "/sensor/imin";
  sens_pmx_topic = device_id + "/sensor/pmax";
  sens_pmn_topic = device_id + "/sensor/pmin";
  sens_tmx_topic = device_id + "/sensor/tmax";
  sens_tmn_topic = device_id + "/sensor/tmin";

  set_config_topic = device_id + "/set_config/#";
  set_dls_time_topic = device_id + "/set_config/dls_time";
  set_timezone_topic = device_id + "/set_config/timezone";
  set_lo_temp_topic = device_id + "/set_config/fan_offtemp";
  set_hi_temp_topic = device_id + "/set_config/fan_ontemp";
  set_off_time_topic = device_id + "/set_config/l1_offtime";
  set_on_time_topic = device_id + "/set_config/l1_ontime";
  set_override_topic = device_id + "/set_config/l1_override";
  set_off_time2_topic = device_id + "/set_config/l2_offtime";
  set_on_time2_topic = device_id + "/set_config/l2_ontime";
  set_override2_topic = device_id + "/set_config/l2_override";
}

void mqtt_blip(bool on){
  static bool prevR = 0;
  static bool prevG = 0;
  static bool prevB = 0;
  if(on){
    prevR = digitalRead(PIN_LEDR);
    prevG = digitalRead(PIN_LEDG);
    prevB = digitalRead(PIN_LEDB);
    digitalWrite(PIN_LEDR, LOW);
    digitalWrite(PIN_LEDG, HIGH);
    digitalWrite(PIN_LEDB, LOW);
  }else{
    digitalWrite(PIN_LEDR, prevR);
    digitalWrite(PIN_LEDG, prevG);
    digitalWrite(PIN_LEDB, prevB);
  }
}

#define PUBLISH_DWELL 10
void mqtt_task(){
  static uint8_t retries = 0;

  if(!internet_connected)return;

  // if (WiFi.status() == WL_CONNECTED){
  //   if(!wifi_connected){
  //     start_mqtt=true;
  //   }
  //   wifi_connected=true;
  // }else{
  //   wifi_connected=false;
  //   //abort if not connected
  //   return;
  // }

  mqttClient.loop();

  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 5000) return;
  lastRun = now;  // reset timer

  static uint8_t idx=0;

  update_topics();

  String fan_state = String(FAN_state);
  String load1_state = String(LOAD1_state);
  String load2_state = String(LOAD2_state);
  String L1_TOPIC = String(DEVICE_ID) + "/inverter/state";
  String L2_TOPIC = device_id + "/cig/state";

  // StaticJsonDocument<64> doc;
  JsonDocument doc;

  // float current, voltage = 0.0;
  char output[110];

  IPAddress ip = WiFi.localIP();
  String ipStr = ip.toString();

  char buffer[5];  // 4 digits + null terminator
  sprintf(buffer, "%04d", off_time);
  String milOffTime = String(buffer);
  sprintf(buffer, "%04d", on_time);
  String milOnTime = String(buffer);
  sprintf(buffer, "%04d", off_time2);
  String milOffTime2 = String(buffer);
  sprintf(buffer, "%04d", on_time2);
  String milOnTime2 = String(buffer);

  size_t freeRam = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t totalRam = heap_caps_get_total_size(MALLOC_CAP_8BIT);
  size_t usedRam = totalRam - freeRam;
  size_t ram_usage = 100 * usedRam / totalRam;
  char ramMsg[50];
  sprintf(ramMsg, "Free:%u Used:%u Total:%u", (unsigned int)freeRam, (unsigned int)usedRam, (unsigned int)totalRam);
  mqttClient.publish("system/ram", ramMsg);

  if (!mqttClient.connected()){
    // Serial.print("Reconnecting to MQTT Server: ");
    // Serial.println(mqttServer);
    retries++;
    reconnect();
  } else {
    retries=0;
    switch(idx){
      case 0:
        // measure_battery_consumption(current, voltage);
        doc["ssid"] = ssid;
        doc["pass"] = password;
        serializeJsonPretty(doc, output);

        Serial.print("Publishing MQTT Data - [");
        Serial.print(device_id);
        Serial.println("]/esp/#");
        mqtt_blip(true);
        mqttClient.publish(fw_topic.c_str(), FirmwareVer.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(batt_topic.c_str(), String(supplyVoltage).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(L1_TOPIC.c_str(), String(load1_state).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(L2_TOPIC.c_str(), String(load2_state).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(wifi_topic.c_str(), output);
        mqtt_blip(false);        
      break;
      case 1:
        Serial.print("Publishing Device Data - [");
        Serial.print(device_id);
        Serial.println("]/device/#");
        mqtt_blip(true);
        mqttClient.publish(dev_mac_topic.c_str(), WiFi.macAddress().c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(dev_fw_topic.c_str(), FirmwareVer.c_str());
        delay(PUBLISH_DWELL);
        // mqttClient.publish(dev_ram_topic.c_str(), String(totalRam).c_str());
        // delay(PUBLISH_DWELL);
        // mqttClient.publish(dev_free_ram_topic.c_str(), String(freeRam).c_str());
        // delay(PUBLISH_DWELL);
        // mqttClient.publish(dev_used_ram_topic.c_str(), String(usedRam).c_str());
        mqttClient.publish(dev_used_ram_topic.c_str(), String(ram_usage).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(dev_used_flash_topic.c_str(), String(nvs_usage).c_str());
        mqtt_blip(false);
      break;
      case 2:
        Serial.print("Publishing Config Data - [");
        Serial.print(device_id);
        Serial.println("]/config/#");
        mqtt_blip(true);
        mqttClient.publish(cfg_dls_time_topic.c_str(), String(daylight_savings).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_timezone_topic.c_str(), String(timezone).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_lo_temp_topic.c_str(), String(lo_temp).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_hi_temp_topic.c_str(), String(hi_temp).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_off_time_topic.c_str(), milOffTime.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_on_time_topic.c_str(), milOnTime.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_override_topic.c_str(), String(overrideSeconds1).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_off_time2_topic.c_str(), milOffTime2.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_on_time2_topic.c_str(), milOnTime2.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(cfg_override2_topic.c_str(), String(overrideSeconds2).c_str());
        mqtt_blip(false);
      break;
      case 3:
        Serial.print("Publishing Status Data - [");
        Serial.print(device_id);
        Serial.println("]/status/#");
        mqtt_blip(true);
        mqttClient.publish(stat_boots_topic.c_str(), String(boots).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_reason_topic.c_str(), String(reasonStr).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_lastboot_topic.c_str(), String(secondsSinceBoot).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_disconnect_topic.c_str(), String(lastWifiDisconnectTime).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_connect_topic.c_str(), String(lastWifiConnectTime).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_time_topic.c_str(), String(timeStr).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(stat_currtime_topic.c_str(), String(curr_time).c_str());
        mqtt_blip(false);
      break;
      case 4:
        Serial.print("Publishing Network Data - [");
        Serial.print(device_id);
        Serial.println("]/network/#");
        mqtt_blip(true);
        mqttClient.publish(net_ip_topic.c_str(), ipStr.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_rssi_topic.c_str(), String(WiFi.RSSI()).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_ssid_topic.c_str(), ssid.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_pw_topic.c_str(), password.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_ssid2_topic.c_str(), ssid2.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_pw2_topic.c_str(), password2.c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(net_enet_topic.c_str(), String(eth_connected).c_str());
        mqtt_blip(false);
      break;
      case 5:
        Serial.print("Publishing Sensors Data - [");
        Serial.print(device_id);
        Serial.println("]/sensor/#");
        mqtt_blip(true);
        mqttClient.publish(sens_lv_topic.c_str(), String(lowSupplyVoltage).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_v_topic.c_str(), String(supplyVoltage).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_i_topic.c_str(), String(supplyCurrent).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_p_topic.c_str(), String(supplyPower).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_tp_topic.c_str(), String(therm_temp).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_t_topic.c_str(), String(th_temp).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_h_topic.c_str(), String(th_humid).c_str());
        mqtt_blip(false);
      break;
      case 6:
        Serial.print("Publishing Load Data - [");
        Serial.print(device_id);
        Serial.println("]/load/#");
        mqtt_blip(true);
        mqttClient.publish(load_fan_topic.c_str(), String(fan_state).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(load_1_topic.c_str(), String(load1_state).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(load_2_topic.c_str(), String(load2_state).c_str());
        mqtt_blip(false);
      break;
      case 7:
        Serial.print("Publishing Extreme Data - [");
        Serial.print(device_id);
        Serial.println("]/sensor/#");
        mqtt_blip(true);
        mqttClient.publish(sens_vmx_topic.c_str(), String(max_volts).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_vmn_topic.c_str(), String(min_volts).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_imx_topic.c_str(), String(max_amps).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_imn_topic.c_str(), String(min_amps).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_pmx_topic.c_str(), String(max_watts).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_pmn_topic.c_str(), String(min_watts).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_tmx_topic.c_str(), String(max_temp).c_str());
        delay(PUBLISH_DWELL);
        mqttClient.publish(sens_tmn_topic.c_str(), String(min_temp).c_str());
        mqtt_blip(false);
      break;
      default:
      break;
    }
    idx++;
    if(idx>=7)idx=0;
  }

  if(retries%3==2){
    connectToWiFi(true, false);
  }

  if(retries>20)ESP.restart();
}

void rgb_task(){
  static uint8_t tick = 0;
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 100) return;
  lastRun = now;  // reset timer

  // RGB LED Behavior Matrix
  //   WHT SOLID       POST
  //   OFF             SLEEPING
  //   RED SOLID       DISCONNECTED
  //   BLU SOLID       BLE
  //   GRN SOLID       ENET
  //   CYN SOLID       WIFI
  //   MAG BLINK       MQTT
  //   YLW SOLID       WEB SERVER
  //   MAG SOLID       OTA

  //if sleeping
  //  off
      // digitalWrite(PIN_LEDR, HIGH);
      // digitalWrite(PIN_LEDG, HIGH);
      // digitalWrite(PIN_LEDB, HIGH);
  //else if BLE
  //  blue
      // digitalWrite(PIN_LEDR, HIGH);
      // digitalWrite(PIN_LEDG, HIGH);
      // digitalWrite(PIN_LEDB, LOW);
  //else if OTA
  //  magenta
      // digitalWrite(PIN_LEDR, LOW);
      // digitalWrite(PIN_LEDG, HIGH);
      // digitalWrite(PIN_LEDB, LOW);
  //else
  if(apModeActive){
      digitalWrite(PIN_LEDR, LOW);
      digitalWrite(PIN_LEDG, LOW);
      digitalWrite(PIN_LEDB, HIGH);
  }else if(eth_connected){
      digitalWrite(PIN_LEDR, HIGH);
      digitalWrite(PIN_LEDG, LOW);
      digitalWrite(PIN_LEDB, HIGH);
  }else if(wifi_connected){
      digitalWrite(PIN_LEDR, HIGH);
      digitalWrite(PIN_LEDG, LOW);
      digitalWrite(PIN_LEDB, LOW);
  }else{ //disconnected
      digitalWrite(PIN_LEDR, LOW);
      digitalWrite(PIN_LEDG, HIGH);
      digitalWrite(PIN_LEDB, HIGH);
  }
  tick++;
}

void config_task(){
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < SAVE_INTERVAL) return;
  if(config_changed && supplyVoltage >= SAFE_VOLTAGE){
    save_settings(PRIMARY_NS);
    save_settings(BACKUP_NS);
    config_changed = false;
    lastRun = now;  // reset timer
  }
}

void reset_task(){
  static uint8_t ticks = 0;
  static unsigned long lastRun = 0;
  unsigned long now = millis();
  if (now - lastRun < 500) return;
  lastRun = now;  // reset timer

  if(!digitalRead(PIN_BTN)) ticks++;
  else ticks = 0;

  // Factory reset button
  if(ticks>=20) {   //this is a 10 second button hold 
    ticks=0;

    // clear_settings(PRIMARY_NS);
    // clear_settings(BACKUP_NS);

    ssid.clear();
    password.clear();
    ssid2.clear();
    password2.clear();
    email.clear();
    hi_temp = 100;            // Fan on at 100°F
    lo_temp = 90;             // Fan off at 90°F
    off_time = 2300;          // Load 1 off at 11 PM
    on_time = 700;            // Load 1 on at 7 AM
    off_time2 = 2300;         // Load 2 off at 11 PM
    on_time2 = 700;           // Load 2 on at 7 AM
    timezone = -8;            // Pacific Time
    daylight_savings = true;  // DLS Enabled
    save_settings(PRIMARY_NS);
    save_settings(BACKUP_NS);
    Serial.println("Cleared preferences, restarting...");
    delay(3000);
    ESP.restart();
  }
}

// Helper: publish any state change (call this instead of mqttClient.publish)
void serial_publish(const String& topic, const String& payload) {
  Serial2.println(topic + " " + payload);
}

void serial_publish(const String& topic, float value) {
  serial_publish(topic, String(value, 2));
}

void serial_publish(const String& topic, int value) {
  serial_publish(topic, String(value));
}

void serial_publish(const String& topic, uint32_t value) {
  serial_publish(topic, String(value));
}

void print_serial_help() {
  Serial2.println("Available topics:");
  Serial2.println("  device/id");
  Serial2.println("  device/firmware");

  Serial2.println("  status/boots");
  Serial2.println("  status/reboot_reason");
  Serial2.println("  status/time");
  Serial2.println("  status/current_time");

  Serial2.println("  sensor/lowvoltage");
  Serial2.println("  sensor/voltage");
  Serial2.println("  sensor/current");
  Serial2.println("  sensor/power");
  Serial2.println("  sensor/tempprobe");
  Serial2.println("  sensor/temperature");
  Serial2.println("  sensor/humidity");
  Serial2.println("  sensor/vmax");
  Serial2.println("  sensor/vmin");
  Serial2.println("  sensor/imax");
  Serial2.println("  sensor/imin");
  Serial2.println("  sensor/pmax");
  Serial2.println("  sensor/pmin");
  Serial2.println("  sensor/tmax");
  Serial2.println("  sensor/tmin");

  Serial2.println("  load/fan");
  Serial2.println("  load/1");
  Serial2.println("  load/2");

  Serial2.println("  config/dls_time");
  Serial2.println("  config/timezone");
  Serial2.println("  config/fan_offtemp");
  Serial2.println("  config/fan_ontemp");
  Serial2.println("  config/l1_offtime");
  Serial2.println("  config/l1_ontime");
  Serial2.println("  config/l2_offtime");
  Serial2.println("  config/l2_ontime");
  Serial2.println("  config/l1_override");
  Serial2.println("  config/l2_override");

  Serial2.println("  set_config/dls_time (0 or 1)");
  Serial2.println("  set_config/timezone (-2 to -12)");
  Serial2.println("  set_config/fan_offtemp 90");
  Serial2.println("  set_config/fan_ontemp 100");
  Serial2.println("  set_config/l1_ontime 0700");
  Serial2.println("  set_config/l1_offtime 2300");
  Serial2.println("  set_config/l2_ontime 0700");
  Serial2.println("  set_config/l2_offtime 2300");
  Serial2.println("  set_config/l1_override 3600");
  Serial2.println("  set_config/l2_override 3600");
  Serial2.println("  reboot");
  Serial2.println("  ping");
  Serial2.println("  help");
}

void process_serial_command(const String& cmd) {
  Serial.print("> ");          // Echo received command
  Serial.println(cmd);

  int spaceIndex = cmd.indexOf(' ');
  String topic = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
  String payload = (spaceIndex == -1) ? "" : cmd.substring(spaceIndex + 1);

  // === STATUS COMMANDS ===
  if (topic == "device/id") serial_publish("device/id", device_id);
  else if (topic == "device/firmware") serial_publish("device/firmware", FirmwareVer);

  else if (topic == "status/boots") serial_publish("status/boots", boots);
  else if (topic == "status/reboot_reason") serial_publish("status/reboot_reason", reasonStr);
  else if (topic == "status/time") serial_publish("status/time", timeStr);
  else if (topic == "status/current_time") serial_publish("status/current_time", curr_time);

  else if (topic == "sensor/lowvoltage") serial_publish("sensor/lowvoltage", lowSupplyVoltage);
  else if (topic == "sensor/voltage") serial_publish("sensor/voltage", supplyVoltage);
  else if (topic == "sensor/current") serial_publish("sensor/current", supplyCurrent);
  else if (topic == "sensor/power") serial_publish("sensor/power", supplyPower);
  else if (topic == "sensor/tempprobe") serial_publish("sensor/tempprobe", therm_temp);
  else if (topic == "sensor/temperature") serial_publish("sensor/temperature", th_temp);
  else if (topic == "sensor/humidity") serial_publish("sensor/humidity", th_humid);
  else if (topic == "sensor/vmax") serial_publish("sensor/vmax", max_volts);
  else if (topic == "sensor/vmin") serial_publish("sensor/vmin", min_volts);
  else if (topic == "sensor/imax") serial_publish("sensor/imax", max_amps);
  else if (topic == "sensor/imin") serial_publish("sensor/imin", min_amps);
  else if (topic == "sensor/pmax") serial_publish("sensor/pmax", max_watts);
  else if (topic == "sensor/pmin") serial_publish("sensor/pmin", min_watts);
  else if (topic == "sensor/tmax") serial_publish("sensor/tmax", max_temp);
  else if (topic == "sensor/tmin") serial_publish("sensor/tmin", min_temp);

  else if (topic == "load/fan") serial_publish("load/fan", FAN_state);
  else if (topic == "load/1") serial_publish("load/1", LOAD1_state);
  else if (topic == "load/2") serial_publish("load/2", LOAD2_state);

  else if (topic == "config/dls_time") serial_publish("config/dls_time", daylight_savings);
  else if (topic == "config/timezone") serial_publish("config/timezone", timezone);
  else if (topic == "config/fan_offtemp") serial_publish("config/fan_offtemp", lo_temp);
  else if (topic == "config/fan_ontemp") serial_publish("config/fan_ontemp", hi_temp);
  else if (topic == "config/l1_offtime") serial_publish("config/l1_offtime", off_time);
  else if (topic == "config/l1_ontime") serial_publish("config/l1_ontime", on_time);
  else if (topic == "config/l2_offtime") serial_publish("config/l2_offtime", off_time2);
  else if (topic == "config/l2_ontime") serial_publish("config/l2_ontime", on_time2);
  else if (topic == "config/l1_override") serial_publish("config/l1_override", overrideSeconds1);
  else if (topic == "config/l2_override") serial_publish("config/l2_override", overrideSeconds2);

  // === SETTABLE CONFIG COMMANDS ===
  else if (topic == "set_config/dls_time") {
    daylight_savings = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/dls_time", payload);
  }
  else if (topic == "set_config/timezone") {
    timezone = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/timezone", payload);
  }
  else if (topic == "set_config/fan_offtemp") {
    lo_temp = payload.toFloat();
    save_settings(PRIMARY_NS);
    serial_publish("config/fan_offtemp", payload);
  }
  else if (topic == "set_config/fan_ontemp") {
    hi_temp = payload.toFloat();
    save_settings(PRIMARY_NS);
    serial_publish("config/fan_ontemp", payload);
  }
  else if (topic == "set_config/l1_ontime") {
    on_time = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/l1_ontime", payload);
  }
  else if (topic == "set_config/l1_offtime") {
    off_time = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/l1_offtime", payload);
  }
  else if (topic == "set_config/l2_ontime") {
    on_time2 = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/l2_ontime", payload);
  }
  else if (topic == "set_config/l2_offtime") {
    off_time2 = payload.toInt();
    save_settings(PRIMARY_NS);
    serial_publish("config/l2_offtime", payload);
  }

  else if (topic == "set_config/l1_override") {
    overrideSeconds1 = payload.toInt();
    serial_publish("config/l1_override", payload);
  }
  else if (topic == "set_config/l2_override") {
    overrideSeconds1 = payload.toInt();
    serial_publish("config/l2_override", payload);
  }

  // // === LOAD CONTROL ===
  // else if (topic == "load/fan") {
  //   if (payload == "on" || payload == "1") set_fan_on();
  //   else if (payload == "off" || payload == "0") set_fan_off();
  //   else if (payload == "auto") set_fan_auto();
  //   // status will be published automatically by your existing fan_task()
  // }
  // else if (topic == "load/1") {
  //   if (payload == "on" || payload == "1") set_load1_on();
  //   else if (payload == "off" || payload == "0") set_load1_off();
  //   // status published by load_task()
  // }
  // else if (topic == "load/2") {
  //   if (payload == "on" || payload == "1") set_load2_on();
  //   else if (payload == "off" || payload == "0") set_load2_off();
  // }

  // === SPECIAL COMMANDS ===
  else if (topic == "reboot") {
    Serial.println("Rebooting from serial proxy command...");
    delay(200);
    ESP.restart();
  }
  else if (topic == "help") {
    print_serial_help();
  }
  else if (topic == "ping") {
    Serial2.println("pong");
  }
  else {
    Serial2.println("Invalid");
  }
}

void serial_proxy_init() {
  // Serial2 on GPIO16 (RX) and GPIO17 (TX) for command proxy
  Serial2.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD);  // Baud, config, RX pin, TX pin
  Serial.println("Serial proxy active.");
}

void serial_proxy_task() {
  static String inputLine = "";

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        inputLine.trim();
        process_serial_command(inputLine);
        inputLine = "";
      }
    } else if (c >= 32 && c <= 126) {  // Printable ASCII only
      inputLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("");
  Serial.println("--- WOLFPAK+ DUAL-LOAD CONTROLLER --- ");

  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: reasonStr = "Power-On"; break;
    case ESP_RST_EXT:     reasonStr = "External Reset"; break;
    case ESP_RST_SW:      reasonStr = "Software Reset"; break;
    case ESP_RST_PANIC:   reasonStr = "Brownout / Panic"; break;
    case ESP_RST_INT_WDT: reasonStr = "Interrupt Watchdog"; break;
    case ESP_RST_TASK_WDT:reasonStr = "Task Watchdog"; break;
    // case ESP_RST_OTHER:   reasonStr = "Other"; break;
    default:              reasonStr = "Unknown"; break;
  }

  Serial.print("Firmware Version: ");
  Serial.println(FirmwareVer);

  rgb_init();
  fan_init();
  load_init();

  // Initialize Preferences
  // preferences.begin("Creds", false);

  //Store Device ID if version is 1.0 otherwise if its a firmware update, read the previously stored Device ID...
  // if (FirmwareVer.toFloat() <= FW_VERSION) {
  //   preferences.putString("device_id", device_id);
  // } else {
  //   device_id = preferences.getString("device_id", "");
  // }

  // if (device_id == "") {
  //   Serial.println("device_id no longer exists.");
  // } else {
  //   Serial.println("device_id is still present: " + deviceID);
  // }

  // Load saved credentials
  // ssid = preferences.getString("ssid", "");
  // password = preferences.getString("password", "");
  // email = preferences.getString("email", "");



  // Load settings
  sync_settings();

  local_ssid = "WOLFPAK_" + device_id + "_Config";

  btn_init();
  adc_init();
  th_init();
  rtc_init();
  power_init();
  eth_init();
  // ethernet_init();

  apModeActive = false;  // Ensure clean state
  connectTime = millis()/1000;
  disconnectTime = millis()/1000;
  connectToWiFi(false, false);

  // connectToWiFi(true, false);

  // if (WiFi.status() == WL_CONNECTED) {
    // if(!wifi_connected){
    //   start_mqtt=true;
    // }
    // wifi_connected=true;
  //   Serial.println("Connected to WiFi!");
  //   // digitalWrite(wifi_led, HIGH);

  //   if(syncTimeWithNTP()) {
  //     printLocalTime();
  //   } else {
  //     Serial.println("NTP time fetch failed");
  //   }

  //   if (FirmwareVersionCheck()) {
  //     firmwareUpdate();
  //   }

  //   mqttClient.setServer(mqttServer, 1883);
  //   mqttClient.setCallback(mqttCallback);
    
  // }else{
  //   Serial.println("NOT Connected to WiFi!");
  //   // digitalWrite(wifi_led, LOW);
  //   wifi_connected=false;
  // }

  esp_err_t result = nvs_get_stats(NULL, &nvsStats);
  nvs_usage = 100*nvsStats.free_entries/nvsStats.total_entries;
  Serial.print("NVS Usage (%): ");
  Serial.println(nvs_usage);

  serial_proxy_init();

  delay(1000);
  digitalWrite(PIN_LEDR, HIGH);
  digitalWrite(PIN_LEDG, HIGH);
  digitalWrite(PIN_LEDB, HIGH);
}

void loop() {
  internet_task();
  time_task();
  power_task();

  config_task();
  reset_task();

  // btn_task();
  adc_task();
  th_task();
  temp_task();
  fan_task();
  load_task();

  wifi_task();
  eth_task();
  // ethernet_task();
  server_task();
  mqtt_task();
  rgb_task();

  serial_proxy_task();
}
