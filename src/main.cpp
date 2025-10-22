#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <APRS-Decoder.h>
#include <math.h>
#include <string>
#include <ElegantOTA.h>
#include <SPIFFS.h>
#include "board_pins.h"
#include <WebSerial.h>
//---------igate rx--> inet ------
#define IGATE_BIDIRECTIONAL false // Set to true to enable bidirectional iGate (transmit packets received via LoRa to RF)

//----hesla--------
String web_username = "admin"; // Výchozí uživatelské jméno
String web_password = "admin"; // Výchozí heslo

// Function prototypes
double convertToDecimalDegrees_la(const String& gpsString);
double convertToDecimalDegrees_lo(const String& gpsString);

//-------- Definice LED --------
const int PLED1 = 25; // Pin pro LED na Trackeru

//-------- Wi-Fi -----------------
String IP = "0.0.0.0";
String ssid = "Vlas_dolni_vlkys";
String password = "xxxxx";
String ap_password = "mojeheslo123"; // Heslo pro AP

//-------- Verze -----------------
String verze = "5.0.00"; // Aktualizováno pro editaci config.txt

//-------- APRS ID ---------------
String call = "OK5TVR-17";
String lon = "4947.18N";
String lat = "01317.10E";
String sym = "L";
String tool = "RX_Igate_LoRA by OK5TVR";
float Alt = 324.0;
String aprs_filter = "10m";

//-------- APRS Setup ------------
char servername[] = "rotate.aprs.net";
long aprs_port = 14580;
String user = call;
String password_aprs = "xxxxx";

//-------- IP Address ------------
bool pouzitPevnouIP = false;
String ipString = "192.168.1.189";
IPAddress local_IP(192, 168, 1, 189);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 1, 1);
IPAddress secondaryDNS(192, 168, 1, 1);

//-------- Buffer ----------------
#define BUFFER_SIZE 5
char buffer[BUFFER_SIZE][10];
char buffer_SN[BUFFER_SIZE][10];
char buffer_RSSI[BUFFER_SIZE][10];
char buffer_pak[BUFFER_SIZE][128];
char buffer_cas[BUFFER_SIZE][10];
char buffer_vzdalenost[BUFFER_SIZE][10];
char buffer_azimut[BUFFER_SIZE][10];
char buffer_icona[BUFFER_SIZE][400];
double buffer_lat[BUFFER_SIZE]; // Array for latitude
double buffer_lon[BUFFER_SIZE]; // Array for longitude
byte cnt = 0;
int filled = 0; 

//-----bufer zpravy-----------
//-------- Buffer pro zprávy --------
#define MESSAGE_BUFFER_SIZE 5
char buffer_zpravy[MESSAGE_BUFFER_SIZE][128]; // Text zprávy
char buffer_zpravy_cas[MESSAGE_BUFFER_SIZE][10]; // Čas přijetí
char buffer_zpravy_odesilatel[MESSAGE_BUFFER_SIZE][10]; // Odesílatel
long buffer_zpravy_age[MESSAGE_BUFFER_SIZE]; // Věk zprávy
int message_cnt = 0;
int posGt = 0;
String call_d = ""; // Call sign of the station

//-------- Digi/iGate/AP ------------
int digi_mode = 1;
int digi_AP = 0;
String lastStation = "";
double lastStationLat = NAN;   // << přidáno
double lastStationLon = NAN;   // << přidáno

//-------- OLED Display -----------
#ifndef OLED_SDA
  #define OLED_SDA 21
#endif
#ifndef OLED_SCL
  #define OLED_SCL 22
#endif
#ifndef OLED_RST
  #define OLED_RST 16
#endif
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//-------- LoRa ------------------
#ifdef LORA_SS
  #ifndef lora_SS
    #define lora_SS LORA_SS
  #endif
#endif
#ifdef LORA_RST
  #ifndef lora_RST
    #define lora_RST LORA_RST
  #endif
#endif
#ifdef LORA_DIO0
  #ifndef lora_DIO0
    #define lora_DIO0 LORA_DIO0
  #endif
#endif
#ifdef LORA_DIO1
  #ifndef lora_DIO1
    #define lora_DIO1 LORA_DIO1
  #endif
#endif
#ifdef LORA_DIO2
  #ifndef lora_DIO2
    #define lora_DIO2 LORA_DIO2
  #endif
#endif
int spreading_factor = 12;
long bandwidth = 125000;
int coding_rate = 5;

//-------- Čas a telemetrie ------
long cas_new = 0;
long cas_old = 0;
long cas_reset = 0;
long cas_telemetry = 0;
long cas_dif = 0;
long rx_cnt = 0;
long rf_inet = 0;
long live = 1;
String live_s = "001";
long dx_dist = 0;
float temp_cpu = 0;
double latitude2 = 0;
double longitude2 = 0;
long buffer_age[BUFFER_SIZE]; // Array to track the age of each station

//-------- Konfigurace -----------
const int MAX_SETTINGS = 26; // Pro ap_password
String fileContent = "";
String nastaveni[MAX_SETTINGS];

//-------- Wi-Fi a NTP -----------
WiFiClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "0.cz.pool.ntp.org", 3600, 60000);
AsyncWebServer server(80);

// --- APRS-IS stav / poslední TX ---
unsigned long last_inet_tx_sec = 0;   // v sekundách (epoch v iGate; uptime v AP/DIGI)
String aprsStatusString() {
  if (digi_AP == 1)      return "Off (AP mode)";
  if (digi_mode == 1)    return "Off (DIGI only)";
  // iGate:
  return client.connected() ? "Connected" : "Disconnected";
}
String aprsServerString() {
  if (digi_AP == 1 || digi_mode == 1) return "-";
  return String(servername) + ":" + String(aprs_port);
}
String inetTxString() {
  // zobrazení „kdy naposledy šel rámec do Internetu“ (jen iGate)
  if (digi_AP == 1 || digi_mode == 1) return "Off in this mode";
  if (last_inet_tx_sec == 0) return "no traffic yet";

  unsigned long nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
  unsigned long diff = (nowSec >= last_inet_tx_sec) ? (nowSec - last_inet_tx_sec) : 0;

  if (diff < 2)      return "just now";
  if (diff < 60)     return String(diff) + " s ago";
  if (diff < 3600)   return String(diff/60) + " min ago";
  return String(diff/3600) + " h ago";
  
}

///-------------------aprs symboly base64
bool iconSet = false;
String digi = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAAvklEQVQ4ja2UsQ6DMBBDDbrBQ4d8asd+AmM/laGDhwwMUSm9hAsCLLbED8enZLDRcE0GQFmn/TSOFyMA6CA4kROlKGaEWJ0kTyLSO61ZgiANhCRJnP7+XHBN0GCjlYk4T1d6CQCNP0RNmZ+zS+H8qIe6LhR/eqTyOdx2G+ou3LKylFUozQ0NRFd1ox7h6qCRxvmzWwq2E6n9B+u8e6jBidAqEvFN9aNp+YvCa/a1Bf4O4qB2uzioe16t4frzuwDPVmVTXnQbfgAAAABJRU5ErkJggg==";

String wx = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAA0klEQVQ4ja2UMRLEIAhFfxwKihS5/yktUlCk2AKXIGrirEuFCN9HlGyUCGtGAOSSn+uZOC0iFIqBfIzINS3RFvt4KxQbGdU/JKTn7RmVapVPAHLs7JY49tuxHIA7FExxDxDneJ89SEDnWsWWDCCf0suJEi0F1y3wsaPOiTeiJ5TmbRl2g3XuIJ8CsMMuEYVSX7m6FOW0hpb1pr7xiuWWkEt7Lid433DM9290o0Q2qZNPC+6Zx0kdDdKoXi1+i1eVNqGDrklLwz6JY/aHv9a2/vv9AESJS86PXo3VAAAAAElFTkSuQmCC";

String car = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAA6klEQVQ4jd1UOwrDMAx9EW/wmGP4ABk8htKDe/RggsYcoYMpGTOE0sGQuHFKS7PVPIyR9XmSJTcU4twigHmZf7Y3NHKSAoA/d6GqhqaEqr71QeHyWErEIVKY7invGbVaBoV7FqrqOndLt0NqXyXiOhdiqPVCDK5znxO5Xq4Ufok1ka27VdV7PxbeLTDWMQELhBimaWrbFkBDYW5wQzNWqocu8tVqdfCoFrAAKvtVvi/FepqX2dAA6PsegPV+LIjYQo7XydxYqGpWKsMeBt9Pdq5tuqeyryiMQ6z7rZZTuJXzt/VP/0Vz/vt9Agg9sUUoI+f4AAAAAElFTkSuQmCC";

String chodec = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAAv0lEQVQ4jdVUOw6DMAw11hvecTp04DioYuBIDAw5TgeGHieDhTpYQghcPs2E5SV2bL/Yz6mgkDKBiGTLf8cT1EIIInKXFAQJlqLom8eOd2+ixOdMgRhFauvU1jOELvUr4wGK7b2+6VJ7EYWINMP7pFGgsMlWOryeNhl0hI5Q+DFUKOIU7lvqryxQXBhq2MugF04kgh6WLR8uYcgL71nnwQTdki0mWAXFsg7BbdmZ4KFrjSKEvf+Wuyz7oVTl3+8XnnyPapycuG4AAAAASUVORK5CYII=";

String sanita = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAA3ElEQVQ4jd1UMRLDIAxTOA8aGHhGn8nA9X0d8wyGDB64Xgda4gTSoenQqy8DUWwhW5BJnOBcCAAt+nE9he6kBAB/Q7HaQeH71KOpbxzNS26EgNp18IHCIcumEZK2fvZh9qFxxRSHSidxUrkpbCpI3kz2ZcnBhyqzLlpfFO6OZt2fqnvBWlRVAeZFa1/jRl6ate7cJBhSNTN6xv6CWHozXT3AOxU2z74e4cNGACCmaLNtgf20hjgp99IecSJO0jX1eAV7fDW1RTW/P0VDvDcVw+L3+G/c1C9QTOd/vw+f+Xxz2dKM7wAAAABJRU5ErkJggg==";

String red_dot = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAAfElEQVQ4je2UPQ6AMAhGX5tv4Dg9kkOP5eCROE4HBxfTOFj/mhgHvwkCvBACBEXRJwFlLo/rTRY7WwB+xHuIbC5zGdkeIbL5tJo+cUAJitpdLZelagOQ9tJOVssPYhs1EWlYKb5x7yEYSy1LA4zNO2rO4qL+M/sgIvS/3wXFgxtzEcFPQAAAAABJRU5ErkJggg==";

String kolo = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAAq0lEQVQ4je1UMQrDMAyUxQ0aOuT/L/JgOusZHkzxIEIHgXBCY2hDC4EKY7BPdzqB7AQGnQsQUbf+MV8gfNICEf0lfi8hkLclBDLSuvWJymw058VfSwTHy9ZWiWi5LX4c0ZHFIz+X7LTIdn7k1FZzyXt3YNhqthoY7dFi91XuxRN2qFP8ftOIqtZWVTXcCiRcjOgYCYxIOup2ggpk42L+6o/QCw341yXS+e/3CUchZQSPlRXZAAAAAElFTkSuQmCC";

String igate = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAYAAADEtGw7AAAAoElEQVRIib3VSw4EIQgE0ILUgvufthemmdUkxPaDPTqsqReDEoVKx4EiAFzl2ooaDbpVDJWGjQaj7YUjmMWncAvK4EM4AuUuKHdJ4124Dro73H3YM4VXLqnX+4BX0FFGZw1v8f8syC+rXWcfJ36DtzLNUazgvd7ujOuAiEBE0gcYXl4MUgkqU+gU7gGZUaWeW4Sy8+e8ZQ381rEFkVOf6Qe4gj5VFW9A0wAAAABJRU5ErkJggg==";

String air = "iVBORw0KGgoAAAANSUhEUgAAABcAAAAXCAIAAABvSEP3AAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAAFiUAABYlAUlSJPAAAABxSURBVDhP7c7RCoAgDIXhCB9demppKQdkHIeLFkJQfF00tp+2tKcgEWmVcpbHFlbykSsaan/F5lRwr9EC2BW6NOl9rtCqa0kFn3o4TvqQK6axQr5VQWIecio6AbQAswrdd7RWOf9y09uVoFapb/QRuQCUYKj+Z5tT/AAAAABJRU5ErkJggg==";

String lgate = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAYAAADEtGw7AAAAn0lEQVRIib2V0Q6FIAxDK85k//+1JmjufTIhuI2i4J7bQxlbWCTJDxNKAGA/9qFQFUUaSiyKBqsoVHQsuASy8CbYAjHwEKyiyGdGPnM33AUzqSKNCe55JE97A/dAI09qCZ7Cv1mQN6tde2+Jn8Atj3jCumf1LG/rFgZxe8wkjzRm4tJ4Jes9uDkVFoC5DTVuJYh93LAVHpypaQuyzPpM/6BgO2Ce0xXHAAAAAElFTkSuQmCC";

String nahrada = "iVBORw0KGgoAAAANSUhEUgAAABYAAAAWCAIAAABL1vtsAAAAfElEQVQ4je2UPQ6AMAhGX5tv4Dg9kkOP5eCROE4HBxfTOFj/mhgHvwkCvBACBEXRJwFlLo/rTRY7WwB+xHuIbC5zGdkeIbL5tJo+cUAJitpdLZelagOQ9tJOVssPYhs1EWlYKb5x7yEYSy1LA4zNO2rO4qL+M/sgIvS/3wXFgxtzEcFPQAAAAABJRU5ErkJggg==";

//-------- HTML for Main Page --------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<meta charset="windows-1250">
<title>LoRa RX iGate</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
h2 {font-size: 1.8rem; color: white; text-align: center; margin: 0; padding: 15px 0; line-height: 1.2;}
h4 {font-size: 1rem;}
h3 {font-size: 1rem;}
.topnav {overflow: visible; background-color: #1b78e2; position: relative; padding: 15px 0; min-height: 60px;}
.card {background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); text-align: center;}
.cards {max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));}
.button-container {position: absolute; top: 50%; right: 20px; transform: translateY(-50%); display: flex; gap: 10px;}
.setup-button, .ota-button, .map-button, .messages-button {padding: 12px 24px; background-color: #1b78e2; color: white; border: 2px solid white; border-radius: 10px; cursor: pointer; font-weight: bold; z-index: 1000; font-size: 1rem; box-shadow: 0 2px 4px rgba(0,0,0,0.3); text-decoration: none;}
.setup-button:hover, .ota-button:hover, .map-button:hover, .messages-button:hover {background-color: #145ca1;}
</style>
</head><body>
<div class="topnav">
<h2>LoRa RX iGate - %CALL%</h2>
<div class="button-container">
  <a href="/nastaveni"><button class="setup-button">Setup</button></a>
  <a href="/update"><button class="ota-button">OTA</button></a>
  <a href="/zpravy"><button class="messages-button">Messages</button></a>
  %OBJECTS_BUTTON%
  %MAP_BUTTON%
  %TRACKER_BUTTON%
  %SERIAL_BUTTON%
</div>
</div>
</br>
<div class="card"><h3>
<center>
<table border="0">
    <tr>
        <td colspan="2">%DATUM%</td>
    </tr>
    <tr>
        <td>Location:</td>
        <td>%lon% %lat%</td>
    </tr>
    <tr>
        <td>Altitude:</td>
        <td>%vys% m a.s.l.</td>
    </tr>
</table>
</center>
</h3></div>
<div class="card" id="pak1">
  <h4><img src="data:image/jpeg;base64,%icona64_0%" alt="Base64 image"> %PAK1% - %vz0% km - %az0% &deg;</h4>
  <button onclick="toggleTable()">Show Data</button>
  <table id="myTable" style="display: none;">
     <tr>
    <td>Time:</td>
    <td>%T0%</td>
    <td>RSSI:</td>
    <td>%RS0%</td>
    <td>S/N:</td>
    <td>%SN0%</td>
  </tr>
  <tr>
     <td colspan="6">%PA0%</td>
  </tr>
  </table>
</div>
<script>
  var tableVisible = false;
  function toggleTable() {
    if (tableVisible) {
      document.getElementById("myTable").style.display = "none";
    } else {
      document.getElementById("myTable").style.display = "table";
    }
    tableVisible = !tableVisible;
  }
</script>
<div class="card" id="pak2">
  <h4><img src="data:image/jpeg;base64,%icona64_1%" alt="Base64 image"> %PAK2% - %vz1% km - %az1% &deg;</h4>
  <button onclick="toggleTable1()">Show Data</button>
  <table id="myTable1" style="display: none;">
     <tr>
    <td>Time:</td>
    <td>%T1%</td>
    <td>RSSI:</td>
    <td>%RS1%</td>
    <td>S/N:</td>
    <td>%SN1%</td>
  </tr>
  <tr>
     <td colspan="6">%PA1%</td>
  </tr>
  </table>
</div>
<script>
  var tableVisible = false;
  function toggleTable1() {
    if (tableVisible) {
      document.getElementById("myTable1").style.display = "none";
    } else {
      document.getElementById("myTable1").style.display = "table";
    }
    tableVisible = !tableVisible;
  }
</script>
<div class="card" id="pak3">
  <h4><img src="data:image/jpeg;base64,%icona64_2%" alt="Base64 image"> %PAK3% - %vz2% km - %az2% &deg;</h4>
  <button onclick="toggleTable2()">Show Data</button>
  <table id="myTable2" style="display: none;">
     <tr>
    <td>Time:</td>
    <td>%T2%</td>
    <td>RSSI:</td>
    <td>%RS2%</td>
    <td>S/N:</td>
    <td>%SN2%</td>
  </tr>
  <tr>
     <td colspan="6">%PA2%</td>
  </tr>
  </table>
</div>
<script>
  var tableVisible = false;
  function toggleTable2() {
    if (tableVisible) {
      document.getElementById("myTable2").style.display = "none";
    } else {
      document.getElementById("myTable2").style.display = "table";
    }
    tableVisible = !tableVisible;
  }
</script>
<div class="card" id="pak4">
  <h4><img src="data:image/jpeg;base64,%icona64_3%" alt="Base64 image"> %PAK4% - %vz3% km - %az3% &deg;</h4>
  <button onclick="toggleTable3()">Show Data</button>
  <table id="myTable3" style="display: none;">
     <tr>
    <td>Time:</td>
    <td>%T3%</td>
    <td>RSSI:</td>
    <td>%RS3%</td>
    <td>S/N:</td>
    <td>%SN3%</td>
  </tr>
  <tr>
     <td colspan="6">%PA3%</td>
  </tr>
  </table>
</div>
<script>
  var tableVisible = false;
  function toggleTable3() {
    if (tableVisible) {
      document.getElementById("myTable3").style.display = "none";
    } else {
      document.getElementById("myTable3").style.display = "table";
    }
    tableVisible = !tableVisible;
  }
</script>
<div class="card" id="pak5">
  <h4><img src="data:image/jpeg;base64,%icona64_4%" alt="Base64 image"> %PAK5% - %vz4% km - %az4% &deg;</h4>
  <button onclick="toggleTable4()">Show Data</button>
  <table id="myTable4" style="display: none;">
     <tr>
    <td>Time:</td>
    <td>%T4%</td>
    <td>RSSI:</td>
    <td>%RS4%</td>
    <td>S/N:</td>
    <td>%SN4%</td>
  </tr>
  <tr>
     <td colspan="6">%PA4%</td>
  </tr>
  </table>
</div>
<script>
  var tableVisible = false;
  function toggleTable4() {
    if (tableVisible) {
      document.getElementById("myTable4").style.display = "none";
    } else {
      document.getElementById("myTable4").style.display = "table";
    }
    tableVisible = !tableVisible;
  }
</script>
</br>
<p>Connection:</p>
<p>RSSI: %RSSI% dBm</p>
<p>IP: %IP_adr%</p>
<p>APRS-IS: %APRS_SERVER%</p>
<p>Status: %APRS_STATUS%</p>
<p>Inet TX: %INET_TX%</p>
<p>Time Diff: %cas_dif%</p>
<p>CPU Temperature: %URL1%</p>
</body></html>
)rawliteral";

//-------- HTML for Settings Page --------
const char nastaveni_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<meta charset="windows-1250">
<title>LoRa RX iGate - Settings</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
h2 {font-size: 1.8rem; color: white; text-align: center;}
h3 {font-size: 1.2rem; text-align: center;}
.topnav {overflow: hidden; background-color: #1b78e2;}
.card {background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); padding: 20px; margin: 20px auto; max-width: 700px;}
label {display: inline-block; width: 200px; font-weight: bold;}
input, select {width: 300px; padding: 5px; margin: 5px 0;}
button {padding: 10px 20px; background-color: #1b78e2; color: white; border: none; cursor: pointer;}
button:hover {background-color: #145ca1;}
</style>
</head><body>
<div class="topnav">
<h2>LoRa RX iGate - Settings</h2>
</div>
<div class="card">
<h3>Edit Configuration</h3>
<form action="/nastaveni" method="POST">
  <label>WiFi SSID:</label><input type="text" name="ssid" value="%SSID%"><br>
  <label>WiFi Password:</label><input type="text" name="password" value="%PASSWORD%"><br>
  <label>Callsign:</label><input type="text" name="call" value="%CALL%"><br>
  <label>Longitude:</label><input type="text" name="lon" value="%LON%"><br>
  <label>Latitude:</label><input type="text" name="lat" value="%LAT%"><br>
  <label>Symbol:</label><input type="text" name="sym" value="%SYM%"><br>
  <label>Comment:</label><input type="text" name="tool" value="%TOOL%"><br>
  <label>Altitude (m):</label><input type="text" name="Alt" value="%ALT%"><br>
  <label>APRS Filter:</label><input type="text" name="aprs_filter" value="%APRS_FILTER%"><br>
  <label>APRS Server:</label><input type="text" name="servername" value="%SERVERNAME%"><br>
  <label>APRS Port:</label><input type="text" name="aprs_port" value="%APRS_PORT%"><br>
  <label>APRS Password:</label><input type="text" name="password_aprs" value="%PASSWORD_APRS%"><br>
  <label>Use Static IP:</label>
  <select name="pouzitPevnouIP">
    <option value="true" %POUZIT_PEVNOU_IP_TRUE%>true</option>
    <option value="false" %POUZIT_PEVNOU_IP_FALSE%>false</option>
  </select><br>
  <label>Static IP:</label><input type="text" name="local_IP" value="%LOCAL_IP%"><br>
  <label>Gateway:</label><input type="text" name="gateway" value="%GATEWAY%"><br>
  <label>Subnet Mask:</label><input type="text" name="subnet" value="%SUBNET%"><br>
  <label>Primary DNS:</label><input type="text" name="primaryDNS" value="%PRIMARY_DNS%"><br>
  <label>Secondary DNS:</label><input type="text" name="secondaryDNS" value="%SECONDARY_DNS%"><br>
  <label>Digi Mode:</label>
  <select name="digi">
    <option value="0" %DIGI_0%>0 (iGate)</option>
    <option value="1" %DIGI_1%>1 (Digi)</option>
  </select><br>
  <label>AP Mode:</label>
  <select name="digi_AP">
    <option value="0" %DIGI_AP_0%>0 (Off)</option>
    <option value="1" %DIGI_AP_1%>1 (On)</option>
  </select><br>
  <label>AP Password:</label><input type="text" name="ap_password" value="%AP_PASSWORD%"><br>
  <label>LoRa Spreading Factor:</label>
    <select name='spreading_factor'>
      <option value='6' %SPREADING_FACTOR_6%>6</option>
      <option value='7' %SPREADING_FACTOR_7%>7</option>
      <option value='8' %SPREADING_FACTOR_8%>8</option>
      <option value='9' %SPREADING_FACTOR_9%>9</option>
      <option value='10' %SPREADING_FACTOR_10%>10</option>
      <option value='11' %SPREADING_FACTOR_11%>11</option>
      <option value='12' %SPREADING_FACTOR_12%>12</option>
    </select>
    <label>LoRa Bandwidth (Hz):</label>
    <select name='bandwidth'>
      <option value='7800' %BANDWIDTH_7800%>7800</option>
      <option value='10400' %BANDWIDTH_10400%>10400</option>
      <option value='15600' %BANDWIDTH_15600%>15600</option>
      <option value='20800' %BANDWIDTH_20800%>20800</option>
      <option value='31250' %BANDWIDTH_31250%>31250</option>
      <option value='41700' %BANDWIDTH_41700%>41700</option>
      <option value='62500' %BANDWIDTH_62500%>62500</option>
      <option value='125000' %BANDWIDTH_125000%>125000</option>
      <option value='250000' %BANDWIDTH_250000%>250000</option>
      <option value='500000' %BANDWIDTH_500000%>500000</option>
    </select>
    <label>LoRa Coding Rate:</label>
    <select name='coding_rate'>
      <option value='5' %CODING_RATE_5%>5</option>
      <option value='6' %CODING_RATE_6%>6</option>
      <option value='7' %CODING_RATE_7%>7</option>
      <option value='8' %CODING_RATE_8%>8</option>
    </select>
    <label>Web Username:</label><input type='text' name='web_username' value='%WEB_USERNAME%'>
    <label>Web Password:</label><input type='password' name='web_password' value='%WEB_PASSWORD%'>
  <button type="submit">Save and Restart</button>
</form>
</div>
</body></html>
)rawliteral";

//-------- HTML for Map Page --------
const char map_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html lang="cs">
<head>
  <meta charset="UTF-8">
  <title>LoRa RX iGate - Mapa</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <script>
    var latitude = %LATITUDE%;
    var longitude = %LONGITUDE%;
    var callsign = "%CALLSIGN%";
    var stations = [
      {
        lat: %LATITUDE0%,
        lon: %LONGITUDE0%,
        callsign: "%CALLSIGN0%",
        rssi: "%RSSI0%",
        snr: "%SNR0%",
        distance: "%DISTANCE0%",
        azimuth: "%AZIMUTH0%"
      },
      {
        lat: %LATITUDE1%,
        lon: %LONGITUDE1%,
        callsign: "%CALLSIGN1%",
        rssi: "%RSSI1%",
        snr: "%SNR1%",
        distance: "%DISTANCE1%",
        azimuth: "%AZIMUTH1%"
      },
      {
        lat: %LATITUDE2%,
        lon: %LONGITUDE2%,
        callsign: "%CALLSIGN2%",
        rssi: "%RSSI2%",
        snr: "%SNR2%",
        distance: "%DISTANCE2%",
        azimuth: "%AZIMUTH2%"
      },
      {
        lat: %LATITUDE3%,
        lon: %LONGITUDE3%,
        callsign: "%CALLSIGN3%",
        rssi: "%RSSI3%",
        snr: "%SNR3%",
        distance: "%DISTANCE3%",
        azimuth: "%AZIMUTH3%"
      },
      {
        lat: %LATITUDE4%,
        lon: %LONGITUDE4%,
        callsign: "%CALLSIGN4%",
        rssi: "%RSSI4%",
        snr: "%SNR4%",
        distance: "%DISTANCE4%",
        azimuth: "%AZIMUTH4%"
      }
    ];
  </script>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin=""/>
  <style>
    body {
      margin: 0;
      font-family: Arial, sans-serif;
      background-color: #f0f0f0;
    }
    h2 {
      font-size: 1.8rem;
      color: white;
      text-align: center;
      margin: 0;
      padding: 15px 0;
    }
    .topnav {
      overflow: visible;
      background-color: #1b78e2;
      position: relative;
      padding: 15px 0;
      min-height: 60px;
    }
    .button-container {
      position: absolute;
      top: 50%;
      right: 20px;
      transform: translateY(-50%);
      display: flex;
      gap: 10px;
    }
    .back-button {
      padding: 12px 24px;
      background-color: #1b78e2;
      color: white;
      border: 2px solid white;
      border-radius: 10px;
      cursor: pointer;
      font-weight: bold;
      z-index: 1000;
      font-size: 1rem;
      box-shadow: 0 2px 4px rgba(0,0,0,0.3);
      text-decoration: none;
    }
    .back-button:hover {
      background-color: #145ca1;
    }
  </style>
</head>
<body>
  <div class="topnav">
    <h2>LoRa RX iGate - Mapa Stanic</h2>
    <div class="button-container">
      <a href="/"><button class="back-button">Back ... </button></a>
    </div>
  </div>
  <div id="map" style="height: calc(100vh - 90px); width: 100%; max-width: 1200px; margin: 20px auto;"></div>
  <div style="
  position:absolute; right:12px; bottom:12px;
  background:#fff; padding:8px 10px; border-radius:8px;
  box-shadow:0 2px 6px rgba(0,0,0,.2); font-size:.9rem; line-height:1.4;">
  <b>RSSI (barva linky)</b><br>
  <span style="color:#2ecc71;">●</span> ≥ −60 dBm&nbsp;&nbsp;
  <span style="color:#f1c40f;">●</span> −61 až −80 dBm<br>
  <span style="color:#e67e22;">●</span> −81 až −95 dBm&nbsp;&nbsp;
  <span style="color:#e74c3c;">●</span> &lt; −95 dBm
</div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
  <script>
    var map = L.map('map').setView([latitude, longitude], 10);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    var redIcon = L.icon({
      iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
      iconSize: [25, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
      shadowSize: [41, 41]
    });
    var marker = L.marker([latitude, longitude], {icon: redIcon}).addTo(map);
    marker.bindPopup(
      "<b>" + callsign + "</b><br>" +
      "RSSI: -90 dBm<br>" +
      "SNR: 10 dB<br>" +
      "Vzdálenost: 0 km<br>" +
      "Azimut: 0&deg;<br>" +
      "<a href='https://aprs.fi/info/a/" + callsign + "' class='aprs-link' target='_blank'>View on aprs.fi</a>"
    );

    var blueIcon = L.icon({
      iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png',
      iconSize: [25, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
      shadowSize: [41, 41]
    });
 
    function rssiColor(rssiStr){
    var r = parseFloat(rssiStr);
    if (isNaN(r)) return '#888';     // fallback
    if (r >= -60) return '#2ecc71';  // silný signál
    if (r >= -80) return '#f1c40f';  // střední
    if (r >= -95) return '#e67e22';  // slabší
    return '#e74c3c';                // velmi slabý
  }

    stations.forEach(function(station) {
      if (station.lat !== 0 && station.lon !== 0 && station.callsign !== "") {
        var stationMarker = L.marker([station.lat, station.lon], {icon: blueIcon}).addTo(map);
        stationMarker.bindPopup(
          "<b>" + station.callsign + "</b><br>" +
          "RSSI: " + station.rssi + " dBm<br>" +
          "SNR: " + station.snr + " dB<br>" +
          "Vzdálenost: " + station.distance + " km<br>" +
          "Azimut: " + station.azimuth + "&deg;<br>"+
          "<a href='https://aprs.fi/info/a/" + station.callsign + "' class='aprs-link' target='_blank'>View on aprs.fi</a>"
        );
                // Add dashed line between iGate and station
 var col = rssiColor(station.rssi);
    L.polyline(
      [[latitude, longitude], [station.lat, station.lon]],
      { color: col, weight: 3, opacity: 0.85, dashArray: "4,6" }
    ).addTo(map);
      }
    });
  </script>
</body>
</html>
)rawliteral";
//-----------web messege
const char zpravy_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<meta charset="windows-1250">
<title>LoRa RX iGate - Messages</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { margin: 0; font-family: Arial, sans-serif; background-color: #f0f0f0; }
h2 { font-size: 1.8rem; color: white; text-align: center; margin: 0; padding: 15px 0; }
.topnav { overflow: visible; background-color: #1b78e2; position: relative; padding: 15px 0; min-height: 60px; }
.button-container { position: absolute; top: 50%; right: 20px; transform: translateY(-50%); display: flex; gap: 10px; }
.back-button { padding: 12px 24px; background-color: #1b78e2; color: white; border: 2px solid white; border-radius: 10px; cursor: pointer; font-weight: bold; z-index: 1000; font-size: 1rem; box-shadow: 0 2px 4px rgba(0,0,0,0.3); text-decoration: none; }
.back-button:hover { background-color: #145ca1; }
.card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); padding: 20px; margin: 20px auto; max-width: 700px; }
label { display: inline-block; width: 150px; font-weight: bold; }
input, textarea { width: 300px; padding: 5px; margin: 5px 0; }
button { padding: 10px 20px; background-color: #1b78e2; color: white; border: none; cursor: pointer; }
button:hover { background-color: #145ca1; }
button:disabled { background-color: #cccccc; cursor: not-allowed; }
.message-list { margin-top: 20px; }
.message-item { border-bottom: 1px solid #ddd; padding: 10px 0; }
.message-item:last-child { border-bottom: none; }
.error { color: red; font-weight: bold; }
.message-sender {
  font-weight: bold;
  color: #1b78e2;
  cursor: pointer;
  text-decoration: none;
  background-color: #e6f0fa; /* Světle modré pozadí */
  padding: 2px 6px;
  border-radius: 4px;
  display: inline-block;
}
.message-sender:hover {
  background-color: #d1e3f6; /* Tmavší pozadí při přejetí */
  color: #145ca1;
}
</style>
</head><body>
<div class="topnav">
<h2>LoRa RX iGate - Messages</h2>
<div class="button-container">
  <a href="/"><button class="back-button">Back</button></a>
</div>
</div>
<div class="card">
<h3>Send APRS Message</h3>
<form action="/zpravy" method="POST">
  <label>Recipient Callsign:</label><input type="text" id="recipient" name="recipient" oninput="this.value = this.value.toUpperCase()"><br>
  <label>Message:</label><textarea name="message" rows="4" placeholder="Enter your message"></textarea><br>
  <button type="submit" %SEND_BUTTON_STATE%>Send Message</button>
</form>
<p class="error">%ERROR_MESSAGE%</p>
</div>
<div class="card">
<h3>Received Messages</h3>
<div class="message-list">
  %MESSAGES%
</div>
</div>
<script>
function fillRecipient(callsign) {
  document.getElementById('recipient').value = callsign.toUpperCase();
}
</script>
</body></html>
)rawliteral";
//----------pozice pro mapu---------
double convertToDecimalDegreesLat(const String& lat) {
  if (lat.length() < 6) return 50.0; // Výchozí hodnota při neplatném vstupu

  // Extrakce stupňů (první 2 znaky) a minut (zbytek až před 'N'/'S')
  String degStr = lat.substring(0, 2);
  String minStr = lat.substring(2, lat.length() - 1);
  char direction = lat[lat.length() - 1];

  // Konverze na číselné hodnoty
  double degrees = degStr.toInt();
  double minutes = minStr.toFloat();
  
  // Kontrola platnosti
  if (degrees < 0 || degrees > 90 || minutes < 0 || minutes >= 60 || (direction != 'N' && direction != 'S')) {
    Serial.println("Neplatná zeměpisná šířka: " + lat + ", použita výchozí: 50.0");
    return 50.0;
  }

  // Konverze na desetinné stupně
  double decimal = degrees + (minutes / 60.0);
  if (direction == 'S') {
    decimal = -decimal;
  }

  return decimal;
}

double convertToDecimalDegreesLon(const String& lon) {
  if (lon.length() < 7) return 14.0; // Výchozí hodnota při neplatném vstupu

  // Extrakce stupňů (první 3 znaky) a minut (zbytek až před 'E'/'W')
  String degStr = lon.substring(0, 3);
  String minStr = lon.substring(3, lon.length() - 1);
  char direction = lon[lon.length() - 1];

  // Konverze na číselné hodnoty
  double degrees = degStr.toInt();
  double minutes = minStr.toFloat();
  
  // Kontrola platnosti
  if (degrees < 0 || degrees > 180 || minutes < 0 || minutes >= 60 || (direction != 'E' && direction != 'W')) {
    Serial.println("Neplatná zeměpisná délka: " + lon + ", použita výchozí: 14.0");
    return 14.0;
  }

  // Konverze na desetinné stupně
  double decimal = degrees + (minutes / 60.0);
  if (direction == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

//-------- Funkce pro webový server --------
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String procesor(const String& var) {
  // Existing placeholders for index_html and nastaveni_html
  if (var == "lat") return lat;
  if (var == "lon") return lon;
  if (var == "vys") return String(Alt);
  if (var == "CALL") return call;
  if (var == "RSSI") return String(WiFi.RSSI());
  if (var == "IP_adr") return IP;
  if (var == "APRS_SERVER") return aprsServerString();
  if (var == "APRS_STATUS") return aprsStatusString();
  if (var == "INET_TX")     return inetTxString();
  if (var == "PAK1") return buffer[0];
  if (var == "T0") return buffer_cas[0];
  if (var == "RS0") return buffer_RSSI[0];
  if (var == "SN0") return buffer_SN[0];
  if (var == "PA0") return buffer_pak[0];
  if (var == "PAK2") return buffer[1];
  if (var == "T1") return buffer_cas[1];
  if (var == "RS1") return buffer_RSSI[1];
  if (var == "SN1") return buffer_SN[1];
  if (var == "PA1") return buffer_pak[1];
  if (var == "PAK3") return buffer[2];
  if (var == "T2") return buffer_cas[2];
  if (var == "RS2") return buffer_RSSI[2];
  if (var == "SN2") return buffer_SN[2];
  if (var == "PA2") return buffer_pak[2];
  if (var == "PAK4") return buffer[3];
  if (var == "T3") return buffer_cas[3];
  if (var == "RS3") return buffer_RSSI[3];
  if (var == "SN3") return buffer_SN[3];
  if (var == "PA3") return buffer_pak[3];
  if (var == "PAK5") return buffer[4];
  if (var == "T4") return buffer_cas[4];
  if (var == "RS4") return buffer_RSSI[4];
  if (var == "SN4") return buffer_SN[4];
  if (var == "PA4") return buffer_pak[4];
  if (var == "cas_dif") return String(cas_dif);
  if (var == "vz0") return buffer_vzdalenost[0];
  if (var == "vz1") return buffer_vzdalenost[1];
  if (var == "vz2") return buffer_vzdalenost[2];
  if (var == "vz3") return buffer_vzdalenost[3];
  if (var == "vz4") return buffer_vzdalenost[4];
  if (var == "az0") return buffer_azimut[0];
  if (var == "az1") return buffer_azimut[1];
  if (var == "az2") return buffer_azimut[2];
  if (var == "az3") return buffer_azimut[3];
  if (var == "az4") return buffer_azimut[4];
  if (var == "icona64_0") return buffer_icona[0];
  if (var == "icona64_1") return buffer_icona[1];
  if (var == "icona64_2") return buffer_icona[2];
  if (var == "icona64_3") return buffer_icona[3];
  if (var == "icona64_4") return buffer_icona[4];
  if (var == "URL1") return String(temp_cpu);
  if (var == "DATUM") return (digi_mode == 0 && digi_AP == 0) ? timeClient.getFormattedTime() : String(cas_new / 60) + "m";
  
  // Settings page placeholders
  if (var == "SSID") return ssid;
  if (var == "PASSWORD") return password;
  if (var == "CALL") return call;
  if (var == "LON") return lon;
  if (var == "LAT") return lat;
  if (var == "SYM") return sym;
  if (var == "TOOL") return tool;
  if (var == "ALT") return String(Alt);
  if (var == "APRS_FILTER") return aprs_filter;
  if (var == "SERVERNAME") return String(servername);
  if (var == "APRS_PORT") return String(aprs_port);
  if (var == "PASSWORD_APRS") return password_aprs;
  if (var == "POUZIT_PEVNOU_IP_TRUE") return pouzitPevnouIP ? "selected" : "";
  if (var == "POUZIT_PEVNOU_IP_FALSE") return !pouzitPevnouIP ? "selected" : "";
  if (var == "LOCAL_IP") return local_IP.toString();
  if (var == "GATEWAY") return gateway.toString();
  if (var == "SUBNET") return subnet.toString();
  if (var == "PRIMARY_DNS") return primaryDNS.toString();
  if (var == "SECONDARY_DNS") return secondaryDNS.toString();
  if (var == "DIGI_0") return (digi_mode == 0) ? "selected" : "";
  if (var == "DIGI_1") return (digi_mode == 1) ? "selected" : "";
  if (var == "DIGI_AP_0") return (digi_AP == 0) ? "selected" : "";
  if (var == "DIGI_AP_1") return (digi_AP == 1) ? "selected" : "";
  if (var == "AP_PASSWORD") return ap_password;
  // Spreading Factor
  if (var == "SPREADING_FACTOR_6") return spreading_factor == 6 ? "selected" : "";
  if (var == "SPREADING_FACTOR_7") return spreading_factor == 7 ? "selected" : "";
  if (var == "SPREADING_FACTOR_8") return spreading_factor == 8 ? "selected" : "";
  if (var == "SPREADING_FACTOR_9") return spreading_factor == 9 ? "selected" : "";
  if (var == "SPREADING_FACTOR_10") return spreading_factor == 10 ? "selected" : "";
  if (var == "SPREADING_FACTOR_11") return spreading_factor == 11 ? "selected" : "";
  if (var == "SPREADING_FACTOR_12") return spreading_factor == 12 ? "selected" : "";
  
  // Bandwidth
  if (var == "BANDWIDTH_7800") return bandwidth == 7800 ? "selected" : "";
  if (var == "BANDWIDTH_10400") return bandwidth == 10400 ? "selected" : "";
  if (var == "BANDWIDTH_15600") return bandwidth == 15600 ? "selected" : "";
  if (var == "BANDWIDTH_20800") return bandwidth == 20800 ? "selected" : "";
  if (var == "BANDWIDTH_31250") return bandwidth == 31250 ? "selected" : "";
  if (var == "BANDWIDTH_41700") return bandwidth == 41700 ? "selected" : "";
  if (var == "BANDWIDTH_62500") return bandwidth == 62500 ? "selected" : "";
  if (var == "BANDWIDTH_125000") return bandwidth == 125000 ? "selected" : "";
  if (var == "BANDWIDTH_250000") return bandwidth == 250000 ? "selected" : "";
  if (var == "BANDWIDTH_500000") return bandwidth == 500000 ? "selected" : "";
  
  // Coding Rate
  if (var == "CODING_RATE_5") return coding_rate == 5 ? "selected" : "";
  if (var == "CODING_RATE_6") return coding_rate == 6 ? "selected" : "";
  if (var == "CODING_RATE_7") return coding_rate == 7 ? "selected" : "";
  if (var == "CODING_RATE_8") return coding_rate == 8 ? "selected" : "";

  // web autentification
    if (var == "WEB_USERNAME") return web_username;
    if (var == "WEB_PASSWORD") return web_password;
  
  if (var == "MAP_BUTTON") {
    if (digi_mode == 0 && digi_AP == 0) {
      return "<a href=\"/map\"><button class=\"map-button\">Map</button></a>";
    }
    return "";
  }

  // Map page placeholders for iGate
  if (var == "LATITUDE") {
    String lat_val = String(convertToDecimalDegreesLat(lon), 6);
    Serial.println("LATITUDE: " + lat_val);
    return lat_val;
  }
  if (var == "LONGITUDE") {
    String lon_val = String(convertToDecimalDegreesLon(lat), 6);
    Serial.println("LONGITUDE: " + lon_val);
    return lon_val;
  }
  if (var == "CALLSIGN") {
    Serial.println("CALLSIGN: " + call);
    return call;
  }

  // Map page placeholders for stations
  if (var == "LATITUDE0") return String(buffer_lat[0], 6);
  if (var == "LONGITUDE0") return String(buffer_lon[0], 6);
  if (var == "CALLSIGN0") return String(buffer[0]);
  if (var == "RSSI0") return String(buffer_RSSI[0]);
  if (var == "SNR0") return String(buffer_SN[0]);
  if (var == "DISTANCE0") return String(buffer_vzdalenost[0]);
  if (var == "AZIMUTH0") return String(buffer_azimut[0]);
  

  if (var == "LATITUDE1") return String(buffer_lat[1], 6);
  if (var == "LONGITUDE1") return String(buffer_lon[1], 6);
  if (var == "CALLSIGN1") return String(buffer[1]);
  if (var == "RSSI1") return String(buffer_RSSI[1]);
  if (var == "SNR1") return String(buffer_SN[1]);
  if (var == "DISTANCE1") return String(buffer_vzdalenost[1]);
  if (var == "AZIMUTH1") return String(buffer_azimut[1]);
  

  if (var == "LATITUDE2") return String(buffer_lat[2], 6);
  if (var == "LONGITUDE2") return String(buffer_lon[2], 6);
  if (var == "CALLSIGN2") return String(buffer[2]);
  if (var == "RSSI2") return String(buffer_RSSI[2]);
  if (var == "SNR2") return String(buffer_SN[2]);
  if (var == "DISTANCE2") return String(buffer_vzdalenost[2]);
  if (var == "AZIMUTH2") return String(buffer_azimut[2]);
  

  if (var == "LATITUDE3") return String(buffer_lat[3], 6);
  if (var == "LONGITUDE3") return String(buffer_lon[3], 6);
  if (var == "CALLSIGN3") return String(buffer[3]);
  if (var == "RSSI3") return String(buffer_RSSI[3]);
  if (var == "SNR3") return String(buffer_SN[3]);
  if (var == "DISTANCE3") return String(buffer_vzdalenost[3]);
  if (var == "AZIMUTH3") return String(buffer_azimut[3]);
  

  if (var == "LATITUDE4") return String(buffer_lat[4], 6);
  if (var == "LONGITUDE4") return String(buffer_lon[4], 6);
  if (var == "CALLSIGN4") return String(buffer[4]);
  if (var == "RSSI4") return String(buffer_RSSI[4]);
  if (var == "SNR4") return String(buffer_SN[4]);
  if (var == "DISTANCE4") return String(buffer_vzdalenost[4]);
  if (var == "AZIMUTH4") return String(buffer_azimut[4]);

  //----object---------
  if (var == "OBJECTS_BUTTON") {
    return "<a href=\"/objects\"><button class=\"map-button\">Objects</button></a>";
  }

  //--------tracker----------
  if (var == "TRACKER_BUTTON") {
  // tlačítko zobrazíme vždy; v iGate jen pro zobrazení, v Digi/AP+Digi umožní TX
  return "<a href=\"/tracker\"><button class='setup-button'>Tracker</button></a>";
}
  //-------serial----
  if (var == "SERIAL_BUTTON") {
  return "<a href=\"/serial\"><button class=\"map-button\">Serial</button></a>";
}
   //---------------mesenger------------
  if (var == "SEND_BUTTON_STATE") {
  return (digi_AP == 1) ? "" : "disabled";
}
if (var == "ERROR_MESSAGE") {
  return "";
}
if (var == "MESSAGES") {
  String messages = "";
  if (message_cnt == 0) {
    messages = "<p>No messages received.</p>";
  } else {
    int start_idx = (message_cnt > MESSAGE_BUFFER_SIZE) ? (message_cnt - MESSAGE_BUFFER_SIZE) : 0;
    for (int i = start_idx; i < message_cnt; i++) {
      int idx = i % MESSAGE_BUFFER_SIZE;
      messages += "<div class='message-item'>";
      messages += "<div class='message-sender' onclick=\"fillRecipient('" + String(buffer_zpravy_odesilatel[idx]) + "')\">" + String(buffer_zpravy_odesilatel[idx]) + "</div>";
      messages += "<div class='message-time'>" + String(buffer_zpravy_cas[idx]) + "</div>";
      messages += "<div class='message-text'>" + String(buffer_zpravy[idx]) + "</div>";
      messages += "</div>";
    }
  }
  return messages;
}

  return String();
}
//----------webserial-----------
#define WLOGln(x) do { Serial.println(x); WebSerial.println(x); } while(0)
#define WLOGf(...) do { Serial.printf(__VA_ARGS__); WebSerial.printf(__VA_ARGS__); } while(0)
//-------- Funkce pro Wi-Fi a APRS --------
void wifi() {
  WiFi.setHostname("lora_RX_igate_");
  WiFi.hostname("lora_RX_igate");
  if (digi_AP == 1) {
    // Režim přístupového bodu (AP)
    String apSSID = call + "_AP";
    IPAddress apIP(192, 168, 4, 1);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(apSSID.c_str(), ap_password.c_str());
    IP = apIP.toString();
    WLOGln("WiFi AP started, SSID: " + apSSID + ", IP: " + IP);
  } else {
    // Režim klienta (připojení k Wi-Fi síti)
    if (pouzitPevnouIP) {
      if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        WLOGln("STA Failed to configure");
      }
    }
    WiFi.begin(ssid.c_str(), password.c_str());
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    IP = WiFi.localIP().toString();
    Serial.println("WiFi connected, IP: " + IP);
  }
}

///------------conect aprs-------------
void con_aprs() {
  WLOGln("\nStarting APRS connection...");
  WLOGln("Připojuji se k APRS serveru: " + String(servername) + ":" + String(aprs_port));
  if (client.connect(servername, aprs_port)) {
    Serial.println("Connected to APRS server: " + String(servername) + ":" + String(aprs_port));
    client.println("");
    client.println("user " + user + " pass " + password_aprs + " vers OK5TVR_RX_Igate_LoRA filter m/1");
    client.flush();
    client.println("");
    client.println(call + ">APZ023,TCPIP*:!" + lon + sym + lat + "&PHG01000/" + tool);
    client.flush();
    client.println(call + ">APZ023,TCPIP*::" + call + ":PARM.RxPkts,RF->Inet,DX_dist.,live,Temp_core");
    client.flush();
    client.println(call + ">APZ023,TCPIP*::" + call + ":UNIT.Pkts,Pkts,km,min.,C");
    client.flush();
    client.println(call + ">APZ023,TCPIP*::" + call + ":EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0");
    client.flush();
    client.println(call + ">APZ023,TCPIP*:>https://github.com/ok5tvr/LoRa_RX_Igate");
    client.flush();
    last_inet_tx_sec = timeClient.getEpochTime(); // spojení + hlavičky = „TX“
  } else {
    WLOGln("APRS connection failed to: " + String(servername) + ":" + String(aprs_port));
  }
}

//-------- Funkce pro Digi mód --------
bool uzJsemDigipeatoval(String paket, String digiCall) {
  if (paket.indexOf("*") != -1) {
    WLOGln("Paket již obsahuje zpracovaný prvek cesty (*), ignoruji.");
    return true;
  }
  return false;
}

String upravDigipeaterPath(String paket, String digiCall) {
  if (uzJsemDigipeatoval(paket, digiCall)) {
    return paket;
  }

  int startIndex = paket.indexOf(">");
  int endIndex = paket.indexOf(":");
  if (startIndex == -1 || endIndex == -1 || endIndex <= startIndex) {
    WLOGln("Chybný formát paketu, cesta se neupraví.");
    return paket;
  }

  String sourceCall = paket.substring(0, startIndex);
  String pathAndDest = paket.substring(startIndex + 1, endIndex);
  String data = paket.substring(endIndex);

  int firstComma = pathAndDest.indexOf(",");
  String destination;
  String path;
  if (firstComma == -1) {
    destination = pathAndDest;
    path = "";
  } else {
    destination = pathAndDest.substring(0, firstComma);
    path = pathAndDest.substring(firstComma + 1);
  }

  if (path.length() == 0) {
    WLOGln("Prázdná cesta, přidávám: " + digiCall + "*");
    String finalPath = destination + "," + digiCall + "*";
    WLOGln("Původní cesta: " + pathAndDest);
    WLOGln("Upravená cesta: " + finalPath);
    return sourceCall + ">" + finalPath + data;
  }

  String pathElements[8];
  int elementCount = 0;
  int lastComma = -1;
  for (int i = 0; i <= path.length(); i++) {
    if (i == path.length() || path.charAt(i) == ',') {
      if (lastComma + 1 < i) {
        pathElements[elementCount] = path.substring(lastComma + 1, i);
        elementCount++;
      }
      lastComma = i;
    }
    if (elementCount >= 8) break;
  }

  bool processed = false;
  String newPath = "";
  for (int i = 0; i < elementCount; i++) {
    String element = pathElements[i];
    if (element.indexOf("*") != -1) {
      newPath += (newPath.length() > 0 ? "," : "") + element;
      continue;
    }
    if (!processed) {
      if (element == "WIDE1-1" || element == digiCall) {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*";
        processed = true;
      } else if (element == "WIDE2-2") {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*,WIDE2-1";
        processed = true;
      } else if (element == "WIDE2-1") {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*";
        processed = true;
      } else if (element == "TRACE1-1") {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*";
        processed = true;
      } else if (element == "TRACE2-2") {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*,TRACE2-1";
        processed = true;
      } else if (element == "TRACE2-1") {
        newPath += (newPath.length() > 0 ? "," : "") + digiCall + "*";
        processed = true;
      } else if (element == "APLT00-1") {
        newPath += (newPath.length() > 0 ? "," : "") + element + "," + digiCall + "*";
        processed = true;
      } else {
        newPath += (newPath.length() > 0 ? "," : "") + element;
      }
    } else {
      newPath += (newPath.length() > 0 ? "," : "") + element;
    }
  }

  String finalPath = destination;
  if (newPath.length() > 0) {
    finalPath += "," + newPath;
  }

  if (processed) {
    WLOGln("Původní cesta: " + pathAndDest);
    WLOGln("Upravená cesta: " + finalPath);
    return sourceCall + ">" + finalPath + data;
  }
  WLOGln("Žádný alias k přeposlání, paket ignorován: " + pathAndDest);
  return paket;
}

void posliBeacon() {
  String beacon = call + ">APZ023:!" + lon + sym + lat + "&PHG01000 " + tool + " " + (digi_AP == 1 ? "AP" : (digi_mode ? "DIGI" : "iGate"));
  if (digi_mode == 1 || digi_AP == 1) {
    digitalWrite(PLED1, HIGH);
    LoRa.beginPacket();
    LoRa.print("<" + String((char)0xFF) + String((char)0x01) + beacon);
    LoRa.endPacket();
    digitalWrite(PLED1, LOW);
    WLOGln("BEACON vyslán přes RF: " + beacon);
  } else {
    if (client.connected()) {
      client.println(beacon);
      client.flush();
      WLOGln("BEACON poslán na APRS-IS: " + beacon);
       last_inet_tx_sec = timeClient.getEpochTime(); // spojení + hlavičky = „TX“
    } else {
      WLOGln("BEACON – není spojení k APRS-IS, reconnect...");
      if (WiFi.status() == WL_CONNECTED) {
        client.stop();
        con_aprs();
      }
    }
  }
}

//-------- Převod GPS na desetinné stupně --------
double convertToDecimalDegrees_la(const String& gpsString) {
  double degrees = gpsString.substring(0, 2).toDouble();
  double minutes = gpsString.substring(2).toDouble();
  return degrees + (minutes / 60.0);
}

double convertToDecimalDegrees_lo(const String& gpsString) {
  double degrees = gpsString.substring(1, 3).toDouble();
  double minutes = gpsString.substring(3).toDouble();
  return degrees + (minutes / 60.0);
}

//-------- Výpočty polohy --------
double toRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371.0;
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(toRadians(lat1)) * cos(toRadians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

double calculateAzimuth(double lat1, double lon1, double lat2, double lon2) {
  double dLon = toRadians(lon2 - lon1);
  double y = sin(dLon) * cos(toRadians(lat2));
  double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) - sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);
  double azimuth = atan2(y, x);
  azimuth = fmod((azimuth + 2 * M_PI), (2 * M_PI));
  return toDegrees(azimuth);
}

//-------- Ověření ASCII --------
bool isASCII(const String& str) {
  for (size_t i = 0; i < str.length(); i++) {
    if (str.charAt(i) > 127) {
      return false;
    }
  }
  return true;
}

//--------intex call-------------
int pickIndexFor(const String& call_d) {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (buffer[i][0] != '\0' && strcmp(buffer[i], call_d.c_str()) == 0) {
      return i; // existující stanice
    }
  }
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (buffer[i][0] == '\0') {
      return i; // volný slot
    }
  }
  int oldestIndex = 0;
  long oldestAge = buffer_age[0];
  for (int i = 1; i < BUFFER_SIZE; i++) {
    if (buffer_age[i] < oldestAge) {
      oldestAge = buffer_age[i];
      oldestIndex = i;
    }
  }
  return oldestIndex; // nejstarší záznam
}

//-------- Interní teplota --------
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

//--------object--------
// === Objects scheduling (5 slots) ===
struct AprsObjectCfg {
  bool    enabled = false;
  String  name    = "";
  String  latstr  = "";   // DDMM.mmN
  String  lonstr  = "";   // DDDMM.mmE
  char    tableCh = '/';  // '/' nebo '\'
  char    symCh   = 'L';  // 1 znak
  bool    killFlag = false;   // false = LIVE '*', true = KILL '_'
  uint32_t intervalMin = 60;  // perioda v minutách
  uint32_t nextDueSec  = 0;   // další termín (sekundy epoch/uptime)
  String comment = "";
};

AprsObjectCfg OBJ[5];
const char* OBJECTS_FILE = "/objects.cfg";

// --- Pomocné z minula (nech jen jednou v kódu) ---
String padObjectName9(const String& name) {
  String n = name;
  if (n.length() > 9) return n.substring(0, 9);
  while (n.length() < 9) n += " ";
  return n;
}
String aprsObjectTimestamp() {
  char buf[8] = {0};
  if (digi_mode == 0 && digi_AP == 0) {
    time_t t = timeClient.getEpochTime();
    struct tm *utc = gmtime(&t);
    snprintf(buf, sizeof(buf), "%02d%02d%02dz", utc->tm_mday, utc->tm_hour, utc->tm_min);
  } else {
    unsigned long s = millis() / 1000;
    int hh = (s % 86400) / 3600;
    int mm = (s % 3600) / 60;
    int ss = s % 60;
    snprintf(buf, sizeof(buf), "%02d%02d%02dz", hh, mm, ss);
  }
  return String(buf);
}
String buildAprsObject(const String& name9, bool killFlag,
                       const String& lat_ddmm, const String& lon_dddmm,
                       char tableChar, char symChar,
                       const String& comment) {
  String payload = ";";
  payload += name9;
  payload += (killFlag ? "_" : "*");
  payload += aprsObjectTimestamp();
  payload += lat_ddmm;
  payload += tableChar;
  payload += lon_dddmm;
  payload += symChar;
  if (comment.length() > 0) { payload += " "; payload += comment; }
  return payload;
}
void sendAprsObject(const String& payload) {
  String frame;
  if (digi_mode == 0 && digi_AP == 0) {              // iGate → APRS-IS
    if (!client.connected()) { client.stop(); con_aprs(); }
    frame = call + ">APZ023,TCPIP*:" + payload;
    client.println(frame); 
    client.flush();
    WLOGln("OBJECT → APRS-IS: " + frame);
    last_inet_tx_sec = timeClient.getEpochTime(); // spojení + hlavičky = „TX“
  } else {                                           // Digi / Digi+AP → RF
    frame = call + ">APZ023:" + payload;
    digitalWrite(PLED1, HIGH);
    LoRa.beginPacket();
    LoRa.print("<" + String((char)0xFF) + String((char)0x01) + frame);
    LoRa.endPacket();
    digitalWrite(PLED1, LOW);
    WLOGln("OBJECT → RF: <FF 01 " + frame + ">");
  }
}

// --- Uložení/načtení do SPIFFS (/objects.cfg) ---
// Formát: pro každý objekt 9 tagů: <EN><NAME><LAT><LON><TABLE><SYM><COMMENT><KILL><INTERVAL>
void saveObjectsConfig() {
  String s = "";
  for (int i = 0; i < 5; i++) {
    s += "<" + String(OBJ[i].enabled ? "1" : "0") + ">";
    s += "<" + OBJ[i].name + ">";
    s += "<" + OBJ[i].latstr + ">";
    s += "<" + OBJ[i].lonstr + ">";
    s += "<"; s += OBJ[i].tableCh; s += ">";
    s += "<"; s += OBJ[i].symCh;   s += ">";
    s += "<" + OBJ[i].comment + ">";
    s += "<" + String(OBJ[i].killFlag ? "1" : "0") + ">";
    s += "<" + String(OBJ[i].intervalMin) + ">";
  }
  File f = SPIFFS.open(OBJECTS_FILE, FILE_WRITE);
  if (!f) { WLOGln("saveObjectsConfig: open FAIL"); return; }
  f.print(s);
  f.close();
  Serial.println("Objects saved.");
}

String nextToken(String &src) {
  int a = src.indexOf('<');
  int b = src.indexOf('>');
  if (a == -1 || b == -1 || b <= a) return "";
  String t = src.substring(a+1, b);
  src.remove(0, b+1);
  return t;
}
void loadObjectsConfig() {
  if (!SPIFFS.exists(OBJECTS_FILE)) {
    WLOGln("Objects file not found, defaults used.");
    return;
  }
  File f = SPIFFS.open(OBJECTS_FILE, FILE_READ);
  if (!f) { WLOGln("loadObjectsConfig: open FAIL"); return; }
  String content = f.readString();
  f.close();
  for (int i = 0; i < 5; i++) {
    String en  = nextToken(content);
    String nm  = nextToken(content);
    String la  = nextToken(content);
    String lo  = nextToken(content);
    String tb  = nextToken(content);
    String sy  = nextToken(content);
    String cm  = nextToken(content);
    String ki  = nextToken(content);
    String in  = nextToken(content);
    if (in == "") break; // konec/poškozeno

    OBJ[i].enabled = (en == "1");
    OBJ[i].name    = nm;
    OBJ[i].latstr  = la.length() ? la : lon; // výchozí předvyplnění: tvoje current lon(lat)
    OBJ[i].lonstr  = lo.length() ? lo : lat; // a current lat(lon) – viz tvé prohozené názvy
    OBJ[i].tableCh = (tb.length() ? tb[0] : '/');
    OBJ[i].symCh   = (sy.length() ? sy[0] : 'L');
    OBJ[i].comment = cm;
    OBJ[i].killFlag = (ki == "1");
    OBJ[i].intervalMin = in.toInt(); if (OBJ[i].intervalMin == 0) OBJ[i].intervalMin = 60;
  }
  WLOGln("Objects loaded.");
}

// Pomocná: bezpečné HTML escapování pro value=""
String htmlEscape(const String& in) {
  String out = in;
  out.replace("&","&amp;"); out.replace("\"","&quot;"); out.replace("<","&lt;"); out.replace(">","&gt;");
  return out;
}

// Sestavení HTML stránky „Objects“ dynamicky (bez šablony)
String buildObjectsHtml() {
String h = R"(
<!DOCTYPE html><html lang="cs"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>LoRa RX iGate - Objects</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#f0f0f0}
h2{font-size:1.8rem;color:#fff;text-align:center;margin:0;padding:15px 0}
.topnav{background:#1b78e2;position:relative;min-height:60px}
.button-container{position:absolute;top:50%;right:20px;transform:translateY(-50%);display:flex;gap:10px}
.btn{padding:10px 18px;background:#1b78e2;color:#fff;border:2px solid #fff;border-radius:10px;cursor:pointer;font-weight:bold;box-shadow:0 2px 4px rgba(0,0,0,.3)}
.card{background:#fff;box-shadow:2px 2px 12px 1px rgba(140,140,140,.5);padding:16px;margin:16px 0;max-width:1500px;text-align:left;}
table{width:100%;border-collapse:collapse}
th,td{border-bottom:1px solid #eee;padding:6px;text-align:left;font-size:14px}
small{color:#555}
.act{white-space:nowrap}
mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,"Liberation Mono",monospace}
.compact{width:auto;padding:4px 6px}
.w1ch{width:2.6ch}   /* 1 znak (+padding) */
.w8ch{width:9.2ch}   /* 8 znaků (+.)    */
.w9ch{width:10.2ch}  /* 9 znaků         */
.w10ch{width:11.2ch} /* 10 znaků        */
.w9name{width:12ch}  /* Name 9 znaků    */
.wnum{width:7ch}     /* Interval        */
</style>
</head><body>
<div class="topnav"><h2>LoRa RX iGate - Objects</h2>
<div class="button-container">
<a href="/"><button class="btn">Back</button></a>
</div></div>
<div class="card">
<form action="/objects" method="POST">
<table>
<tr><th>#</th><th>Enable</th><th>NCall</th><th>Lat (DDMM.mmN)</th><th>Lon (DDDMM.mmE)</th><th>Table</th><th>Sym</th><th>Action</th><th>Comment</th><th>Interval (min)</th><th class="act">Now</th></tr>
)";
  for (int i=0;i<5;i++){
    h += "<tr>";
    h += "<td>"+String(i+1)+"</td>";
    h += "<td><input type='checkbox' name='o"+String(i)+"_en' "+String(OBJ[i].enabled?"checked":"")+"></td>";
    h += "<td><input class='compact mono w9name' type='text' maxlength='9' name='o"+String(i)+"_name' value='"+htmlEscape(OBJ[i].name)+"' oninput='this.value=this.value.toUpperCase()'></td>";
    h += "<td><input  class='compact mono w9ch' type='text' name='o"+String(i)+"_lat' value='"+htmlEscape(OBJ[i].latstr.length()?OBJ[i].latstr:lon)+"'></td>";
    h += "<td><input class='compact mono w10ch' type='text' name='o"+String(i)+"_lon' value='"+htmlEscape(OBJ[i].lonstr.length()?OBJ[i].lonstr:lat)+"'></td>";
    h += "<td><select name='o"+String(i)+"_tab'><option value='/'"+String(OBJ[i].tableCh=='/'?" selected":"")+">/</option><option value='\\'"+String(OBJ[i].tableCh=='\\'?" selected":"")+">\\</option></select></td>";
    h += "<td><input class='compact mono w1ch' type='text' maxlength='1' name='o"+String(i)+"_sym' value='"+String(OBJ[i].symCh)+"' oninput='this.value=this.value.toUpperCase()'></td>";
    h += "<td><select name='o"+String(i)+"_act'><option value='live'"+String(!OBJ[i].killFlag?" selected":"")+">LIVE *</option><option value='kill'"+String(OBJ[i].killFlag?" selected":"")+">KILL _</option></select></td>";
    h += "<td><input type='text' maxlength='60' name='o"+String(i)+"_cmt' value='"+htmlEscape(OBJ[i].comment)+"'></td>";
    h += "<td><input class='compact mono wnum' type='number' min='1' max='10080' name='o"+String(i)+"_int' value='"+String(OBJ[i].intervalMin)+"' style='width:90px'></td>";
    h += "<td class='act'><button type='submit' class='btn' name='send_now' value='"+String(i)+"'>Send now</button></td>";
    h += "</tr>";
  }
  h += R"(</table><br>
<small>Tip: Interval je perioda opakovaného odesílání. Po restartu se první odeslání provede ručně („Send now“) nebo až po uplynutí intervalu.</small><br><br>
<button type="submit" class="btn">Save</button>
</form></div></body></html>)";
  return h;
}

///------------------tracker---------------
#include "setup_tracker.h" 

// Vloží správný q-construct (qAR nebo qAO) do hlavičky APRS rámce.
// Pokud už hlavička obsahuje ",qA?" (tj. q-construct), paket se nechá beze změny.
String injectQConstruct(const String& pkt, const String& igateCall, bool bidirectional) {
  int colon = pkt.indexOf(':');
  if (colon < 0) return pkt;  // žádný payload → nic nevkládat

  String head = pkt.substring(0, colon);
  // Pokud už existuje nějaký q-construct, nic nepřidávej
  if (head.indexOf(",qA") >= 0) return pkt;

  String q = bidirectional ? "qAR" : "qAO";
  // vlož ",qAR,IGATE" nebo ",qAO,IGATE" těsně před ':'
  return head + "," + q + "," + igateCall + pkt.substring(colon);
}


void setup() {
  // Inicializace LED pinu
  pinMode(PLED1, OUTPUT);
  digitalWrite(PLED1, LOW);

  pinMode(lora_DIO0, INPUT);
  Serial.begin(9600);
  delay(500);

  // Initialize buffers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    buffer[i][0] = '\0';
    buffer_SN[i][0] = '\0';
    buffer_RSSI[i][0] = '\0';
    buffer_pak[i][0] = '\0';
    buffer_cas[i][0] = '\0';
    buffer_vzdalenost[i][0] = '\0';
    buffer_azimut[i][0] = '\0';
    buffer_icona[i][0] = '\0';
    buffer_lat[i] = 0.0; // Initialize latitude
    buffer_lon[i] = 0.0; // Initialize longitude
  }

  // Initialize message buffers
for (int i = 0; i < MESSAGE_BUFFER_SIZE; i++) {
  buffer_zpravy[i][0] = '\0';
  buffer_zpravy_cas[i][0] = '\0';
  buffer_zpravy_odesilatel[i][0] = '\0';
  buffer_zpravy_age[i] = 0;
}

  //-------- SPIFFS --------
  if (!SPIFFS.begin(true)) {
    WLOGln("Chyba SPIFFS");
    return;
  }

  //-------- Načítání konfigurace --------
  File file = SPIFFS.open("/config.txt");
  if (file) {
    fileContent = "";
    while (file.available()) {
      fileContent += char(file.read());
    }
    file.close();
    WLOGln("Soubor konfigurace:");
    WLOGln(fileContent);

    for (int i = 0; i < MAX_SETTINGS; i++) {
      int start = fileContent.indexOf("<");
      int end = fileContent.indexOf(">");
      if (start != -1 && end != -1 && end > start) {
        nastaveni[i] = fileContent.substring(start + 1, end);
        fileContent = fileContent.substring(end + 1);
      } else {
        nastaveni[i] = "";
      }
      WLOGln("nastaveni[");
      WLOGln(i);
      WLOGln("]: ");
      WLOGln(nastaveni[i]);
    }

    // Přiřazení nastavení
    if (nastaveni[0].length() > 0) ssid = nastaveni[0];
    if (nastaveni[1].length() > 0) password = nastaveni[1];
    if (nastaveni[2].length() > 0) call = nastaveni[2];
    if (nastaveni[3].length() > 0) lon = nastaveni[3];
    if (nastaveni[4].length() > 0) lat = nastaveni[4];
    if (nastaveni[5].length() > 0) sym = nastaveni[5];
    if (nastaveni[6].length() > 0) tool = nastaveni[6];
    if (nastaveni[7].length() > 0) Alt = nastaveni[7].toFloat();
    if (nastaveni[8].length() > 0) aprs_filter = nastaveni[8];
    if (nastaveni[9].length() > 0) strncpy(servername, nastaveni[9].c_str(), sizeof(servername));
    if (nastaveni[10].length() > 0) aprs_port = nastaveni[10].toInt();
    if (nastaveni[11].length() > 0) password_aprs = nastaveni[11];
    if (nastaveni[12].length() > 0) pouzitPevnouIP = (nastaveni[12] == "true");
    if (nastaveni[13].length() > 0) {
      int dot1 = nastaveni[13].indexOf(".");
      int dot2 = nastaveni[13].indexOf(".", dot1 + 1);
      int dot3 = nastaveni[13].indexOf(".", dot2 + 1);
      if (dot1 != -1 && dot2 != -1 && dot3 != -1) {
        local_IP = IPAddress(nastaveni[13].substring(0, dot1).toInt(),
                            nastaveni[13].substring(dot1 + 1, dot2).toInt(),
                            nastaveni[13].substring(dot2 + 1, dot3).toInt(),
                            nastaveni[13].substring(dot3 + 1).toInt());
      }
    }
    if (nastaveni[14].length() > 0) {
      int dot1 = nastaveni[14].indexOf(".");
      int dot2 = nastaveni[14].indexOf(".", dot1 + 1);
      int dot3 = nastaveni[14].indexOf(".", dot2 + 1);
      if (dot1 != -1 && dot2 != -1 && dot3 != -1) {
        gateway = IPAddress(nastaveni[14].substring(0, dot1).toInt(),
                          nastaveni[14].substring(dot1 + 1, dot2).toInt(),
                          nastaveni[14].substring(dot2 + 1, dot3).toInt(),
                          nastaveni[14].substring(dot3 + 1).toInt());
      }
    }
    if (nastaveni[15].length() > 0) {
      int dot1 = nastaveni[15].indexOf(".");
      int dot2 = nastaveni[15].indexOf(".", dot1 + 1);
      int dot3 = nastaveni[15].indexOf(".", dot2 + 1);
      if (dot1 != -1 && dot2 != -1 && dot3 != -1) {
        subnet = IPAddress(nastaveni[15].substring(0, dot1).toInt(),
                          nastaveni[15].substring(dot1 + 1, dot2).toInt(),
                          nastaveni[15].substring(dot2 + 1, dot3).toInt(),
                          nastaveni[15].substring(dot3 + 1).toInt());
      }
    }
    if (nastaveni[16].length() > 0) {
      int dot1 = nastaveni[16].indexOf(".");
      int dot2 = nastaveni[16].indexOf(".", dot1 + 1);
      int dot3 = nastaveni[16].indexOf(".", dot2 + 1);
      if (dot1 != -1 && dot2 != -1 && dot3 != -1) {
        primaryDNS = IPAddress(nastaveni[16].substring(0, dot1).toInt(),
                              nastaveni[16].substring(dot1 + 1, dot2).toInt(),
                              nastaveni[16].substring(dot2 + 1, dot3).toInt(),
                              nastaveni[16].substring(dot3 + 1).toInt());
        secondaryDNS = primaryDNS;
      }
    }
    if (nastaveni[18].length() > 0) digi_mode = nastaveni[18].toInt();
    if (nastaveni[19].length() > 0) digi_AP = nastaveni[19].toInt();
    if (nastaveni[20].length() > 0) ap_password = nastaveni[20];
    if (nastaveni[21].length() > 0) spreading_factor = nastaveni[21].toInt();
    if (nastaveni[22].length() > 0) bandwidth = nastaveni[22].toInt();
    if (nastaveni[23].length() > 0) coding_rate = nastaveni[23].toInt();
     if (nastaveni[24].length() > 0) web_username = nastaveni[24];
    if (nastaveni[25].length() > 0) web_password = nastaveni[25];
    
    Serial.print("web_username: ");
    Serial.println(web_username);
    Serial.print("web_password: ");
    Serial.println(web_password);
    Serial.print("digi_mode: ");
    Serial.println(digi_mode);
    Serial.print("digi_AP: ");
    Serial.println(digi_AP);
    Serial.print("ap_password: ");
    Serial.println(ap_password);
    Serial.print("spreading_factor: ");
    Serial.println(spreading_factor);
    Serial.print("bandwidth: ");
    Serial.println(bandwidth);
    Serial.print("coding_rate: ");
    Serial.println(coding_rate);
    user = call;
  } else {
    WLOGln("Soubor config.txt nelze přečíst, použity výchozí hodnoty.");
  }
///-----------konfigurace object-------
 // Po SPIFFS.begin(...) a po načtení hlavní konfigurace:
  loadObjectsConfig();

  // Inic. časů nextDueSec při prvním startu (když jsou nulové)
  uint32_t nowSecInit = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
  for (int i=0;i<5;i++){
    if (OBJ[i].nextDueSec == 0) OBJ[i].nextDueSec = nowSecInit + (OBJ[i].intervalMin*60);
  }
  //-------- Inicializace OLED --------
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
    Serial.println(F("SSD1306 error"));
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(call);
  display.setTextSize(1);
  display.setCursor(5, 27);
  display.print(F("RX LoRa"));
  display.setCursor(5, 36);
  display.print(F("(c) OK5TVR"));
  display.setCursor(5, 46);
  display.print(F("ver.: "));
  display.setCursor(40, 46);
  display.print(verze);
  display.display();
  delay(3000);

  //-------- Inicializace podle režimu --------
  if (digi_AP == 1) {
    wifi();
    tracker_setup(&server);   // zaregistruje /tracker a načte/vytvoří /tracker.cfg
    ElegantOTA.begin(&server,web_username.c_str(), web_password.c_str());
    // WebSerial – pověsit na AsyncWebServer a zapnout BasicAuth
    WebSerial.begin(&server);  // vytvoří stránku /webserial
    WebSerial.setAuthentication(web_username.c_str(), web_password.c_str());
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      WLOGln("Client připojen: " + request->client()->remoteIP().toString());
      request->send_P(200, "text/html", index_html, procesor);
    });
    // přesměrování /serial -> /webserial (aby seděl odkaz v UI)
server.on("/serial", HTTP_GET, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();
  request->redirect("/webserial");
});
    server.on("/nastaveni", HTTP_GET, [](AsyncWebServerRequest *request) {
      WLOGln("Client připojen k nastavení: " + request->client()->remoteIP().toString());
       if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
      return request->requestAuthentication();
    }
      request->send_P(200, "text/html", nastaveni_html, procesor);
    });
    server.on("/nastaveni", HTTP_POST, [](AsyncWebServerRequest *request) {
      String newConfig = "";
      if (request->hasParam("ssid", true)) newConfig += "<" + request->getParam("ssid", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("password", true)) newConfig += "<" + request->getParam("password", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("call", true)) newConfig += "<" + request->getParam("call", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("lon", true)) newConfig += "<" + request->getParam("lon", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("lat", true)) newConfig += "<" + request->getParam("lat", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("sym", true)) newConfig += "<" + request->getParam("sym", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("tool", true)) newConfig += "<" + request->getParam("tool", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("Alt", true)) newConfig += "<" + request->getParam("Alt", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("aprs_filter", true)) newConfig += "<" + request->getParam("aprs_filter", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("servername", true)) newConfig += "<" + request->getParam("servername", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("aprs_port", true)) newConfig += "<" + request->getParam("aprs_port", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("password_aprs", true)) newConfig += "<" + request->getParam("password_aprs", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("pouzitPevnouIP", true)) newConfig += "<" + request->getParam("pouzitPevnouIP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("local_IP", true)) newConfig += "<" + request->getParam("local_IP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("gateway", true)) newConfig += "<" + request->getParam("gateway", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("subnet", true)) newConfig += "<" + request->getParam("subnet", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("primaryDNS", true)) newConfig += "<" + request->getParam("primaryDNS", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("secondaryDNS", true)) newConfig += "<" + request->getParam("secondaryDNS", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("digi", true)) newConfig += "<" + request->getParam("digi", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("digi_AP", true)) newConfig += "<" + request->getParam("digi_AP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("ap_password", true)) newConfig += "<" + request->getParam("ap_password", true)->value() + ">";
      else newConfig += "<>";
           if (request->hasParam("spreading_factor", true)) newConfig += "<" + request->getParam("spreading_factor", true)->value() + ">";
    else newConfig += "<12>";
    if (request->hasParam("bandwidth", true)) newConfig += "<" + request->getParam("bandwidth", true)->value() + ">";
    else newConfig += "<125000>";
    if (request->hasParam("coding_rate", true)) newConfig += "<" + request->getParam("coding_rate", true)->value() + ">";
    else newConfig += "<5>";
    if (request->hasParam("web_username", true)) newConfig += "<" + request->getParam("web_username", true)->value() + ">";
    else newConfig += "<admin>";
    if (request->hasParam("web_password", true)) newConfig += "<" + request->getParam("web_password", true)->value() + ">";
    else newConfig += "<admin>";

      File file = SPIFFS.open("/config.txt", FILE_WRITE);
      if (file) {
        file.print(newConfig);
        file.close();
        WLOGln("Konfigurace uložena do config.txt: " + newConfig);
        request->send(200, "text/plain", "Konfigurace uložena, zařízení se restartuje...");
        delay(1000);
        ESP.restart();
      } else {
        WLOGln("Chyba při ukládání config.txt");
        request->send(500, "text/plain", "Chyba při ukládání konfigurace");
      }
    });
    // === Objects UI ===
server.on("/objects", HTTP_GET, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();
  String page = buildObjectsHtml();
  request->send(200, "text/html", page);
});

server.on("/objects", HTTP_POST, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();

  // Pokud přišel parametr 'send_now', víme, že se klikal konkrétní řádek.
  int sendIdx = -1;
  if (request->hasParam("send_now", true)) {
    sendIdx = request->getParam("send_now", true)->value().toInt();
  }

  // 1) Načti hodnoty z formuláře do OBJ[*] (stejně jako dřív)
  for (int i=0;i<5;i++){
    OBJ[i].enabled  = request->hasParam("o"+String(i)+"_en", true);
    OBJ[i].name     = request->getParam("o"+String(i)+"_name", true)->value();
    OBJ[i].latstr   = request->getParam("o"+String(i)+"_lat",  true)->value();
    OBJ[i].lonstr   = request->getParam("o"+String(i)+"_lon",  true)->value();
    OBJ[i].tableCh  = request->getParam("o"+String(i)+"_tab",  true)->value()[0];
    OBJ[i].symCh    = request->getParam("o"+String(i)+"_sym",  true)->value()[0];
    OBJ[i].comment  = request->getParam("o"+String(i)+"_cmt",  true)->value();
    String act      = request->getParam("o"+String(i)+"_act",  true)->value();
    OBJ[i].killFlag = (act == "kill");
    String iv       = request->getParam("o"+String(i)+"_int",  true)->value();
    uint32_t mins   = iv.toInt(); if (mins==0) mins=60;
    OBJ[i].intervalMin = mins;

    // normalizace
    if (OBJ[i].name.length() > 9) OBJ[i].name = OBJ[i].name.substring(0,9);
    OBJ[i].name.toUpperCase();
    char c = OBJ[i].symCh; if (c>='a' && c<='z') OBJ[i].symCh = c-32;
  }

  // 2) Pokud se klikal "Send now", pošli IHNEĎ jen určený objekt (bez ukládání)
  if (sendIdx >= 0 && sendIdx < 5) {
    String payload = buildAprsObject(
      padObjectName9(OBJ[sendIdx].name),
      OBJ[sendIdx].killFlag,
      OBJ[sendIdx].latstr.length()?OBJ[sendIdx].latstr:lon,
      OBJ[sendIdx].lonstr.length()?OBJ[sendIdx].lonstr:lat,
      OBJ[sendIdx].tableCh,
      OBJ[sendIdx].symCh,
      OBJ[sendIdx].comment
    );
    sendAprsObject(payload);

    // posuň interní timer pro daný objekt
    uint32_t nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
    OBJ[sendIdx].nextDueSec = nowSec + (OBJ[sendIdx].intervalMin*60);

    // (volitelné) Nechceme měnit config při "Send now" → neukládej
    // saveObjectsConfig();  // <- vynecháno

    // vrať znovu stránku (stejnou), případně by šlo přidat "flash" hlášku
    request->send(200, "text/html", buildObjectsHtml());
    return;
  }

  // 3) Jinak je to běžné "Save" → uložit a vrátit stránku
  saveObjectsConfig();

  // po uložení nastavíme příští termíny
  uint32_t nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
  for (int i=0;i<5;i++){
    OBJ[i].nextDueSec = nowSec + (OBJ[i].intervalMin*60);
  }

  request->send(200, "text/html", buildObjectsHtml());
});
///-------------------zpravy------------------------
    server.on("/zpravy", HTTP_GET, [](AsyncWebServerRequest *request) {
  WLOGln("Client připojen k zprávám: " + request->client()->remoteIP().toString());
  if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
    return request->requestAuthentication();
  }
  request->send_P(200, "text/html", zpravy_html, procesor);
});
server.on("/zpravy", HTTP_POST, [](AsyncWebServerRequest *request) {
  if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
    return request->requestAuthentication();
  }
  if (digi_AP != 1) {
    request->send(403, "text/plain", "Sending messages is only allowed in AP+DIGI mode");
    return;
  }
  String recipient = request->hasParam("recipient", true) ? request->getParam("recipient", true)->value() : "";
  String message = request->hasParam("message", true) ? request->getParam("message", true)->value() : "";
  if (recipient.length() == 0 || message.length() == 0) {
    request->send(400, "text/plain", "Recipient and message cannot be empty");
    return;
  }
  if (message.length() > 67) { // APRS zpráva max 67 znaků
    request->send(400, "text/plain", "Message too long (max 67 characters)");
    return;
  }
  // Odeslání zprávy přes LoRa
  String aprs_message = call + ">APZ023::" + recipient + " :" + message + "{01";
  digitalWrite(PLED1, HIGH);
  LoRa.beginPacket();
  LoRa.print("<" + String((char)0xFF) + String((char)0x01) + aprs_message);
  LoRa.endPacket();
  digitalWrite(PLED1, LOW);
  WLOGln("Zpráva odeslána přes RF: " + aprs_message);
  request->send_P(200, "text/html", zpravy_html, procesor);
});
    server.onNotFound(notFound);
    server.begin();
    cas_new = millis() / 1000;
    WLOGln("AP MODE spuštěn – WiFi AP aktivní, APRS-IS vypnuto.");
  } else if (digi_mode == 0) {
    wifi();
    tracker_setup(&server);   // zaregistruje /tracker a načte/vytvoří /tracker.cfg
    ElegantOTA.begin(&server, web_username.c_str(), web_password.c_str());
        // WebSerial – pověsit na AsyncWebServer a zapnout BasicAuth
    WebSerial.begin(&server);  // vytvoří stránku /webserial
    WebSerial.setAuthentication(web_username.c_str(), web_password.c_str());
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      WLOGln("Client připojen: " + request->client()->remoteIP().toString());
      request->send_P(200, "text/html", index_html, procesor);
    });
    // přesměrování /serial -> /webserial (aby seděl odkaz v UI)
server.on("/serial", HTTP_GET, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();
  request->redirect("/webserial");
});
    server.on("/nastaveni", HTTP_GET, [](AsyncWebServerRequest *request) {
      Serial.println("Client připojen k nastavení: " + request->client()->remoteIP().toString());
       if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
      return request->requestAuthentication();
    }
      request->send_P(200, "text/html", nastaveni_html, procesor);
    });
   server.on("/map", HTTP_GET, [](AsyncWebServerRequest *request) {
    WLOGln("Client connected to map: " + request->client()->remoteIP().toString());
    request->send_P(200, "text/html", map_html, procesor);
  });
    server.on("/nastaveni", HTTP_POST, [](AsyncWebServerRequest *request) {
      String newConfig = "";
      if (request->hasParam("ssid", true)) newConfig += "<" + request->getParam("ssid", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("password", true)) newConfig += "<" + request->getParam("password", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("call", true)) newConfig += "<" + request->getParam("call", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("lon", true)) newConfig += "<" + request->getParam("lon", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("lat", true)) newConfig += "<" + request->getParam("lat", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("sym", true)) newConfig += "<" + request->getParam("sym", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("tool", true)) newConfig += "<" + request->getParam("tool", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("Alt", true)) newConfig += "<" + request->getParam("Alt", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("aprs_filter", true)) newConfig += "<" + request->getParam("aprs_filter", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("servername", true)) newConfig += "<" + request->getParam("servername", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("aprs_port", true)) newConfig += "<" + request->getParam("aprs_port", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("password_aprs", true)) newConfig += "<" + request->getParam("password_aprs", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("pouzitPevnouIP", true)) newConfig += "<" + request->getParam("pouzitPevnouIP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("local_IP", true)) newConfig += "<" + request->getParam("local_IP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("gateway", true)) newConfig += "<" + request->getParam("gateway", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("subnet", true)) newConfig += "<" + request->getParam("subnet", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("primaryDNS", true)) newConfig += "<" + request->getParam("primaryDNS", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("secondaryDNS", true)) newConfig += "<" + request->getParam("secondaryDNS", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("digi", true)) newConfig += "<" + request->getParam("digi", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("digi_AP", true)) newConfig += "<" + request->getParam("digi_AP", true)->value() + ">";
      else newConfig += "<>";
      if (request->hasParam("ap_password", true)) newConfig += "<" + request->getParam("ap_password", true)->value() + ">";
      else newConfig += "<>";
         if (request->hasParam("spreading_factor", true)) newConfig += "<" + request->getParam("spreading_factor", true)->value() + ">";
    else newConfig += "<12>";
    if (request->hasParam("bandwidth", true)) newConfig += "<" + request->getParam("bandwidth", true)->value() + ">";
    else newConfig += "<125000>";
    if (request->hasParam("coding_rate", true)) newConfig += "<" + request->getParam("coding_rate", true)->value() + ">";
    else newConfig += "<5>";
      if (request->hasParam("web_username", true)) newConfig += "<" + request->getParam("web_username", true)->value() + ">";
    else newConfig += "<admin>";
    if (request->hasParam("web_password", true)) newConfig += "<" + request->getParam("web_password", true)->value() + ">";
    else newConfig += "<admin>";

      File file = SPIFFS.open("/config.txt", FILE_WRITE);
      if (file) {
        file.print(newConfig);
        file.close();
        WLOGln("Konfigurace uložena do config.txt: " + newConfig);
        request->send(200, "text/plain", "Konfigurace uložena, zařízení se restartuje...");
        delay(1000);
        ESP.restart();
      } else {
        WLOGln("Chyba při ukládání config.txt");
        request->send(500, "text/plain", "Chyba při ukládání konfigurace");
      }
    });
server.on("/zpravy", HTTP_GET, [](AsyncWebServerRequest *request) {
  WLOGln("Client připojen k zprávám: " + request->client()->remoteIP().toString());
  if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
    return request->requestAuthentication();
  }
  request->send_P(200, "text/html", zpravy_html, procesor);
});
server.on("/zpravy", HTTP_POST, [](AsyncWebServerRequest *request) {
  if (!request->authenticate(web_username.c_str(), web_password.c_str())) {
    return request->requestAuthentication();
  }
  request->send(403, "text/plain", "Sending messages is only allowed in AP+DIGI mode");
});
/////---------object------

server.on("/objects", HTTP_GET, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();
  String page = buildObjectsHtml();
  request->send(200, "text/html", page);
});

server.on("/objects", HTTP_POST, [](AsyncWebServerRequest *request){
  if (!request->authenticate(web_username.c_str(), web_password.c_str()))
    return request->requestAuthentication();

  // Pokud přišel parametr 'send_now', víme, že se klikal konkrétní řádek.
  int sendIdx = -1;
  if (request->hasParam("send_now", true)) {
    sendIdx = request->getParam("send_now", true)->value().toInt();
  }

  // 1) Načti hodnoty z formuláře do OBJ[*] (stejně jako dřív)
  for (int i=0;i<5;i++){
    OBJ[i].enabled  = request->hasParam("o"+String(i)+"_en", true);
    OBJ[i].name     = request->getParam("o"+String(i)+"_name", true)->value();
    OBJ[i].latstr   = request->getParam("o"+String(i)+"_lat",  true)->value();
    OBJ[i].lonstr   = request->getParam("o"+String(i)+"_lon",  true)->value();
    OBJ[i].tableCh  = request->getParam("o"+String(i)+"_tab",  true)->value()[0];
    OBJ[i].symCh    = request->getParam("o"+String(i)+"_sym",  true)->value()[0];
    OBJ[i].comment  = request->getParam("o"+String(i)+"_cmt",  true)->value();
    String act      = request->getParam("o"+String(i)+"_act",  true)->value();
    OBJ[i].killFlag = (act == "kill");
    String iv       = request->getParam("o"+String(i)+"_int",  true)->value();
    uint32_t mins   = iv.toInt(); if (mins==0) mins=60;
    OBJ[i].intervalMin = mins;

    // normalizace
    if (OBJ[i].name.length() > 9) OBJ[i].name = OBJ[i].name.substring(0,9);
    OBJ[i].name.toUpperCase();
    char c = OBJ[i].symCh; if (c>='a' && c<='z') OBJ[i].symCh = c-32;
  }

  // 2) Pokud se klikal "Send now", pošli IHNEĎ jen určený objekt (bez ukládání)
  if (sendIdx >= 0 && sendIdx < 5) {
    String payload = buildAprsObject(
      padObjectName9(OBJ[sendIdx].name),
      OBJ[sendIdx].killFlag,
      OBJ[sendIdx].latstr.length()?OBJ[sendIdx].latstr:lon,
      OBJ[sendIdx].lonstr.length()?OBJ[sendIdx].lonstr:lat,
      OBJ[sendIdx].tableCh,
      OBJ[sendIdx].symCh,
      OBJ[sendIdx].comment
    );
    sendAprsObject(payload);

    // posuň interní timer pro daný objekt
    uint32_t nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
    OBJ[sendIdx].nextDueSec = nowSec + (OBJ[sendIdx].intervalMin*60);

    // (volitelné) Nechceme měnit config při "Send now" → neukládej
    // saveObjectsConfig();  // <- vynecháno

    // vrať znovu stránku (stejnou), případně by šlo přidat "flash" hlášku
    request->send(200, "text/html", buildObjectsHtml());
    return;
  }

  // 3) Jinak je to běžné "Save" → uložit a vrátit stránku
  saveObjectsConfig();

  // po uložení nastavíme příští termíny
  uint32_t nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
  for (int i=0;i<5;i++){
    OBJ[i].nextDueSec = nowSec + (OBJ[i].intervalMin*60);
  }

  request->send(200, "text/html", buildObjectsHtml());
});

    server.onNotFound(notFound);
    server.begin();
    con_aprs();
    timeClient.begin();
    timeClient.update();
    cas_new = timeClient.getEpochTime();
    Serial.println("iGate MODE spuštěn.");
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    cas_new = millis() / 1000;
    Serial.println("DIGI MODE spuštěn – WiFi a APRS-IS vypnuto.");
  }
  cas_old = cas_new;
  cas_reset = cas_new;
  cas_telemetry = cas_new;

  //-------- Inicializace LoRa --------
  LoRa.setPins(lora_SS, lora_RST, lora_DIO0);
  if (!LoRa.begin(433775000)) {
    WLOGln("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(spreading_factor);
  LoRa.setSignalBandwidth(bandwidth);
  LoRa.setCodingRate4(coding_rate);
  LoRa.disableCrc();
  WLOGln("LoRa started successfully.");

  //-------- Úvodní zobrazení na OLED --------
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(call);
  display.setCursor(70, 0);
  display.print(digi_AP == 1 ? "AP+DIGI" : (digi_mode ? "DIGI" : "iGate"));
  if (digi_mode == 0 || digi_AP == 1) {
    display.setCursor(2, 9);
    display.print(IP);
  }
  display.setCursor(5, 27);
  display.print(F("RX LoRa"));
  display.setCursor(5, 36);
  display.print(F("ALL OK"));
  display.setCursor(5, 46);
  display.print(F("ver.: "));
  display.setCursor(40, 46);
  display.print(verze);
  display.display();

  ///--oveření pozice---
  WLOGln("Lat: " + String(convertToDecimalDegreesLat(lon), 6));
  WLOGln("Lon: " + String(convertToDecimalDegreesLon(lat), 6));

  //---- Úvodní beacon
  posliBeacon();
}

void loop() {

/////------------tracker.-----------
tracker_loop(); 
//--object--------
// === Objects scheduler (každý průchod loop) ===
  uint32_t nowSec = (digi_mode==0 && digi_AP==0) ? timeClient.getEpochTime() : (millis()/1000);
  for (int i=0;i<5;i++){
    if (!OBJ[i].enabled) continue;
    if (OBJ[i].intervalMin == 0) continue;
    if (nowSec >= OBJ[i].nextDueSec) {
      String payload = buildAprsObject(
        padObjectName9(OBJ[i].name),
        OBJ[i].killFlag,
        OBJ[i].latstr.length()?OBJ[i].latstr:lon,
        OBJ[i].lonstr.length()?OBJ[i].lonstr:lat,
        OBJ[i].tableCh,
        OBJ[i].symCh,
        OBJ[i].comment
      );
      sendAprsObject(payload);
      OBJ[i].nextDueSec = nowSec + (OBJ[i].intervalMin*60);
      // (nepovinné) průběžně ukládat: saveObjectsConfig();
    }
  }

    // -- aktualizace času (sekundy)
  unsigned long currentTime = millis() / 1000;

  if (digi_mode == 0 && digi_AP == 0) {
    timeClient.update();
    cas_new = timeClient.getEpochTime();
  } else {
    cas_new = currentTime;
  }
  cas_dif = cas_new - cas_old;

  // -- kontrola Wi-Fi v iGate módu
  if (digi_mode == 0 && digi_AP == 0 && WiFi.status() != WL_CONNECTED) {
    WLOGln("WiFi no connect...");
    wifi();
  }

  // -- watchdog/restart po 2h
  if (cas_new - cas_reset > 7200) {
    esp_restart();
  }

  // -- telemetrie každých 900 s
  if (cas_new - cas_telemetry > 900) {
    temp_cpu = (temprature_sens_read() - 32) / 1.8;

    live_s = "";
    if (live < 100) live_s += "0";
    if (live < 10)  live_s += "0";
    live_s += live;

    if (digi_mode == 0 && digi_AP == 0) {
      if (client.connected()) {
        client.println(call + ">APZ023,TCPIP*:T#" + live_s + "," + rx_cnt + "," + rf_inet + "," + dx_dist + "," + live + "," + temp_cpu + ",00000000");
        client.flush();
        last_inet_tx_sec = timeClient.getEpochTime(); // spojení + hlavičky = „TX“
      } else {
        WLOGln("Connection to APRS server lost");
        client.stop();
        con_aprs();
      }
    } else {
      String telemetry = "<" + String((char)0xFF) + String((char)0x01) + call + ">APZ023:T#" + live_s + "," + rx_cnt + "," + rf_inet + "," + dx_dist + "," + live + "," + temp_cpu + ",00000000";
      digitalWrite(PLED1, HIGH);
      LoRa.beginPacket();
      LoRa.print(telemetry);
      LoRa.endPacket();
      digitalWrite(PLED1, LOW);
      WLOGln("Telemetry sent via RF: " + telemetry);
    }

    live++;
    rx_cnt   = 0;
    rf_inet  = 0;
    dx_dist  = 0;
    if (live >= 96) live = 0;
    cas_telemetry = cas_new;
  }

  // -- beacon každých 1200 s
  if (cas_new - cas_old > 1200) {
    posliBeacon();
    cas_old = cas_new;
  }

  // -- příjem LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String packet;
    while (LoRa.available()) packet += (char)LoRa.read();

    WLOGln("Received packet: ");
    WLOGln(packet);
    WLOGln(" with RSSI ");
    WLOGln(LoRa.packetRssi());
    WLOGln(" with SNR ");
    WLOGln(LoRa.packetSnr());

    // Odřízni neabecední prefix
    int firstLetterIndex = -1;
    for (int i = 0; i < packet.length(); i++) {
      if (isAlpha(packet.charAt(i))) { firstLetterIndex = i; break; }
    }
    if (firstLetterIndex != -1) {
      packet = packet.substring(firstLetterIndex);
     WLOGln("Cleaned packet: ");
      WLOGln(packet);
    } else {
      WLOGln("No letter found, packet ignored.");
      rx_cnt++;
      return;
    }

    // ASCII kontrola
    if (!isASCII(packet)) {
      WLOGln("Packet contains non-ASCII characters.");
      rx_cnt++;
      return;
    }
    rx_cnt++;

    //---------zpracování zprávy------------
// Extrakce volací značky odesílatele (call_d)
int posGt = packet.indexOf(">");
if (posGt == -1) {
  WLOGln("Invalid packet format, missing '>'");
  return;
}
call_d = packet.substring(0, posGt);

// Rozpoznání APRS zprávy
int colonIndex = packet.indexOf(':');
if (colonIndex != -1 && packet[colonIndex + 1] == ':' && colonIndex + 11 < packet.length()) {
  String destCall = packet.substring(colonIndex + 2, colonIndex + 11);
  destCall.trim();
  if (destCall == call) { // Zpráva je určena pro nás
    String message = packet.substring(colonIndex + 11);
    int braceIndex = message.indexOf('{');
    if (braceIndex != -1) {
      message = message.substring(0, braceIndex); // Odstraníme ID zprávy
    }
    // Uložení zprávy do bufferu
    int msg_idx = message_cnt % MESSAGE_BUFFER_SIZE;
    snprintf(buffer_zpravy[msg_idx], sizeof(buffer_zpravy[msg_idx]), "%s", message.c_str());
    snprintf(buffer_zpravy_odesilatel[msg_idx], sizeof(buffer_zpravy_odesilatel[msg_idx]), "%s", call_d.c_str());
    if (digi_mode == 0 && digi_AP == 0) {
      String t = timeClient.getFormattedTime();
      snprintf(buffer_zpravy_cas[msg_idx], sizeof(buffer_zpravy_cas[msg_idx]), "%s", t.c_str());
    } else {
      snprintf(buffer_zpravy_cas[msg_idx], sizeof(buffer_zpravy_cas[msg_idx]), "%ldm", currentTime / 60);
    }
    buffer_zpravy_age[msg_idx] = cas_new;
    message_cnt++;
  }
}
    // iGate upload (qAS), nebo digi RF forward
    if (digi_mode == 0 && digi_AP == 0) {
      String toSend = packet;
      toSend.trim();
      //int colonIndex = toSend.indexOf(":");
      //if (colonIndex >= 0) {
       // toSend = toSend.substring(0, colonIndex) + ",qAS," + call + toSend.substring(colonIndex) + "\n";
      //}

       // vlož q-construct podle přepínače (RX-only = qAO, obousměrné = qAR)
      toSend = injectQConstruct(toSend, call, IGATE_BIDIRECTIONAL);

      if (client.connected()) {
        WLOGln("Sending packet to iGate: " + toSend);
        client.println("");
        client.println(toSend);
        client.flush();
        last_inet_tx_sec = timeClient.getEpochTime(); // spojení + hlavičky = „TX“
        rf_inet++;
      } else {
        WLOGln("Connection to APRS server lost");
        client.stop();
        con_aprs();
      }
    } else {
      if (!uzJsemDigipeatoval(packet, call)) {
        String modifiedPacket = upravDigipeaterPath(packet, call);
        digitalWrite(PLED1, HIGH);
        LoRa.beginPacket();
        LoRa.print("<" + String((char)0xFF) + String((char)0x01) + modifiedPacket);
        LoRa.endPacket();
        digitalWrite(PLED1, LOW);
        WLOGln("Packet forwarded via RF: <" + String((char)0xFF) + String((char)0x01) + modifiedPacket);
      } else {
        WLOGln("Packet already digipeated, ignoring.");
      }
    }

    // --- rozbor packetu a zápis do bufferu 5 unikátních stanic ---
    posGt = packet.indexOf(">");
    if (posGt == -1) return;

    String call_d = packet.substring(0, posGt);
    int idx = pickIndexFor(call_d);   // <<< klíčová volba slotu

    // Čas do slotu
    if (digi_mode == 0 && digi_AP == 0) {
      String t = timeClient.getFormattedTime();
      snprintf(buffer_cas[idx], sizeof(buffer_cas[idx]), "%s", t.c_str());
    } else {
      snprintf(buffer_cas[idx], sizeof(buffer_cas[idx]), "%ldm", currentTime / 60);
    }

    // RSSI / SNR
    char buf_rssi[10], buf_snr[10];
    dtostrf(LoRa.packetRssi(), 1, 1, buf_rssi);
    dtostrf(LoRa.packetSnr(), 1, 1, buf_snr);

    snprintf(buffer[idx],          sizeof(buffer[idx]),          "%s", call_d.c_str());
    snprintf(buffer_RSSI[idx],     sizeof(buffer_RSSI[idx]),     "%s", buf_rssi);
    snprintf(buffer_SN[idx],       sizeof(buffer_SN[idx]),       "%s", buf_snr);
    snprintf(buffer_pak[idx],      sizeof(buffer_pak[idx]),      "%s", packet.c_str());

    // Pozice stanice z packetu (jen pokud nejde o '@' timestamp)
    latitude2 = 0; longitude2 = 0;
    int startIndex = packet.indexOf(':');
    if (startIndex != -1 && packet.indexOf('@') == -1 && startIndex + 2 < packet.length()) {
      String icon;
      // Komprimovaná vs. nekomprimovaná pozice
      if ((packet.substring(startIndex + 2, startIndex + 3) == "/") || (packet.substring(startIndex + 2, startIndex + 3) == "\\")) {
        // komprese
        icon = packet.substring(startIndex + 2, startIndex + 3) + packet.substring(startIndex + 11, startIndex + 12);
        String k_lat = packet.substring(startIndex + 3, startIndex + 7);
        String k_lon = packet.substring(startIndex + 7, startIndex + 11);
        latitude2  =  90.0 - ((((((k_lat.charAt(0) - 33) * 91.0) + (k_lat.charAt(1) - 33)) * 91.0) + (k_lat.charAt(2) - 33)) * 91.0 + (k_lat.charAt(3) - 33)) / 380926.0;
        longitude2 = -180.0 + ((((((k_lon.charAt(0) - 33) * 91.0) + (k_lon.charAt(1) - 33)) * 91.0) + (k_lon.charAt(2) - 33)) * 91.0 + (k_lon.charAt(3) - 33)) / 190463.0;
      } else {
        // nekomprimovaná
        if (startIndex + 21 < packet.length()) {
          String gpsLat = packet.substring(startIndex + 2,  startIndex + 9);
          String gpsLon = packet.substring(startIndex + 11, startIndex + 19);
          icon = packet.substring(startIndex + 10, startIndex + 11) + packet.substring(startIndex + 20, startIndex + 21);
          latitude2  = convertToDecimalDegrees_la(gpsLat.c_str());
          longitude2 = convertToDecimalDegrees_lo(gpsLon.c_str());
        }
      }

      // Ulož souřadnice do bufferu (i když jsou 0 – mapový kód si to ohlídá)
      buffer_lat[idx] = latitude2;
      buffer_lon[idx] = longitude2;

      // Výpočet vzdálenosti/azimutu (POZOR: proměnné lat/lon jsou ve tvém kódu "prohozené" podle názvu – zachovávám původní vzorec)
      double latitude1  = convertToDecimalDegrees_la(lon.c_str());
      double longitude1 = convertToDecimalDegrees_lo(lat.c_str());
      double distance   = calculateDistance(latitude1, longitude1, latitude2, longitude2);
      double azimuth    = calculateAzimuth(latitude1, longitude1, latitude2, longitude2);

      char buf_dist[10], buf_az[10];
      dtostrf(distance, 1, 2, buf_dist);
      dtostrf(azimuth,  1, 1, buf_az);

      snprintf(buffer_vzdalenost[idx], sizeof(buffer_vzdalenost[idx]), "%s", buf_dist);
      snprintf(buffer_azimut[idx],     sizeof(buffer_azimut[idx]),     "%s", buf_az);

      if (distance > dx_dist) {
        dx_dist = round(distance);
        if (dx_dist > 2000) dx_dist = 0;
      }
      
  if (icon.length() > 0) {
  if      (icon == "/#") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", digi.c_str());
  else if (icon == "R#") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", digi.c_str());
  else if (icon == "/r") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", digi.c_str());
  else if (icon == "1#") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", digi.c_str());
  else if (icon == "/>") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", car.c_str());
  else if (icon == "\\>") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", car.c_str());
  else if (icon == "L&") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", lgate.c_str());
  else if (icon == "I&") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", igate.c_str());
  else if (icon == "/_") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", wx.c_str());
  else if (icon == "/[") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", chodec.c_str());
  else if (icon == "//") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", red_dot.c_str());
  else if (icon == "/a") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", sanita.c_str());
  else if (icon == "/b") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", kolo.c_str());
  else if (icon == "/'") snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", air.c_str());
  else snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", nahrada.c_str());
} else {
  // když není ikona vůbec → náhradní
  snprintf(buffer_icona[idx], sizeof(buffer_icona[idx]), "%s", nahrada.c_str());
}

}



    // věk záznamu
    buffer_age[idx] = cas_new;
    lastStation = call_d;

    lastStationLat  = buffer_lat[idx];
    lastStationLon  = buffer_lon[idx];

    cas_reset   = cas_new;

    
    // OLED refresh (seznam 5 položek)
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(call);
    display.setCursor(70, 0);
    display.print(digi_AP == 1 ? "AP+DIGI" : (digi_mode ? "DIGI" : "iGate"));
    if (digi_mode == 0 || digi_AP == 1) {
      display.setCursor(2, 9);
      display.print(IP);
    }
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int y = 18 + (i * 9);
      display.setCursor(2,  y);
      display.print(buffer[i]);
      display.setCursor(60, y);
      display.print(buffer_RSSI[i]);
      display.setCursor(95, y);
      display.print(buffer_SN[i]);
    }
    display.setCursor(0, (digi_mode == 1 && digi_AP == 0) ? 9 : 54);
    display.print("Ver: ");
    display.setCursor(40, (digi_mode == 1 && digi_AP == 0) ? 9 : 54);
    display.print(verze);
    display.display();
  }
}