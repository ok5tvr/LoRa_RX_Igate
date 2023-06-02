#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "ESPAsyncWebServer.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "HTTPClient.h"
#include <APRS-Decoder.h>
#include <math.h>
#include <string>
#include <AsyncElegantOTA.h>
#include "SPIFFS.h"


//--------wifi-----------------
String IP = "000.000.000.000";
const char *ssid     = "Vlas_dolni_vlkys";
const char *password = "xxxxxxx";

///--------------verze---------
String verze = "2.1.0";

/// ------- ID APRS -------------------------
String call = "OK5TVR-15";
String lon = "4947.18N"; //4947.18NI01317.10E
String lat = "01317.10E";
String sym = "L";
String tool = " OK5TVR_RX_Igate_LoRA";
float Alt = 324.0;
String aprs_filter ="";

//---------------aprs setup--------------
char servername[] = "czech.aprs2.net";
long aprs_port = 14580;
String user = call;
String password_aprs = "xxxxxxx";

///-------------IP adress------------
// Pokud chcete použít pevnou IP adresu
bool pouzitPevnouIP = true;
String ipString = "192.168.1.189";

#define BUFFER_SIZE 5
char buffer[BUFFER_SIZE][10]; // 5 řetězců o maximální délce 20 znaků call
char buffer_SN[BUFFER_SIZE][10]; // 5 řetězců o maximální délce 20 znaků S/N
char buffer_RSSI[BUFFER_SIZE][10]; // 5 řetězců o maximální délce 20 znaků RSSI
char buffer_pak[BUFFER_SIZE][256]; // 5 řetězců o maximální délce 256 znaků pakety
char buffer_cas[BUFFER_SIZE][10]; // 5 řetězců o maximální délce 20 znaků cas odeslání
char buffer_vzdalenost[BUFFER_SIZE][10];
char buffer_azimut[BUFFER_SIZE][10];
char buffer_icona[BUFFER_SIZE][400];

byte cnt = 0;

#define OLED_SDA 21
#define OLED_SCL 22 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 189);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 1, 1);   //optional
IPAddress secondaryDNS(192, 168, 1, 1); //optional

WiFiClient client;
WiFiUDP ntpUDP;


NTPClient timeClient(ntpUDP, "0.cz.pool.ntp.org", 3600, 60000);
AsyncWebServer server(80);


///-------------------aprs symboly base64
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

long cas_dif =0;
float temp_cpu = 0;
double latitude2 = 0;
double longitude2 = 0;
////-----------nastaveni z config.txt------------
const int MAX_SETTINGS = 17; // Maximální počet proměnných
String fileContent = "";
String nastaveni[MAX_SETTINGS]; // Pole pro uložení proměnných

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<meta charset="windows-1250">
<title>LoRa RX IGate</title>
<meta name="viewport content=width=device-width,initial-scale=1">
<style>
h2 {font-size: 1.8rem; color: white; text-align: center;}
h4 { font-size: 1rem;}
h3 { font-size: 1rem;}
.topnav { overflow: hidden; background-color: #1b78e2;}
.card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);text-align: center;}
.cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));}
</style>
</head/<body>
<div class="topnav">
<h2>LoRa RX IGate - %CALL%</h2>
</div>
</br>
<div class="card"><h3>
<center>
<table border="0">
    <tr>
        <td colspan="2">%DATUM%</td>
       
    </tr>
    <tr>
        <td>Poloha:</td>
        <td>%lon% %lat%</td>
    </tr>
        <tr>
        <td>Nadmorska vyska:</td>
        <td>%vys% m.n.m.</td>
    </tr>
</table>
</center>
</h3></div>

</center>
</h3></div>

<div class="card" id="pak1">
  <h4><img src="data:image/jpeg;base64,%icona64_0%" alt="Base64 obrázek"> %PAK1% - %vz0% km - %az0% &deg;</h4>
  <button onclick="toggleTable()">Zobrazit data</button>
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
  <h4><img src="data:image/jpeg;base64,%icona64_1%" alt="Base64 obrázek">%PAK2% - %vz1% km - %az1% &deg;</h4>
  <button onclick="toggleTable1()">Zobrazit data</button>
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
  <h4><img src="data:image/jpeg;base64,%icona64_2%" alt="Base64 obrázek">%PAK3% - %vz2% km - %az2% &deg;</h4>
  <button onclick="toggleTable2()">Zobrazit data</button>
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
  <h4><img src="data:image/jpeg;base64,%icona64_3%" alt="Base64 obrázek">%PAK4% - %vz3% km - %az3% &deg;</h4>
  <button onclick="toggleTable3()">Zobrazit data</button>
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
  <h4><img src="data:image/jpeg;base64,%icona64_4%" alt="Base64 obrázek">%PAK5% - %vz4% km - %az4% &deg;</h4>
  <button onclick="toggleTable4()">Zobrazit data</button>
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
<p> Pripojeni: </p>
<p>RSSI: %RSSI% dBm</p>
<p>IP: %IP_adr%</p>
<p>cas dif: %cas_dif%</p>
<p>URL: %URL1%</p>
</body></html>
)rawliteral";

void notFound(AsyncWebServerRequest *request){
  request->send(404, "text/plain", "Not found");
}

String procesor (const String& var){
  Serial.println(var);
if (var == "lat"){
 return String(lat);
 }
 if (var == "lon"){
 return String(lon);
 }
 if (var == "vys"){
 return String(Alt);
 }
else if (var == "CALL"){
    return String(call);
  }
else if (var == "RSSI"){
    return String(WiFi.RSSI());
  }
else if (var == "IP_adr"){
    return String(IP);
  }
  else if (var == "PAK1"){   ///------ pa1
    return String(buffer[0]);
  }
  else if (var == "T0"){
    return String(buffer_cas[0]);
  }
  else if (var == "RS0"){
    return String(buffer_RSSI[0]);
  }
   else if (var == "SN0"){
    return String(buffer_SN[0]);
  }
  else if (var == "PA0"){
    return String(buffer_pak[0]);
  }
   else if (var == "PAK2"){         ////----- pa2
    return String(buffer[1]);
  }
    else if (var == "T1"){
    return String(buffer_cas[1]);
  }
  else if (var == "RS1"){
    return String(buffer_RSSI[1]);
  }
   else if (var == "SN1"){
    return String(buffer_SN[1]);
  }
  else if (var == "PA1"){
    return String(buffer_pak[1]);
  }
   else if (var == "PAK3"){  ////----- pa3
    return String(buffer[2]);
  }
    else if (var == "T2"){
    return String(buffer_cas[2]);
  }
  else if (var == "RS2"){
    return String(buffer_RSSI[2]);
  }
   else if (var == "SN2"){
    return String(buffer_SN[2]);
  }
  else if (var == "PA2"){
    return String(buffer_pak[2]);
  }
   else if (var == "PAK4"){  ////------pa4
    return String(buffer[3]);
  }
    else if (var == "T3"){
    return String(buffer_cas[3]);
  }
  else if (var == "RS3"){
    return String(buffer_RSSI[3]);
  }
   else if (var == "SN3"){
    return String(buffer_SN[3]);
  }
  else if (var == "PA3"){
    return String(buffer_pak[3]);
  }
   else if (var == "PAK5"){  ////------pa5
    return String(buffer[4]);
  }
    else if (var == "T4"){
    return String(buffer_cas[4]);
  }
  else if (var == "RS4"){
    return String(buffer_RSSI[4]);
  }
   else if (var == "SN4"){
    return String(buffer_SN[4]);
  }
  else if (var == "PA4"){
    return String(buffer_pak[4]);
  }
    else if (var == "cas_dif"){
    return String(cas_dif);
  }
   else if (var == "vz0"){
    return String(buffer_vzdalenost[0]);
  }
     else if (var == "vz1"){
    return String(buffer_vzdalenost[1]);
  }
     else if (var == "vz2"){
    return String(buffer_vzdalenost[2]);
  }
     else if (var == "vz3"){
    return String(buffer_vzdalenost[3]);
  }
     else if (var == "vz4"){
    return String(buffer_vzdalenost[4]);
  }
     else if (var == "az0"){
    return String(buffer_azimut[0]);
  }
       else if (var == "az1"){
    return String(buffer_azimut[1]);
  }
       else if (var == "az2"){
    return String(buffer_azimut[2]);
  }
       else if (var == "az3"){
    return String(buffer_azimut[3]);
  }
       else if (var == "az4"){
    return String(buffer_azimut[4]);
  }
  else if (var == "icona64_0"){
    return String(buffer_icona[0]);
  }
    else if (var == "icona64_1"){
    return String(buffer_icona[1]);
  }
    else if (var == "icona64_2"){
    return String(buffer_icona[2]);
  }
    else if (var == "icona64_3"){
    return String(buffer_icona[3]);
  }
    else if (var == "icona64_4"){
    return String(buffer_icona[4]);
  }
    else if (var == "URL1"){
    return String(temp_cpu);
  }
return String();
}

void wifi();
void con_aprs();



//--cas--
long cas_new = 0;
long cas_old = 0;
long cas_reset = 0;
long cas_telemetry = 0;

//------telemetrie--------
long rx_cnt = 0;
long rf_inet = 0;
long live = 1;
String live_s = "001";
long dx_dist = 0;

// --- lora ---
const int lora_SS = 18;    // LoRa radio chip select
const int lora_RST = 23;   // LoRa radio reset
const int lora_DIO0 = 26;  // LoRa radio DIO0
const int lora_DIO1 = 33;  // LoRa radio DIO1
const int lora_DIO2 = 32;  // LoRa radio DIO2

// --- převod gps na double -----
double convertToDecimalDegrees_la(const String& gpsString) {
  // Extrahování stupňů a minut z řetězce
  double degrees = gpsString.substring(0, 2).toDouble();
  double minutes = gpsString.substring(2).toDouble();

  // Výpočet zeměpisných souřadnic ve stupních (desetinné číslo)
  double decimalDegrees = degrees + (minutes / 60.0);

  return decimalDegrees;
}

double convertToDecimalDegrees_lo(const String& gpsString) {
  // Extrahování stupňů a minut z řetězce
  double degrees = gpsString.substring(1, 3).toDouble();
  double minutes = gpsString.substring(3).toDouble();

  // Výpočet zeměpisných souřadnic ve stupních (desetinné číslo)
  double decimalDegrees = degrees + (minutes / 60.0);

  return decimalDegrees;
}
/// ----- výpočty z pozic
// Funkce pro převod stupňů na radiány
double toRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}
// Funkce pro převod radiánů na stupn
double toDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

// Funkce pro výpočet vzdálenosti mezi dvěma geografickými pozicemi
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371.0; // Poloměr Země v kilometrech

  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  double a = sin(dLat / 2) * sin(dLat / 2) + cos(toRadians(lat1)) * cos(toRadians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  double distance = R * c; // Vzdálenost mezi pozicemi v kilometrech

  return distance;
}

// Funkce pro výpočet azimutu (směru) mezi dvěma geografickými pozicemi
double calculateAzimuth(double lat1, double lon1, double lat2, double lon2) {
  double dLon = toRadians(lon2 - lon1);

  double y = sin(dLon) * cos(toRadians(lat2));
  double x = cos(toRadians(lat1)) * sin(toRadians(lat2)) - sin(toRadians(lat1)) * cos(toRadians(lat2)) * cos(dLon);

  double azimuth = atan2(y, x);
  azimuth = fmod((azimuth + 2 * M_PI), (2 * M_PI)); // Převod na rozsah 0-2π

  azimuth = toDegrees(azimuth); // Převod na stupně

  return azimuth;
}

///---------ověřuje spránost paketů 
bool isASCII(const String& str) {
  for (size_t i = 0; i < str.length(); i++) {
    if (str.charAt(i) > 127) {
      return false;
    }
  }
  return true;
}

//// -- interní teplota -------
#ifdef __cplusplus

extern "C" {

#endif

uint8_t temprature_sens_read();

#ifdef __cplusplus

}

#endif

uint8_t temprature_sens_read();

void setup() {

pinMode(lora_DIO0, INPUT);

Serial.begin(9600);
delay(500);

///---------------------spiffs-------------------
Serial.println("nastavuji z config.txt");

if(!SPIFFS.begin(true)){
    Serial.println("chyba SPIFFS");
    return;
  }

///------------------test cteni souboruí nastaveni----------
  File file = SPIFFS.open("/config.txt");
  if(!file){
    Serial.println("Soubor config.txt neni mozne precist");
    return;
  }
  
  Serial.println("Soubor konfigurace:");
  fileContent = "";
  while (file.available()) {
    fileContent += char(file.read());
  }
  file.close();
  Serial.println(fileContent);

  //------------- Hledání proměnných v textu--------------------------------------
 if (fileContent.length()>1){
 nastaveni[1] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[2] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[3] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[4] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[5] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
  fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[6] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[7] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
  fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[8] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[9] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[10] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[11] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
  fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[12] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[13] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
  fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[14] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[15] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
  fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[16] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
 fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[17] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));
fileContent = fileContent.substring(fileContent.indexOf(">")+1,fileContent.indexOf("!"));
 nastaveni[18] =fileContent.substring(fileContent.indexOf("<")+1,fileContent.indexOf(">"));

 Serial.println("promene:");
 Serial.println(nastaveni[1]);
 Serial.println(nastaveni[2]);
 Serial.println(nastaveni[3]);
 Serial.println(nastaveni[4]);
 Serial.println(nastaveni[5]);
 Serial.println(nastaveni[6]);
 Serial.println(nastaveni[7]);
 Serial.println(nastaveni[8]);
 Serial.println(nastaveni[9]);
 Serial.println(nastaveni[10]);
 Serial.println(nastaveni[11]);
 Serial.println(nastaveni[12]);
 Serial.println(nastaveni[13]);
 Serial.println(nastaveni[14]);
 Serial.println(nastaveni[15]);
 Serial.println(nastaveni[16]);
 Serial.println(nastaveni[17]);
Serial.println(nastaveni[18]);

ssid = nastaveni[1].c_str();
password = nastaveni[2].c_str();

call = nastaveni[3].c_str();
lon = nastaveni[4].c_str(); 
lat = nastaveni[5].c_str();
sym = nastaveni[6].c_str();
tool = nastaveni[7].c_str();
Alt = nastaveni[8].toFloat();
aprs_filter =nastaveni[9].c_str();

nastaveni[10].toCharArray(servername,nastaveni[10].length()+1);

aprs_port = nastaveni[11].toInt();
user = call;
password_aprs = nastaveni[12].c_str();

pouzitPevnouIP = nastaveni[13].c_str();

// Set your Static IP address
uint8_t octed1;
uint8_t octed2;
uint8_t octed3;
uint8_t octed4;

int dot1 = nastaveni[14].indexOf(".");
int dot2 = nastaveni[14].indexOf(".",dot1 + 1);
int dot3 = nastaveni[14].indexOf(".",dot2 + 1);

octed1 = nastaveni[14].substring(0,dot1).toInt();
octed2 = nastaveni[14].substring(dot1+1,dot2).toInt();
octed3 = nastaveni[14].substring(dot2+1,dot3).toInt();
octed4 = nastaveni[14].substring(dot3+1).toInt();

IPAddress local_IP(octed1, octed2, octed3, octed4);

// Set your Gateway IP address
dot1 = nastaveni[15].indexOf(".");
dot2 = nastaveni[15].indexOf(".",dot1 + 1);
dot3 = nastaveni[15].indexOf(".",dot2 + 1);

octed1 = nastaveni[15].substring(0,dot1).toInt();
octed2 = nastaveni[15].substring(dot1+1,dot2).toInt();
octed3 = nastaveni[15].substring(dot2+1,dot3).toInt();
octed4 = nastaveni[15].substring(dot3+1).toInt();
IPAddress gateway(octed1, octed2, octed3, octed4);

dot1 = nastaveni[16].indexOf(".");
dot2 = nastaveni[16].indexOf(".",dot1 + 1);
dot3 = nastaveni[16].indexOf(".",dot2 + 1);

octed1 = nastaveni[16].substring(0,dot1).toInt();
octed2 = nastaveni[16].substring(dot1+1,dot2).toInt();
octed3 = nastaveni[16].substring(dot2+1,dot3).toInt();
octed4 = nastaveni[16].substring(dot3+1).toInt();
IPAddress subnet(octed1, octed2, octed3, octed4);

dot1 = nastaveni[17].indexOf(".");
dot2 = nastaveni[17].indexOf(".",dot1 + 1);
dot3 = nastaveni[17].indexOf(".",dot2 + 1);

octed1 = nastaveni[17].substring(0,dot1).toInt();
octed2 = nastaveni[17].substring(dot1+1,dot2).toInt();
octed3 = nastaveni[17].substring(dot2+1,dot3).toInt();
octed4 = nastaveni[17].substring(dot3+1).toInt();

IPAddress primaryDNS(octed1, octed2, octed3, octed4);   //optional
IPAddress secondaryDNS(octed1, octed2, octed3, octed4); //optional

Serial.println("nastaveno");
}



Serial.print("Start digi_RX wifi");

////-----------nastavení pevná vs dhcp ip adresa-------
if (pouzitPevnouIP) {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  } else {
    Serial.print("Používá se IP adresa přidělená pomocí DHCP: ");
  }



Wire.begin(OLED_SDA, OLED_SCL);

if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
Serial.println(F("SSD1306 error"));
for(;;); // Don't proceed, loop forever
}

//display.ssd1306_command(0xAF); // display on

display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(2);
display.setCursor(0,0);
display.print(call);
display.setTextSize(1);
display.setCursor(5,27);
display.print(F("RX LoRa"));
display.setCursor(5,36);
display.print(F("(c) OK5TVR"));
display.setCursor(5,46);
display.print(F("ver.: "));
display.setCursor(40,46);
display.print(verze);
display.display();
delay(3000);

wifi();


LoRa.setPins(lora_SS, lora_RST, lora_DIO0); // set CS, reset, IRQ pin
if (!LoRa.begin(433775000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  LoRa.setCodingRate4(5);
  //LoRa.enableCrc();
  LoRa.disableCrc();
  Serial.println("LoRa started successfully.");

timeClient.begin();
timeClient.update();
cas_new = timeClient.getEpochTime();
cas_old = timeClient.getEpochTime();
cas_reset = timeClient.getEpochTime();
cas_telemetry = timeClient.getEpochTime();

AsyncElegantOTA.begin(&server);
server.begin();
con_aprs(); 

server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send_P(200, "text/html",index_html,procesor);
});

//server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
//  request->send(200, "text/plain", "update");;
//});





 display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(2);
display.setCursor(0,0);
display.print(call);
display.setTextSize(1);
display.setCursor(5,27);
display.print(F("RX LoRa"));
display.setCursor(5,36);
display.print(F("ALL OK"));
display.setCursor(5,46);
display.print(F("ver.: "));
display.setCursor(40,46);
display.print(verze);
display.setCursor(5,55);
display.print(F("IP"));
display.setCursor(20,55);
display.print(IP);
display.display();

}

void loop() {
jump:
AsyncElegantOTA.loop();
String paket ="";
String paket1 ="";
String gpslan = "";
String gpslon ="";
String icona ="";

timeClient.update();
cas_new = timeClient.getEpochTime();
cas_dif = cas_new-cas_old;
if (WiFi.status() == WL_CONNECTED){
}
else {
  Serial.println ("wifi no connect....");
  wifi();
}
if(cas_new-cas_reset>7200){
  esp_restart();
}

if(cas_new-cas_telemetry>900){
  temp_cpu = (temprature_sens_read() - 32) / 1.8;
  Serial.print ("teplota int: ");
  Serial.println (temp_cpu);
  if (client.connected()) {
  live_s="";
  if(live<100) { live_s += "0"; }
  if(live<10)  { live_s+= "0"; }
  live_s = live_s + live;
  client.println(call + ">APZ023,TCPIP*:T#"+live_s+","+rx_cnt+","+rf_inet+","+dx_dist+","+live+","+temp_cpu+",00000000");
  client.flush();
  live = live + 1;
  rx_cnt = 0;
  rf_inet = 0;
  dx_dist = 0;
  if (live >= 96){
    live=0;
    }
  cas_telemetry = timeClient.getEpochTime();
  } else {
    Serial.println("Spojení s APRS serverem bylo přerušeno");
    // Připojení znovu
    client.stop();
    con_aprs();
     live_s="";
  if(live<100) { live_s += "0"; }
  if(live<10)  { live_s+= "0"; }
  live_s = live_s + live;
  client.println(call + ">APZ023,TCPIP*:T#"+live_s+","+rx_cnt+","+rf_inet+","+dx_dist+","+live+","+temp_cpu+",00000000");
  client.flush();
  live = live + 1;
  rx_cnt = 0;
  rf_inet = 0;
  dx_dist = 0;
  if (live >= 96){
    live=0;
    }
  cas_telemetry = timeClient.getEpochTime();
  }
}
if((cas_new - cas_old) > 1200){
if (client.connected()) {
  
  client.println(call + ">APZ023,TCPIP*:!" + lon + sym + lat +"&PHG01000/"+tool); 
  client.flush();
  while (client.available()) {
      String response = client.readStringUntil('\n');
      Serial.println("Odpověď od APRS serveru:");
      Serial.println(response);
    }
  } else {
    Serial.println("Spojení s APRS serverem bylo přerušeno");
    // Připojení znovu
    client.stop();
    con_aprs();
  }
cas_old = timeClient.getEpochTime();
}

int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    // WebSerial.print("Received packet: ");

    while (LoRa.available()) {
      
      paket = paket + (char)LoRa.read();
     // WebSerial.print((char)LoRa.read());
    }
    Serial.println(paket);

    Serial.print(" with RSSI ");
    // WebSerial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
    // WebSerial.println(LoRa.packetRssi());

      Serial.print(" with SN");
    // WebSerial.print(" with RSSI ");
    Serial.println(LoRa.packetSnr());
    // WebSerial.println(LoRa.packetRssi());

     //---- uprava pro rf to inet
    //paket.replace("*", "");
    paket.trim();
    int colonIndex = paket.indexOf(":");
    if(colonIndex >= 0) {
    paket = paket.substring(0, colonIndex) +",qAS," +call + paket.substring(colonIndex)+ " DS " + LoRa.packetSnr() + " RS " + LoRa.packetRssi() +"\n";
    paket = paket.substring(3);
    //paket = "}" + paket;
    //paket.replace(",", ">");
  }
    //---otestuje zda pokat obsahuje pouze asci znaky
  if (isASCII(paket)) {
    Serial.println("Řetězec obsahuje pouze znaky z ASCII tabulky.");
    rx_cnt = rx_cnt + 1;
    rf_inet = rf_inet + 1;
  } else {
    Serial.println("Řetězec obsahuje znaky mimo rozsah ASCII tabulky.");
      // Kód, na který skočíme
    Serial.println("Skok na řádek s error paket");
    rx_cnt = rx_cnt + 1;
    goto jump;
  }



    Serial.println(timeClient.getFormattedTime());
    Serial.println(">>>" + paket);

if (client.connected()) {
  
  client.println(paket); 
  client.flush();
  while (client.available()) {
      String response = client.readStringUntil('\n');
      Serial.println("Odpověď od APRS serveru:");
      Serial.println(response);
    }
  } else {
    Serial.println("Spojení s APRS serverem bylo přerušeno PA " + client.connected());
    // Připojení znovu
    client.stop();
    con_aprs();
  }

     int pos = paket.indexOf(">");
    // Pokud je pozice nalezena
     if (pos != -1) {
    // Vytvoření nového řetězce pouze s volacím znakem
    String call_d = paket.substring(0, pos);
    
    strcpy(buffer[cnt], call_d.c_str());
    strcpy(buffer_pak[cnt],paket.c_str());
    strcpy(buffer_cas[cnt],timeClient.getFormattedTime().c_str());

    char buffer_RSSIa[10];
    dtostrf(LoRa.packetRssi(), 1, 1, buffer_RSSIa);

    char buffer_SNa[10];
    dtostrf(LoRa.packetSnr(), 1, 1, buffer_SNa);

    strcpy(buffer_RSSI[cnt], buffer_RSSIa);
    strcpy(buffer_SN[cnt], buffer_SNa);


    ////--------------------dekodování pozice--------------
  
  // Zde dekódujte APRS paket
  
  
       int startIndex = paket.indexOf(':'); // Index znaku ':' označující začátek tela paketu
      if (paket.indexOf('@') == -1) {     // paket nemá hodiny
        if ((paket.substring(startIndex + 2, startIndex + 3)=="/") || (paket.substring(startIndex + 2, startIndex + 3)=="\\")){
        Serial.println("comprese pozice");
        icona =  paket.substring(startIndex + 2, startIndex + 3) + paket.substring(startIndex + 11, startIndex + 12);
        Serial.println(icona);

        String k_lan =paket.substring(startIndex + 3, startIndex + 7);
        String k_lon =paket.substring(startIndex + 7, startIndex + 11);
        Serial.println(k_lon);
        Serial.println(k_lan);
  
        // Compute lat/long z komprimovaných dat
        latitude2 = 90.0-((((((((k_lan.charAt(0)-33)*91.0)+(k_lan.charAt(1)-33))*91.0)+(k_lan.charAt(2)-33))*91.0)+(k_lan.charAt(3)-33))/380926.0);
        longitude2 = -180.0+((((((((k_lon.charAt(0)-33)*91.0)+(k_lon.charAt(1)-33))*91.0)+(k_lon.charAt(2)-33))*91.0)+(k_lon.charAt(4)-33))/190463.0);
        
        }
        else {
         gpslan = paket.substring(startIndex + 2, startIndex + 9);
         gpslon = paket.substring(startIndex + 11, startIndex + 19);
         icona =  paket.substring(startIndex + 10, startIndex + 11) + paket.substring(startIndex + 20, startIndex + 21); 
          Serial.println("");
          Serial.println(gpslon);
          Serial.println(gpslan);
          Serial.println(icona);
          latitude2 = convertToDecimalDegrees_la(gpslan.c_str());
          longitude2 = convertToDecimalDegrees_lo(gpslon.c_str());
      }   
      }      

    ///---- výpočet vzdálenosti
      
      double latitude1 = convertToDecimalDegrees_la(lon.c_str());
      double longitude1 = convertToDecimalDegrees_lo(lat.c_str());
 
    ///--- výpočtyazimutu a vzdálenosti
      double distance = calculateDistance(latitude1, longitude1, latitude2, longitude2);
      double azimuth = calculateAzimuth(latitude1, longitude1, latitude2, longitude2);
      Serial.println("");
      Serial.print(distance); 
      Serial.println(" km");
      Serial.print(azimuth); 
      Serial.println(" stupen");

       // Převod na string
     char buffer_vzdalenosta[10];
    dtostrf(distance, 1, 2, buffer_vzdalenosta);
    
    if (distance > dx_dist) {             ////------------------dx dist ---- telemetrie
      dx_dist = round(distance);
      if (dx_dist > 2000){dx_dist=0;};
    }

    char buffer_azimuta[10];
    dtostrf(azimuth, 1, 1, buffer_azimuta);

    strcpy(buffer_vzdalenost[cnt], buffer_vzdalenosta);
    strcpy(buffer_azimut[cnt], buffer_azimuta);

    ////-----------icona stanice ---------
    if (icona == "/#") { strcpy(buffer_icona[cnt], digi.c_str());};
    if (icona == "R#") { strcpy(buffer_icona[cnt], digi.c_str());};
    if (icona == "/r") { strcpy(buffer_icona[cnt], digi.c_str());};
    if (icona == "1#") { strcpy(buffer_icona[cnt], digi.c_str());};
    if (icona == "/>") { strcpy(buffer_icona[cnt], car.c_str());};
    if (icona == "L&") { strcpy(buffer_icona[cnt], lgate.c_str());};
    if (icona == "I&") { strcpy(buffer_icona[cnt], igate.c_str());};
    if (icona == "/_") { strcpy(buffer_icona[cnt], wx.c_str());};
    if (icona == "/[") { strcpy(buffer_icona[cnt], chodec.c_str());};
    if (icona == "//") { strcpy(buffer_icona[cnt], red_dot.c_str());};
    if (icona == "/a") { strcpy(buffer_icona[cnt], sanita.c_str());};
    if (icona == "/b") { strcpy(buffer_icona[cnt], kolo.c_str());};
    if (icona == "/'") { strcpy(buffer_icona[cnt], air.c_str());};

   ///--------- vymazání času pro reset ESP32
      cas_reset = timeClient.getEpochTime();
   ///---pocet pro buffer----
    cnt = cnt + 1;
    if (cnt==5) {
      cnt=0;
      }

      

      ///--- zobrazení oled
display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(1);
display.setCursor(0,0);
display.print(call);
display.setTextSize(1);
display.setCursor(2,9);
display.print(IP);
display.setCursor(2,18);
display.print(buffer[0]);
display.setCursor(60,18);
display.print(buffer_RSSI[0]);
display.setCursor(95,18);
display.print(buffer_SN[0]);
display.setCursor(2,27);
display.print(buffer[1]);
display.setCursor(60,27);
display.print(buffer_RSSI[1]);
display.setCursor(95,27);
display.print(buffer_SN[1]);
display.setCursor(2,36);
display.print(buffer[2]);
display.setCursor(60,36);
display.print(buffer_RSSI[2]);
display.setCursor(95,36);
display.print(buffer_SN[2]);
display.setCursor(2,46);
display.print(buffer[3]);
display.setCursor(60,46);
display.print(buffer_RSSI[3]);
display.setCursor(95,46);
display.print(buffer_SN[3]);
display.setCursor(2,55);
display.print(buffer[4]);
display.setCursor(60,55);
display.print(buffer_RSSI[4]);
display.setCursor(95,55);
display.print(buffer_SN[4]);
display.display();

    }  
  }
}

void con_aprs() {
Serial.println("\nStarting connection...");
    // if you get a connection, report back via serial:
    
    if (client.connect(servername, aprs_port)) {
      Serial.println("connected");
      // Make a HTTP request:
      client.println("");
      client.println("user " + user + " pass " + password_aprs + " vers " + " OK5TVR_RX_Igate_LoRA");
      client.flush();
      client.println("");
      client.println(call + ">APZ023,TCPIP*:!" + lon + sym + lat +"&PHG01000/" + tool);
      client.flush();
      //live_s="";
  //if(live<100) { live_s += "0"; }
 // if(live<10)  { live_s+= "0"; }
 // live_s = live_s + live;
      //client.println(call + ">APZ023,TCPIP*:T#"+live_s+","+rx_cnt+","+rf_inet+","+dx_dist+","+live+",0,00000000");
      //Serial.println(call + ">APZ023,TCPIP*:T#"+live_s+","+rx_cnt+","+rf_inet+","+dx_dist+","+live+",0,00000000");
     // client.flush();
      client.println(call + ">APZ023,TCPIP*"+"::"+call+":PARM.RxPkts,RF->Inet,DX_dist.,live,Temp_core");
      client.flush();
      client.println(call + ">APZ023,TCPIP*"+"::"+call+":UNIT.Pkts,Pkts,km,min.,C");
      client.flush();
      client.println(call + ">APZ023,TCPIP*"+"::"+call+":EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0");
      client.flush();
      }
}

void wifi(){
    WiFi.setHostname("lora_RX_igate");
    WiFi.hostname("lora_RX_igate");
    WiFi.begin(ssid, password);
   
    

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

IP = WiFi.localIP().toString();
Serial.println (WiFi.localIP());
}

