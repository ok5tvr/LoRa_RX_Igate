#ifndef SETUP_TRACKER_H
#define SETUP_TRACKER_H

/*
  setup_tracker.h  —  APRS Tracker modul (POST toggle varianta)

  Funkce:
   - tracker_setup(AsyncWebServer* srv)  : registruje /tracker (pokud srv!=nullptr),
                                           načte / vytvoří /tracker.cfg, inicializuje GPS
   - tracker_loop()                      : čte NMEA, SmartBeacon rozhodování, TX, OLED

  Závislosti (v hlavním kódu):
    #include <Arduino.h>
    #include <LoRa.h>
    #include <ESPAsyncWebServer.h>
    #include <Adafruit_SSD1306.h>
    #include <NTPClient.h>
    #include <SPIFFS.h>

  Externí symboly:
    extern AsyncWebServer server;
    extern Adafruit_SSD1306 display;
    extern NTPClient timeClient;
    extern WiFiClient client;
    extern String web_username, web_password;
    extern String call;
    extern String lastStation;
    extern int digi_mode;      // 0=iGate, 1=Digi
    extern int digi_AP;        // 1=AP + Digi
    extern const int PLED1;    // LED pin
    extern double calculateDistance(double lat1,double lon1,double lat2,double lon2);

  Poznámky:
    - V iGate režimu (digi_mode==0 && digi_AP==0) tracker NIKDY neTX (jen UI/NMEA/OLED).
    - V Digi nebo AP+Digi tracker vysílá přes LoRa.
*/

#include <HardwareSerial.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// ---------- Konfigurace / výchozí hodnoty ----------
#ifndef GPS_RX_PIN_DEFAULT
  #define GPS_RX_PIN_DEFAULT 13
#endif
#ifndef GPS_TX_PIN_DEFAULT
  #define GPS_TX_PIN_DEFAULT 14
#endif
#ifndef GPS_BAUD_DEFAULT
  #define GPS_BAUD_DEFAULT   9600
#endif

#define TRACKER_CFG_PATH     "/tracker.cfg"
#define TRACKER_OLED_ENABLED 1

// kolik ms bez NMEA než zahlásíme výpadek
#define GPS_SILENCE_MS       8000UL
// perioda update OLED (ms)
#define OLED_REFRESH_MS      500UL

// --- monitoring po načtení stránky (čisté čtení bez TX) ---
#define TRACKER_MONITOR_MS   60000UL   // 60 s

// ---------- Stav GPS/Tracker ----------
struct TrackerCfg {
  int      gpsRxPin       = GPS_RX_PIN_DEFAULT;
  int      gpsTxPin       = GPS_TX_PIN_DEFAULT;
  uint32_t gpsBaud        = GPS_BAUD_DEFAULT;

  // vysílání
  bool     enabled        = false;     // poslední stav (pamatuje se a obnoví při startu)
  bool     compressed     = true;      // komprimovaná poloha
  char     tableCh        = '/';       // '/' nebo '\\'
  char     symCh          = '>';       // symbol
  int      commentMode    = 1;         // 0=vlastní comment, 1=rychlost+výška
  String   comment        = "";        // vlastní komentář
  String   trackerCall    = "";        // prázdné = použij globální call

  // SmartBeacon
  bool     sbActive       = true;
  uint16_t slowRate       = 120;       // s
  float    slowSpeed      = 10.0f;     // km/h
  uint16_t fastRate       = 60;        // s
  float    fastSpeed      = 70.0f;     // km/h
  uint16_t minTxDist      = 100;       // m
  uint16_t minDeltaBeacon = 12;        // s
  float    turnMinDeg     = 10.0f;     // °
  float    turnSlope      = 80.0f;     // (° * h / km)
};

static TrackerCfg _cfg;

// stavové proměnné (runtime)
static volatile bool   _running        = false;
static HardwareSerial  GPS(2);
static String          _lastRMC = "";
static String          _lastGGA = "";
static uint32_t        _lastNmeaMs = 0;

static bool            _haveFix = false;
static double          _lat = 0.0, _lon = 0.0;      // dec deg
static double          _altM = 0.0;                 // m
static double          _speedKmh = 0.0;             // km/h
static double          _courseDeg = NAN;            // degrees

static uint32_t        _lastTxMs = 0;
static double          _lastTxLat = 0.0, _lastTxLon = 0.0;
static double          _lastTxCourse = NAN;
static uint32_t        _lastOledMs = 0;

// --- monitoring (čtení GPS po otevření stránky bez TX) ---
static volatile bool     _monitor         = false;
static uint32_t          _monitorUntilMs  = 0;

// --- SSE (Server-Sent Events) pro živé NMEA ---
static AsyncEventSource  _sse("/api/gps"); // pozor na kolize se statikou

//----externí promenné-------
extern String lastStation;
extern double lastStationLat;   // << přidáno
extern double lastStationLon;   // << přidán

// ---------- Pomocné funkce ----------
static double _toRad(double d){ return d * (M_PI/180.0); }
static double _norm360(double a){ while(a<0) a+=360; while(a>=360) a-=360; return a; }
static double _angleDelta(double a,double b) {
  // nejmenší rozdíl směrů (0..180)
  double d = fabs(_norm360(a) - _norm360(b));
  return d>180 ? 360.0 - d : d;
}

// ddmm.mm + N/S
static String _fmtLat(double lat) {
  char buf[16];
  char NS = (lat>=0) ? 'N' : 'S';
  lat = fabs(lat);
  int deg = (int)lat;
  double minutes = (lat - deg) * 60.0;
  snprintf(buf, sizeof(buf), "%02d%05.2f%c", deg, minutes, NS);
  return String(buf);
}

// dddmm.mm + E/W
static String _fmtLon(double lon) {
  char buf[16];
  char EW = (lon>=0) ? 'E' : 'W';
  lon = fabs(lon);
  int deg = (int)lon;
  double minutes = (lon - deg) * 60.0;
  snprintf(buf, sizeof(buf), "%03d%05.2f%c", deg, minutes, EW);
  return String(buf);
}

// Bezpečné komprimované kódování (4+4 base91)
static void _encodeCompressed(double lat, double lon, char out8[9]) {
  // Clamp podle APRS spec (vyhnout se přesně 90/180 kvůli přetečení rozsahu)
  if (isnan(lat) || isnan(lon)) { for (int i=0;i<8;i++) out8[i]='!'; out8[8]='\0'; return; }
  if (lat >  89.999999) lat =  89.999999;
  if (lat < -89.999999) lat = -89.999999;
  if (lon > 179.999999) lon = 179.999999;
  if (lon < -179.999999) lon = -179.999999;

  // Y = 90 - lat; X = 180 + lon
  double Y = 90.0 - lat;
  double X = 180.0 + lon;

  // Přepočet na celočíselné mřížky dle spec
  uint32_t y = (uint32_t)floor(380926.0 * Y + 0.5);  // zaokrouhlení pomáhá stabilitě
  uint32_t x = (uint32_t)floor(190463.0 * X + 0.5);

  // 4 znaky lat (y), 4 znaky lon (x), každý 0..90 → +33 do ASCII
  for (int i = 3; i >= 0; --i) { out8[i]   = (char)((y % 91U) + 33); y /= 91U; }
  for (int i = 7; i >= 4; --i) { out8[i]   = (char)((x % 91U) + 33); x /= 91U; }
  out8[8] = '\0';
}


// rychlý parser CSV pole
static int _splitCSV(const String& s, String* f, int maxf) {
  int cnt=0, start=0;
  for (int i=0; i<=s.length(); i++){
    if (i==s.length() || s[i]==',') {
      if (cnt<maxf) f[cnt++] = s.substring(start, i);
      start = i+1;
    }
  }
  return cnt;
}

// --- uložení/načtení /tracker.cfg (3 řádky s <> tokeny) ---
static String _nextToken(String &src) {
  int a = src.indexOf('<');
  int b = src.indexOf('>');
  if (a == -1 || b == -1 || b <= a) return "";
  String t = src.substring(a+1, b);
  src.remove(0, b+1);
  return t;
}

static void _saveCfg() {
  // Řádek 1: <rx><tx><baud><legacy><compressed><table><sym><commentMode><comment>
  String l1;
  l1 += "<"+String(_cfg.gpsRxPin)+">";
  l1 += "<"+String(_cfg.gpsTxPin)+">";
  l1 += "<"+String(_cfg.gpsBaud)+">";
  l1 += "<30>";
  l1 += "<"+String(_cfg.compressed?1:0)+">";
  l1 += "<"; l1 += _cfg.tableCh; l1 += ">";
  l1 += "<"; l1 += _cfg.symCh;   l1 += ">";
  l1 += "<"+String(_cfg.commentMode)+">";
  l1 += "<"+_cfg.comment+">";

  // Řádek 2: SmartBeacon <active><slowRate><slowSpeed><fastRate><fastSpeed><minTXdist><minDeltaBeacon><turnMinDeg><turnSlope>
  String l2;
  l2 += "<"+String(_cfg.sbActive?1:0)+">";
  l2 += "<"+String(_cfg.slowRate)+">";
  l2 += "<"+String(_cfg.slowSpeed,1)+">";
  l2 += "<"+String(_cfg.fastRate)+">";
  l2 += "<"+String(_cfg.fastSpeed,1)+">";
  l2 += "<"+String(_cfg.minTxDist)+">";
  l2 += "<"+String(_cfg.minDeltaBeacon)+">";
  l2 += "<"+String(_cfg.turnMinDeg,0)+">";
  l2 += "<"+String(_cfg.turnSlope,0)+">";

  // Řádek 3: <enabled><trackerCall>
  String l3;
  l3 += "<"+String(_cfg.enabled?1:0)+">";
  l3 += "<"+(_cfg.trackerCall.length()? _cfg.trackerCall : call)+">";

  File f = SPIFFS.open(TRACKER_CFG_PATH, FILE_WRITE);
  if (!f) { Serial.println("tracker.cfg: open FAIL"); return; }
  f.println(l1);
  f.println(l2);
  f.println(l3);
  f.close();
  Serial.println("tracker.cfg saved.");
}

static void _loadCfg() {
  if (!SPIFFS.exists(TRACKER_CFG_PATH)) {
    Serial.println("tracker.cfg not found, using defaults (will be created on Save/Start).");
    return;
  }
  File f = SPIFFS.open(TRACKER_CFG_PATH, FILE_READ);
  if (!f) { Serial.println("tracker.cfg: open FAIL"); return; }

  String L1 = f.readStringUntil('\n');
  String L2 = f.readStringUntil('\n');
  String L3 = f.readStringUntil('\n');
  f.close();

  { // line 1
    String s = L1, t;
    t=_nextToken(s); if (t!="") _cfg.gpsRxPin = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.gpsTxPin = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.gpsBaud  = t.toInt();
    t=_nextToken(s); /* legacy ignore */
    t=_nextToken(s); if (t!="") _cfg.compressed = (t!="0");
    t=_nextToken(s); if (t!="") _cfg.tableCh = t[0];
    t=_nextToken(s); if (t!="") _cfg.symCh   = t[0];
    t=_nextToken(s); if (t!="") _cfg.commentMode = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.comment = t;
  }
  { // line 2
    String s = L2, t;
    t=_nextToken(s); if (t!="") _cfg.sbActive = (t!="0");
    t=_nextToken(s); if (t!="") _cfg.slowRate = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.slowSpeed= t.toFloat();
    t=_nextToken(s); if (t!="") _cfg.fastRate = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.fastSpeed= t.toFloat();
    t=_nextToken(s); if (t!="") _cfg.minTxDist= t.toInt();
    t=_nextToken(s); if (t!="") _cfg.minDeltaBeacon = t.toInt();
    t=_nextToken(s); if (t!="") _cfg.turnMinDeg = t.toFloat();
    t=_nextToken(s); if (t!="") _cfg.turnSlope  = t.toFloat();
  }
  { // line 3
    String s = L3, t;
    t=_nextToken(s); if (t!="") _cfg.enabled = (t!="0");
    t=_nextToken(s); if (t!="") _cfg.trackerCall = t;
  }

  Serial.println("tracker.cfg loaded.");
}

// ---------- TX (APRSi přes LoRa RF) ----------
static String _trackerCall() {
  if (_cfg.trackerCall.length()>0) return _cfg.trackerCall;
  return call;
}

static String _buildComment() {
  if (_cfg.commentMode==0) return _cfg.comment;
  char buf[64];
  snprintf(buf, sizeof(buf), "spd %.0f km/h alt %.0f m", _speedKmh, _altM);
  return String(buf);
}

static String _buildPayload_Uncompressed() {
  String payload = "!";
  payload += _fmtLat(_lat);
  payload += _cfg.tableCh;
  payload += _fmtLon(_lon);
  payload += _cfg.symCh;
  String cmt = _buildComment();
  if (cmt.length()) { payload += " "; payload += cmt; }
  return payload;
}

static String _buildPayload_Compressed() {
  char b91[9];
  _encodeCompressed(_lat, _lon, b91);

  // Zkonstruuj payload přímo po znacích → méně alokací, méně fragmentace
  String payload;
  payload.reserve(1 + 1 + 8 + 1 + 1 + 32); // ! + table + 8 + sym + mezera + komentář (odhad)
  payload += '!';
  payload += _cfg.tableCh;       // '/' nebo '\\'

  // přidej přesně 8 znaků base91 (4 lat + 4 lon)
  for (int i = 0; i < 8; ++i) payload += b91[i];

  payload += _cfg.symCh;         // APRS symbol

  String cmt = _buildComment();
  if (cmt.length()) { payload += ' '; payload += cmt; }
  return payload;
}


static void _sendPosition() {
  if (!_haveFix) return;
  if (digi_mode==0 && digi_AP==0) return; // iGate – netx

  String payload = _cfg.compressed ? _buildPayload_Compressed()
                                   : _buildPayload_Uncompressed();

  String frame = _trackerCall() + ">APZ023:" + payload;

  digitalWrite(PLED1, HIGH);
  LoRa.beginPacket();
  LoRa.print("<" + String((char)0xFF) + String((char)0x01) + frame);
  LoRa.endPacket();
  digitalWrite(PLED1, LOW);

  _lastTxMs = millis();
  _lastTxLat = _lat;
  _lastTxLon = _lon;
  _lastTxCourse = _courseDeg;

  Serial.println("TRACKER TX: " + frame);
}

// ---------- SmartBeacon rozhodování ----------
static uint32_t _smartInterval(double vKmh) {
  if (!_cfg.sbActive) return _cfg.slowRate;
  if (vKmh <= _cfg.slowSpeed) return _cfg.slowRate;
  if (vKmh >= _cfg.fastSpeed) return _cfg.fastRate;
  double r = _cfg.slowRate + ( (_cfg.fastRate - _cfg.slowRate) *
              ( (vKmh - _cfg.slowSpeed) / (_cfg.fastSpeed - _cfg.slowSpeed) ) );
  if (r < _cfg.fastRate) r = _cfg.fastRate;
  if (r > _cfg.slowRate) r = _cfg.slowRate;
  return (uint32_t)r;
}

static void _maybeBeacon() {
  if (!_running || !_haveFix) return;

  const uint32_t nowMs      = millis();
  const uint32_t sinceTx    = nowMs - _lastTxMs;
  const uint32_t minDeltaMs = (uint32_t)_cfg.minDeltaBeacon * 1000UL;

  // vzdálenost od posledního TX (m)
  double distM = calculateDistance(_lastTxLat, _lastTxLon, _lat, _lon) * 1000.0;

  // --- první beacon po startu (jakmile je fix) ---
  if (_lastTxMs == 0) {
    _sendPosition();
    return;
  }

  // --- SmartBeacon OFF: čistě periodické vysílání každých slowRate s (s respektem k minDelta) ---
  if (!_cfg.sbActive) {
    uint32_t periodMs = (uint32_t)_cfg.slowRate * 1000UL;
    uint32_t needMs   = (periodMs > minDeltaMs) ? periodMs : minDeltaMs; // ekvivalent max(periodMs, minDeltaMs)
    if (sinceTx >= needMs) {
      _sendPosition();
    }
    return;
  }

  // --- SmartBeacon ON ---
  // 1) Zatáčka (dynamický práh dle rychlosti)
  bool turnTrig = false;
  if (!isnan(_courseDeg) && !isnan(_lastTxCourse)) {
    double degDelta = _angleDelta(_courseDeg, _lastTxCourse);
    double v = (_speedKmh > 0.5) ? _speedKmh : 0.5; // ochrana proti dělení 0
    double dynTurn = _cfg.turnMinDeg + (_cfg.turnSlope / v);
    if (degDelta >= dynTurn) turnTrig = true;
  }

  // 2) Časový interval dle rychlosti
  uint32_t needSec = _smartInterval(_speedKmh);
  bool timeTrig = (sinceTx >= needSec * 1000UL);

  // 3) Ujetá vzdálenost
  bool distTrig = (distM >= _cfg.minTxDist);

  // --- OR logika s pojistkou minDelta (aby to nespamovalo) ---
  bool shouldTx = false;
  if (turnTrig  && sinceTx >= minDeltaMs) shouldTx = true;
  if (distTrig  && sinceTx >= minDeltaMs) shouldTx = true;
  if (timeTrig  && sinceTx >= minDeltaMs) shouldTx = true;

  if (shouldTx) {
    _sendPosition();
  }
}

// ---------- NMEA parser (RMC + GGA) ----------
static void _parseRMC(const String& line) {
  // $GPRMC,hhmmss.s,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,...
  String f[16]; int n = _splitCSV(line, f, 16);
  if (n < 12) return;
  String status = f[2];
  _haveFix = (status == "A");
  if (f[3].length()>=4 && f[4].length()==1 && f[5].length()>=5 && f[6].length()==1) {
    double rawLat = f[3].toFloat();
    double rawLon = f[5].toFloat();
    double lat = floor(rawLat/100.0);
    lat += (rawLat - lat*100.0) / 60.0;
    if (f[4] == "S") lat = -lat;
    double lon = floor(rawLon/100.0);
    lon += (rawLon - lon*100.0) / 60.0;
    if (f[6] == "W") lon = -lon;
    _lat = lat; _lon = lon;
  }
  if (f[7].length()) _speedKmh = f[7].toFloat() * 1.852;
  if (f[8].length()) _courseDeg = f[8].toFloat();

  _lastRMC = line;
  _lastNmeaMs = millis();
}

static void _parseGGA(const String& line) {
  // $GPGGA,hhmmss,llll.ll,a,yyyyy.yy,a,fix,sats,hdop,alt,M,geoid,M,...
  String f[16]; int n = _splitCSV(line, f, 16);
  if (n < 10) return;
  int fix = f[6].toInt(); // 0=no fix
  if (fix>0) _haveFix = true;
  if (f[9].length()) _altM = f[9].toFloat();
  _lastGGA = line;
  _lastNmeaMs = millis();
}

static void _gpsRead() {
  static String line="";
  while (GPS.available()) {
    char c = (char)GPS.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) _parseRMC(line);
      else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) _parseGGA(line);

      // — hned pošli přes SSE do prohlížeče
      if (line.startsWith("$GP") || line.startsWith("$GN")) {
        _sse.send(line.c_str(), "nmea", millis());
      }

      line = "";
    } else {
      if (line.length() < 120) line += c;
    }
  }
}
//--------- kompas-------
static double _bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  if (isnan(lat1) || isnan(lon1) || isnan(lat2) || isnan(lon2)) return NAN;
  const double d2r = M_PI / 180.0;
  const double r2d = 180.0 / M_PI;
  double phi1 = lat1 * d2r, phi2 = lat2 * d2r;
  double dLon = (lon2 - lon1) * d2r;
  double y = sin(dLon) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
  double brng = atan2(y, x) * r2d;      // -180..+180 od severu
  if (brng < 0) brng += 360.0;
  return brng;                          // 0..360 (0=N)
}

static void _drawCompass(int cx, int cy, int r, double azDeg) {
  // obrys kruhu + značky stran
  display.drawCircle(cx, cy, r, WHITE);
  display.drawLine(cx, cy - r, cx, cy - r + 3, WHITE); // N
  display.drawLine(cx + r, cy, cx + r - 3, cy, WHITE); // E
  display.drawLine(cx, cy + r, cx, cy + r - 3, WHITE); // S
  display.drawLine(cx - r, cy, cx - r + 3, cy, WHITE); // W

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(cx - 2, cy - r - 8); display.print("N");

  // šipka (0°=N). GFX 0 rad = +X → korekce
  double rad = (90.0 - azDeg) * M_PI / 180.0;
  int x2 = cx + (int)round((r - 3) * cos(rad));
  int y2 = cy - (int)round((r - 3) * sin(rad));
  display.drawLine(cx, cy, x2, y2, WHITE);

  // hlavička šipky
  double radL = rad + 0.18, radR = rad - 0.18; // ~±10°
  int hx1 = x2 - (int)round(6 * cos(radL));
  int hy1 = y2 + (int)round(6 * sin(radL));
  int hx2 = x2 - (int)round(6 * cos(radR));
  int hy2 = y2 + (int)round(6 * sin(radR));
  display.drawLine(x2, y2, hx1, hy1, WHITE);
  display.drawLine(x2, y2, hx2, hy2, WHITE);
}

// ---------- OLED ----------
static void _oledDraw() {
#if TRACKER_OLED_ENABLED
  if (!_running && !_monitor) return;
  uint32_t now = millis();
  if (now - _lastOledMs < OLED_REFRESH_MS) return;
  _lastOledMs = now;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0,0);
  String hdr = _trackerCall() + String(" TRK");
  display.print(hdr);
  if (_haveFix && ((now/400)%2==0)) { // blikající *
    display.setCursor(118,0);
    display.print("*");
  }

  display.setCursor(0,12);
  char ln1[32]; snprintf(ln1,sizeof(ln1),"Lat %.5f", _lat);
  display.print(ln1);
  display.setCursor(0,22);
  char ln2[32]; snprintf(ln2,sizeof(ln2),"Lon %.5f", _lon);
  display.print(ln2);

  display.setCursor(0,34);
  char ln3[32]; snprintf(ln3,sizeof(ln3),"Alt %.0f m  Spd %.0f", _altM, _speedKmh);
  display.print(ln3);

  display.setCursor(0,54);
  String rx = "RX: " + (lastStation.length()? lastStation : String("-"));
  if (rx.length()>21) rx = rx.substring(0,21);
  display.print(rx);

  // --- kompas vpravo: azimut z aktuální GPS → poslední stanice ---
if (_haveFix && !isnan(lastStationLat) && !isnan(lastStationLon)) {
  double az = _bearingDeg(_lat, _lon, lastStationLat, lastStationLon);

  // kompas vpravo uprostřed
  int cx = 104;  // střed X
  int cy = 28;   // střed Y
  int r  = 22;   // poloměr tak, aby se vešel
  _drawCompass(cx, cy, r, az);

  // číselná hodnota
  display.setTextSize(1);
  display.setCursor(cx - 12, cy + r + 8);
  char azbuf[8]; snprintf(azbuf, sizeof(azbuf), "%3.0f", az);
  display.print(azbuf); display.print((char)247); // ° znak
}

  display.display();
#endif
}

// ---------- WEB UI ----------
static String _htmlEscape(const String& in){
  String o=in;
  o.replace("&","&amp;"); o.replace("\"","&quot;"); o.replace("<","&lt;"); o.replace(">","&gt;");
  return o;
}

static String _buildPage() {
  String tNow;
  if (digi_mode==0 && digi_AP==0) tNow = timeClient.getFormattedTime();
  else tNow = String(millis()/1000) + " s";

  bool gpsOk = (millis() - _lastNmeaMs) < GPS_SILENCE_MS;

  String page;
  page  = "<!DOCTYPE html><html lang='cs'><head><meta charset='utf-8'>";
  page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>Tracker</title>";
  page += "<style>body{font-family:Arial;margin:0;background:#f6f7fb}"
          ".top{background:#1b78e2;color:#fff;padding:12px 16px;font-size:18px}"
          ".wrap{max-width:900px;margin:14px auto;padding:0 10px}"
          ".card{background:#fff;box-shadow:0 2px 12px rgba(0,0,0,.1);border-radius:10px;padding:14px;margin:12px 0}"
          "label{display:inline-block;min-width:220px;font-weight:600}"
          "input,select{padding:6px 8px;margin:4px 0}"
          ".btn{background:#1b78e2;color:#fff;border:0;border-radius:8px;padding:9px 14px;font-weight:700;cursor:pointer}"
          ".row{margin:6px 0}"
          ".warn{background:#fff3f3;color:#b80000;border:1px solid #f3c6c6;padding:8px;border-radius:8px;margin:8px 0}"
          ".ok{color:#2c7a3f}"
          "pre{font-family:monospace}"
          "</style>";
  // SSE klient – přímé streamování NMEA
  page += "<script>"
          "let nmeaEl;"
          "document.addEventListener('DOMContentLoaded',()=>{"
            "nmeaEl=document.getElementById('nmea');"
            "try{"
              "const es=new EventSource('/api/gps');"
              "es.addEventListener('nmea',e=>{"
                "const atBottom=nmeaEl.scrollTop+nmeaEl.clientHeight>=nmeaEl.scrollHeight-5;"
                "nmeaEl.textContent+=e.data+'\\n';"
                "if(atBottom) nmeaEl.scrollTop=nmeaEl.scrollHeight;"
              "});"
            "}catch(e){console.log('SSE error',e);}"
          "});"
          "</script>";
  page += "</head><body>";
  page += "<div class='top'>APRS Tracker &nbsp;—&nbsp; čas: "+tNow+"</div>";
  page += "<div class='wrap'>";

  // Status
  page += "<div class='card'><h3>Status</h3>";
  page += "<div class='row'>Status: <b>";
  page += (_running ? "<span class='ok'>ENABLED</span>" : (_monitor ? "MONITOR" : "STOP"));
  page += "</b> &nbsp;|&nbsp; Mode: ";
  page += ( (digi_mode==0 && digi_AP==0) ? "iGate (no TX)" : "Digi/RF TX");
  page += "</div>";
  if (!gpsOk) page += "<div class='warn'>GPS: no NMEA frames (check wiring/pins/baud rate)</div>";
  page += "<div class='row'>Fix: <b>";
  page += (_haveFix ? "YES" : "NO");
  page += "</b> &nbsp; Lat: "+String(_lat,5)+" Lon: "+String(_lon,5)+"&nbsp; Alt: "+String(_altM,0)+" m &nbsp; Spd: "+String(_speedKmh,0)+" km/h</div>";
  page += "<div class='row'>NMEA(RMC): "+_htmlEscape(_lastRMC)+"</div>";
  page += "<div class='row'>NMEA(GGA): "+_htmlEscape(_lastGGA)+"</div>";
  page += "<div class='row'><a href='/'><button class='btn'>← Back</button></a> ";

  // POST toggle button (nahrazuje GET /tracker/toggle)
  page += "<form method='POST' action='/tracker' style='display:inline'>";
  page += "<button class='btn' name='toggle' value='1'>";
  page += (_running? "Stop tracker" : "Start tracker");
  page += "</button>";
  page += "</form>";

  // Volitelné: Send now
  page += " <form method='POST' action='/tracker' style='display:inline'>";
  page += "<button class='btn' name='send_now' value='1'>Send now</button>";
  page += "</form>";

  page += "</div>";

  // živý log (SSE)
  page += "<div class='row'><h4>NMEA (live)</h4>"
          "<pre id='nmea' style='max-height:240px;overflow:auto;background:#111;color:#0f0;padding:8px;border-radius:8px;white-space:pre-wrap;'></pre></div>";

  page += "</div>";

  // Form konfigurace
  page += "<div class='card'><h3>Setup</h3>";
  page += "<form method='POST' action='/tracker'>";
  page += "<div class='row'><label>Tracker CALL</label><input name='trk_call' value='"+_htmlEscape(_trackerCall())+"' oninput='this.value=this.value.toUpperCase()'></div>";
  page += "<div class='row'><label>GPS RX pin</label><input type='number' name='gps_rx' value='"+String(_cfg.gpsRxPin)+"'> ";
  page += "<label>GPS TX pin</label><input type='number' name='gps_tx' value='"+String(_cfg.gpsTxPin)+"'></div>";
  page += "<div class='row'><label>GPS Baud</label><input type='number' name='gps_baud' value='"+String(_cfg.gpsBaud)+"'></div>";
  page += "<div class='row'><label>Compressed position</label><select name='compressed'><option value='1' ";
  page += (_cfg.compressed?"selected":"");
  page += ">ano</option><option value='0' ";
  page += (!_cfg.compressed?"selected":"");
  page += ">ne</option></select></div>";
  page += "<div class='row'><label>Symbol table</label><select name='table'><option value='/' ";
  page += (_cfg.tableCh=='/'?"selected":"");
  page += ">/</option><option value='\\\\' ";
  page += (_cfg.tableCh=='\\'?"selected":"");
  page += ">\\</option></select> ";
  page += "<label>Symbol</label><input maxlength='1' name='sym' value='";
  page += String(_cfg.symCh);
  page += "' oninput='this.value=this.value.toUpperCase()'></div>";

  page += "<div class='row'><label>Comment beacon</label><select name='cmmode'>"
          "<option value='0' "; page += (_cfg.commentMode==0?"selected":""); page += ">Comment text</option>"
          "<option value='1' "; page += (_cfg.commentMode==1?"selected":""); page += ">speed + alt</option>"
          "</select></div>";
  page += "<div class='row'><label>Comment text</label><input name='comment' maxlength='60' value='"+_htmlEscape(_cfg.comment)+"'></div>";

  page += "<hr><h4>Smart Beacon</h4>";
  page += "<div class='row'><label>ON</label><select name='sb_on'><option value='1' ";
  page += (_cfg.sbActive?"selected":""); page += ">ano</option><option value='0' ";
  page += (!_cfg.sbActive?"selected":""); page += ">ne</option></select></div>";

  page += "<div class='row'><label>Slow (rate, s)</label><input type='number' name='sb_slowrate' value='"+String(_cfg.slowRate)+"'> ";
  page += "<div class='row'><label>Slow (speed, km/h)</label><input type='number' name='sb_slowspeed' value='"+String(_cfg.slowSpeed,0)+"'></div>";

  page += "<div class='row'><label>Fast (rate, s)</label><input type='number' name='sb_fastrate' value='"+String(_cfg.fastRate)+"'> ";
  page += "<label>Fast (speed, km/h)</label><input type='number' name='sb_fastspeed' value='"+String(_cfg.fastSpeed,0)+"'></div>";

  page += "<div class='row'><label>Min TX distance (m)</label><input type='number' name='sb_mindist' value='"+String(_cfg.minTxDist)+"'> ";
  page += "<label>Min delta (s)</label><input type='number' name='sb_mindelta' value='"+String(_cfg.minDeltaBeacon)+"'></div>";

  page += "<div class='row'><label>Min turn (deg)</label><input type='number' name='sb_turnmin' value='"+String(_cfg.turnMinDeg,0)+"'> ";
  page += "<label>Turn slope</label><input type='number' name='sb_turnslope' value='"+String(_cfg.turnSlope,0)+"'></div>";

  page += "<div class='row'><button class='btn' type='submit'>Save</button></div>";
  page += "</form></div>";

  page += "</div></body></html>";
  return page;
}

static void _applyGpsSerial() {
  GPS.end();
  delay(20);
  GPS.begin(_cfg.gpsBaud, SERIAL_8N1, _cfg.gpsRxPin, _cfg.gpsTxPin);
  _lastNmeaMs = millis(); // reset timer
}

// --- monitoring start: čti GPS bez TX ihned po načtení stránky ---
static void _startMonitor() {
  _monitor = true;
  _monitorUntilMs = millis() + TRACKER_MONITOR_MS;
  if (!_running) {
    _applyGpsSerial(); // otevři jen UART (bez TX logiky)
  }
}

static void _startTracker() {
  if (_running) return;
  _applyGpsSerial();
  _running = true;
  _cfg.enabled = true;
  _saveCfg();
  _lastTxMs = 0;
  _lastTxLat = _lat; _lastTxLon = _lon;
  _lastTxCourse = _courseDeg;
  Serial.println("TRACKER: started");
}

static void _stopTracker() {
  if (!_running) return;
  _running = false;
  GPS.end();
  _cfg.enabled = false;
  _saveCfg();
  Serial.println("TRACKER: stopped");
}

// ---------- PUBLIC API ----------
static void tracker_setup(AsyncWebServer* srv) {
  _loadCfg();
  if (_cfg.enabled) _startTracker();

  if (srv) {
    // SSE handler (zaregistrovat dřív, než případnou statiku/catch-all)
    srv->addHandler(&_sse);
    _sse.onConnect([](AsyncEventSourceClient *client){
      if (client->connected()) client->send("connected", "info", millis());
    });

    // /tracker HTML
    srv->on("/tracker", HTTP_GET, [](AsyncWebServerRequest *req){
      extern String web_username, web_password;
      if (!req->authenticate(web_username.c_str(), web_password.c_str()))
        return req->requestAuthentication();

      _startMonitor(); // ihned po otevření stránky začni číst GPS (bez TX)
      req->send(200, "text/html", _buildPage());
    });

    // Save/Actions konfigurace (POST)
    srv->on("/tracker", HTTP_POST, [](AsyncWebServerRequest *req){
      extern String web_username, web_password;
      if (!req->authenticate(web_username.c_str(), web_password.c_str()))
        return req->requestAuthentication();

      // 💡 AKČNÍ TLAČÍTKA PŘEDEM (toggle, send_now...)
      if (req->hasParam("toggle", true)) {
        if (_running) _stopTracker(); else _startTracker();
        return req->redirect("/tracker");
      }
      if (req->hasParam("send_now", true)) {
        _sendPosition();          // pošli okamžitě (pokud je fix a režim dovolí TX)
        _startMonitor();          // zobraz živý log
        return req->send(200, "text/html", _buildPage());
      }

      auto getV=[&](const char* n)->String{
        return req->hasParam(n,true) ? req->getParam(n,true)->value() : String("");
      };
      String v;

      v=getV("trk_call");     if (v.length()) _cfg.trackerCall = v;
      v=getV("gps_rx");       if (v.length()) _cfg.gpsRxPin = v.toInt();
      v=getV("gps_tx");       if (v.length()) _cfg.gpsTxPin = v.toInt();
      v=getV("gps_baud");     if (v.length()) _cfg.gpsBaud  = v.toInt();
      v=getV("compressed");   if (v.length()) _cfg.compressed = (v!="0");
      v=getV("table");        if (v.length()) _cfg.tableCh = v[0];
      v=getV("sym");          if (v.length()) _cfg.symCh   = (char)toupper(v[0]);
      v=getV("cmmode");       if (v.length()) _cfg.commentMode = v.toInt();
      v=getV("comment");      _cfg.comment = v;

      v=getV("sb_on");        if (v.length()) _cfg.sbActive = (v!="0");
      v=getV("sb_slowrate");  if (v.length()) _cfg.slowRate = (uint16_t)v.toInt();
      v=getV("sb_slowspeed"); if (v.length()) _cfg.slowSpeed= v.toFloat();
      v=getV("sb_fastrate");  if (v.length()) _cfg.fastRate = (uint16_t)v.toInt();
      v=getV("sb_fastspeed"); if (v.length()) _cfg.fastSpeed= v.toFloat();
      v=getV("sb_mindist");   if (v.length()) _cfg.minTxDist= (uint16_t)v.toInt();
      v=getV("sb_mindelta");  if (v.length()) _cfg.minDeltaBeacon = (uint16_t)v.toInt();
      v=getV("sb_turnmin");   if (v.length()) _cfg.turnMinDeg = v.toFloat();
      v=getV("sb_turnslope"); if (v.length()) _cfg.turnSlope  = v.toFloat();

      _saveCfg();
      if (_running) _applyGpsSerial(); // přenastav UART bez restartu
      _startMonitor();                 // a hned zase čti

      req->send(200, "text/html", _buildPage());
    });

    // (volitelné) Kompatibilita: GET toggle
    srv->on("/tracker/toggle", HTTP_GET, [](AsyncWebServerRequest *req){
      extern String web_username, web_password;
      if (!req->authenticate(web_username.c_str(), web_password.c_str()))
        return req->requestAuthentication();

      if (_running) _stopTracker();
      else _startTracker();

      req->redirect("/tracker");
    });
  }
}

static void tracker_loop() {
  // čtení GPS
  if (_running || _monitor) {
    _gpsRead();
    if (_running) _maybeBeacon();   // TX logika jen v běžném režimu
  }

  // auto-vypnutí monitoringu po timeoutu (pokud neběží tracker)
  if (_monitor && !_running && millis() > _monitorUntilMs) {
    _monitor = false;
    GPS.end(); // zavři UART kvůli spotřebě
  }

  // OLED
  _oledDraw();
}

#endif // SETUP_TRACKER_H
