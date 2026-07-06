/*
 * CYD28 (ESP32) Plane Spotter - Direct TFT Rendering
 * --------------------------------------------------------------------------
 * UPDATE: Merged Weather & System Status into a single unified screen.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =======================================================================
// CONFIGURATION — loaded from config.env
// =======================================================================
#include "config.env"

// Runtime-configurable location & radar (persisted via NVS)
// Defaults below; overridden by web UI and saved to flash.
double homeLat = DEFAULT_HOME_LAT;
double homeLon = DEFAULT_HOME_LON;
double searchRadiusDeg = DEFAULT_SEARCH_RADIUS;
float  radarMaxKm = DEFAULT_RADAR_MAX_KM;

SemaphoreHandle_t configMutex = NULL;  // protects homeLat/Lon/Radius/radarMaxKm
WebServer server(80);
Preferences prefs;
// =======================================================================

// CYD Touch Pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass touchSpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite radarSpr = TFT_eSprite(&tft);

// Data models
struct Aircraft {
  char callsign[10]; char country[24];
  double lat; double lon; float altitudeM; float velocityMs;
  float trackDeg; float vrateMs; bool onGround; int category;
  double distanceKm; double bearingDeg; bool valid;
  char dep[5]; char arr[5]; bool hasRoute;
};
Aircraft nearest;

// Top 5 Tracker
const uint8_t MAX_TOP5 = 5;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

// Blips for Radar
struct Blip { double lat; double lon; float track; float speedMs; char callsign[10]; float altitudeM; };
const uint8_t MAX_BLIPS = 20;
Blip blips[MAX_BLIPS];
uint8_t blipCount = 0;
uint32_t lastDataMs = 0;

// Async data fetch (FreeRTOS task on Core 0)
SemaphoreHandle_t dataMutex = NULL;
TaskHandle_t fetchTaskHandle = NULL;
volatile bool newDataReady = false;
volatile bool fetchInProgress = false;

// Route Cache
struct CachedRoute {
  char callsign[10];
  char dep[5]; char arr[5];
  bool valid; uint32_t fetchTime;
};
CachedRoute routeCache[15];

struct Weather {
  float tempC; float windKmh; int humidity; int code; bool valid = false;
} weather;
uint32_t lastWeatherPoll = 0;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  double closestEver = 1e9; uint32_t lastUpdateMs = 0;
} stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastScreenSwap = 0;
uint32_t lastTouchMs = 0;
bool firstWeatherDone = false;
const uint8_t NUM_SCREENS = 4; // Reduced from 5 to 4 screens
const uint32_t SCREEN_SWAP_MS = 10000;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

void projectLatLon(double lat, double lon, float trackDeg, double distM, double& outLat, double& outLon) {
  double dr = distM / 6371000.0; double b = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla); outLon = rad2deg(nlo);
}

// ---------------------------------------------------------------------------
// Network & APIs
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);
  tft.drawString("Connecting WiFi...", 20, 100);
  while (WiFi.status() != WL_CONNECTED) { delay(400); }
  tft.fillScreen(TFT_BLACK);
}

bool getRoute(const char* callsign, char* dep, char* arr) {
  if (strlen(callsign) == 0 || strcmp(callsign, "(no id)") == 0) return false;
  
  for (int i=0; i<15; i++) {
    if (routeCache[i].valid && strcmp(routeCache[i].callsign, callsign) == 0) {
      if (routeCache[i].dep[0]) {
         strcpy(dep, routeCache[i].dep);
         strcpy(arr, routeCache[i].arr);
         return true;
      }
      return false;
    }
  }

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  String url = String("https://hexdb.io/api/v1/route/icao/") + callsign;
  bool found = false;
  dep[0] = '\0'; arr[0] = '\0';

  if (https.begin(client, url)) {
    int code = https.GET();
    if (code == HTTP_CODE_OK) {
      JsonDocument d;
      if (!deserializeJson(d, https.getString())) {
        const char* r = d["route"] | "";
        const char* dash = strchr(r, '-');
        if (r[0] && dash) {
          size_t dl = dash - r;
          if (dl < 5) {
            strncpy(dep, r, dl); dep[dl] = '\0';
            strncpy(arr, dash + 1, 4); arr[4] = '\0';
            found = true;
          }
        }
      }
    }
    https.end();
  }

  int replaceIdx = 0;
  uint32_t oldest = 0xFFFFFFFF;
  for (int i=0; i<15; i++) {
    if (!routeCache[i].valid) { replaceIdx = i; break; }
    if (routeCache[i].fetchTime < oldest) { oldest = routeCache[i].fetchTime; replaceIdx = i; }
  }
  routeCache[replaceIdx].valid = true;
  strcpy(routeCache[replaceIdx].callsign, callsign);
  strcpy(routeCache[replaceIdx].dep, dep);
  strcpy(routeCache[replaceIdx].arr, arr);
  routeCache[replaceIdx].fetchTime = millis();

  return found;
}

// Fetches aircraft data into caller-provided local buffers (thread-safe — no globals touched)
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut) {
  // Snapshot config under mutex (Core 0 reads, Core 1 may write via web UI)
  double hLat, hLon, sRad; float rMax;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; sRad = searchRadiusDeg; rMax = radarMaxKm;
  if (configMutex) xSemaphoreGive(configMutex);

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  String url = "https://opensky-network.org/api/states/all?lamin=" + String(hLat - sRad, 4) + 
         "&lomin=" + String(hLon - sRad, 4) + "&lamax=" + String(hLat + sRad, 4) + 
         "&lomax=" + String(hLon + sRad, 4) + "&extended=1";

  if (!https.begin(client, url)) { stats.requestsFail++; return false; }
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); stats.requestsFail++; return false; }

  JsonDocument filter;
  JsonArray el = filter["states"].to<JsonArray>().add<JsonArray>();
  for (int i = 0; i <= 17; i++) el[i] = false;
  el[0] = el[1] = el[2] = el[5] = el[6] = el[7] = el[8] = el[9] = el[10] = el[11] = el[13] = el[17] = true;

  String payload = https.getString(); https.end();
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) { stats.requestsFail++; return false; }

  JsonArray states = doc["states"].as<JsonArray>();
  uint16_t count = 0; 
  blipCntOut = 0;
  top5CntOut = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;

    // Skip aircraft on the ground — only track those in flight
    if (s[8] | false) continue;

    double lon = s[5].as<double>(), lat = s[6].as<double>();
    double d = haversineKm(hLat, hLon, lat, lon);
    double brg = bearingDeg(hLat, hLon, lat, lon);
    count++;

    if (blipCntOut < MAX_BLIPS) {
      blipsOut[blipCntOut].lat = lat; blipsOut[blipCntOut].lon = lon;
      blipsOut[blipCntOut].track = s[10] | 0.0f;
      blipsOut[blipCntOut].speedMs = (s[8] | false) ? 0.0f : (s[9] | 0.0f);
      blipsOut[blipCntOut].altitudeM = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
      const char* cs = s[1] | "";
      strncpy(blipsOut[blipCntOut].callsign, cs, 9);
      blipsOut[blipCntOut].callsign[9] = '\0';
      blipCntOut++;
    }

    Aircraft a;
    a.distanceKm = d; a.lat = lat; a.lon = lon; a.bearingDeg = brg;
    a.onGround = s[8] | false; a.category = s[17] | 0;
    a.altitudeM = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
    a.velocityMs = s[9] | 0.0f; a.trackDeg = s[10] | 0.0f; a.vrateMs = s[11] | 0.0f;
    a.hasRoute = false; a.dep[0] = '\0'; a.arr[0] = '\0';
    a.valid = true;
    
    strncpy(a.callsign, s[1] | "", 9); a.callsign[9] = '\0';
    for (int i = strlen(a.callsign) - 1; i >= 0 && a.callsign[i] == ' '; i--) a.callsign[i] = '\0';
    if (a.callsign[0] == '\0') strcpy(a.callsign, "(no id)");
    strncpy(a.country, s[2] | "?", 23); a.country[23] = '\0';

    int insertIdx = -1;
    for (int i = 0; i < MAX_TOP5; i++) {
      if (i >= top5CntOut || a.distanceKm < top5Out[i].distanceKm) {
        insertIdx = i; break;
      }
    }
    if (insertIdx != -1) {
      for (int i = MAX_TOP5 - 1; i > insertIdx; i--) { top5Out[i] = top5Out[i-1]; }
      top5Out[insertIdx] = a;
      if (top5CntOut < MAX_TOP5) top5CntOut++;
    }
  }

  stats.inView = count; if (count > stats.maxInView) stats.maxInView = count;
  stats.requestsOk++;

  if (top5CntOut > 0) {
    nearOut = top5Out[0];
    if (nearOut.distanceKm < stats.closestEver)
      stats.closestEver = nearOut.distanceKm;
  } else {
    nearOut.valid = false;
  }

  return true;
}

// ===================================================================
// Background Data Fetcher — runs on Core 0 so Core 1 stays smooth
// ===================================================================
void dataFetcherTask(void* param) {
  // Local work buffers — all blocking HTTP happens here
  Aircraft locTop5[MAX_TOP5];
  Blip locBlips[MAX_BLIPS];
  uint8_t locTop5Count = 0;
  uint8_t locBlipCount = 0;
  Aircraft locNearest;
  locNearest.valid = false;

  for (;;) {
    // Wait for the next poll interval (sleep in 1s chunks so task is responsive)
    for (int t = 0; t < UPDATE_INTERVAL_MS / 1000; t++) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    fetchInProgress = true;
    bool ok = fetchAircraftTo(locTop5, locTop5Count, locNearest, locBlips, locBlipCount);
    fetchInProgress = false;

    if (ok) {
      // Fetch routes for the top 5 (quick per-call, but may still block a bit)
      for (int i = 0; i < locTop5Count; i++) {
        locTop5[i].hasRoute = getRoute(locTop5[i].callsign, locTop5[i].dep, locTop5[i].arr);
        locNearest = locTop5[0]; // nearest = closest after route resolution
      }

      // Atomically publish results to the globals used by render()
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(blips, locBlips, sizeof(locBlips));
      blipCount = locBlipCount;
      memcpy(top5, locTop5, sizeof(locTop5));
      top5Count = locTop5Count;
      nearest = locNearest;
      lastDataMs = millis();
      stats.lastUpdateMs = lastDataMs;
      newDataReady = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

bool fetchWeather() {
  // Snapshot config under mutex
  double hLat, hLon;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon;
  if (configMutex) xSemaphoreGive(configMutex);

  WiFiClientSecure client; client.setInsecure(); HTTPClient https; https.setReuse(false);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(hLat, 4) + "&longitude=" + String(hLon, 4) + "&current=temperature_2m,relative_humidity_2m,wind_speed_10m";
  if (!https.begin(client, url)) return false;
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  String payload = https.getString(); https.end();

  JsonDocument doc; if (deserializeJson(doc, payload)) return false;
  JsonObject c = doc["current"]; if (c.isNull()) return false;
  
  weather.tempC = c["temperature_2m"] | 0.0f; weather.humidity = c["relative_humidity_2m"] | 0;
  weather.windKmh = c["wind_speed_10m"] | 0.0f; weather.valid = true;
  return true;
}

// ---------------------------------------------------------------------------
// UI & Drawing
// ---------------------------------------------------------------------------
bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

void drawHeader(const char* title, bool newPage) {
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  
  if (newPage) {
    char left[30]; snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, NUM_SCREENS);
    tft.setTextPadding(200);
    tft.drawString(left, 10, 5);
    tft.drawLine(0, 25, 320, 25, TFT_DARKGREY);
  }
  
  char t[10]; fmtClock(t, sizeof(t), true);
  tft.setTextPadding(80);
  tft.drawString(t, 240, 5);
  tft.setTextPadding(0);
}

void drawArrow(int cx, int cy, int r, double angleDeg, uint16_t color) {
  double a = deg2rad(angleDeg);
  int tx = cx + (int)(sin(a) * r), ty = cy - (int)(cos(a) * r);
  int bx = cx - (int)(sin(a) * r), by = cy + (int)(cos(a) * r);
  tft.drawLine(bx, by, tx, ty, color);
  double left = a + deg2rad(150), right = a - deg2rad(150);
  tft.drawLine(tx, ty, tx + (int)(sin(left) * (r/2)), ty - (int)(cos(left) * (r/2)), color);
  tft.drawLine(tx, ty, tx + (int)(sin(right)* (r/2)), ty - (int)(cos(right)* (r/2)), color);
}

void screenTargetIntel(bool newPage) {
  drawHeader("TARGET INTEL", newPage);
  tft.setTextPadding(300);
  
  if (!nearest.valid) {
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGET IN RANGE", 40, 100);
    return;
  }

  tft.setTextFont(4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(nearest.callsign, 20, 40);
  
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  if (nearest.hasRoute) {
    char rte[32]; snprintf(rte, sizeof(rte), "Route: %s -> %s", nearest.dep, nearest.arr);
    tft.drawString(rte, 160, 45);
  } else {
    tft.drawString("Route: Unknown", 160, 45);
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[64];
  snprintf(buf, sizeof(buf), "Country: %s", nearest.country);
  tft.drawString(buf, 20, 80);
  
  snprintf(buf, sizeof(buf), "Distance: %.1f km (%s)", nearest.distanceKm, compass(nearest.bearingDeg));
  tft.drawString(buf, 20, 110);
  
  float altFeet = nearest.altitudeM * 3.28084f;
  if (nearest.onGround) snprintf(buf, sizeof(buf), "Altitude: On Ground");
  else snprintf(buf, sizeof(buf), "Alt: %.0f ft (FL%03.0f)", altFeet, altFeet / 100.0f);
  tft.drawString(buf, 20, 140);
  
  snprintf(buf, sizeof(buf), "Speed: %.0f km/h", nearest.velocityMs * 3.6);
  tft.drawString(buf, 20, 170);

  tft.fillRect(220, 110, 60, 60, TFT_BLACK); 
  drawArrow(250, 140, 25, nearest.trackDeg, TFT_CYAN);
  snprintf(buf, sizeof(buf), "HDG %03.0f", nearest.trackDeg);
  tft.drawString(buf, 225, 180);
  
  tft.setTextPadding(0);
}

void screenTop5(bool newPage) {
  drawHeader("TOP 5 IN RANGE", newPage);
  
  if (top5Count == 0) {
    tft.setTextFont(4); tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGETS", 100, 100);
    return;
  }
  
  if (newPage) {
    tft.setTextFont(2); tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("FLIGHT", 10, 40);
    tft.drawString("ROUTE", 100, 40);
    tft.drawString("ALTITUDE", 190, 40);
    tft.drawLine(10, 60, 310, 60, TFT_DARKGREY);
  }
  
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(80); 
  
  int y = 70;
  for (int i=0; i<5; i++) {
    if (i < top5Count) {
      tft.drawString(top5[i].callsign, 10, y);
      
      char rte[16];
      if (top5[i].hasRoute) snprintf(rte, sizeof(rte), "%s->%s", top5[i].dep, top5[i].arr);
      else strcpy(rte, "N/A");
      tft.drawString(rte, 100, y);
      
      char alt[32];
      tft.setTextPadding(120); 
      if (top5[i].onGround) strcpy(alt, "Ground");
      else {
        float ft = top5[i].altitudeM * 3.28084f;
        snprintf(alt, sizeof(alt), "%.0f ft (FL%03.0f)", ft, ft/100.0f);
      }
      tft.drawString(alt, 190, y);
      tft.setTextPadding(80);
      
    } else {
      tft.drawString(" ", 10, y);
      tft.drawString(" ", 100, y);
      tft.setTextPadding(120);
      tft.drawString(" ", 190, y);
      tft.setTextPadding(80);
    }
    y += 30;
  }
  tft.setTextPadding(0);
}

void screenRadar(bool newPage) {
  drawHeader("RADAR PPI", newPage);

  // Snapshot config under mutex
  double hLat, hLon; float rMax;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  if (configMutex) xSemaphoreGive(configMutex);

  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextPadding(110);
  char buf[32]; snprintf(buf, sizeof(buf), "Contacts: %u", blipCount);
  tft.drawString(buf, 5, 50);
  if(nearest.valid) {
    snprintf(buf, sizeof(buf), "TGT: %s", nearest.callsign);
    tft.drawString(buf, 5, 80);
    snprintf(buf, sizeof(buf), "%.1f km", nearest.distanceKm);
    tft.drawString(buf, 5, 110);
  } else {
    tft.drawString("No TGT", 5, 80);
    tft.drawString(" ", 5, 110);
  }
  tft.setTextPadding(0);

  const int cx = 100, cy = 100, R = 95;
  float elapsed = (millis() - lastDataMs) / 1000.0f;

  radarSpr.fillSprite(TFT_BLACK);
  radarSpr.drawCircle(cx, cy, R, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R*2/3, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R/3, TFT_DARKGREY);
  radarSpr.drawLine(cx-R, cy, cx+R, cy, TFT_DARKGREY);
  radarSpr.drawLine(cx, cy-R, cx, cy+R, TFT_DARKGREY);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  radarSpr.drawLine(cx, cy, cx + (int)(sin(sw)*R), cy - (int)(cos(sw)*R), TFT_GREEN);

  radarSpr.setTextFont(1);
  radarSpr.setTextColor(TFT_WHITE, TFT_BLACK);

  for (uint8_t i = 0; i < blipCount; i++) {
    double la = blips[i].lat, lo = blips[i].lon;
    if (blips[i].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedMs * elapsed, la, lo);

    double dist = haversineKm(hLat, hLon, la, lo);
    float fr = (float)(dist / rMax);
    if (fr > 1) continue;

    int rr = (int)(fr * R);
    double brg = bearingDeg(hLat, hLon, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);

    float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
    if (behind < 30) radarSpr.fillCircle(bx, by, 3, TFT_YELLOW);
    else radarSpr.fillRect(bx, by, 2, 2, TFT_GREEN);

    // Draw callsign label in smallest legible font
    if (blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0) {
      int lx = bx + 4;
      int ly = by - 4;
      if (lx + 54 > 200) lx = bx - 54;
      if (ly < 0) ly = 0;
      if (ly > 192) ly = 192;
      radarSpr.drawString(blips[i].callsign, lx, ly);

      // Draw FL below callsign
      float altFt = blips[i].altitudeM * 3.28084f;
      char flBuf[10];
      snprintf(flBuf, sizeof(flBuf), "FL%03.0f", altFt / 100.0f);
      int ly2 = ly + 8;
      if (ly2 > 192) ly2 = 192;
      radarSpr.drawString(flBuf, lx, ly2);

      // Draw speed (knots) below FL
      float speedKt = blips[i].speedMs * 1.94384f;
      char spBuf[10];
      snprintf(spBuf, sizeof(spBuf), "%ukn", (unsigned int)(speedKt + 0.5f));
      int ly3 = ly + 16;
      if (ly3 > 192) ly3 = 192;
      radarSpr.drawString(spBuf, lx, ly3);
    }
  }
  radarSpr.pushSprite(115, 35);

  // --- Range indicator & +/- buttons (bottom-left, side by side) ---
  // Range label
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(80);
  char rng[16]; snprintf(rng, sizeof(rng), "RNG %d km", (int)rMax);
  tft.drawString(rng, 10, 193);

  // "-" button (left, 30x20)
  tft.fillRoundRect(10, 215, 30, 20, 4, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextPadding(30);
  tft.drawString("-", 20, 216);

  // "+" button (right of "-", 30x20)
  tft.fillRoundRect(50, 215, 30, 20, 4, TFT_DARKGREY);
  tft.drawString("+", 60, 216);

  tft.setTextPadding(0);
}

// Unified Weather and System Screen
void screenWeatherSystem(bool newPage) {
  drawHeader("WEATHER & SYSTEM", newPage);
  
  if (newPage) {
    // Divider line between weather and system info
    tft.drawLine(10, 115, 310, 115, TFT_DARKGREY);
  }

  // --- TOP HALF: WEATHER ---
  if (!weather.valid) { 
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO WX DATA", 20, 50); 
  } else {
    tft.setTextFont(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextPadding(100);
    char buf[32]; snprintf(buf, sizeof(buf), "%.1f C", weather.tempC);
    tft.drawString(buf, 20, 45);

    tft.setTextFont(2); 
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextPadding(150);
    snprintf(buf, sizeof(buf), "Humidity: %d%%", weather.humidity);
    tft.drawString(buf, 160, 45);
    snprintf(buf, sizeof(buf), "Wind: %.0f km/h", weather.windKmh);
    tft.drawString(buf, 160, 75);
  }

  // --- BOTTOM HALF: SYSTEM ---
  tft.setTextFont(2); 
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextPadding(160);
  
  char bufSys[64];
  uint32_t up = millis() / 1000;
  snprintf(bufSys, sizeof(bufSys), "Uptime: %02lu:%02lu:%02lu", up / 3600, (up % 3600) / 60, up % 60);
  tft.drawString(bufSys, 20, 130);
  
  snprintf(bufSys, sizeof(bufSys), "WiFi RSSI: %d dBm", WiFi.RSSI());
  tft.drawString(bufSys, 180, 130);
  
  snprintf(bufSys, sizeof(bufSys), "RAM Free: %d KB", ESP.getFreeHeap() / 1024);
  tft.drawString(bufSys, 20, 160);
  
  tft.setTextPadding(300); // Larger padding for API count
  snprintf(bufSys, sizeof(bufSys), "API Req: %lu OK / %lu FAIL", stats.requestsOk, stats.requestsFail);
  tft.drawString(bufSys, 20, 190);
  
  tft.setTextPadding(0);
}

void render() {
  bool newPage = false;
  if (screen != lastScreen) {
    tft.fillScreen(TFT_BLACK);
    lastScreen = screen;
    newPage = true;
  }
  
  // Take mutex for the duration of rendering so we see a consistent snapshot
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen) {
    case 0: screenTargetIntel(newPage); break;
    case 1: screenTop5(newPage);        break;
    case 2: screenRadar(newPage);       break;
    case 3: screenWeatherSystem(newPage); break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
}

void checkTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    
    // STRICT FILTER: 
    // 1. p.z > 400 ignores light ghost touches and case pinching.
    // 2. p.x and p.y bounds ignore the 0 and 4095 coordinate spikes caused by SPI noise.
    if (p.z > 400 && p.x > 100 && p.x < 4000 && p.y > 100 && p.y < 4000) {
      
      if (millis() - lastTouchMs > 400) { // 400ms debounce to prevent double-skips
        lastTouchMs = millis();

        // Map raw touch to screen coordinates (CYD rotation 1)
        // Calibrated: raw ~200-3900, X-axis inverted for this panel
        int sx = 320 - (int)((p.y - 200.0f) * 320.0f / 3700.0f);
        int sy = 240 - (int)((p.x - 200.0f) * 240.0f / 3700.0f);
        sx = constrain(sx, 0, 319);
        sy = constrain(sy, 0, 239);

        // Debug: print touch coords for calibration
        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 2000) {
          Serial.printf("Touch: raw(%d,%d,%d) -> screen(%d,%d)\n", p.x, p.y, p.z, sx, sy);
          lastDebug = millis();
        }

        // Screen-specific touch handling
        if (screen == 2) {
          // Radar screen: check +/- range buttons (bottom-left, stacked vertically)
          // "+" button: sx=10..50, sy=160..205
          if (sx >= 10 && sx <= 50 && sy >= 160 && sy <= 205) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            prefs.putFloat("range", radarMaxKm);
            Serial.printf("Range increased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
          // "-" button: sx=10..50, sy=206..240
          if (sx >= 10 && sx <= 50 && sy >= 206 && sy <= 240) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            prefs.putFloat("range", radarMaxKm);
            Serial.printf("Range decreased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
        }

        // Default: cycle to next screen
        screen = (screen + 1) % NUM_SCREENS;
        lastScreenSwap = millis();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Web Server — Config Page
// ---------------------------------------------------------------------------
void initWebServer() {
  // GET / — serve config page
  server.on("/", []() {
    // Snapshot current values
    double hLat, hLon, sRad; float rMax;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; sRad = searchRadiusDeg; rMax = radarMaxKm;
    if (configMutex) xSemaphoreGive(configMutex);

    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CYD Plane Spotter</title>
<style>
  body{font-family:Arial,sans-serif;background:#0a0a1a;color:#e0e0e0;margin:0;padding:20px}
  h1{color:#00ff88;text-align:center;font-size:1.4em}
  .card{background:#1a1a2e;border-radius:12px;padding:20px;margin:12px 0;max-width:420px;margin-left:auto;margin-right:auto}
  label{display:block;margin:10px 0 4px;color:#aaa;font-size:0.85em}
  input{width:100%;padding:10px;border:1px solid #333;border-radius:6px;background:#0d0d1f;color:#fff;font-size:1em;box-sizing:border-box}
  input:focus{border-color:#00ff88;outline:none}
  button{width:100%;padding:12px;margin-top:16px;background:#00ff88;color:#000;border:none;border-radius:8px;font-size:1.1em;font-weight:bold;cursor:pointer}
  button:hover{background:#00cc6a}
  .info{text-align:center;color:#666;font-size:0.8em;margin-top:12px}
  .saved{color:#00ff88;text-align:center;display:none;margin-top:8px}
</style></head><body>
<h1>&#9992; CYD Plane Spotter</h1>
<div class="card">
  <form id="cfg" onsubmit="save(event)">
    <label>Home Latitude (&deg;)</label>
    <input type="number" step="any" name="lat" id="lat" value=")rawliteral" + String(hLat, 6) + R"rawliteral(" required>
    <label>Home Longitude (&deg;)</label>
    <input type="number" step="any" name="lon" id="lon" value=")rawliteral" + String(hLon, 6) + R"rawliteral(" required>
    <label>Search Radius (&deg;)</label>
    <input type="number" step="any" name="radius" id="radius" value=")rawliteral" + String(sRad, 4) + R"rawliteral(" required>
    <label>Radar Max Range (km)</label>
    <input type="number" step="any" name="range" id="range" value=")rawliteral" + String(rMax, 1) + R"rawliteral(" required>
    <button type="submit">Save Settings</button>
  </form>
  <div class="saved" id="msg">&#10003; Settings saved!</div>
</div>
<div class="info">IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
<script>
function save(e){
  e.preventDefault();
  var x=new XMLHttpRequest();
  x.open('POST','/save',true);
  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  x.onload=function(){
    if(x.status==200){document.getElementById('msg').style.display='block';
    setTimeout(function(){document.getElementById('msg').style.display='none'},3000)}
  };
  x.send('lat='+encodeURIComponent(document.getElementById('lat').value)+
         '&lon='+encodeURIComponent(document.getElementById('lon').value)+
         '&radius='+encodeURIComponent(document.getElementById('radius').value)+
         '&range='+encodeURIComponent(document.getElementById('range').value));
}
</script></body></html>)rawliteral";

    server.send(200, "text/html", html);
  });

  // POST /save — persist new config
  server.on("/save", []() {
    if (!server.hasArg("lat") || !server.hasArg("lon") || !server.hasArg("radius") || !server.hasArg("range")) {
      server.send(400, "text/plain", "Missing fields");
      return;
    }

    double newLat = server.arg("lat").toDouble();
    double newLon = server.arg("lon").toDouble();
    double newRad = server.arg("radius").toDouble();
    float  newRng = server.arg("range").toFloat();

    // Basic validation
    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRad <= 0 || newRad > 10 || newRng <= 0 || newRng > 500) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }

    // Atomically update
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    homeLat = newLat;
    homeLon = newLon;
    searchRadiusDeg = newRad;
    radarMaxKm = newRng;
    if (configMutex) xSemaphoreGive(configMutex);

    // Persist to NVS
    prefs.putDouble("lat", newLat);
    prefs.putDouble("lon", newLon);
    prefs.putDouble("radius", newRad);
    prefs.putFloat("range", newRng);

    Serial.printf("Config saved: lat=%.6f lon=%.6f radius=%.4f range=%.1f\n",
                  newLat, newLon, newRad, newRng);
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Web server started on http://" + WiFi.localIP().toString() + "/");
}

// ---------------------------------------------------------------------------
// Main Arduino Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, -1);
  ts.begin(touchSpi); 
  ts.setRotation(1);

  tft.begin();
  tft.setRotation(1); 
  tft.invertDisplay(false); 
  
  radarSpr.setColorDepth(16);
  radarSpr.createSprite(200, 200);

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN);
  tft.drawString("CYD PLANE SPOTTER", 20, 100);
  delay(1500);

  for (int i=0; i<15; i++) routeCache[i].valid = false;

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;
  newDataReady = false;
  fetchInProgress = false;

  // Load persisted config (or use defaults)
  prefs.begin("cyd-spotter", false);
  homeLat = prefs.getDouble("lat", homeLat);
  homeLon = prefs.getDouble("lon", homeLon);
  searchRadiusDeg = prefs.getDouble("radius", searchRadiusDeg);
  radarMaxKm = prefs.getFloat("range", radarMaxKm);
  Serial.printf("Config loaded: lat=%.6f lon=%.6f radius=%.4f range=%.1f\n",
                homeLat, homeLon, searchRadiusDeg, radarMaxKm);

  // Create mutexes before the task (task uses them)
  dataMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();

  // Start web config server
  initWebServer();

  // Launch background fetcher on Core 0 (Core 1 = Arduino loop)
  xTaskCreatePinnedToCore(
    dataFetcherTask,   // Function
    "DataFetch",       // Name
    16384,             // Stack (16 KB — enough for JSON + SSL)
    NULL,              // Parameter
    1,                 // Priority
    &fetchTaskHandle,  // Handle
    0                  // Core 0 — leaves Core 1 free for rendering
  );
}

void loop() {
  uint32_t now = millis();

  // WiFi health — reconnect if needed (draws to TFT so must be Core 1)
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  // Weather fetch still runs in loop (it's infrequent, every 10 min)
  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now; firstWeatherDone = true;
  }

  checkTouch();

  // Serve web config requests
  server.handleClient();

  // Auto-screen rotation (re-enabled)
  //if (now - lastScreenSwap >= SCREEN_SWAP_MS) {
  //  screen = (screen + 1) % NUM_SCREENS;
  //  lastScreenSwap = now;
  //}

  render();
  delay(33);
}