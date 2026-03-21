#include <WiFi.h>
#include <driver/gpio.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// -----------------------------
// Hardware pin mapping (ESP32-C3 SuperMini HW-466AB)
// -----------------------------
static const int PIN_OLED_SCK = 4;
static const int PIN_OLED_SDA = 6;   // MOSI
static const int PIN_OLED_CS = 7;
static const int PIN_OLED_DC = 8;
static const int PIN_OLED_RES = 9;

static const int PIN_RELAY = 21;     // Relay transistor (BC337) base resistor -> this GPIO
static const int PIN_ONEWIRE = 10;   // DS18B20 data pin
// ADC vstup pro žebřík tlačítek.
// Na ESP32-C3 má GPIO3 ADC1_CH3 a HW pull-up/pull-down (narozdíl od „jen vstupních“ pinů
// GPIO34–39 u klasického ESP32). Slabý interní pull-up (~45 kΩ) často nestačí na klidný
// analogový uzel — spolehlivější je odpor 10k–47k z ADC uzlu na 3V3 (+ volitelně 100nF na GND).
// Jádro někdy po analogRead() stáhne pull-up; před čtením ho proto znovu zapínáme.
static const int PIN_BUTTONS_ADC = 3; // 3 buttons over resistor ladder to GND
static const int ADC_BUTTON_SAMPLES = 16; // průměr proti šumu ADC

// Kalibrace tlačítek: na Serial každých 500 ms vypíše surovou hodnotu ADC (0..4095).
// Po doladění prahů nastav na 0 a znovu nahraj firmware.
#ifndef PRINT_ADC_CALIBRATION
#define PRINT_ADC_CALIBRATION 1
#endif
static const unsigned long ADC_CALIB_PRINT_MS = 500;
static unsigned long lastAdcCalibPrintMs = 0;

// -----------------------------
// OLED setup
// -----------------------------
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
// SH1106 128x64 přes hardware SPI (stejné pořadí pinů jako u Adafruit_SSD1306)
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, PIN_OLED_DC, PIN_OLED_RES, PIN_OLED_CS);

// -----------------------------
// Temperature sensor
// -----------------------------
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);

// -----------------------------
// Web + config storage
// -----------------------------
WebServer server(80);
Preferences prefs;

// -----------------------------
// Runtime settings/state
// -----------------------------
String wifiSsid;
String wifiPass;
bool apMode = false;

int schedOnHour = 8;
int schedOnMinute = 0;
int schedOffHour = 18;
int schedOffMinute = 0;

bool relayState = false;
bool manualOverride = false;
bool manualRelayState = false;

float waterTempC = NAN;
unsigned long lastTempReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastNtpSyncMs = 0;
unsigned long lastWifiRetryMs = 0;

// -----------------------------
// Time config
// -----------------------------
// Pro CZ použij "CET-1CEST,M3.5.0/2,M10.5.0/3"
static const char *TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const unsigned long NTP_SYNC_INTERVAL_MS = 60UL * 60UL * 1000UL; // 1h

// -----------------------------
// Button handling (ADC ladder)
// -----------------------------
enum ButtonId {
  BTN_NONE = 0,
  BTN_1 = 1,
  BTN_2 = 2,
  BTN_3 = 3
};

struct ButtonState {
  ButtonId stableBtn = BTN_NONE;
  ButtonId rawBtn = BTN_NONE;
  unsigned long changedMs = 0;
  unsigned long pressedMs = 0;
  bool shortPressFired = false;
  bool longPressFired = false;
};

ButtonState btn;
static const unsigned long BUTTON_DEBOUNCE_MS = 40;
static const unsigned long BUTTON_LONG_PRESS_MS = 1800;

// ADC thresholds (12-bit ADC, 0..4095).
// Dle reálných odporů je potřeba případně doladit.
static const int ADC_BTN1_MAX = 50;
static const int ADC_BTN2_MIN = 80;
static const int ADC_BTN2_MAX = 100;
static const int ADC_BTN3_MIN = 130;
static const int ADC_BTN3_MAX = 500;

// -----------------------------
// Menu state
// -----------------------------
enum UiMode {
  UI_HOME = 0,
  UI_EDIT_ON_H,
  UI_EDIT_ON_M,
  UI_EDIT_OFF_H,
  UI_EDIT_OFF_M,
  UI_SAVE
};

UiMode uiMode = UI_HOME;

// -----------------------------
// Utility
// -----------------------------
String twoDigits(int v) {
  if (v < 10) return "0" + String(v);
  return String(v);
}

String timeHmString(int h, int m) {
  return twoDigits(h) + ":" + twoDigits(m);
}

bool isNowInSchedule(int nowMinutes) {
  const int onMin = schedOnHour * 60 + schedOnMinute;
  const int offMin = schedOffHour * 60 + schedOffMinute;

  if (onMin == offMin) {
    // stejný čas -> vypnuto celý den (bezpečný default)
    return false;
  }

  if (onMin < offMin) {
    return (nowMinutes >= onMin && nowMinutes < offMin);
  }
  // Přes půlnoc
  return (nowMinutes >= onMin || nowMinutes < offMin);
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(PIN_RELAY, relayState ? HIGH : LOW);
}

void applyRelayLogic() {
  if (manualOverride) {
    setRelay(manualRelayState);
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 20)) {
    // Když není čas dostupný, nech aktuální stav.
    return;
  }
  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  setRelay(isNowInSchedule(nowMinutes));
}

void loadSettings() {
  prefs.begin("poolctl", true);
  schedOnHour = prefs.getInt("onH", 8);
  schedOnMinute = prefs.getInt("onM", 0);
  schedOffHour = prefs.getInt("offH", 18);
  schedOffMinute = prefs.getInt("offM", 0);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();

  schedOnHour = constrain(schedOnHour, 0, 23);
  schedOffHour = constrain(schedOffHour, 0, 23);
  schedOnMinute = constrain(schedOnMinute, 0, 59);
  schedOffMinute = constrain(schedOffMinute, 0, 59);
}

void saveSchedule() {
  prefs.begin("poolctl", false);
  prefs.putInt("onH", schedOnHour);
  prefs.putInt("onM", schedOnMinute);
  prefs.putInt("offH", schedOffHour);
  prefs.putInt("offM", schedOffMinute);
  prefs.end();
}

void saveWifiCredentials(const String &ssid, const String &pass) {
  prefs.begin("poolctl", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void startApMode() {
  apMode = true;
  WiFi.disconnect(true, true);
  delay(200);

  WiFi.mode(WIFI_AP);
  String apName = "PoolCtrl-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.softAP(apName.c_str(), "12345678");
}

bool connectSta() {
  if (wifiSsid.isEmpty()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 20000)) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

void syncTimeNow() {
  if (WiFi.status() != WL_CONNECTED) return;

  configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  struct tm t;
  if (getLocalTime(&t, 4000)) {
    lastNtpSyncMs = millis();
  }
}

// -----------------------------
// Buttons
// -----------------------------
static void adcButtonsEnsurePullUp() {
  pinMode(PIN_BUTTONS_ADC, INPUT_PULLUP);
  gpio_pullup_en((gpio_num_t)PIN_BUTTONS_ADC);
}

static int readAnalogButtonsAveraged() {
  adcButtonsEnsurePullUp();
  uint32_t sum = 0;
  for (int i = 0; i < ADC_BUTTON_SAMPLES; i++) {
    sum += analogRead(PIN_BUTTONS_ADC);
  }
  return (int)(sum / ADC_BUTTON_SAMPLES);
}

ButtonId readButtonRaw() {
  int v = readAnalogButtonsAveraged();
  if (v <= ADC_BTN1_MAX) return BTN_1;
  if (v >= ADC_BTN2_MIN && v <= ADC_BTN2_MAX) return BTN_2;
  if (v >= ADC_BTN3_MIN && v <= ADC_BTN3_MAX) return BTN_3;
  return BTN_NONE;
}

void processShortPress(ButtonId b);
void processLongPress(ButtonId b);

void updateButtons() {
  ButtonId nowRaw = readButtonRaw();
  unsigned long nowMs = millis();

  if (nowRaw != btn.rawBtn) {
    btn.rawBtn = nowRaw;
    btn.changedMs = nowMs;
  }

  if ((nowMs - btn.changedMs) < BUTTON_DEBOUNCE_MS) return;

  if (btn.stableBtn != btn.rawBtn) {
    // state changed after debounce
    if (btn.stableBtn != BTN_NONE && btn.rawBtn == BTN_NONE) {
      // released
      if (!btn.longPressFired) {
        processShortPress(btn.stableBtn);
      }
    } else if (btn.stableBtn == BTN_NONE && btn.rawBtn != BTN_NONE) {
      // pressed
      btn.pressedMs = nowMs;
      btn.longPressFired = false;
    }
    btn.stableBtn = btn.rawBtn;
  }

  if (btn.stableBtn != BTN_NONE && !btn.longPressFired) {
    if ((nowMs - btn.pressedMs) >= BUTTON_LONG_PRESS_MS) {
      btn.longPressFired = true;
      processLongPress(btn.stableBtn);
    }
  }
}

// -----------------------------
// UI / Menu
// -----------------------------
void processShortPress(ButtonId b) {
  if (uiMode == UI_HOME) {
    if (b == BTN_3) {
      uiMode = UI_EDIT_ON_H;
    }
    return;
  }

  if (uiMode == UI_SAVE) {
    if (b == BTN_1) {
      uiMode = UI_HOME; // cancel
      return;
    }
    if (b == BTN_2 || b == BTN_3) {
      saveSchedule();
      uiMode = UI_HOME;
      return;
    }
    return;
  }

  int *target = nullptr;
  int minVal = 0;
  int maxVal = 59;

  if (uiMode == UI_EDIT_ON_H) {
    target = &schedOnHour;
    minVal = 0;
    maxVal = 23;
  } else if (uiMode == UI_EDIT_ON_M) {
    target = &schedOnMinute;
  } else if (uiMode == UI_EDIT_OFF_H) {
    target = &schedOffHour;
    minVal = 0;
    maxVal = 23;
  } else if (uiMode == UI_EDIT_OFF_M) {
    target = &schedOffMinute;
  }

  if (target) {
    if (b == BTN_1) {
      *target = (*target <= minVal) ? maxVal : (*target - 1);
    } else if (b == BTN_2) {
      *target = (*target >= maxVal) ? minVal : (*target + 1);
    } else if (b == BTN_3) {
      uiMode = static_cast<UiMode>(static_cast<int>(uiMode) + 1);
    }
  }
}

void processLongPress(ButtonId b) {
  if (b == BTN_1) {
    manualOverride = true;
    manualRelayState = !relayState;
    setRelay(manualRelayState);
  } else if (b == BTN_3) {
    startApMode();
  }
}

void drawHomeScreen() {
  struct tm timeinfo;
  bool hasTime = getLocalTime(&timeinfo, 10);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.print("Time: ");
  if (hasTime) {
    display.print(twoDigits(timeinfo.tm_hour));
    display.print(":");
    display.print(twoDigits(timeinfo.tm_min));
    display.print(":");
    display.print(twoDigits(timeinfo.tm_sec));
  } else {
    display.print("--:--:--");
  }

  display.setCursor(0, 12);
  display.print("Water: ");
  if (isnan(waterTempC)) {
    display.print("--.- C");
  } else {
    display.print(String(waterTempC, 1));
    display.print(" C");
  }

  display.setCursor(0, 24);
  display.print("Relay: ");
  display.print(relayState ? "ON" : "OFF");
  if (manualOverride) display.print(" (MAN)");

  display.setCursor(0, 36);
  display.print("Sch ");
  display.print(timeHmString(schedOnHour, schedOnMinute));
  display.print("-");
  display.print(timeHmString(schedOffHour, schedOffMinute));

  display.setCursor(0, 48);
  display.print(apMode ? "AP mode" : "BTN3=menu, long BTN3=AP");

  display.display();
}

void drawEditScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.print("Schedule setup");

  display.setCursor(0, 14);
  display.print("ON : ");
  display.print(timeHmString(schedOnHour, schedOnMinute));

  display.setCursor(0, 26);
  display.print("OFF: ");
  display.print(timeHmString(schedOffHour, schedOffMinute));

  display.setCursor(0, 40);
  if (uiMode == UI_EDIT_ON_H) display.print("Edit ON hour");
  if (uiMode == UI_EDIT_ON_M) display.print("Edit ON minute");
  if (uiMode == UI_EDIT_OFF_H) display.print("Edit OFF hour");
  if (uiMode == UI_EDIT_OFF_M) display.print("Edit OFF minute");
  if (uiMode == UI_SAVE) display.print("Save? B1=No B2/B3=Yes");

  display.setCursor(0, 54);
  display.print("B1- B2+ B3 next");

  display.display();
}

void updateDisplay() {
  if (millis() - lastDisplayMs < 250) return;
  lastDisplayMs = millis();

  if (uiMode == UI_HOME) drawHomeScreen();
  else drawEditScreen();
}

// -----------------------------
// Web server
// -----------------------------
String htmlPage() {
  String ipInfo = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String modeInfo = apMode ? "AP" : "STA";

  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'><title>Pool Controller</title></head><body>";
  s += "<h2>Pool Controller</h2>";
  s += "<p><b>Mode:</b> " + modeInfo + " | <b>IP:</b> " + ipInfo + "</p>";
  s += "<p><b>Water temp:</b> " + String(isnan(waterTempC) ? -999.0f : waterTempC, 1) + " C</p>";
  s += "<p><b>Relay:</b> " + String(relayState ? "ON" : "OFF") + "</p>";
  s += "<p><b>Manual override:</b> " + String(manualOverride ? "ON" : "OFF") + "</p>";
  s += "<p><b>Schedule:</b> " + timeHmString(schedOnHour, schedOnMinute) + " - " + timeHmString(schedOffHour, schedOffMinute) + "</p>";
  s += "<hr>";
  s += "<h3>WiFi config</h3>";
  s += "<form method='POST' action='/savewifi'>";
  s += "SSID: <input name='ssid' value='" + wifiSsid + "'><br>";
  s += "Password: <input name='pass' type='password' value='" + wifiPass + "'><br>";
  s += "<button type='submit'>Save WiFi & Restart</button></form>";
  s += "<hr>";
  s += "<h3>Schedule config</h3>";
  s += "<form method='POST' action='/saveschedule'>";
  s += "ON (HH:MM): <input name='onh' size='2' value='" + String(schedOnHour) + "'> : ";
  s += "<input name='onm' size='2' value='" + String(schedOnMinute) + "'><br>";
  s += "OFF (HH:MM): <input name='offh' size='2' value='" + String(schedOffHour) + "'> : ";
  s += "<input name='offm' size='2' value='" + String(schedOffMinute) + "'><br>";
  s += "<button type='submit'>Save schedule</button></form>";
  s += "<hr>";
  s += "<p><a href='/toggle'>Toggle relay (manual)</a> | <a href='/auto'>Back to auto</a> | <a href='/status'>JSON status</a></p>";
  s += "</body></html>";
  return s;
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", htmlPage());
}

void handleStatus() {
  struct tm t;
  bool hasTime = getLocalTime(&t, 10);
  String nowStr = hasTime ? (twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min) + ":" + twoDigits(t.tm_sec)) : "--:--:--";

  String json = "{";
  json += "\"mode\":\"" + String(apMode ? "AP" : "STA") + "\",";
  json += "\"ip\":\"" + String(apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
  json += "\"time\":\"" + nowStr + "\",";
  json += "\"temperature_c\":" + String(isnan(waterTempC) ? -999.0f : waterTempC, 2) + ",";
  json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
  json += "\"manual_override\":" + String(manualOverride ? "true" : "false") + ",";
  json += "\"schedule_on\":\"" + timeHmString(schedOnHour, schedOnMinute) + "\",";
  json += "\"schedule_off\":\"" + timeHmString(schedOffHour, schedOffMinute) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSaveWiFi() {
  if (!server.hasArg("ssid")) {
    server.send(400, "text/plain", "Missing ssid");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  saveWifiCredentials(ssid, pass);
  server.send(200, "text/plain", "Saved. Restarting...");
  delay(800);
  ESP.restart();
}

void handleSaveSchedule() {
  if (!server.hasArg("onh") || !server.hasArg("onm") || !server.hasArg("offh") || !server.hasArg("offm")) {
    server.send(400, "text/plain", "Missing params");
    return;
  }
  schedOnHour = constrain(server.arg("onh").toInt(), 0, 23);
  schedOnMinute = constrain(server.arg("onm").toInt(), 0, 59);
  schedOffHour = constrain(server.arg("offh").toInt(), 0, 23);
  schedOffMinute = constrain(server.arg("offm").toInt(), 0, 59);
  saveSchedule();
  server.send(200, "text/plain", "Schedule saved");
}

void handleToggleRelay() {
  manualOverride = true;
  manualRelayState = !relayState;
  setRelay(manualRelayState);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirect");
}

void handleAutoMode() {
  manualOverride = false;
  applyRelayLogic();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirect");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/savewifi", HTTP_POST, handleSaveWiFi);
  server.on("/saveschedule", HTTP_POST, handleSaveSchedule);
  server.on("/toggle", HTTP_GET, handleToggleRelay);
  server.on("/auto", HTTP_GET, handleAutoMode);
  server.begin();
}

void printAdcCalibration() {
#if PRINT_ADC_CALIBRATION
  unsigned long now = millis();
  if (now - lastAdcCalibPrintMs < ADC_CALIB_PRINT_MS) return;
  lastAdcCalibPrintMs = now;
  int v = readAnalogButtonsAveraged();
  Serial.print("ADC buttons (GPIO");
  Serial.print(PIN_BUTTONS_ADC);
  Serial.print(") = ");
  Serial.println(v);
#endif
}

void updateTemperature() {
  if (millis() - lastTempReadMs < 5000) return;
  lastTempReadMs = millis();
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t > -55.0f && t < 125.0f) {
    waterTempC = t;
  }
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  adcButtonsEnsurePullUp();

  Serial.begin(115200);
  delay(200);

  loadSettings();

  ds18b20.begin();

  // Na ESP32-C3 je potřeba navázat SPI na konkrétní piny (SCK, MISO=-1, MOSI).
  SPI.begin(PIN_OLED_SCK, -1, PIN_OLED_SDA);

  // U SPI je první argument begin() ignorován; druhý = hard reset přes RES.
  if (!display.begin(0x3C, true)) {
    // Pokud OLED nenaběhne, pokračuj bez něj.
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Booting...");
    display.display();
  }

  bool connected = connectSta();
  if (connected) {
    apMode = false;
    // ---- IP address output to Serial added here ----
    Serial.print("WiFi connected, IP address: ");
    Serial.println(WiFi.localIP());
    // ------------------------------------------------
    syncTimeNow();
  } else {
    startApMode();
  }

  setupWebServer();
  // Úspávání WiFi zvyšuje šum na ADC — pro klidnější žebřík tlačítek vypnout sleep.
  WiFi.setSleep(false);
  applyRelayLogic();
}

void loop() {
  server.handleClient();
  printAdcCalibration();
  updateButtons();
  updateTemperature();

  if (!apMode && WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiRetryMs > 10000) {
      lastWifiRetryMs = millis();
      WiFi.reconnect();
    }
  }

  if (!apMode && WiFi.status() == WL_CONNECTED) {
    if ((millis() - lastNtpSyncMs) > NTP_SYNC_INTERVAL_MS) {
      syncTimeNow();
    }
  }

  applyRelayLogic();
  updateDisplay();
  delay(10);
}
