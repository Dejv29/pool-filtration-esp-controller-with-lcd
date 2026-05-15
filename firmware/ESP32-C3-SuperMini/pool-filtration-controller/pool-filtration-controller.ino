#include <WiFi.h>
#include <driver/gpio.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Hardware pin mapping (ESP32-C3 SuperMini HW-466AB)
static const int PIN_OLED_SCK = 21;
static const int PIN_OLED_SDA = 20;   // MOSI
static const int PIN_OLED_CS = 8;
static const int PIN_OLED_DC = 9;
static const int PIN_OLED_RES = 10;

static const int PIN_RELAY = 7;     // Relay transistor (BC337) base resistor
static const int PIN_ONEWIRE = 4;   // DS18B20 data pin
static const int PIN_SAFETY_SWITCH = 1;

// Na ESP32-C3 má GPIO3 ADC1_CH3 a HW pull-up/pull-down.
static const int PIN_BUTTONS_ADC = 3; // 4 buttons over resistor ladder to GND
static const int ADC_BUTTON_SAMPLES = 16; // průměr proti šumu ADC

// Kalibrace tlačítek: na Serial každých 500 ms vypíše surovou hodnotu ADC (0..4095).
#ifndef PRINT_ADC_CALIBRATION
#define PRINT_ADC_CALIBRATION 0
#endif
static const unsigned long ADC_CALIB_PRINT_MS = 500;
static unsigned long lastAdcCalibPrintMs = 0;

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
// SH1106 128x64 přes hardware SPI
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, PIN_OLED_DC, PIN_OLED_RES, PIN_OLED_CS);

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);

WebServer server(80);
Preferences prefs;

String wifiSsid;
String wifiPass;
bool apMode = false;
bool prefForceAp = false; // preferovaný režim (true = AP, false = Klient)
bool wifiModeChanged = false; // příznak změny režimu pro restart po opuštění menu

int schedOnHour = 8;
int schedOnMinute = 0;
int schedOffHour = 18;
int schedOffMinute = 0;
bool timerEnabled = true;

bool relayState = false;
bool manualOverride = false;
bool manualRelayState = false;
bool prevScheduleNow = false; // detekce přechodu plánovače ON->OFF

float waterTempC = NAN;
float pumpTempC = NAN;
DeviceAddress tempAddr0 = {0};
DeviceAddress tempAddr1 = {0};
bool hasTempAddr0 = false;
bool hasTempAddr1 = false;
bool tempSensorsSwap = false;
unsigned long lastTempReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastNtpSyncMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastUserActivityMs = 0;
bool prevWifiConnected = false;

static const unsigned long FLOOD_MANUAL_GRACE_MS = 60000;
int floodProtectTimeoutIdx = 2;
static const unsigned long FLOOD_PROTECT_TIMEOUTS_MS[] = {5000, 10000, 15000, 30000, 0};
static const char* FLOOD_PROTECT_TIMEOUT_LABELS[] = {"5s", "10s", "15s", "30s", "OFF"};

int tempProtectLimitIdx = 4;
static const int TEMP_PROTECT_LIMITS_C[] = {40, 50, 60, 70, 80, 90, 100, 0};
static const char* TEMP_PROTECT_LIMIT_LABELS[] = {"40 C", "50 C", "60 C", "70 C", "80 C", "90 C", "100 C", "OFF"};

enum FaultType {
  FAULT_NONE = 0,
  FAULT_NOT_FLOODED,
  FAULT_OVERHEAT
};

FaultType lockoutFault = FAULT_NONE;
FaultType errorScreenFault = FAULT_NONE;

unsigned long notFloodedSinceMs = 0;
bool manualRecoveryActive = false;
FaultType manualRecoveryReason = FAULT_NONE;
unsigned long manualRecoveryStartMs = 0;

// Pro CZ použij "CET-1CEST,M3.5.0/2,M10.5.0/3"
static const char *TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const unsigned long NTP_SYNC_INTERVAL_MS = 60UL * 60UL * 1000UL; // 1h

enum ButtonId {
  BTN_NONE = 0,
  BTN_1 = 1,
  BTN_2 = 2,
  BTN_3 = 3,
  BTN_4 = 4
};

struct ButtonState {
  ButtonId stableBtn = BTN_NONE;
  ButtonId rawBtn = BTN_NONE;
  unsigned long changedMs = 0;
  bool longPressHandled = false;
};

ButtonState btn;
static const unsigned long BUTTON_DEBOUNCE_MS = 40;
static const unsigned long BUTTON_LONG_PRESS_MS = 1000;

// ADC thresholds (12-bit ADC, 0..4095).
static const int ADC_BTN1_MAX = 450;
static const int ADC_BTN2_MIN = 650;
static const int ADC_BTN2_MAX = 750;
static const int ADC_BTN3_MIN = 920;
static const int ADC_BTN3_MAX = 1050;
static const int ADC_BTN4_MIN = 1200;
static const int ADC_BTN4_MAX = 1300;

static const int SOFTKEY_W = 31;
static const int SOFTKEY_H = 13;
static const int SOFTKEY_Y = 51;
static const int SOFTKEY_BITMAP_SIZE = (SOFTKEY_W * SOFTKEY_H + 7) / 8;

const uint8_t bitmap_filtrationOn[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x38, 0x84, 0x39, 0x38, 0x44, 0xc4, 0x42, 0x84, 0x44, 0xa4, 
	0xe1, 0x0e, 0x44, 0xa4, 0xb8, 0x3a, 0x44, 0xa4, 0xaf, 0xea, 0x44, 0x94, 0xaa, 0xaa, 0x44, 0x94, 
	0xaa, 0xaa, 0x44, 0x94, 0x6a, 0xac, 0x44, 0x94, 0x3a, 0xb8, 0x44, 0x8c, 0x0f, 0xe0, 0x38, 0x84, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_filtrationOff[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x73, 0xde, 0x39, 0x38, 0x8a, 0x10, 0x42, 0x84, 0x8a, 0x10, 
	0xe1, 0x0e, 0x8a, 0x10, 0xb8, 0x3a, 0x8a, 0x10, 0xaf, 0xea, 0x8b, 0x9c, 0xaa, 0xaa, 0x8a, 0x10, 
	0xaa, 0xaa, 0x8a, 0x10, 0x6a, 0xac, 0x8a, 0x10, 0x3a, 0xb8, 0x8a, 0x10, 0x0f, 0xe0, 0x72, 0x10, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_settings[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x1a, 0x58, 0x00, 0x00, 0x36, 0x6c, 0x00, 
	0x00, 0x20, 0x04, 0x00, 0x00, 0x11, 0x88, 0x00, 0x00, 0x0a, 0x50, 0x00, 0x00, 0x11, 0x88, 0x00, 
	0x00, 0x20, 0x04, 0x00, 0x00, 0x36, 0x6c, 0x00, 0x00, 0x1a, 0x58, 0x00, 0x00, 0x01, 0x80, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_plus[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 
	0x00, 0x03, 0x80, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x00, 
	0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_minus[SOFTKEY_BITMAP_SIZE] PROGMEM = {	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_arrowUp[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x07, 0xc0, 0x00, 
	0x00, 0x0f, 0xe0, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x3b, 0xb8, 0x00, 0x00, 0x73, 0x9c, 0x00, 
	0x00, 0xe3, 0x8e, 0x00, 0x00, 0xc3, 0x86, 0x00, 0x00, 0x83, 0x82, 0x00, 0x00, 0x03, 0x80, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_arrowDown[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0x07, 0x04, 0x00, 0x01, 0x87, 0x0c, 0x00, 
	0x01, 0xc7, 0x1c, 0x00, 0x00, 0xe7, 0x38, 0x00, 0x00, 0x77, 0x70, 0x00, 0x00, 0x3f, 0xe0, 0x00, 
	0x00, 0x1f, 0xc0, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_arrowLeft[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x7f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_arrowRight[SOFTKEY_BITMAP_SIZE] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x00, 0xe0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x3f, 0xf0, 0x00, 
	0x00, 0x00, 0xe0, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x07, 0x00, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_back[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 
	0x03, 0xc0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 
	0x03, 0xc0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 
	0x00, 0x00, 0x00};
const uint8_t bitmap_ok[SOFTKEY_BITMAP_SIZE] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xc3, 0x00, 0x01, 0xf8, 0xc7, 0x00, 0x03, 0x9c, 0xce, 0x00, 
	0x03, 0x0c, 0xdc, 0x00, 0x03, 0x0c, 0xf8, 0x00, 0x03, 0x0c, 0xf0, 0x00, 0x03, 0x0c, 0xf8, 0x00, 
	0x03, 0x0c, 0xdc, 0x00, 0x03, 0x9c, 0xce, 0x00, 0x01, 0xf8, 0xc7, 0x00, 0x00, 0xf0, 0xc3, 0x00, 
	0x00, 0x00, 0x00};

static const int WIFI_ICON_W = 11;
static const int WIFI_ICON_H = 11;
static const int WIFI_ICON_BYTES = ((WIFI_ICON_W + 7) / 8) * WIFI_ICON_H;

const uint8_t bitmap_wifiConnected[WIFI_ICON_BYTES] PROGMEM = {0x07, 0x00, 0x18, 0xc0, 0x22, 0x20, 0x0d, 0x80, 0x10, 0x40, 0x07, 0x00, 0x08, 0x80, 0x02, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Připojeno k WiFi
const uint8_t bitmap_wifiAp[WIFI_ICON_BYTES] PROGMEM = {0x73, 0xe0, 0x8a, 0x20, 0x8a, 0x20, 0x8a, 0x20, 0x8a, 0x20, 0xfb, 0xe0, 0x8a, 0x00, 0x8a, 0x00, 
	0x8a, 0x00, 0x8a, 0x00, 0x8a, 0x00};        // Režim AP

static const int TEMP_DIGIT_W = 24;
static const int TEMP_DIGIT_H = 64;
static const int TEMP_DIGIT_BYTES = (TEMP_DIGIT_W * TEMP_DIGIT_H + 7) / 8;
static const int TEMP_DEG_W = 40;
static const int TEMP_DEG_H = 64;
static const int TEMP_DEG_BYTES = (TEMP_DEG_W * TEMP_DEG_H + 7) / 8;
static const int TEMP_COMMA_W = 8;
static const int TEMP_COMMA_H = 10;
static const int TEMP_COMMA_BYTES = (TEMP_COMMA_W * TEMP_COMMA_H + 7) / 8;
static const int TEMP_MINUS_W = 18;
static const int TEMP_MINUS_H = 4;
static const int TEMP_MINUS_BYTES = (TEMP_MINUS_W * TEMP_MINUS_H + 7) / 8;
static const int TEMP_COMMA_Y = SCREEN_HEIGHT - TEMP_COMMA_H;
static const int TEMP_MINUS_Y = (SCREEN_HEIGHT - TEMP_MINUS_H) / 2;
static const int TEMP_CHAR_GAP = 3; // mezera mezi „znaky“ na celoobrazovce teploty

const uint8_t epd_bitmap_num_0[TEMP_DIGIT_BYTES] PROGMEM = {0x03, 0xff, 0xc0, 0x0f, 0xff, 0xf0, 0x3f, 0xff, 0xfc, 0x7e, 0x00, 0x7e, 0x78, 0x00, 0x3e, 0x78, 
	0x00, 0x1e, 0x70, 0x00, 0x1e, 0xe0, 0x00, 0x0f, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xc0, 0x00, 
	0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 
	0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 
	0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 
	0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 
	0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 
	0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 
	0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 
	0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 
	0x00, 0x03, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x0f, 0x70, 0x00, 0x1e, 0x78, 0x00, 
	0x1e, 0x78, 0x00, 0x3e, 0x7e, 0x00, 0x7e, 0x3f, 0xff, 0xfc, 0x0f, 0xff, 0xf0, 0x03, 0xff, 0xc0};
const uint8_t epd_bitmap_num_1[TEMP_DIGIT_BYTES] PROGMEM = {0x00, 0x07, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x7f, 0x00, 0x00, 
	0xf3, 0x00, 0x01, 0xe3, 0x00, 0x03, 0xc3, 0x00, 0x03, 0x03, 0x00, 0x02, 0x03, 0x00, 0x00, 0x03, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00};
const uint8_t epd_bitmap_num_2[TEMP_DIGIT_BYTES] PROGMEM = {0x1f, 0xff, 0x80, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xf0, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x38, 0x00, 
	0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x06, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 
	0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 
	0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x1e, 0x00, 0x00, 
	0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x38, 
	0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x70, 0x00, 0x00, 0xe0, 0x00, 0x01, 0xc0, 0x00, 
	0x01, 0xc0, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x07, 0x80, 0x00, 0x07, 0x80, 0x00, 0x0f, 
	0x00, 0x00, 0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x78, 0x00, 
	0x00, 0xf8, 0x00, 0x01, 0xf0, 0x00, 0x01, 0xe0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x07, 
	0x80, 0x00, 0x07, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x7c, 0x00, 
	0x00, 0x78, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const uint8_t epd_bitmap_num_3[TEMP_DIGIT_BYTES] PROGMEM = {0x00, 0x3f, 0x80, 0x7f, 0xff, 0xe0, 0xff, 0xff, 0xf0, 0xff, 0x00, 0xf8, 0x04, 0x00, 0x3c, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 
	0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 
	0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 
	0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 
	0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0xf8, 0x00, 0x7f, 0xf8, 0x0f, 0xff, 0xf0, 
	0x1f, 0xff, 0xf0, 0x0f, 0xff, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x1c, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 
	0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 
	0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 
	0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x1c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x78, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0x80};
const uint8_t epd_bitmap_num_4[TEMP_DIGIT_BYTES] PROGMEM = {0x00, 0x01, 0x80, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 
	0x07, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 
	0x00, 0x00, 0x38, 0x00, 0x00, 0x38, 0x00, 0x00, 0x38, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 
	0x00, 0x60, 0x00, 0x00, 0xe0, 0x1c, 0x00, 0xe0, 0x1c, 0x01, 0xc0, 0x1c, 0x01, 0xc0, 0x1c, 0x01, 
	0xc0, 0x1c, 0x03, 0xc0, 0x1c, 0x03, 0x80, 0x1c, 0x03, 0x80, 0x1c, 0x03, 0x00, 0x1c, 0x07, 0x00, 
	0x1c, 0x07, 0x00, 0x1c, 0x0e, 0x00, 0x1c, 0x06, 0x00, 0x1c, 0x0e, 0x00, 0x1c, 0x1c, 0x00, 0x1c, 
	0x3c, 0x00, 0x1c, 0x3c, 0x00, 0x1c, 0x3c, 0x00, 0x1c, 0x38, 0x00, 0x1c, 0x78, 0x00, 0x1c, 0x78, 
	0x00, 0x1c, 0x78, 0x00, 0x1c, 0xf0, 0x00, 0x1c, 0xe0, 0x00, 0x1c, 0xc0, 0x00, 0x1c, 0xe0, 0x00, 
	0x1c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 
	0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c};
const uint8_t epd_bitmap_num_5[TEMP_DIGIT_BYTES] PROGMEM = {0x7f, 0xff, 0xfe, 0x7f, 0xff, 0xfe, 0x7f, 0xff, 0xfe, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 
	0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 
	0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 
	0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 
	0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 
	0x00, 0x63, 0xff, 0xc0, 0x67, 0xff, 0xf0, 0x7f, 0xff, 0xfc, 0xfe, 0x00, 0x3c, 0xfc, 0x00, 0x1c, 
	0xf8, 0x00, 0x0e, 0xf0, 0x00, 0x06, 0xf0, 0x00, 0x06, 0xe0, 0x00, 0x06, 0xc0, 0x00, 0x07, 0xc0, 
	0x00, 0x03, 0xc0, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 0x00, 
	0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x7c, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xc0};
const uint8_t epd_bitmap_num_6[TEMP_DIGIT_BYTES] PROGMEM = {0x03, 0xff, 0xc0, 0x07, 0xff, 0xfe, 0x0f, 0xff, 0xfe, 0x1e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x38, 
	0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 
	0x00, 0xe0, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xc0, 0x00, 0x00, 
	0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 
	0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0xc0, 0x00, 0x00, 0xc7, 0xff, 0xc0, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xf8, 0xe0, 0x00, 0x7c, 
	0xc0, 0x00, 0x3e, 0xc0, 0x00, 0x1e, 0xc0, 0x00, 0x0e, 0xc0, 0x00, 0x06, 0xc0, 0x00, 0x07, 0xc0, 
	0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 
	0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 
	0xc0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0x60, 
	0x00, 0x07, 0x60, 0x00, 0x07, 0x78, 0x00, 0x06, 0x78, 0x00, 0x06, 0x78, 0x00, 0x0e, 0x38, 0x00, 
	0x1e, 0x3e, 0x00, 0x3c, 0x1f, 0x00, 0x78, 0x0f, 0xff, 0xf8, 0x07, 0xff, 0xf8, 0x03, 0xff, 0xc0};
const uint8_t epd_bitmap_num_7[TEMP_DIGIT_BYTES] PROGMEM = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 
	0x00, 0x1f, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 
	0x00, 0x00, 0x38, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 
	0x00, 0x78, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xe0, 0x00, 0x00, 
	0xc0, 0x00, 0x01, 0xc0, 0x00, 0x01, 0xc0, 0x00, 0x01, 0xc0, 0x00, 0x01, 0xc0, 0x00, 0x01, 0xc0, 
	0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 
	0x03, 0x80, 0x00, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 
	0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 
	0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x3c, 0x00, 0x00, 0x38, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 
	0x00, 0x00, 0x78, 0x00, 0x00, 0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0xe0, 0x00, 0x00, 0xe0, 0x00};
const uint8_t epd_bitmap_num_8[TEMP_DIGIT_BYTES] PROGMEM = {0x03, 0xff, 0xc0, 0x0f, 0xff, 0xe0, 0x3f, 0xff, 0xfc, 0x3e, 0x00, 0x7c, 0x3c, 0x00, 0x1c, 0x78, 
	0x00, 0x1e, 0x60, 0x00, 0x1e, 0xe0, 0x00, 0x0e, 0xe0, 0x00, 0x06, 0xe0, 0x00, 0x06, 0xc0, 0x00, 
	0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 
	0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 
	0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x06, 0xe0, 0x00, 
	0x06, 0xe0, 0x00, 0x1e, 0x78, 0x00, 0x1e, 0x7c, 0x00, 0x7c, 0x3f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 
	0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xfc, 0x3e, 0x00, 0x3e, 0x7c, 0x00, 0x1e, 0x60, 0x00, 0x1e, 0xe0, 
	0x00, 0x06, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 
	0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 
	0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xc0, 
	0x00, 0x07, 0xc0, 0x00, 0x07, 0xe0, 0x00, 0x06, 0xe0, 0x00, 0x0e, 0xe0, 0x00, 0x1e, 0x78, 0x00, 
	0x1e, 0x3c, 0x00, 0x3c, 0x3e, 0x00, 0x7c, 0x3f, 0xff, 0xfc, 0x0f, 0xff, 0xe0, 0x03, 0xff, 0xc0};
const uint8_t epd_bitmap_num_9[TEMP_DIGIT_BYTES] PROGMEM = {0x03, 0xff, 0x80, 0x0f, 0xff, 0xe0, 0x3f, 0xff, 0xe0, 0x3e, 0x00, 0xf0, 0x3c, 0x00, 0x78, 0x78, 
	0x00, 0x7c, 0x70, 0x00, 0x3c, 0x60, 0x00, 0x3c, 0x60, 0x00, 0x1e, 0xe0, 0x00, 0x1e, 0xe0, 0x00, 
	0x1e, 0xe0, 0x00, 0x0e, 0xe0, 0x00, 0x06, 0xe0, 0x00, 0x06, 0xe0, 0x00, 0x06, 0xe0, 0x00, 0x07, 
	0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 0x00, 0x07, 0xe0, 
	0x00, 0x07, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x03, 0xe0, 0x00, 
	0x03, 0xe0, 0x00, 0x03, 0x60, 0x00, 0x03, 0x60, 0x00, 0x03, 0x78, 0x00, 0x03, 0x7c, 0x00, 0x03, 
	0x3e, 0x00, 0x07, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff, 0x03, 0xff, 0xc3, 0x00, 0x00, 0x03, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 
	0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0e, 0x00, 
	0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 0x00, 0x00, 
	0x3c, 0x00, 0x00, 0x78, 0x00, 0x00, 0xf0, 0x7f, 0xff, 0xe0, 0x7f, 0xff, 0xe0, 0x07, 0xff, 0xc0};
const uint8_t epd_bitmap_num_degrees_of_c[TEMP_DEG_BYTES] PROGMEM = {0x03, 0xc0, 0x00, 0x7f, 0x80, 0x0f, 0xe0, 0x07, 0xff, 0xfc, 0x3c, 0x38, 0x0f, 0xff, 0xfe, 0x70, 
	0x18, 0x0f, 0x80, 0x1e, 0x60, 0x1c, 0x1f, 0x00, 0x08, 0x60, 0x0c, 0x1e, 0x00, 0x00, 0xc0, 0x0e, 
	0x3c, 0x00, 0x00, 0xc0, 0x0e, 0x38, 0x00, 0x00, 0xc0, 0x0e, 0x70, 0x00, 0x00, 0xc0, 0x0e, 0x70, 
	0x00, 0x00, 0x60, 0x1c, 0x70, 0x00, 0x00, 0x30, 0x3c, 0x60, 0x00, 0x00, 0x1f, 0xf8, 0x60, 0x00, 
	0x00, 0x0f, 0xf0, 0x60, 0x00, 0x00, 0x07, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 
	0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 
	0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 
	0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
	0x30, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x1c, 
	0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x08, 0x00, 0x00, 0x0f, 0xc0, 
	0x1e, 0x00, 0x00, 0x07, 0xff, 0xfe, 0x00, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0x80};
// °C 39×64 px = 2496 bitů → přesně 312 B (export může mít 315+ B — oříznout na 312)
const uint8_t epd_bitmap_num_comma[TEMP_COMMA_BYTES] PROGMEM = {0x30, 0x70, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xe0, 0xc0};
// čárka 5×10 → 50 bitů → 7 B (ne 10)
const uint8_t epd_bitmap_num_minus[TEMP_MINUS_BYTES] PROGMEM = {0xff, 0xfe, 0x00, 0x7f, 0xff, 0x00, 0x3f, 0xff, 0x80};
// mínus 18×4 → 72 b → 9 B (ne 12)

static const uint8_t *const epd_bitmap_digits[10] = {
    epd_bitmap_num_0, epd_bitmap_num_1, epd_bitmap_num_2, epd_bitmap_num_3, epd_bitmap_num_4,
    epd_bitmap_num_5, epd_bitmap_num_6, epd_bitmap_num_7, epd_bitmap_num_8, epd_bitmap_num_9};

enum UiScreen {
  SCREEN_HOME = 0,
  SCREEN_TIMER,
  SCREEN_TEMP_FULL,
  SCREEN_SETTINGS,
  SCREEN_TIME_SET,
  SCREEN_INFO,
  SCREEN_ERROR
};

enum TimerEditPart {
  TIMER_EDIT_NONE = 0,
  TIMER_EDIT_HOUR,
  TIMER_EDIT_MINUTE
};

UiScreen uiScreen = SCREEN_HOME;
int timerSelectedRow = 0; // 0=ON, 1=OFF, 2=Aktivni
TimerEditPart timerEditPart = TIMER_EDIT_NONE;

int settingsSelectedRow = 0; // 0=WifiMode, 1=FullscreenTemp, 2=Hlidat zahlceni, 3=Tep. ochrana, 4=Informace, 5=Nastavit cas, 6=Prohodit cidla, 7=RESET CIDEL
int timeSetHour = 12;
int timeSetMinute = 0;
TimerEditPart timeSetEditPart = TIMER_EDIT_NONE;
UiScreen timeSetReturnScreen = SCREEN_SETTINGS;
bool timeSetAfterSaveShowTimer = false;
int fullscreenTimeoutIdx = 1; // default 15s (index 1)
static const int FULLSCREEN_TIMEOUTS[] = {0, 15000, 30000, 45000, 60000};
static const char* FULLSCREEN_TIMEOUT_LABELS[] = {"OFF", "15s", "30s", "45s", "60s"};

String twoDigits(int v) {
  if (v < 10) return "0" + String(v);
  return String(v);
}

String timeHmString(int h, int m) {
  return twoDigits(h) + ":" + twoDigits(m);
}

bool isNowInSchedule(int nowMinutes) {
  if (!timerEnabled) return false;

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

/** V auto režimu: zda by časovač právě chtěl relé zapnuté. */
bool scheduleWouldBeOnNow() {
  struct tm t;
  if (!getLocalTime(&t, 10)) return false;
  return isNowInSchedule(t.tm_hour * 60 + t.tm_min);
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(PIN_RELAY, relayState ? HIGH : LOW);
}

void applyRelayLogic() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 20)) {
    if (manualOverride) setRelay(manualRelayState);
    else if (lockoutFault != FAULT_NONE) setRelay(false);
    return;
  }

  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  bool scheduleNow = isNowInSchedule(nowMinutes);

  if (manualOverride) {
    // Po přechodu časovače do OFF automaticky zrušíme MAN režim, pokud bylo relé vypnuto ručně během ON periody.
    if (!manualRelayState && prevScheduleNow && !scheduleNow) {
      manualOverride = false;
    } else {
      setRelay(manualRelayState);
      prevScheduleNow = scheduleNow;
      return;
    }
  }

  if (lockoutFault != FAULT_NONE) {
    setRelay(false);
    prevScheduleNow = scheduleNow;
    return;
  }

  setRelay(scheduleNow);
  prevScheduleNow = scheduleNow;
}

void loadSettings() {
  prefs.begin("poolctl", true);
  schedOnHour = prefs.getInt("onH", 8);
  schedOnMinute = prefs.getInt("onM", 0);
  schedOffHour = prefs.getInt("offH", 18);
  schedOffMinute = prefs.getInt("offM", 0);
  timerEnabled = prefs.getBool("timerEn", true);
  prefForceAp = prefs.getBool("forceAp", false);
  fullscreenTimeoutIdx = prefs.getInt("fsTmo", 1);
  floodProtectTimeoutIdx = prefs.getInt("floodTmo", 2);
  tempProtectLimitIdx = prefs.getInt("tempLim", 4);
  tempSensorsSwap = prefs.getBool("tSwap", false);
  hasTempAddr0 = false;
  hasTempAddr1 = false;
  if (prefs.getBytesLength("t0") == 8) {
    prefs.getBytes("t0", tempAddr0, 8);
    hasTempAddr0 = true;
  }
  if (prefs.getBytesLength("t1") == 8) {
    prefs.getBytes("t1", tempAddr1, 8);
    hasTempAddr1 = true;
  }
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();

  schedOnHour = constrain(schedOnHour, 0, 23);
  schedOffHour = constrain(schedOffHour, 0, 23);
  schedOnMinute = constrain(schedOnMinute, 0, 59);
  schedOffMinute = constrain(schedOffMinute, 0, 59);
  fullscreenTimeoutIdx = constrain(fullscreenTimeoutIdx, 0, 4);
  floodProtectTimeoutIdx = constrain(floodProtectTimeoutIdx, 0, 4);
  tempProtectLimitIdx = constrain(tempProtectLimitIdx, 0, 7);
}

void saveSchedule() {
  prefs.begin("poolctl", false);
  prefs.putInt("onH", schedOnHour);
  prefs.putInt("onM", schedOnMinute);
  prefs.putInt("offH", schedOffHour);
  prefs.putInt("offM", schedOffMinute);
  prefs.putBool("timerEn", timerEnabled);
  prefs.end();
}

void saveWifiMode() {
  prefs.begin("poolctl", false);
  prefs.putBool("forceAp", prefForceAp);
  prefs.end();
}

void saveFullscreenSettings() {
  prefs.begin("poolctl", false);
  prefs.putInt("fsTmo", fullscreenTimeoutIdx);
  prefs.end();
}

void saveProtectionSettings() {
  prefs.begin("poolctl", false);
  prefs.putInt("floodTmo", floodProtectTimeoutIdx);
  prefs.putInt("tempLim", tempProtectLimitIdx);
  prefs.end();
}

void saveTempSensorSwap() {
  prefs.begin("poolctl", false);
  prefs.putBool("tSwap", tempSensorsSwap);
  prefs.end();
}

void clearTempSensorAddresses() {
  prefs.begin("poolctl", false);
  prefs.remove("t0");
  prefs.remove("t1");
  prefs.putBool("tSwap", false);
  prefs.end();
  hasTempAddr0 = false;
  hasTempAddr1 = false;
  tempSensorsSwap = false;
}

void saveTempSensorAddresses() {
  prefs.begin("poolctl", false);
  prefs.putBytes("t0", tempAddr0, 8);
  prefs.putBytes("t1", tempAddr1, 8);
  prefs.end();
  hasTempAddr0 = true;
  hasTempAddr1 = true;
}

void pairTempSensorsIfNeeded() {
  if (hasTempAddr0 && hasTempAddr1) return;
  int count = ds18b20.getDeviceCount();
  if (count < 2) return;

  DeviceAddress a = {0};
  bool got0 = false;
  bool got1 = false;

  for (int i = 0; i < count; i++) {
    if (!ds18b20.getAddress(a, i)) continue;
    if (!got0) {
      memcpy(tempAddr0, a, 8);
      got0 = true;
    } else if (!got1) {
      memcpy(tempAddr1, a, 8);
      got1 = true;
      break;
    }
  }

  if (got0 && got1) {
    saveTempSensorAddresses();
  }
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
  WiFi.softAP("PoolControlSetup");
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

static void setSystemTimeFromLocalHm(int h, int m) {
  setenv("TZ", TZ_INFO, 1);
  tzset();

  struct tm t;
  if (!getLocalTime(&t, 10)) {
    memset(&t, 0, sizeof(t));
    t.tm_year = 2024 - 1900;
    t.tm_mon = 0;
    t.tm_mday = 1;
  }

  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = 0;
  t.tm_isdst = -1;

  time_t epoch = mktime(&t);
  if (epoch < 0) return;

  timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  lastNtpSyncMs = millis();
}

static bool hasValidSystemTime() {
  struct tm t;
  if (!getLocalTime(&t, 10)) return false;
  return t.tm_year >= (2023 - 1900);
}

static bool isFlooded() {
  return digitalRead(PIN_SAFETY_SWITCH) == LOW;
}

static unsigned long floodProtectionTimeoutMs() {
  return FLOOD_PROTECT_TIMEOUTS_MS[floodProtectTimeoutIdx];
}

static int tempProtectionLimitC() {
  return TEMP_PROTECT_LIMITS_C[tempProtectLimitIdx];
}

static void triggerFault(FaultType fault) {
  setRelay(false);
  manualOverride = false;
  manualRelayState = false;
  manualRecoveryActive = false;
  notFloodedSinceMs = 0;
  lockoutFault = fault;
  errorScreenFault = fault;
  uiScreen = SCREEN_ERROR;
}

void updateProtections() {
  unsigned long now = millis();
  bool floodedNow = isFlooded();
  unsigned long floodTmoMs = floodProtectionTimeoutMs();
  int tempLimitC = tempProtectionLimitC();

  if (manualRecoveryActive) {
    if (manualRecoveryReason == FAULT_NOT_FLOODED) {
      if (floodedNow) {
        manualRecoveryActive = false;
        lockoutFault = FAULT_NONE;
        notFloodedSinceMs = 0;
      } else if (now - manualRecoveryStartMs >= FLOOD_MANUAL_GRACE_MS) {
        triggerFault(FAULT_NOT_FLOODED);
      }
      return;
    }
    if (manualRecoveryReason == FAULT_OVERHEAT) {
      if (tempLimitC > 0 && !isnan(pumpTempC) && pumpTempC >= (float)tempLimitC) {
        triggerFault(FAULT_OVERHEAT);
        return;
      }
      manualRecoveryActive = false;
      lockoutFault = FAULT_NONE;
    }
  }

  if (relayState) {
    if (floodTmoMs > 0) {
      if (!floodedNow) {
        if (notFloodedSinceMs == 0) notFloodedSinceMs = now;
        if (now - notFloodedSinceMs >= floodTmoMs) {
          triggerFault(FAULT_NOT_FLOODED);
          return;
        }
      } else {
        notFloodedSinceMs = 0;
      }
    } else {
      notFloodedSinceMs = 0;
    }

    if (tempLimitC > 0 && !isnan(pumpTempC) && pumpTempC >= (float)tempLimitC) {
      triggerFault(FAULT_OVERHEAT);
      return;
    }
  } else {
    notFloodedSinceMs = 0;
  }
}

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
  if (v >= ADC_BTN4_MIN && v <= ADC_BTN4_MAX) return BTN_4;
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
    if (btn.stableBtn != BTN_NONE && btn.rawBtn == BTN_NONE) {
      if (!btn.longPressHandled) {
        processShortPress(btn.stableBtn);
      }
      btn.longPressHandled = false;
    }
    btn.stableBtn = btn.rawBtn;
  } else if (btn.stableBtn != BTN_NONE && !btn.longPressHandled) {
    if ((nowMs - btn.changedMs) >= BUTTON_LONG_PRESS_MS) {
      btn.longPressHandled = true;
      processLongPress(btn.stableBtn);
    }
  }
}

void drawSoftkeys(const uint8_t *k1, const uint8_t *k2, const uint8_t *k3, const uint8_t *k4) {
  const uint8_t *keys[4] = {k1, k2, k3, k4};
  display.drawFastHLine(0, SOFTKEY_Y - 1, SCREEN_WIDTH, SH110X_WHITE);
  for (int i = 0; i < 4; i++) {
    if (keys[i] == nullptr) continue;
    int x = i * 32;
    display.drawBitmap(x, SOFTKEY_Y, keys[i], SOFTKEY_W, SOFTKEY_H, SH110X_WHITE);
  }
}

void drawLabeledTimeRow(const char *label, int y, int h, int m, bool rowSelected, bool editHour, bool editMinute) {
  const bool invertRow = rowSelected && !editHour && !editMinute;
  if (invertRow) {
    display.fillRect(0, y - 1, SCREEN_WIDTH, 10, SH110X_WHITE);
  }

  display.setTextColor(invertRow ? SH110X_BLACK : SH110X_WHITE);
  display.setCursor(0, y);
  display.print(label);
  display.print(": ");

  int xHour = 24;
  int xMinute = 42;
  if (editHour) {
    display.fillRect(xHour - 1, y - 1, 14, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(xHour, y);
    display.print(twoDigits(h));
    display.setTextColor(SH110X_WHITE);
  } else {
    display.setTextColor(invertRow ? SH110X_BLACK : SH110X_WHITE);
    display.setCursor(xHour, y);
    display.print(twoDigits(h));
  }

  display.setTextColor(invertRow ? SH110X_BLACK : SH110X_WHITE);
  display.setCursor(36, y);
  display.print(":");

  if (editMinute) {
    display.fillRect(xMinute - 1, y - 1, 14, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(xMinute, y);
    display.print(twoDigits(m));
    display.setTextColor(SH110X_WHITE);
  } else {
    display.setTextColor(invertRow ? SH110X_BLACK : SH110X_WHITE);
    display.setCursor(xMinute, y);
    display.print(twoDigits(m));
  }

  display.setTextColor(SH110X_WHITE);
}

void noteUserActivity() {
  lastUserActivityMs = millis();
}

/** Po nastaveném čase bez stisku tlačítka přejde z hlavní obrazovky nebo časovače na celoobrazovkovou teplotu. */
void checkIdleToTempScreen() {
  if (uiScreen == SCREEN_TEMP_FULL) return;
  if (uiScreen != SCREEN_HOME && uiScreen != SCREEN_TIMER) return;
  
  int timeoutMs = FULLSCREEN_TIMEOUTS[fullscreenTimeoutIdx];
  if (timeoutMs <= 0) return; // OFF

  if (millis() - lastUserActivityMs < (unsigned long)timeoutMs) return;
  uiScreen = SCREEN_TEMP_FULL;
}

void processLongPress(ButtonId b) {
  noteUserActivity();
  if (uiScreen == SCREEN_HOME && b == BTN_4) {
    uiScreen = SCREEN_SETTINGS;
  }
}

void processShortPress(ButtonId b) {
  noteUserActivity();

  if (uiScreen == SCREEN_ERROR) {
    if (b == BTN_4) {
      errorScreenFault = FAULT_NONE;
      uiScreen = SCREEN_HOME;
    }
    return;
  }

  if (uiScreen == SCREEN_TEMP_FULL) {
    uiScreen = SCREEN_HOME;
    return;
  }

  if (uiScreen == SCREEN_TIME_SET) {
    if (timeSetEditPart == TIMER_EDIT_NONE) {
      if (b == BTN_1) {
        uiScreen = timeSetReturnScreen;
        return;
      }
      if (b == BTN_4) {
        timeSetEditPart = TIMER_EDIT_HOUR;
        return;
      }
      return;
    }

    if (b == BTN_1) {
      timeSetEditPart = TIMER_EDIT_NONE;
      return;
    }

    if (timeSetEditPart == TIMER_EDIT_HOUR) {
      if (b == BTN_2) timeSetHour = (timeSetHour + 23) % 24;
      if (b == BTN_3) timeSetHour = (timeSetHour + 1) % 24;
      if (b == BTN_4) timeSetEditPart = TIMER_EDIT_MINUTE;
      return;
    }

    if (timeSetEditPart == TIMER_EDIT_MINUTE) {
      if (b == BTN_2) timeSetMinute = (timeSetMinute + 59) % 60;
      if (b == BTN_3) timeSetMinute = (timeSetMinute + 1) % 60;
      if (b == BTN_4) {
        timeSetEditPart = TIMER_EDIT_NONE;
        setSystemTimeFromLocalHm(timeSetHour, timeSetMinute);
        if (timeSetAfterSaveShowTimer) {
          timeSetAfterSaveShowTimer = false;
          uiScreen = SCREEN_TIMER;
          timerSelectedRow = 0;
          timerEditPart = TIMER_EDIT_NONE;
        } else {
          uiScreen = SCREEN_SETTINGS;
        }
      }
      return;
    }
  }

  if (uiScreen == SCREEN_SETTINGS) {
    if (b == BTN_1) {
      if (wifiModeChanged) {
        display.clearDisplay();
        display.setCursor(24, 28);
        display.print("Restartuji...");
        display.display();
        delay(1000);
        ESP.restart();
      }
      uiScreen = SCREEN_HOME;
    } else if (b == BTN_2) {
      settingsSelectedRow = (settingsSelectedRow + 7) % 8;
    } else if (b == BTN_3) {
      settingsSelectedRow = (settingsSelectedRow + 1) % 8;
    } else if (b == BTN_4) {
      if (settingsSelectedRow == 0) {
        prefForceAp = !prefForceAp;
        wifiModeChanged = !wifiModeChanged; // toggle flag (při sudém počtu změn se nerestartuje)
        saveWifiMode();
      } else if (settingsSelectedRow == 1) {
        fullscreenTimeoutIdx = (fullscreenTimeoutIdx + 1) % 5;
        saveFullscreenSettings();
      } else if (settingsSelectedRow == 2) {
        floodProtectTimeoutIdx = (floodProtectTimeoutIdx + 1) % 5;
        saveProtectionSettings();
      } else if (settingsSelectedRow == 3) {
        tempProtectLimitIdx = (tempProtectLimitIdx + 1) % 8;
        saveProtectionSettings();
      } else if (settingsSelectedRow == 4) {
        uiScreen = SCREEN_INFO;
      } else if (settingsSelectedRow == 5) {
        struct tm t;
        if (getLocalTime(&t, 10)) {
          timeSetHour = t.tm_hour;
          timeSetMinute = t.tm_min;
        } else {
          timeSetHour = 12;
          timeSetMinute = 0;
        }
        timeSetEditPart = TIMER_EDIT_NONE;
        timeSetReturnScreen = SCREEN_SETTINGS;
        timeSetAfterSaveShowTimer = false;
        uiScreen = SCREEN_TIME_SET;
      } else if (settingsSelectedRow == 6) {
        tempSensorsSwap = !tempSensorsSwap;
        saveTempSensorSwap();
      } else if (settingsSelectedRow == 7) {
        clearTempSensorAddresses();
        display.clearDisplay();
        display.setCursor(20, 28);
        display.print("Reset cidel...");
        display.display();
        delay(800);
        ESP.restart();
      }
    }
    return;
  }

  if (uiScreen == SCREEN_INFO) {
    if (b == BTN_1) {
      uiScreen = SCREEN_SETTINGS;
    } else if (b == BTN_2) {
      uiScreen = SCREEN_SETTINGS;
    } else if (b == BTN_3) {
      // arrowDown - placeholder
    }
    return;
  }

  if (uiScreen == SCREEN_HOME) {
    if (b == BTN_1) {
      // Ikona: ON stav (auto nebo manuál) -> filtrationOff = vypnout / ukončit manuál ON
      //        OFF stav -> filtrationOn = zapnout / ukončit manuál OFF
      if (manualOverride) {
        manualOverride = false;
        manualRecoveryActive = false;
        applyRelayLogic();
      } else if (lockoutFault != FAULT_NONE) {
        if (lockoutFault == FAULT_OVERHEAT) {
          int limitC = tempProtectionLimitC();
          if (limitC > 0 && !isnan(pumpTempC) && pumpTempC >= (float)limitC) {
            errorScreenFault = FAULT_OVERHEAT;
            uiScreen = SCREEN_ERROR;
            return;
          }
        }
        manualOverride = true;
        manualRelayState = true;
        setRelay(true);
        manualRecoveryActive = true;
        manualRecoveryReason = lockoutFault;
        manualRecoveryStartMs = millis();
      } else {
        if (scheduleWouldBeOnNow()) {
          manualOverride = true;
          manualRelayState = false;
          setRelay(false);
        } else {
          manualOverride = true;
          manualRelayState = true;
          setRelay(true);
        }
      }
    } else if (b == BTN_4) {
      if (!hasValidSystemTime()) {
        timeSetHour = 12;
        timeSetMinute = 0;
        timeSetEditPart = TIMER_EDIT_NONE;
        timeSetReturnScreen = SCREEN_HOME;
        timeSetAfterSaveShowTimer = true;
        uiScreen = SCREEN_TIME_SET;
      } else {
        uiScreen = SCREEN_TIMER;
        timerSelectedRow = 0;
        timerEditPart = TIMER_EDIT_NONE;
      }
    }
    return;
  }

  if (uiScreen == SCREEN_TIMER && timerEditPart == TIMER_EDIT_NONE) {
    if (b == BTN_1) {
      uiScreen = SCREEN_HOME;
      return;
    }
    if (b == BTN_2) {
      timerSelectedRow = (timerSelectedRow + 2) % 3;
      return;
    }
    if (b == BTN_3) {
      timerSelectedRow = (timerSelectedRow + 1) % 3;
      return;
    }
    if (b == BTN_4) {
      if (timerSelectedRow == 2) {
        timerEnabled = !timerEnabled;
        saveSchedule();
      } else {
        timerEditPart = TIMER_EDIT_HOUR;
      }
    }
    return;
  }

  if (uiScreen == SCREEN_TIMER && timerEditPart != TIMER_EDIT_NONE) {
    if (b == BTN_1) {
      timerEditPart = TIMER_EDIT_NONE;
      return;
    }

    int *targetHour = (timerSelectedRow == 0) ? &schedOnHour : &schedOffHour;
    int *targetMinute = (timerSelectedRow == 0) ? &schedOnMinute : &schedOffMinute;

    if (timerEditPart == TIMER_EDIT_HOUR) {
      if (b == BTN_2) *targetHour = (*targetHour + 23) % 24;
      if (b == BTN_3) *targetHour = (*targetHour + 1) % 24;
      if (b == BTN_4) timerEditPart = TIMER_EDIT_MINUTE;
    } else if (timerEditPart == TIMER_EDIT_MINUTE) {
      if (b == BTN_2) *targetMinute = (*targetMinute + 55) % 60;
      if (b == BTN_3) *targetMinute = (*targetMinute + 5) % 60;
      if (b == BTN_4) {
        timerEditPart = TIMER_EDIT_NONE;
        saveSchedule();
      }
    }
    return;
  }
}

void drawHomeScreen() {
  struct tm timeinfo;
  bool hasTime = getLocalTime(&timeinfo, 10);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // WiFi ikonka vpravo nahoře (11x11).
  const uint8_t *wifiIcon = nullptr;
  if (apMode) {
    wifiIcon = bitmap_wifiAp;
  } else if (WiFi.status() == WL_CONNECTED) {
    wifiIcon = bitmap_wifiConnected;
  }
  if (wifiIcon != nullptr) {
    display.drawBitmap(SCREEN_WIDTH - WIFI_ICON_W, 0, wifiIcon, WIFI_ICON_W, WIFI_ICON_H, SH110X_WHITE);
  }

  if (lockoutFault != FAULT_NONE) {
    display.setCursor(0, 0);
    display.print("!ERR!");
  }

  display.setCursor(40, 0);
  if (hasTime) {
    display.print(twoDigits(timeinfo.tm_hour));
    display.print(":");
    display.print(twoDigits(timeinfo.tm_min));
    display.print(":");
    display.print(twoDigits(timeinfo.tm_sec));
  } else {
    display.print("--:--:--");
  }

  display.setCursor(0, 10);
  display.print("Voda:");
  if (isnan(waterTempC)) {
    display.print("--.- C");
  } else {
    display.print(String(waterTempC, 1));
    display.print(" C");
  }
  display.print("  Cer:");
  if (isnan(pumpTempC)) {
    display.print("-- C");
  } else {
    display.print((int)lroundf(pumpTempC));
    display.print(" C");
  }

  display.setCursor(0, 20);
  display.print("Filtrace: ");
  display.print(relayState ? "ON" : "OFF");
  if (manualOverride) display.print(" (MAN)");

  display.setCursor(0, 30);
  display.print("Casovac: ");
  display.print(timerEnabled ? "AKTIVNI" : "Vypnuty");

  display.setCursor(27, 40);
  display.print(timeHmString(schedOnHour, schedOnMinute));
  display.print(" - ");
  display.print(timeHmString(schedOffHour, schedOffMinute));

  // Zobrazení odpovídá logickému „je zapnuto“: ON -> ikona vypnutí, OFF -> ikona zapnutí.
  const bool logicalOn = manualOverride ? manualRelayState : (lockoutFault != FAULT_NONE ? false : scheduleWouldBeOnNow());
  const uint8_t *k1 = logicalOn ? bitmap_filtrationOff : bitmap_filtrationOn;
  drawSoftkeys(k1, nullptr, nullptr, bitmap_settings);

  display.display();
}

void drawWaterTempFullScreen() {
  display.clearDisplay();
  if (isnan(waterTempC)) {
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(48, 24);
    display.print("--");
    display.display();
    return;
  }

  float t = waterTempC;
  int t10 = (int)(t * 10.0f + (t >= 0 ? 0.5f : -0.5f));
  bool neg = t10 < 0;
  int abs10 = neg ? -t10 : t10;
  int intPart = abs10 / 10;
  int frac = abs10 % 10;

  char intStr[8];
  snprintf(intStr, sizeof(intStr), "%d", intPart);

  const int digitCount = (int)strlen(intStr); // počet číslic v celočásti
  const int elements = (neg ? 1 : 0) + digitCount + 1 /*comma*/ + 1 /*frac*/ + 1 /*deg*/;
  const int gaps = (elements > 0) ? (elements - 1) * TEMP_CHAR_GAP : 0;

  int widths = 0;
  if (neg) widths += TEMP_MINUS_W;
  widths += digitCount * TEMP_DIGIT_W;
  widths += TEMP_COMMA_W;
  widths += TEMP_DIGIT_W; // desetiny
  widths += TEMP_DEG_W;

  int totalW = widths + gaps;

  int x = (SCREEN_WIDTH - totalW) / 2;
  if (x < 0) x = 0;

  bool first = true;

  if (neg) {
    display.drawBitmap(x, TEMP_MINUS_Y, epd_bitmap_num_minus, TEMP_MINUS_W, TEMP_MINUS_H, SH110X_WHITE);
    x += TEMP_MINUS_W;
    first = false;
  }

  for (const char *p = intStr; *p; ++p) {
    int d = *p - '0';
    if (!first) x += TEMP_CHAR_GAP;
    if (d >= 0 && d <= 9) {
      display.drawBitmap(x, 0, epd_bitmap_digits[d], TEMP_DIGIT_W, TEMP_DIGIT_H, SH110X_WHITE);
      x += TEMP_DIGIT_W;
    } else {
      // fallback, kdyby intStr neobsahovalo jen číslice
      x += TEMP_DIGIT_W;
    }
    first = false;
  }

  if (!first) x += TEMP_CHAR_GAP;
  display.drawBitmap(x, TEMP_COMMA_Y, epd_bitmap_num_comma, TEMP_COMMA_W, TEMP_COMMA_H, SH110X_WHITE);
  x += TEMP_COMMA_W;
  first = false;

  x += TEMP_CHAR_GAP; // desetiny vždy následují po čárce
  display.drawBitmap(x, 0, epd_bitmap_digits[frac], TEMP_DIGIT_W, TEMP_DIGIT_H, SH110X_WHITE);
  x += TEMP_DIGIT_W;

  x += TEMP_CHAR_GAP; // °C vždy za desetiny
  display.drawBitmap(x, 0, epd_bitmap_num_degrees_of_c, TEMP_DEG_W, TEMP_DEG_H, SH110X_WHITE);

  display.display();
}

void drawTimerScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(36, 0);
  display.print("-CASOVAC-");

  drawLabeledTimeRow("ON", 14, schedOnHour, schedOnMinute, timerSelectedRow == 0, timerSelectedRow == 0 && timerEditPart == TIMER_EDIT_HOUR, timerSelectedRow == 0 && timerEditPart == TIMER_EDIT_MINUTE);
  drawLabeledTimeRow("OFF", 26, schedOffHour, schedOffMinute, timerSelectedRow == 1, timerSelectedRow == 1 && timerEditPart == TIMER_EDIT_HOUR, timerSelectedRow == 1 && timerEditPart == TIMER_EDIT_MINUTE);

  bool invertActiveRow = (timerSelectedRow == 2 && timerEditPart == TIMER_EDIT_NONE);
  if (invertActiveRow) {
    display.fillRect(0, 37, SCREEN_WIDTH, 10, SH110X_WHITE);
  }
  display.setTextColor(invertActiveRow ? SH110X_BLACK : SH110X_WHITE);
  display.setCursor(0, 38);
  display.print("Aktivni: ");
  display.print(timerEnabled ? "[X]" : "[ ]");
  display.setTextColor(SH110X_WHITE);

  if (timerEditPart == TIMER_EDIT_NONE) {
    drawSoftkeys(bitmap_back, bitmap_arrowUp, bitmap_arrowDown, bitmap_ok);
  } else {
    drawSoftkeys(bitmap_back, bitmap_minus, bitmap_plus, bitmap_ok);
  }

  display.display();
}

void drawTimeSetScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(18, 0);
  display.print("-NASTAVIT CAS-");

  drawLabeledTimeRow("CAS", 26, timeSetHour, timeSetMinute, true, timeSetEditPart == TIMER_EDIT_HOUR, timeSetEditPart == TIMER_EDIT_MINUTE);

  if (timeSetEditPart == TIMER_EDIT_NONE) {
    drawSoftkeys(bitmap_back, nullptr, nullptr, bitmap_ok);
  } else {
    drawSoftkeys(bitmap_back, bitmap_minus, bitmap_plus, bitmap_ok);
  }

  display.display();
}

void drawSettingsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(6, 0);
  display.print("-NASTAVENI SYSTEMU-");

  const int totalRows = 8;
  const int visibleRows = 4;
  int firstRow = settingsSelectedRow - (visibleRows - 1);
  if (firstRow < 0) firstRow = 0;
  int maxFirstRow = totalRows - visibleRows;
  if (firstRow > maxFirstRow) firstRow = maxFirstRow;

  for (int i = 0; i < visibleRows; i++) {
    int row = firstRow + i;
    int y = 12 + i * 10;

    if (row == settingsSelectedRow) {
      display.fillRect(0, y - 1, SCREEN_WIDTH, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }

    display.setCursor(0, y);
    if (row == 0) {
      display.print("Rezim WiFi: ");
      display.print(prefForceAp ? "AP" : "Klient");
    } else if (row == 1) {
      display.print("Fullscreen temp: ");
      display.print(FULLSCREEN_TIMEOUT_LABELS[fullscreenTimeoutIdx]);
    } else if (row == 2) {
      display.print("Hlidat zahlceni: ");
      display.print(FLOOD_PROTECT_TIMEOUT_LABELS[floodProtectTimeoutIdx]);
    } else if (row == 3) {
      display.print("Tep. ochrana: ");
      display.print(TEMP_PROTECT_LIMIT_LABELS[tempProtectLimitIdx]);
    } else if (row == 4) {
      display.print("Informace");
    } else if (row == 5) {
      display.print("Nastavit cas");
    } else if (row == 6) {
      display.print("Prohodit cidla: ");
      display.print(tempSensorsSwap ? "ANO" : "NE");
    } else if (row == 7) {
      display.print("RESET CIDEL");
    }
  }

  display.setTextColor(SH110X_WHITE);

  drawSoftkeys(bitmap_back, bitmap_arrowUp, bitmap_arrowDown, bitmap_ok);
  display.display();
}

void drawInfoScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(30, 0);
  display.print("-INFORMACE-");

  display.setCursor(0, 14);
  display.print(apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString());

  display.setCursor(0, 24);
  display.print(WiFi.macAddress());

  display.setCursor(0, 34);
  if (apMode) {
    display.print("PoolControlSetup");
  } else {
    display.print(WiFi.SSID().length() > 0 ? WiFi.SSID() : "WIFI Nepripojena");
  }

  drawSoftkeys(bitmap_back, bitmap_arrowRight, bitmap_arrowDown, nullptr);
  display.display();
}

void drawErrorScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(42, 0);
  display.print("-CHYBA-");

  display.setCursor(0, 24);
  if (errorScreenFault == FAULT_NOT_FLOODED) {
    display.print("    CERPADLO NENI\n      ZAHLCENO");
  } else if (errorScreenFault == FAULT_OVERHEAT) {
    display.print("  PREHRATI CERPADLA");
  } else {
    display.print("Neznama chyba");
  }

  drawSoftkeys(nullptr, nullptr, nullptr, bitmap_ok);
  display.display();
}

void updateDisplay() {
  if (millis() - lastDisplayMs < 250) return;
  lastDisplayMs = millis();

  if (uiScreen == SCREEN_HOME) {
    drawHomeScreen();
  } else if (uiScreen == SCREEN_TIMER) {
    drawTimerScreen();
  } else if (uiScreen == SCREEN_TIME_SET) {
    drawTimeSetScreen();
  } else if (uiScreen == SCREEN_SETTINGS) {
    drawSettingsScreen();
  } else if (uiScreen == SCREEN_INFO) {
    drawInfoScreen();
  } else if (uiScreen == SCREEN_ERROR) {
    drawErrorScreen();
  } else {
    drawWaterTempFullScreen();
  }
}

String htmlPage() {
  String ipInfo = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String modeInfo = apMode ? "AP" : "STA";

  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  s += "<title>Pool Controller</title></head><body>";
  s += "<h2>Pool Controller</h2>";
  s += "<p><b>Mode:</b> " + modeInfo + " | <b>IP:</b> " + ipInfo + "</p>";
  s += "<p><b>Time:</b> <span id='time'>--:--:--</span></p>";
  s += "<p><b>Teploty:</b> Voda <span id='water_temp'>N/A</span> | Cerpadlo <span id='pump_temp'>N/A</span></p>";
  s += "<p><b>Relay:</b> <span id='relay'>?</span> | <b>Manual override:</b> <span id='manual_override'>?</span></p>";
  s += "<p><b>Timer:</b> <span id='timer_enabled'>?</span> | <b>Schedule:</b> <span id='schedule'>--:-- - --:--</span></p>";
  s += "<p><b>Safety switch:</b> <span id='safety_ok'>?</span> | <b>Chyba:</b> <span id='fault'>zadna</span></p>";
  s += "<p><b>Hlidat zahlceni:</b> <span id='flood_protect'>?</span> | <b>Tep. ochrana:</b> <span id='temp_protect'>?</span></p>";
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
  s += "Timer enabled: <input type='checkbox' name='ten' ";
  s += (timerEnabled ? "checked" : "");
  s += "><br>";
  s += "<button type='submit'>Save schedule</button></form>";
  s += "<hr>";
  s += "<p><a href='/toggle'>Toggle relay (manual)</a> | <a href='/auto'>Back to auto</a> | <a href='/status'>JSON status</a></p>";
  s += "<script>";
  s += "function fmtTemp(v, dec){ if(v===null||v===undefined||v<-200) return 'N/A'; return v.toFixed(dec)+' C'; }";
  s += "function txt(id, v){ var e=document.getElementById(id); if(e) e.textContent=v; }";
  s += "async function refresh(){";
  s += "try{";
  s += "const r=await fetch('/status',{cache:'no-store'});";
  s += "if(!r.ok) return;";
  s += "const s=await r.json();";
  s += "txt('time', s.time || '--:--:--');";
  s += "txt('water_temp', fmtTemp(s.water_temp_c, 1));";
  s += "txt('pump_temp', fmtTemp(s.pump_temp_c, 0));";
  s += "txt('relay', s.relay ? 'ON' : 'OFF');";
  s += "txt('manual_override', s.manual_override ? 'ON' : 'OFF');";
  s += "txt('timer_enabled', s.timer_enabled ? 'ON' : 'OFF');";
  s += "txt('schedule', (s.schedule_on||'--:--') + ' - ' + (s.schedule_off||'--:--'));";
  s += "txt('safety_ok', (s.safety_ok===true) ? 'OK' : 'OPEN');";
  s += "txt('fault', s.lockout_fault_label || 'zadna');";
  s += "txt('flood_protect', s.flood_protect_label || '?');";
  s += "txt('temp_protect', s.temp_protect_label || '?');";
  s += "}catch(e){}";
  s += "}";
  s += "refresh(); setInterval(refresh, 1000);";
  s += "</script>";
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

  String lockoutKey = "none";
  String lockoutLabel = "zadna";
  if (lockoutFault == FAULT_NOT_FLOODED) {
    lockoutKey = "not_flooded";
    lockoutLabel = "Cerpadlo neni zahlceno";
  } else if (lockoutFault == FAULT_OVERHEAT) {
    lockoutKey = "overheat";
    lockoutLabel = "Prehrati cerpadla";
  }

  bool safetyOk = (digitalRead(PIN_SAFETY_SWITCH) == LOW);

  String json = "{";
  json += "\"mode\":\"" + String(apMode ? "AP" : "STA") + "\",";
  json += "\"ip\":\"" + String(apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
  json += "\"time\":\"" + nowStr + "\",";
  json += "\"has_time\":" + String(hasTime ? "true" : "false") + ",";
  json += "\"temperature_c\":" + String(isnan(waterTempC) ? -999.0f : waterTempC, 2) + ",";
  json += "\"water_temp_c\":" + String(isnan(waterTempC) ? "null" : String(waterTempC, 2)) + ",";
  json += "\"pump_temp_c\":" + String(isnan(pumpTempC) ? "null" : String(pumpTempC, 2)) + ",";
  json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
  json += "\"manual_override\":" + String(manualOverride ? "true" : "false") + ",";
  json += "\"timer_enabled\":" + String(timerEnabled ? "true" : "false") + ",";
  json += "\"schedule_on\":\"" + timeHmString(schedOnHour, schedOnMinute) + "\",";
  json += "\"schedule_off\":\"" + timeHmString(schedOffHour, schedOffMinute) + "\",";
  json += "\"safety_ok\":" + String(safetyOk ? "true" : "false") + ",";
  json += "\"lockout_fault\":\"" + lockoutKey + "\",";
  json += "\"lockout_fault_label\":\"" + lockoutLabel + "\",";
  json += "\"flood_protect_label\":\"" + String(FLOOD_PROTECT_TIMEOUT_LABELS[floodProtectTimeoutIdx]) + "\",";
  json += "\"temp_protect_label\":\"" + String(TEMP_PROTECT_LIMIT_LABELS[tempProtectLimitIdx]) + "\"";
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
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

  // Po uložení nových údajů chceme, aby se ESP pokusilo o připojení v režimu Klient.
  prefForceAp = false;
  saveWifiMode();

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
  timerEnabled = server.hasArg("ten");
  saveSchedule();
  server.send(200, "text/plain", "Schedule saved");
}

void handleToggleRelay() {
  bool desiredOn = !relayState;
  if (desiredOn && lockoutFault != FAULT_NONE) {
    if (lockoutFault == FAULT_OVERHEAT) {
      int limitC = tempProtectionLimitC();
      if (limitC > 0 && !isnan(pumpTempC) && pumpTempC >= (float)limitC) {
        errorScreenFault = FAULT_OVERHEAT;
        uiScreen = SCREEN_ERROR;
        desiredOn = false;
      }
    }
    if (desiredOn) {
      manualOverride = true;
      manualRelayState = true;
      setRelay(true);
      manualRecoveryActive = true;
      manualRecoveryReason = lockoutFault;
      manualRecoveryStartMs = millis();
    }
  } else {
    manualOverride = true;
    manualRelayState = desiredOn;
    setRelay(desiredOn);
    if (!desiredOn) manualRecoveryActive = false;
  }
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
  float t0 = hasTempAddr0 ? ds18b20.getTempC(tempAddr0) : ds18b20.getTempCByIndex(0);
  float t1 = hasTempAddr1 ? ds18b20.getTempC(tempAddr1) : ds18b20.getTempCByIndex(1);

  float tw = tempSensorsSwap ? t1 : t0;
  float tp = tempSensorsSwap ? t0 : t1;

  if (tw > -55.0f && tw < 125.0f) waterTempC = tw;
  else waterTempC = NAN;

  if (tp > -55.0f && tp < 125.0f) pumpTempC = tp;
  else pumpTempC = NAN;
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);
  pinMode(PIN_SAFETY_SWITCH, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  adcButtonsEnsurePullUp();

  Serial.begin(115200);
  delay(200);

  loadSettings();
  setenv("TZ", TZ_INFO, 1);
  tzset();
  ds18b20.begin();
  pairTempSensorsIfNeeded();

  // Na ESP32-C3 je potřeba navázat SPI na konkrétní piny (SCK, MISO=-1, MOSI).
  SPI.begin(PIN_OLED_SCK, -1, PIN_OLED_SDA);

  if (!display.begin(0x3C, true)) {
    // Pokud OLED nenaběhne, pokračuj bez něj.
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(18, 28);
    display.print("Inicializace...");
    display.display();
  }

  bool connected = false;
  if (prefForceAp) {
    startApMode();
  } else {
    apMode = false;
    connected = connectSta();
    if (connected) {
      Serial.print("WiFi connected, IP address: ");
      Serial.println(WiFi.localIP());
      syncTimeNow();
    } else {
      Serial.println("WiFi not connected, will retry in loop...");

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(12, 28);
      display.print("WIFI NEPRIPOJENA");
      display.display();
      delay(1500);
    }
  }

  setupWebServer();
  // Úspávání WiFi zvyšuje šum na ADC — pro klidnější žebřík tlačítek vypnout sleep.
  WiFi.setSleep(false);
  applyRelayLogic();
  lastUserActivityMs = millis();
  prevWifiConnected = (!apMode && WiFi.status() == WL_CONNECTED);
}

void loop() {
  server.handleClient();
  printAdcCalibration();
  updateButtons();
  updateTemperature();

  if (!apMode && WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiRetryMs > 15000) {
      lastWifiRetryMs = millis();
      WiFi.reconnect();
    }
  }

  if (!apMode && WiFi.status() == WL_CONNECTED) {
    if (!prevWifiConnected) {
      syncTimeNow();
    }
    if ((millis() - lastNtpSyncMs) > NTP_SYNC_INTERVAL_MS) {
      syncTimeNow();
    }
  }
  prevWifiConnected = (!apMode && WiFi.status() == WL_CONNECTED);

  applyRelayLogic();
  updateProtections();
  checkIdleToTempScreen();
  updateDisplay();
  delay(10);
}
