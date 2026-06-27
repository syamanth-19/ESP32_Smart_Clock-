#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <time.h>
#include <math.h>

// --- Time Settings ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800; // IST (UTC +5:30)
const int   daylightOffset_sec = 0;

// --- Hardware Pins ---
#define DHTPIN 4      
#define DHTTYPE DHT22   
#define MASTER_BTN 13   
#define ACTION_BTN 14   
#define BACK_BTN 18     
#define LDR_PIN 19      
#define BUZZER_PIN 23   

// --- GPS Hardware Serial ---
#define RXD2 16
#define TXD2 17

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C 

DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); 

// --- API Server Initialization ---
WebServer server(80);

// --- OS States ---
enum ScreenState { 
  SCREEN_CLOCK, 
  SCREEN_ENV, 
  SCREEN_GPS,
  SCREEN_STOPWATCH, 
  SCREEN_TIMER,
  SCREEN_WIFI
};
ScreenState currentScreen = SCREEN_CLOCK;
ScreenState lastScreen = SCREEN_CLOCK;

// --- Global Variables ---
unsigned long lastDisplayUpdate = 0;
String popupMessage = "";
unsigned long popupEndTime = 0;

// --- Screen Sleep Variables ---
const unsigned long INACTIVITY_TIMEOUT = 180000; // 3 minutes in milliseconds
unsigned long lastInteractionTime = 0;
bool displayAsleep = false;
bool justWokeUp = false;

float currentTemp = 0.0;
float currentHum = 0.0;
float currentWetBulb = 0.0;
bool forceSensorRead = false;

bool swRunning = false;
unsigned long swStartTime = 0;
unsigned long swElapsedTime = 0;
bool tmrRunning = false;
long tmrDuration = 0; 
long tmrRemaining = 0;
unsigned long tmrLastCalcTime = 0;

int wifiCursorIndex = 0;
int wifiTotalNetworks = 0;
bool isTimeSynced = false;
bool isWifiConnected = false;
bool isGpsTimeSynced = false;
bool wifiPowerOn = true; 

// Upgraded GPS Radar Variables
int gpsSubPage = 0; // Now 8 pages total (0 to 7)
unsigned long totalNmeaSentencesParsed = 0; 
int activeSatIDs[16] = {0}; 
int activeSatSNRs[16] = {0}; 

const char* targetSSID = "tp";
const char* targetPASS = "55516884";

const int DEBOUNCE_TIME = 35;
const int LONG_PRESS_TIME = 1000;

struct ButtonEngine {
  int pin;
  bool state;
  bool lastState;
  unsigned long pressTime;
  unsigned long releaseTime;
  bool longToneTriggered;
};

ButtonEngine btn1 = {MASTER_BTN, false, false, 0, 0, false};
ButtonEngine btn2 = {ACTION_BTN, false, false, 0, 0, false};
ButtonEngine btn3 = {BACK_BTN, false, false, 0, 0, false}; 

unsigned long buzzerCutoffTime = 0;
bool buzzerIsPlaying = false;
int alarmSequence = 0;
unsigned long nextAlarmStepTime = 0;

// --- Function Prototypes ---
void showPopup(String msg, int duration_ms);
void processButton(ButtonEngine &btn, void (*shortPressHandler)(), void (*longPressHandler)());
void handleButtonLogic();
void updateTimerLogic();
void updateDisplay();
void updateBuzzerLogic();
void processGpsStreamAndExtractSats(); 
void syncTimeFromGps();
String getCardinalDirection(double course);
float calculateWetBulb(float T, float RH);
void playToneNonBlocking(int frequency, int duration_ms);
void playShortBeep();
void playMediumBeep();
void playLongBeep();

void btn1_ShortPress(); void btn1_LongPress();
void btn2_ShortPress(); void btn2_LongPress();
void btn3_ShortPress(); void btn3_LongPress(); 

void drawProgressBar(ButtonEngine &btn);
void drawClockScreen();
void drawSensorScreen();
void drawGpsScreen();
void drawStopwatchScreen();
void drawTimerScreen();
void drawWifiScreen();
void connectAndSyncTime(bool silentBoot);
void handleTelemetryApi(); 

// ==========================================
//           API ENDPOINT HANDLER
// ==========================================
void handleTelemetryApi() {
  String json = "{";
  
  // --- 1. System Information ---
  json += "\"system\":{";
  json += "\"uptime_ms\":" + String(millis()) + ",";
  json += "\"free_heap_bytes\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"wifi_rssi_dbm\":" + String(WiFi.RSSI());
  json += "},";
  
  // --- 2. Environment & LDR ---
  json += "\"environment\":{";
  json += "\"temperature_c\":" + String(currentTemp, 1) + ",";
  json += "\"humidity_percent\":" + String(currentHum, 1) + ",";
  json += "\"wet_bulb_c\":" + String(currentWetBulb, 1) + ",";
  json += "\"ldr_digital_state\":" + String(digitalRead(LDR_PIN));
  json += "},";
  
  // --- 3. GPS Information ---
  bool hasLock = gps.location.isValid() && gps.satellites.value() >= 3;
  json += "\"gps\":{";
  json += "\"has_lock\":" + String(hasLock ? "true" : "false") + ",";
  json += "\"satellites_connected\":" + String(gps.satellites.value()) + ",";
  json += "\"hdop\":" + String(gps.hdop.hdop()) + ",";
  json += "\"latitude\":" + String(gps.location.lat(), 6) + ",";
  json += "\"longitude\":" + String(gps.location.lng(), 6) + ",";
  json += "\"speed_kmph\":" + String(gps.speed.kmph(), 1) + ",";
  json += "\"altitude_m\":" + String(gps.altitude.meters(), 1) + ",";
  json += "\"course\":\"" + getCardinalDirection(gps.course.deg()) + "\",";
  json += "\"nmea_sentences_parsed\":" + String(totalNmeaSentencesParsed) + ",";
  
  // Inject the 16-Slot Radar Data
  json += "\"sat_radar_array\":[";
  for(int i = 0; i < 16; i++) {
    json += "{\"slot\":" + String(i+1) + ",\"id\":" + String(activeSatIDs[i]) + ",\"snr\":" + String(activeSatSNRs[i]) + "}";
    if (i < 15) json += ","; 
  }
  json += "]";
  
  json += "}"; // End GPS object
  json += "}"; // End Master object
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ==========================================
//                 SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  dht.begin();
  pinMode(MASTER_BTN, INPUT_PULLUP);
  pinMode(ACTION_BTN, INPUT_PULLUP);
  pinMode(BACK_BTN, INPUT_PULLUP); 
  pinMode(LDR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 alloc failed"));
    for(;;); 
  }
  
  display.setTextWrap(false);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("SmartOS Anti-Wrap...");
  display.println("16-Slot Radar Unlocked");
  display.println("Connecting WiFi...");
  display.display();

  tone(BUZZER_PIN, 2200, 100);

  currentTemp = dht.readTemperature();
  currentHum = dht.readHumidity();
  if(!isnan(currentTemp) && !isnan(currentHum)) {
    currentWetBulb = calculateWetBulb(currentTemp, currentHum);
  }

  connectAndSyncTime(true); 

  display.clearDisplay();
  display.setCursor(0, 20);
  display.println("Deep Telemetry Active!");
  display.display();
  delay(1200);
  
  lastInteractionTime = millis(); 

  server.on("/api/telemetry", handleTelemetryApi);
  server.begin();
}

// ==========================================
//               MAIN LOOP
// ==========================================
void loop() {
  processGpsStreamAndExtractSats();
  handleButtonLogic();
  updateTimerLogic();
  updateBuzzerLogic();

  if (gps.time.isUpdated() && gps.date.isUpdated() && gps.satellites.value() >= 4) {
    syncTimeFromGps();
  }

  if (wifiPowerOn && isWifiConnected) {
    server.handleClient();
  }

  if (!displayAsleep && (millis() - lastInteractionTime > INACTIVITY_TIMEOUT)) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    displayAsleep = true;
  }

  if (!displayAsleep && (millis() - lastDisplayUpdate > 40)) { 
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// ==========================================
//       MATH & HARDWARE ENGINES
// ==========================================
float calculateWetBulb(float T, float RH) {
  return T * atan(0.151977 * pow(RH + 8.313659, 0.5)) 
       + atan(T + RH) 
       - atan(RH - 1.676331) 
       + 0.00391838 * pow(RH, 1.5) * atan(0.023101 * RH) 
       - 4.686035;
}

void processGpsStreamAndExtractSats() {
  static String nmeaLineBuffer = "";
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c);
    Serial.write(c); 

    if (c != '\r' && c != '\n') {
      nmeaLineBuffer += c;
    } 
    else if (c == '\n') {
      totalNmeaSentencesParsed++;
      if (nmeaLineBuffer.indexOf("GPGSV") != -1 || nmeaLineBuffer.indexOf("GNGSV") != -1) {
        int commaIndex = 0; int lastCommaPos = 0;
        int msgNum = 1;
        
        for (int i = 0; i < nmeaLineBuffer.length(); i++) {
          if (nmeaLineBuffer.charAt(i) == ',') {
            commaIndex++;
            String fieldData = nmeaLineBuffer.substring(lastCommaPos + 1, i);
            lastCommaPos = i;
            
            if (commaIndex == 2) { 
              msgNum = fieldData.toInt(); 
              if (msgNum == 1) {
                for(int k = 0; k < 16; k++) { activeSatIDs[k] = 0; activeSatSNRs[k] = 0; }
              }
            }
            if (commaIndex >= 4) {
              int satIndexInMsg = (commaIndex - 4) / 4;
              int globalIdx = ((msgNum - 1) * 4) + satIndexInMsg;
              
              if (globalIdx < 16 && fieldData.length() > 0) {
                if ((commaIndex - 4) % 4 == 0) {
                  activeSatIDs[globalIdx] = fieldData.toInt(); 
                } else if ((commaIndex - 7) % 4 == 0) {
                  activeSatSNRs[globalIdx] = fieldData.toInt(); 
                }
              }
            }
          }
        }
        if (commaIndex >= 7 && (commaIndex - 6) % 4 == 0) {
          int satIndexInMsg = (commaIndex - 3) / 4;
          int globalIdx = ((msgNum - 1) * 4) + satIndexInMsg;
          if (globalIdx < 16) {
             String lastField = nmeaLineBuffer.substring(lastCommaPos + 1);
             activeSatSNRs[globalIdx] = lastField.toInt(); 
          }
        }
      }
      nmeaLineBuffer = ""; 
    }
  }
}

void syncTimeFromGps() {
  struct tm gpsTime;
  gpsTime.tm_year = gps.date.year() - 1900; gpsTime.tm_mon = gps.date.month() - 1;
  gpsTime.tm_mday = gps.date.day(); gpsTime.tm_hour = gps.time.hour();
  gpsTime.tm_min = gps.time.minute(); gpsTime.tm_sec = gps.time.second(); gpsTime.tm_isdst = 0;
  time_t utc_timestamp = mktime(&gpsTime);
  if (utc_timestamp != (time_t)-1) {
    time_t local_timestamp = utc_timestamp + gmtOffset_sec;
    struct timeval tv = { .tv_sec = local_timestamp, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    isTimeSynced = true; isGpsTimeSynced = true;
  }
}

String getCardinalDirection(double course) {
  if (course >= 337.5 || course < 22.5)   return "N";
  if (course >= 22.5  && course < 67.5)   return "NE";
  if (course >= 67.5  && course < 112.5) return "E";
  if (course >= 112.5 && course < 157.5) return "SE";
  if (course >= 157.5 && course < 202.5) return "S";
  if (course >= 202.5 && course < 247.5) return "SW";
  if (course >= 247.5 && course < 292.5) return "W";
  if (course >= 292.5 && course < 337.5) return "NW";
  return "--";
}

void playToneNonBlocking(int frequency, int duration_ms) {
  tone(BUZZER_PIN, frequency); buzzerCutoffTime = millis() + duration_ms; buzzerIsPlaying = true;
}
void playShortBeep()  { playToneNonBlocking(2700, 50); }
void playMediumBeep() { playToneNonBlocking(2000, 90); }
void playLongBeep()   { playToneNonBlocking(1500, 250); }

void updateBuzzerLogic() {
  if (buzzerIsPlaying && millis() >= buzzerCutoffTime) { noTone(BUZZER_PIN); buzzerIsPlaying = false; }
  if (millis() < popupEndTime && popupMessage == "TIMER DONE!") {
    if (millis() >= nextAlarmStepTime) {
      if (alarmSequence % 2 == 0) { tone(BUZZER_PIN, 3200); nextAlarmStepTime = millis() + 120; } 
      else { noTone(BUZZER_PIN); nextAlarmStepTime = millis() + 120; }
      alarmSequence++;
    }
  } else if (popupMessage == "TIMER DONE!" && millis() >= popupEndTime) { noTone(BUZZER_PIN); }
}

void handleButtonLogic() {
  processButton(btn1, btn1_ShortPress, btn1_LongPress); 
  processButton(btn2, btn2_ShortPress, btn2_LongPress);
  processButton(btn3, btn3_ShortPress, btn3_LongPress); 
}

void processButton(ButtonEngine &btn, void (*shortPressHandler)(), void (*longPressHandler)()) {
  bool reading = (digitalRead(btn.pin) == LOW); 
  if (reading != btn.lastState) { btn.releaseTime = millis(); }
  
  if ((millis() - btn.releaseTime) > DEBOUNCE_TIME) {
    if (reading != btn.state) {
      btn.state = reading;
      
      if (btn.state == true) { 
        if (displayAsleep) {
          display.ssd1306_command(SSD1306_DISPLAYON);
          displayAsleep = false;
          justWokeUp = true; 
        } else {
          justWokeUp = false;
        }
        lastInteractionTime = millis(); 
        btn.pressTime = millis(); 
        btn.longToneTriggered = false; 
      } 
      else {
        unsigned long duration = millis() - btn.pressTime;
        if (!justWokeUp) {
          if (duration < LONG_PRESS_TIME) shortPressHandler(); else longPressHandler();
        }
        lastInteractionTime = millis(); 
      }
    }
  }
  
  if (btn.state == true && !btn.longToneTriggered && !justWokeUp) {
    if ((millis() - btn.pressTime) >= LONG_PRESS_TIME) { 
      tone(BUZZER_PIN, 1500, 60); 
      btn.longToneTriggered = true; 
    }
  }
  btn.lastState = reading;
}

// --- BUTTON 1 (MASTER) ---
void btn1_ShortPress() {
  playShortBeep(); lastScreen = currentScreen; currentScreen = (ScreenState)((currentScreen + 1) % 6); display.clearDisplay();
  if (currentScreen == SCREEN_WIFI && wifiPowerOn) { WiFi.mode(WIFI_STA); WiFi.disconnect(); WiFi.scanNetworks(true); wifiCursorIndex = 0; }
}

void btn1_LongPress() {
  playLongBeep();
  switch(currentScreen) {
    case SCREEN_CLOCK: connectAndSyncTime(false); break;
    case SCREEN_WIFI:  if (wifiPowerOn) { WiFi.scanDelete(); WiFi.scanNetworks(true); wifiCursorIndex = 0; showPopup("Scan...", 1000); } else { showPopup("Enable 1st", 1000); } break;
    case SCREEN_STOPWATCH: swRunning = false; swElapsedTime = 0; break;
    case SCREEN_TIMER:     tmrRunning = false; tmrDuration = 0; tmrRemaining = 0; break;
    case SCREEN_ENV:       showPopup("Reboot...", 1500); delay(1500); ESP.restart(); break;
    case SCREEN_GPS:       gpsSubPage = 0; showPopup("Rst Page", 1000); break;
  }
}

// --- BUTTON 2 (ACTION) ---
void btn2_ShortPress() {
  playMediumBeep();
  switch(currentScreen) {
    case SCREEN_GPS:       gpsSubPage = (gpsSubPage + 1) % 8; break; 
    case SCREEN_ENV:       forceSensorRead = true; break;
    case SCREEN_STOPWATCH: if (swRunning) { swElapsedTime += (millis() - swStartTime); swRunning = false; } else { swStartTime = millis(); swRunning = true; } break;
    case SCREEN_TIMER:     if (!tmrRunning) { tmrDuration += 60000; if (tmrDuration > 3600000) tmrDuration = 0; tmrRemaining = tmrDuration; } break;
    case SCREEN_WIFI:      if(wifiPowerOn){ wifiTotalNetworks = WiFi.scanComplete(); if (wifiTotalNetworks > 0) { wifiCursorIndex++; if (wifiCursorIndex >= wifiTotalNetworks) wifiCursorIndex = 0; } } break;
    default: break;
  }
}

void btn2_LongPress() {
  playLongBeep();
  if (currentScreen == SCREEN_TIMER && tmrRemaining > 0 && !tmrRunning) { 
    tmrRunning = true; tmrLastCalcTime = millis(); showPopup("Start", 1000); 
  } else if (currentScreen == SCREEN_WIFI) {
    wifiPowerOn = !wifiPowerOn;
    if (wifiPowerOn) {
      WiFi.mode(WIFI_STA); WiFi.disconnect(); WiFi.scanNetworks(true); wifiCursorIndex = 0;
      showPopup("WiFi ON", 1000);
    } else {
      WiFi.disconnect(true); WiFi.mode(WIFI_OFF); isWifiConnected = false;
      showPopup("WiFi OFF", 1000);
    }
  }
}

// --- BUTTON 3 (BACK) ---
void btn3_ShortPress() {
  playShortBeep();
  lastScreen = currentScreen; 
  currentScreen = (ScreenState)((currentScreen - 1 + 6) % 6); 
  display.clearDisplay();
  if (currentScreen == SCREEN_WIFI && wifiPowerOn) { 
    WiFi.mode(WIFI_STA); WiFi.disconnect(); WiFi.scanNetworks(true); wifiCursorIndex = 0; 
  }
}

void btn3_LongPress() {
  playLongBeep();
  if (currentScreen == SCREEN_GPS && gpsSubPage > 0) {
    gpsSubPage = 0; 
    showPopup("GPS Main", 1000);
  } else {
    lastScreen = currentScreen;
    currentScreen = SCREEN_CLOCK; 
    showPopup("Home", 1000);
  }
}

// ==========================================
//              UI RENDERING
// ==========================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); 

  if (millis() < popupEndTime) {
    display.setTextSize(2); display.setCursor(10, 25); display.print(popupMessage); display.display();
    return;
  }
  
  switch (currentScreen) {
    case SCREEN_CLOCK:     drawClockScreen();     break;
    case SCREEN_ENV:       drawSensorScreen();    break;
    case SCREEN_GPS:       drawGpsScreen();       break;
    case SCREEN_STOPWATCH: drawStopwatchScreen(); break;
    case SCREEN_TIMER:     drawTimerScreen();     break;
    case SCREEN_WIFI:      drawWifiScreen();      break;
  }
  
  drawProgressBar(btn1); drawProgressBar(btn2); drawProgressBar(btn3);
  display.display();
}

void drawProgressBar(ButtonEngine &btn) {
  if (btn.state == true) {
    unsigned long holdTime = millis() - btn.pressTime;
    if (holdTime > LONG_PRESS_TIME) holdTime = LONG_PRESS_TIME;
    int yPos = 62; 
    if (btn.pin == ACTION_BTN) yPos = 60;
    if (btn.pin == BACK_BTN) yPos = 58;
    int barWidth = map(holdTime, 0, LONG_PRESS_TIME, 0, SCREEN_WIDTH);
    display.fillRect(0, yPos, barWidth, 2, SSD1306_WHITE);
  }
}

void drawClockScreen() {
  struct tm timeinfo;
  bool timeValid = getLocalTime(&timeinfo, 5);
  int centerX = 30, centerY = 32, radius = 23; 
  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(centerX - 5, centerY - radius + 2); display.print("12");
  display.setCursor(centerX + radius - 6, centerY - 3); display.print("3");
  display.setCursor(centerX - 3, centerY + radius - 8); display.print("6");
  display.setCursor(centerX - radius + 2, centerY - 3); display.print("9");

  if (timeValid && isTimeSynced) {
    float secAngle = (timeinfo.tm_sec * PI / 30) - (PI / 2);
    float minAngle = (timeinfo.tm_min * PI / 30) - (PI / 2);
    float hrAngle  = ((timeinfo.tm_hour % 12 + timeinfo.tm_min / 60.0) * PI / 6) - (PI / 2);
    display.drawLine(centerX, centerY, centerX + cos(hrAngle) * 9, centerY + sin(hrAngle) * 9, SSD1306_WHITE); 
    display.drawLine(centerX, centerY, centerX + cos(minAngle) * 13, centerY + sin(minAngle) * 13, SSD1306_WHITE); 
    display.drawLine(centerX, centerY, centerX + cos(secAngle) * 18, centerY + sin(secAngle) * 18, SSD1306_WHITE); 
    display.setCursor(65, 6);  display.printf("%02d/%02d", timeinfo.tm_mday, timeinfo.tm_mon + 1);
    display.setTextSize(2); display.setCursor(62, 18); display.printf("%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    display.setTextSize(1); display.setCursor(110, 32); display.printf(":%02d", timeinfo.tm_sec);
  } else {
    display.drawLine(centerX, centerY, centerX, centerY - 12, SSD1306_WHITE);
    display.setTextSize(1); display.setCursor(64, 20); display.print("NO TIME");
  }
  display.setCursor(62, 46); display.print(isGpsTimeSynced ? "SRC: GPS" : (isWifiConnected ? "SRC: WiFi" : "SRC: Off"));
}

void drawSensorScreen() {
  static unsigned long lastDhtRead = 0;
  if (millis() - lastDhtRead > 2000 || forceSensorRead) { 
    currentTemp = dht.readTemperature(); 
    currentHum = dht.readHumidity(); 
    if(!isnan(currentTemp) && !isnan(currentHum)) {
      currentWetBulb = calculateWetBulb(currentTemp, currentHum);
    }
    lastDhtRead = millis(); forceSensorRead = false; 
  }
  display.setTextSize(1); display.setCursor(20, 0); display.print("ENVIRONMENT");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  display.setTextSize(2);
  if (isnan(currentTemp) || isnan(currentHum)) { 
    display.setCursor(5, 24); display.print("Sens Err"); 
  } else { 
    display.setCursor(0, 14); display.printf("T :%.1fC", currentTemp); 
    display.setCursor(0, 30); display.printf("H :%.1f%%", currentHum); 
    display.setCursor(0, 46); display.printf("Tw:%.1fC", currentWetBulb); 
  }
}

// --- UPDATED: GPS Screen Top Bar ---
void drawGpsScreen() {
  int totalSats = gps.satellites.value();
  double hdopVal = gps.hdop.hdop();
  bool hasLock = gps.location.isValid() && totalSats >= 3;

  display.setTextSize(1);
  display.setCursor(0, 0); 
  
  // Condense GPS data into the top bar
  float dispHdop = (hdopVal == 0 || hdopVal > 99.0) ? 99.9 : hdopVal;
  display.printf("P%d/8 S:%d H:%.1f", gpsSubPage + 1, totalSats, dispHdop);

  // Leave radar icon exactly where it was on the far right
  display.drawRect(114, 0, 2, 2, SSD1306_WHITE);
  if (totalSats >= 1) display.fillRect(114, 1, 2, 1, SSD1306_WHITE);
  display.drawRect(118, -2, 2, 4, SSD1306_WHITE);
  if (totalSats >= 4) display.fillRect(118, -1, 2, 3, SSD1306_WHITE);
  display.drawRect(122, -4, 2, 6, SSD1306_WHITE);
  if (totalSats >= 6) display.fillRect(122, -3, 2, 5, SSD1306_WHITE);
  
  display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

  switch (gpsSubPage) {
    case 0:
      display.setCursor(0, 14); display.print("Fix: "); display.print(hasLock ? "LOCKED" : "SEARCHING");
      display.setCursor(0, 26); display.printf("Sats: %02d", totalSats);
      display.setCursor(0, 38); display.print("HDOP: "); if (hdopVal == 0 || hdopVal > 99.0) display.print("99.9"); else display.printf("%.4f", hdopVal);
      display.setCursor(0, 50); display.printf("NMEA: %lu", totalNmeaSentencesParsed);
      break;
    case 1:
      display.setCursor(0, 14); display.print("SATS [1-4]   #ID(SNR)");
      display.setCursor(0, 28); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[0], activeSatSNRs[0], activeSatIDs[1], activeSatSNRs[1]);
      display.setCursor(0, 42); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[2], activeSatSNRs[2], activeSatIDs[3], activeSatSNRs[3]);
      break;
    case 2:
      display.setCursor(0, 14); display.print("SATS [5-8]   #ID(SNR)");
      display.setCursor(0, 28); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[4], activeSatSNRs[4], activeSatIDs[5], activeSatSNRs[5]);
      display.setCursor(0, 42); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[6], activeSatSNRs[6], activeSatIDs[7], activeSatSNRs[7]);
      break;
    case 3:
      display.setCursor(0, 14); display.print("SATS [9-12]  #ID(SNR)");
      display.setCursor(0, 28); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[8], activeSatSNRs[8], activeSatIDs[9], activeSatSNRs[9]);
      display.setCursor(0, 42); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[10], activeSatSNRs[10], activeSatIDs[11], activeSatSNRs[11]);
      break;
    case 4:
      display.setCursor(0, 14); display.print("SATS [13-16] #ID(SNR)");
      display.setCursor(0, 28); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[12], activeSatSNRs[12], activeSatIDs[13], activeSatSNRs[13]);
      display.setCursor(0, 42); display.printf("#%02d(%02d)   #%02d(%02d)", activeSatIDs[14], activeSatSNRs[14], activeSatIDs[15], activeSatSNRs[15]);
      break;
    case 5:
      display.setCursor(0, 14); display.print("COORDINATES");
      if (hasLock) {
        display.setCursor(0, 26); display.print("LAT:"); display.setTextSize(2); display.setCursor(24, 26); display.printf("%.6f", gps.location.lat());
        display.setTextSize(1); display.setCursor(0, 46); display.print("LON:"); display.setTextSize(2); display.setCursor(24, 46); display.printf("%.6f", gps.location.lng()); display.setTextSize(1);
      } else {
        display.setCursor(0, 30); display.print("Waiting for Fix");
      }
      break;
    case 6:
      display.setCursor(0, 14); display.print("MOTION");
      if (hasLock) {
        display.setCursor(0, 26); display.printf("Spd: %.1f km/h", gps.speed.kmph());
        display.setCursor(0, 38); display.printf("Alt: %.1f m", gps.altitude.meters());
        display.setCursor(0, 50); display.print("Dir: "); display.print(getCardinalDirection(gps.course.deg()));
      } else {
        display.setCursor(0, 32); display.print("No Data");
      }
      break;
    case 7:
      display.setCursor(0, 14); display.print("ATOMIC CLOCK");
      if (gps.time.isValid() && gps.date.isValid()) {
        display.setCursor(0, 26); display.printf("Date: %04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
        display.setCursor(0, 38); display.printf("Time: %02d:%02d:%02d UTC", gps.time.hour(), gps.time.minute(), gps.time.second());
        display.setCursor(0, 50); display.printf("Age: %lu ms", gps.time.age());
      } else {
        display.setCursor(0, 30); display.print("Waiting for Fix");
      }
      break;
  }
}

void drawStopwatchScreen() {
  unsigned long currentVal = swElapsedTime;
  if (swRunning) currentVal += (millis() - swStartTime);
  unsigned long ms = (currentVal % 1000) / 10; unsigned long sec = (currentVal / 1000) % 60; unsigned long min = (currentVal / 60000) % 60;
  display.setTextSize(1); display.setCursor(30, 0); display.print("STOPWATCH");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(15, 24); display.printf("%02d:%02d.%02d", min, sec, ms);
  display.setTextSize(1); display.setCursor(0, 50); display.print(swRunning ? "B2:Pse B1:Rst" : "B2:Ply B1:Rst");
}

void drawTimerScreen() {
  unsigned long sec = (tmrRemaining / 1000) % 60; unsigned long min = (tmrRemaining / 60000) % 60;
  display.setTextSize(1); display.setCursor(35, 0); display.print("TIMER");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  display.setTextSize(3); display.setCursor(22, 20); display.printf("%02d:%02d", min, sec);
  display.setTextSize(1); display.setCursor(0, 50); display.print("B2:+1M B2H:Strt");
}

// --- UPDATED: WIFI Screen Top Bar ---
void drawWifiScreen() {
  display.setTextSize(1); 
  display.setCursor(0, 0); 
  display.print("WIFI ");
  
  // Show IP dynamically if connected, otherwise default text
  if (isWifiConnected) {
    display.print(WiFi.localIP().toString());
  } else {
    display.print("MANAGER");
  }
  
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  if (!wifiPowerOn) {
    display.setCursor(25, 24); display.print("Wi-Fi is OFF");
    display.setCursor(0, 52); display.print("B2H:Pwr ON");
    return;
  }

  int n = WiFi.scanComplete(); 
  if (n == WIFI_SCAN_FAILED) { display.setCursor(0, 24); display.print("Hold B1 to Scan"); } 
  else if (n == WIFI_SCAN_RUNNING) { display.setCursor(0, 24); display.print("Scanning..."); } 
  else if (n == 0) { display.setCursor(0, 24); display.print("No APs found"); } 
  else if (n > 0) {
    wifiTotalNetworks = n; int startIdx = wifiCursorIndex; if (startIdx > n - 3) startIdx = (n - 3 > 0) ? (n - 3) : 0;
    for (int i = 0; i < 3 && (startIdx + i) < n; ++i) {
      int idx = startIdx + i; int yPos = 14 + (i * 12);
      if (idx == wifiCursorIndex) { display.fillRect(0, yPos - 1, 128, 11, SSD1306_WHITE); display.setTextColor(SSD1306_BLACK); } 
      else { display.setTextColor(SSD1306_WHITE); }
      display.setCursor(2, yPos + 1); String ssidName = WiFi.SSID(idx); if(ssidName.length() > 12) ssidName = ssidName.substring(0, 10) + "..";
      display.print(ssidName); display.setCursor(92, yPos + 1); display.printf("%d", WiFi.RSSI(idx));
    }
  }
  
  display.setTextColor(SSD1306_WHITE); display.setCursor(0, 52); display.print("B2H:Pwr B1H:Scan");
}

void connectAndSyncTime(bool silentBoot) {
  if (!wifiPowerOn) return; 
  if (!silentBoot) { display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 10); display.println("Conn SSID:"); display.println(targetSSID); display.display(); }
  WiFi.mode(WIFI_STA); WiFi.begin(targetSSID, targetPASS); int attempts = 0;
  while(WiFi.status() != WL_CONNECTED && attempts < 12) { delay(500); if (!silentBoot) { display.print("."); display.display(); } attempts++; }
  if (WiFi.status() == WL_CONNECTED) {
    isWifiConnected = true;
    if (!silentBoot) { display.clearDisplay(); display.setCursor(0, 15); display.println("WiFi OK!"); display.display(); }
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); struct tm timeinfo; int retry = 0; bool syncSuccess = false;
    while (retry < 8) { if (getLocalTime(&timeinfo, 10)) { syncSuccess = true; break; } delay(300); retry++; }
    if(syncSuccess) { isTimeSynced = true; if (!silentBoot) { showPopup("Time Sync!", 2000); tone(BUZZER_PIN, 2500, 300); } }
  } else { isWifiConnected = false; if (!silentBoot) { showPopup("Fail", 2000); tone(BUZZER_PIN, 1000, 400); } }
  if (!isWifiConnected && wifiPowerOn) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); wifiPowerOn = false; }
  currentScreen = SCREEN_CLOCK;
}

void updateTimerLogic() {
  if (tmrRunning) {
    unsigned long now = millis(); 
    if (tmrRemaining > 0) { tmrRemaining -= (now - tmrLastCalcTime); } 
    tmrLastCalcTime = now;
    
    if (tmrRemaining <= 0) { 
      tmrRemaining = 0; 
      tmrRunning = false; 
      alarmSequence = 0; 
      nextAlarmStepTime = millis(); 
      showPopup("DONE!", 3000); 
      
      if (displayAsleep) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        displayAsleep = false;
        lastInteractionTime = millis();
      }
    }
  }
}

void showPopup(String msg, int duration_ms) { 
  popupMessage = msg; 
  popupEndTime = millis() + duration_ms; 
}