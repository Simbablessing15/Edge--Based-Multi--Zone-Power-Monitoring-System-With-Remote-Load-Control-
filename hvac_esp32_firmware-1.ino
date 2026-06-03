#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// CONFIGURATION
// ============================================================
#define WIFI_SSID       "Simbarashe"
#define WIFI_PASSWORD   "simba12@"
#define FIREBASE_URL    "https://hvacsystem-efaf3-default-rtdb.firebaseio.com"
#define FIREBASE_API    "AIzaSyAblAKHcGvvDZyVnnK-aOMnyuEvB6sSRA4"

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define IR_ROOM1        34    // IR sensor Room 1
#define IR_ROOM2        35    // IR sensor Room 2

#define RELAY_ROOM1_BULB    26
#define RELAY_ROOM1_SOCKET  25
#define RELAY_ROOM2_BULB    14
#define RELAY_ROOM2_SOCKET  27

#define PZEM_RX         16
#define PZEM_TX         17

#define LCD_ADDR        0x27
#define LCD_COLS        20
#define LCD_ROWS        4

// ============================================================
// TIMING CONSTANTS
// ============================================================
#define FIREBASE_INTERVAL   3000
#define PZEM_INTERVAL       2000
#define LCD_REFRESH         1000
#define OCCUPANCY_TIMEOUT   30000  // 30 seconds no detection → socket OFF

// ============================================================
// OBJECTS
// ============================================================
HardwareSerial pzemSerial(2);
PZEM004Tv30 pzem(pzemSerial, PZEM_RX, PZEM_TX);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
FirebaseData fbdo;
FirebaseData fbdoStream;
FirebaseAuth auth;
FirebaseConfig fbConfig;

// ============================================================
// STATE VARIABLES
// ============================================================
bool room1AutoMode      = true;
bool room2AutoMode      = true;

bool room1BulbManual    = false;
bool room1SocketManual  = false;
bool room2BulbManual    = false;
bool room2SocketManual  = false;

bool room1Occupied      = false;
bool room2Occupied      = false;
unsigned long room1LastDetection = 0;
unsigned long room2LastDetection = 0;

// IR edge detection
bool lastIR1State = false;
bool lastIR2State = false;

bool relay_r1_bulb      = false;
bool relay_r1_socket    = false;
bool relay_r2_bulb      = false;
bool relay_r2_socket    = false;

float voltage    = 0;
float current    = 0;
float power      = 0;
float energy     = 0;
float frequency  = 0;
float pf         = 0;

unsigned long lastFirebaseTime = 0;
unsigned long lastPzemTime     = 0;
unsigned long lastLcdTime      = 0;
unsigned long lastIRPrint      = 0;
uint8_t lcdPage = 0;

const String BASE = "/hvac/";

// ============================================================
// RELAY HELPER
// ============================================================
void setRelay(uint8_t pin, bool state) {
  digitalWrite(pin, state ? LOW : HIGH);
}

// ============================================================
// APPLY ROOM LOGIC
// ============================================================
void applyRoomControl() {
  if (room1AutoMode) {
    relay_r1_bulb   = true;           // Bulb always ON
    relay_r1_socket = room1Occupied;  // Socket follows IR detection
  } else {
    relay_r1_bulb   = room1BulbManual;
    relay_r1_socket = room1SocketManual;
  }

  if (room2AutoMode) {
    relay_r2_bulb   = true;           // Bulb always ON
    relay_r2_socket = room2Occupied;  // Socket follows IR detection
  } else {
    relay_r2_bulb   = room2BulbManual;
    relay_r2_socket = room2SocketManual;
  }

  setRelay(RELAY_ROOM1_BULB,   relay_r1_bulb);
  setRelay(RELAY_ROOM1_SOCKET, relay_r1_socket);
  setRelay(RELAY_ROOM2_BULB,   relay_r2_bulb);
  setRelay(RELAY_ROOM2_SOCKET, relay_r2_socket);
}

// ============================================================
// READ IR SENSORS
// IR sensors output LOW when they detect something
// and HIGH when nothing is detected (active low)
// Uses edge detection to catch each detection event
// Socket turns OFF after 30 seconds of no detection
// ============================================================
void readIR() {
  unsigned long now = millis();

  // Read IR sensors — active LOW means LOW = detected
  bool ir1Detected = (digitalRead(IR_ROOM1) == LOW);
  bool ir2Detected = (digitalRead(IR_ROOM2) == LOW);

  // Print every 500ms
  if (now - lastIRPrint >= 500) {
    lastIRPrint = now;
    Serial.print("[IR1]: ");
    Serial.print(ir1Detected ? "DETECTED" : "clear   ");
    Serial.print("  [IR2]: ");
    Serial.println(ir2Detected ? "DETECTED" : "clear   ");
    Serial.print("[R1 Socket]: ");
    Serial.print(room1Occupied ? "ON " : "OFF");
    Serial.print("  [R2 Socket]: ");
    Serial.println(room2Occupied ? "ON " : "OFF");
  }

  // IR1 — detection resets the 30 second countdown
  if (ir1Detected) {
    room1Occupied      = true;
    room1LastDetection = now;
    if (!lastIR1State) {
      Serial.println("[IR1] Detection - Socket ON");
    }
  }
  // No detection for 30 seconds → socket OFF
  if ((now - room1LastDetection) > OCCUPANCY_TIMEOUT) {
    if (room1Occupied) {
      Serial.println("[IR1] 30s timeout - Socket OFF");
    }
    room1Occupied = false;
  }

  // IR2 — detection resets the 30 second countdown
  if (ir2Detected) {
    room2Occupied      = true;
    room2LastDetection = now;
    if (!lastIR2State) {
      Serial.println("[IR2] Detection - Socket ON");
    }
  }
  // No detection for 30 seconds → socket OFF
  if ((now - room2LastDetection) > OCCUPANCY_TIMEOUT) {
    if (room2Occupied) {
      Serial.println("[IR2] 30s timeout - Socket OFF");
    }
    room2Occupied = false;
  }

  // Save states for next loop
  lastIR1State = ir1Detected;
  lastIR2State = ir2Detected;
}

// ============================================================
// READ PZEM
// ============================================================
void readPZEM() {
  float v   = pzem.voltage();
  float c   = pzem.current();
  float p   = pzem.power();
  float e   = pzem.energy();
  float f   = pzem.frequency();
  float pfv = pzem.pf();

  if (!isnan(v))   voltage   = v;
  if (!isnan(c))   current   = c;
  if (!isnan(p))   power     = p;
  if (!isnan(e))   energy    = e;
  if (!isnan(f))   frequency = f;
  if (!isnan(pfv)) pf        = pfv;
}

// ============================================================
// LCD DISPLAY
// ============================================================
void updateLCD() {
  lcd.clear();
  switch (lcdPage) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("  HVAC MONITOR  SIM ");
      lcd.setCursor(0, 1);
      lcd.print("Voltage: ");
      lcd.print(voltage, 1);
      lcd.print(" V   ");
      lcd.setCursor(0, 2);
      lcd.print("Current: ");
      lcd.print(current, 2);
      lcd.print(" A  ");
      lcd.setCursor(0, 3);
      lcd.print("Freq: ");
      lcd.print(frequency, 1);
      lcd.print("Hz  PF:");
      lcd.print(pf, 2);
      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print("  POWER  &  ENERGY  ");
      lcd.setCursor(0, 1);
      lcd.print("Power:  ");
      lcd.print(power, 1);
      lcd.print(" W     ");
      lcd.setCursor(0, 2);
      lcd.print("Energy: ");
      lcd.print(energy, 3);
      lcd.print(" kWh");
      lcd.setCursor(0, 3);
      lcd.print("PF: ");
      lcd.print(pf, 2);
      lcd.print("  F: ");
      lcd.print(frequency, 1);
      lcd.print("Hz");
      break;

    case 2:
      lcd.setCursor(0, 0);
      lcd.print("  ROOM STATUS       ");
      lcd.setCursor(0, 1);
      lcd.print("R1:");
      lcd.print(room1Occupied ? "OCC" : "EMP");
      lcd.print(room1AutoMode ? " AUTO" : " MAN ");
      lcd.print(" B:");
      lcd.print(relay_r1_bulb ? "ON" : "OF");
      lcd.setCursor(0, 2);
      lcd.print("   Skt:");
      lcd.print(relay_r1_socket ? "ON " : "OFF");
      lcd.print(" Pwr:");
      lcd.print(power, 0);
      lcd.print("W");
      lcd.setCursor(0, 3);
      lcd.print("R2:");
      lcd.print(room2Occupied ? "OCC" : "EMP");
      lcd.print(room2AutoMode ? " AUTO" : " MAN ");
      lcd.print(" B:");
      lcd.print(relay_r2_bulb ? "ON" : "OF");
      break;

    case 3:
      lcd.setCursor(0, 0);
      lcd.print("  ROOM 2 DETAIL     ");
      lcd.setCursor(0, 1);
      lcd.print("Occupied: ");
      lcd.print(room2Occupied ? "YES" : "NO ");
      lcd.setCursor(0, 2);
      lcd.print("Bulb:  ");
      lcd.print(relay_r2_bulb ? "ON " : "OFF");
      lcd.print("  Mode:");
      lcd.print(room2AutoMode ? "AUTO" : "MAN ");
      lcd.setCursor(0, 3);
      lcd.print("Socket: ");
      lcd.print(relay_r2_socket ? "ON " : "OFF");
      break;
  }
  lcdPage = (lcdPage + 1) % 4;
}

// ============================================================
// FIREBASE WRITE
// ============================================================
void firebaseWrite() {
  if (!Firebase.ready()) return;

  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/voltage",      voltage);
  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/current",      current);
  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/power",        power);
  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/energy",       energy);
  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/frequency",    frequency);
  Firebase.RTDB.setFloat(&fbdo, BASE + "pzem/pf",           pf);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room1/occupied",    room1Occupied);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room2/occupied",    room2Occupied);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room1/bulbState",   relay_r1_bulb);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room1/socketState", relay_r1_socket);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room2/bulbState",   relay_r2_bulb);
  Firebase.RTDB.setBool(&fbdo,  BASE + "room2/socketState", relay_r2_socket);
}

// ============================================================
// FIREBASE READ
// ============================================================
void firebaseRead() {
  if (!Firebase.ready()) return;

  if (Firebase.RTDB.getBool(&fbdo, BASE + "room1/autoMode"))
    room1AutoMode = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, BASE + "room2/autoMode"))
    room2AutoMode = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, BASE + "room1/manualBulb"))
    room1BulbManual = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, BASE + "room1/manualSocket"))
    room1SocketManual = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, BASE + "room2/manualBulb"))
    room2BulbManual = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, BASE + "room2/manualSocket"))
    room2SocketManual = fbdo.boolData();
}

// ============================================================
// FIREBASE INIT
// ============================================================
void firebaseInit() {
  Firebase.RTDB.setBool(&fbdo, BASE + "room1/autoMode",     true);
  Firebase.RTDB.setBool(&fbdo, BASE + "room2/autoMode",     true);
  Firebase.RTDB.setBool(&fbdo, BASE + "room1/manualBulb",   false);
  Firebase.RTDB.setBool(&fbdo, BASE + "room1/manualSocket", false);
  Firebase.RTDB.setBool(&fbdo, BASE + "room2/manualBulb",   false);
  Firebase.RTDB.setBool(&fbdo, BASE + "room2/manualSocket", false);
  Serial.println("[Firebase] Defaults written.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_ROOM1_BULB,   OUTPUT); digitalWrite(RELAY_ROOM1_BULB,   HIGH);
  pinMode(RELAY_ROOM1_SOCKET, OUTPUT); digitalWrite(RELAY_ROOM1_SOCKET, HIGH);
  pinMode(RELAY_ROOM2_BULB,   OUTPUT); digitalWrite(RELAY_ROOM2_BULB,   HIGH);
  pinMode(RELAY_ROOM2_SOCKET, OUTPUT); digitalWrite(RELAY_ROOM2_SOCKET, HIGH);

  // IR sensors use INPUT_PULLUP because they are active LOW
  pinMode(IR_ROOM1, INPUT_PULLUP);
  pinMode(IR_ROOM2, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("HVAC MONITOR");
  lcd.setCursor(4, 1);
  lcd.print("by Simba");
  lcd.setCursor(1, 2);
  lcd.print("Connecting WiFi...");
  delay(1500);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print("."); tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
    lcd.setCursor(0, 3);
    lcd.print("WiFi OK: ");
    lcd.print(WiFi.localIP().toString().substring(0, 10));
  } else {
    Serial.println("\n[WiFi] FAILED - running offline");
    lcd.setCursor(0, 3);
    lcd.print("WiFi FAILED - Offline");
  }
  delay(2000);
  lcd.clear();

  fbConfig.api_key               = FIREBASE_API;
  fbConfig.database_url          = FIREBASE_URL;
  fbConfig.token_status_callback = tokenStatusCallback;

  auth.user.email    = "simba@test.com";
  auth.user.password = "12345678";

  Firebase.begin(&fbConfig, &auth);
  Firebase.reconnectWiFi(true);

  unsigned long t = millis();
  while (!Firebase.ready() && (millis() - t) < 10000) delay(200);

  if (Firebase.ready()) {
    Serial.println("[Firebase] Connected.");
    firebaseInit();
  } else {
    Serial.println("[Firebase] Connection timeout.");
  }

  pzemSerial.begin(9600, SERIAL_8N1, PZEM_RX, PZEM_TX);
  Serial.println("[PZEM] Serial2 started.");
  Serial.println("[System] HVAC Monitor ready.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  readIR();

  if (now - lastPzemTime >= PZEM_INTERVAL) {
    lastPzemTime = now;
    readPZEM();
    readIR();
  }

  applyRoomControl();

  if (now - lastFirebaseTime >= FIREBASE_INTERVAL) {
    lastFirebaseTime = now;
    readIR();
    firebaseRead();
    readIR();
    applyRoomControl();
    firebaseWrite();
  }

  if (now - lastLcdTime >= LCD_REFRESH * 4) {
    lastLcdTime = now;
    updateLCD();
  }
}