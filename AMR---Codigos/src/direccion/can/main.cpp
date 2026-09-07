/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA DIRECCIÓN  v2.5.2
 *
 *   Historial de versiones:
 *   v2.3 — EEPROM con calibración. MAGIC=0xCA51 (bug int=2bytes).
 *   v2.4 — EEPROM corregida MAGIC=0xCA52, int=4bytes. Filtro N=12.
 *           Tolerancia anti-EMI. Init seguro driver OFF al boot.
 *   v2.5 — Auto-recalibración si RAW actual fuera de rango.
 *           Calibrador serial compatible MAGIC=0xCA52.
 *   v2.5.2 — Emergencia separada:
 *           Escucha 0x211 (emergencia dirección, solo failsafe real)
 *           en lugar de 0x210 para modoEmergencia.
 *           0x210 se drena sin bloquear dirección → permite
 *           maniobrar con dirección cuando CH5=-100 (freno activo).
 *           masterOffline sigue bloqueando como antes.
 *
 *   PINOUT:
 *     R_EN + L_EN  → PIN 5
 *     RPWM         → PIN 6
 *     LPWM         → PIN 9
 *     Potenciómetro→ PIN A0
 *
 *   CAN:
 *     RX 0x100 → comando dirección (0-100%)
 *     RX 0x211 → emergencia dirección (solo failsafe real)
 *     RX 0x210 → drenado, ignorado para bloqueo de dirección
 *     TX 0x101 → feedback posición (0-100%)
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP2515.h>

// ── Pines ──────────────────────────────────────────────────────
#define PIN_EN    5
#define PIN_PWM_A 6
#define PIN_PWM_B 9
#define PIN_POT   A0

// ── OLED ───────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// ── CAN ────────────────────────────────────────────────────────
#define CAN_BAUDRATE    250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define ID_CMD_DIR         0x100
#define ID_FB_DIR          0x101
#define ID_PARO_EMERG_DIR  0x211   // v2.5.2: solo failsafe real
#define ID_EMERG_MOTRIZ    0x210   // se drena, no bloquea dirección

// ── EEPROM ─────────────────────────────────────────────────────
#define EEPROM_SIZE  14
#define MAGIC_VAL    0xCA52
#define ADDR_MAGIC   0
#define ADDR_LEFT    2
#define ADDR_MID     6
#define ADDR_RIGHT   10

// ── Calibración ──────────────────────────────────────────────── version 2.5.3
int rawLeft  = 1270;   // posición IZQUIERDA  (~1198-1201) new 1270   anterior 1242
int rawMid   = 2022;   // posición MEDIA      (~2004-2008) new 2022   anterior 2005
int rawRight = 2630;   // posición DERECHA    (~2738-2743) new 2630   anterior 2865

const int MARGEN_RANGO = 50;

// ── Control ────────────────────────────────────────────────────
const int TOLERANCIA_PCT     = 4;
const int TOLERANCIA_MIN_RAW = 25;

int  cmdRecibido    = 50;
int  rawObjetivo    = 0;
bool modoEmergencia = false;

// ── Watchdog ───────────────────────────────────────────────────
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long lastCanRx = 0;
bool masterOffline = false;

// ── Filtro ─────────────────────────────────────────────────────
const int N = 12;
int  buf[N];
long suma  = 0;
int  idx   = 0;
bool lleno = false;

bool initCompleto = false;

// ================================================================
void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  uint16_t magic;
  EEPROM.get(ADDR_MAGIC, magic);
  if (magic == MAGIC_VAL) {
    EEPROM.get(ADDR_LEFT,  rawLeft);
    EEPROM.get(ADDR_MID,   rawMid);
    EEPROM.get(ADDR_RIGHT, rawRight);
  }
}

void verificarRango(int rawActual) {
  int lo = min(rawLeft, rawRight);
  int hi = max(rawLeft, rawRight);
  if (rawActual < lo - MARGEN_RANGO || rawActual > hi + MARGEN_RANGO) {
    int mitadRango = abs(rawRight - rawLeft) / 2;
    if (rawLeft < rawRight) {
      rawMid   = rawActual;
      rawLeft  = rawActual - mitadRango;
      rawRight = rawActual + mitadRango;
    } else {
      rawMid   = rawActual;
      rawLeft  = rawActual + mitadRango;
      rawRight = rawActual - mitadRango;
    }
    rawLeft  = constrain(rawLeft,  0, 4095);
    rawMid   = constrain(rawMid,   0, 4095);
    rawRight = constrain(rawRight, 0, 4095);
    Serial.println(F("[DIR v2.5.2] Rango recalculado"));
    Serial.print(F("  L=")); Serial.print(rawLeft);
    Serial.print(F(" M=")); Serial.print(rawMid);
    Serial.print(F(" R=")); Serial.println(rawRight);
  }
}

int leerPot() {
  int v = analogRead(PIN_POT);
  suma -= buf[idx];
  buf[idx] = v;
  suma += v;
  idx++;
  if (idx >= N) { idx = 0; lleno = true; }
  return lleno ? (suma / N) : (suma / idx);
}

int rawToPct(int raw) {
  int lo = min(rawLeft, rawRight);
  int hi = max(rawLeft, rawRight);
  raw = constrain(raw, lo, hi);
  return (int)map(raw, rawLeft, rawRight, 0, 100);
}

int pctToRaw(int pct) {
  pct = constrain(pct, 0, 100);
  return (int)map(pct, 0, 100, rawLeft, rawRight);
}

int tolRaw() {
  return max(abs(rawRight - rawLeft) * TOLERANCIA_PCT / 100, TOLERANCIA_MIN_RAW);
}

// ── Motor ──────────────────────────────────────────────────────
void motorStop()        { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, 0); }
void motorFwd(byte pwm) { analogWrite(PIN_PWM_B, 0); analogWrite(PIN_PWM_A, pwm); }
void motorRev(byte pwm) { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, pwm); }

void entrarModoSeguro() { motorStop(); }

void mover() {
  // v2.5.2: modoEmergencia ya NO bloquea — solo masterOffline
  if (!initCompleto || masterOffline) {
    motorStop();
    return;
  }
  // Emergencia real (failsafe) sí bloquea
  if (modoEmergencia) {
    motorStop();
    return;
  }

  int raw   = leerPot();
  int error = rawObjetivo - raw;

  if (abs(error) <= tolRaw()) {
    motorStop();
    return;
  }

  int  rango = abs(rawRight - rawLeft);
  byte pwm   = (byte)constrain(map(abs(error), 0, rango, 135, 195), 0, 255);

  if (error > 0) motorFwd(pwm);
  else           motorRev(pwm);
}

// ── CAN ────────────────────────────────────────────────────────
void enviarFeedbackCAN() {
  if (!canOK) return;
  byte pos = rawToPct(leerPot());
  mcp.beginPacket(ID_FB_DIR);
  mcp.write(pos);
  mcp.endPacket();
}

// ── OLED ───────────────────────────────────────────────────────
void updateOLED() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - DIRECCION"));
  display.setTextColor(SSD1306_WHITE);

  if (masterOffline) {
    display.setCursor(0, 16); display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 28);
    unsigned long ms = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(ms / 1000); display.print('.');
    display.print((ms % 1000) / 100); display.print('s');
    display.setCursor(0, 40); display.print(F("MODO SEGURO:"));
    display.setCursor(0, 50); display.print(F("Motor detenido"));
    display.display(); return;
  }

  if (modoEmergencia) {
    display.setTextSize(2);
    display.setCursor(10, 20); display.print(F("EMERG!"));
    display.setTextSize(1);
    display.setCursor(0, 44); display.print(F("Failsafe SBUS"));
  } else {
    int raw = leerPot();
    int pos = rawToPct(raw);

    display.setCursor(0, 16);
    display.print(F("CMD: ")); display.print(cmdRecibido); display.print('%');
    display.setCursor(64, 16);
    display.print(F("POS: ")); display.print(pos); display.print('%');

    display.setCursor(0, 28);
    display.print(F("ERR: "));
    int err = cmdRecibido - pos;
    if (err > 0) display.print('+');
    display.print(err);
    display.setCursor(64, 28);
    display.print(F("RAW:")); display.print(raw);

    display.setCursor(0, 38);
    display.print(F("L=")); display.print(rawLeft);
    display.print(F(" R=")); display.print(rawRight);
  }

  display.setCursor(0, 50);
  display.print(canOK ? F("CAN OK") : F("CAN ERR"));
  display.setCursor(64, 50);
  display.print(F("TOL:")); display.print(tolRaw());

  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(rawToPct(leerPot()), 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);
  int targetX = map(cmdRecibido, 0, 100, 1, 127);
  display.drawLine(targetX, 53, targetX, 64, SSD1306_WHITE);

  display.display();
}

// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_EN,    OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  digitalWrite(PIN_EN,    LOW);
  digitalWrite(PIN_PWM_A, LOW);
  digitalWrite(PIN_PWM_B, LOW);

  analogWriteFreq(10000);
  analogWriteRange(255);
  analogReadResolution(12);
  motorStop();
  delay(50);

  for (int i = 0; i < N; i++) {
    buf[i] = analogRead(PIN_POT);
    suma += buf[i];
    delay(3);
  }
  lleno = true;

  eepromLoad();
  verificarRango(leerPot());

  rawObjetivo = leerPot();
  cmdRecibido = rawToPct(rawObjetivo);

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 10); display.print(F("DIRECCION v2.5.2"));
    display.setCursor(10, 22);
    display.print(F("L=")); display.print(rawLeft);
    display.print(F(" M=")); display.print(rawMid);
    display.setCursor(10, 32);
    display.print(F("R=")); display.print(rawRight);
    display.setCursor(10, 44); display.print(F("Dir libre en freno"));
    display.display();
    delay(800);
  }

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);
  pinMode(PIN_CAN_RESET, OUTPUT);
  digitalWrite(PIN_CAN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_CAN_RESET, HIGH);
  delay(10);

  if (mcp.begin(CAN_BAUDRATE)) canOK = true;

  if (displayOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(canOK ? F("CAN OK") : F("CAN ERR"));
    display.setCursor(10, 36);
    display.print(F("Esperando master..."));
    display.display();
    delay(500);
  }

  digitalWrite(PIN_EN, HIGH);

  lastCanRx     = millis();
  masterOffline = true;
  initCompleto  = true;
}

// ================================================================
void loop() {
  unsigned long now = millis();

  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;
    uint32_t id = mcp.packetId();

    // v2.5.2: watchdog se resetea con CMD dirección o emerg dirección
    bool idRelevante = (id == ID_CMD_DIR || id == ID_PARO_EMERG_DIR);
    if (idRelevante) {
      lastCanRx = now;
      if (masterOffline) masterOffline = false;
    }

    // Emergencia dirección (failsafe real) — bloquea dirección
    if (id == ID_PARO_EMERG_DIR && mcp.available()) {
      byte em = mcp.read();
      if (em == 1) { modoEmergencia = true;  motorStop(); }
      else         { modoEmergencia = false; }
      enviarFeedbackCAN();
      continue;
    }

    // Emergencia motriz — se drena, no bloquea dirección
    if (id == ID_EMERG_MOTRIZ) {
      while (mcp.available()) mcp.read();
      continue;
    }

    // Saltar si offline o emergencia real
    if (modoEmergencia || masterOffline) {
      while (mcp.available()) mcp.read();
      continue;
    }

    // Comando dirección
    if (id == ID_CMD_DIR && mcp.available()) {
      cmdRecibido = mcp.read();
      cmdRecibido = constrain(cmdRecibido, 0, 100);
      rawObjetivo = pctToRaw(cmdRecibido);
      enviarFeedbackCAN();
      continue;
    }

    while (mcp.available()) mcp.read();
  }

  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    entrarModoSeguro();
  }

  mover();

  static unsigned long lastFeedback = 0;
  if (now - lastFeedback >= 100) {
    lastFeedback = now;
    if (!masterOffline) enviarFeedbackCAN();
  }

  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    if (displayOK) updateOLED();
  }
}
