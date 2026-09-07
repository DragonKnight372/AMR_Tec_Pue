/*
 * ================================================================
 *   SANDBOX TEST — FRENOS  (copia de v2.7.2)
 *   Experimentar AQUÍ. Si falla, flashear freno_can.
 *   No editar src/freno/can ni src/freno/serial.
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA FRENOS  v2.7.2
 *
 *   Historial de versiones:
 *   v2.5   — Freno binario (cmd>=90). Watchdog 1500ms.
 *            Modo recuperación + ACT_FLT. Drenado CAN.
 *   v2.6   — Freno gradual en RAW. Bug: byte no cabe 815-868.
 *   v2.6.3 — Control en %. frenoCmd directo CAN. Sin pctToRaw().
 *            Funciona pero dirección física invertida.
 *   v2.6.4 — Intento fallido: doble inversión → bucle.
 *   v2.6.5 — Fix dirección física: 4 cambios interdependientes.
 *            Bug: POS_EMERGENCIA=0 no frenaba al desactivar CH3.
 *   v2.6.6 — Fix POS_NORMAL=0, POS_EMERGENCIA=100. Versión
 *            definitiva y operativa.
 *   v2.7   — Relé Luz estroboscópica en D4:
 *            D4 HIGH cuando:
 *              - modoEmergencia = true  (CAN 0x210)
 *              - masterOffline  = true  (watchdog)
 *              - frenoCmd >= UMBRAL_RELE (90%) (CH3/CH5 o gradual
 *                al máximo)
 *            D4 LOW en cualquier otro caso (freno gradual < 90%)
 *            Constante UMBRAL_RELE ajustable.
 * 
 *    v2.7.1 -  Cambio de mapeo de RAW_MIN, y RAW_MAX
 *    v2.7.2 — Tope 100% = llegado: si objetivo es 100% y
 *             RAW >= RAW_MAX, motorOff(). No tratar el final
 *             de escala como "fuera de rango" (evita stall).
 *
 *   Mapeo v2.7 (igual que v2.6.6):
 *     RAW_MIN(815) →   0% = sin freno   (actuador retraído)
 *     RAW_MAX(868) → 100% = freno total (actuador extendido)
 *
 *   PINOUT nuevo:
 *     D4 → Relé estroboscópico (HIGH = freno máximo activo)
 *
 *   CAN:
 *     RX 0x200 → freno proporcional (0-100%)
 *     RX 0x210 → emergencia
 *     TX 0x201 → feedback posición (0-100%)
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP2515.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

#define CAN_BAUDRATE    250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define ID_CMD_FRENO    0x200
#define ID_FB_FRENO     0x201
#define ID_EMERGENCIA   0x210

#define PIN_PWM         6
#define PIN_DIR         9
#define PIN_FEEDBACK    A0
#define ACT_FLT         5
#define PIN_RELE        4    // ← NUEVO v2.7: relé estroboscópico

// ── Límites físicos RAW ────────────────────────────────────────
const int RAW_MIN = 830;   // retraído  = sin freno    =   0%
const int RAW_MAX = 900;   // extendido = freno total  = 100%

// ── Posiciones en % ───────────────────────────────────────────
#define POS_NORMAL        0  // % — sin freno
#define POS_EMERGENCIA  100  // % — freno total

// ── Umbral relé estroboscópico ─────────────────────────────────
// D4 HIGH cuando frenoCmd >= este valor, emergencia o masterOffline
const byte UMBRAL_RELE = 90;   // % — ajustable

// ── Control ────────────────────────────────────────────────────
const int  TOLERANCIA              = 8;
const unsigned long TIMEOUT_FUERA_RANGO_MS = 2000;

// ── Watchdog ──────────────────────────────────────────────────
const unsigned long WATCHDOG_TIMEOUT_MS = 1500;
unsigned long lastCanRx = 0;
bool masterOffline      = false;

// ── Estado ────────────────────────────────────────────────────
byte frenoCmd          = 0;
byte posicionObjetivo  = POS_NORMAL;
bool modoEmergencia    = false;
bool errorPot          = false;
bool paroFisico        = false;
bool modoRecuperacion  = false;
unsigned long tFueraDeRango = 0;

// ── Filtro promedio móvil ─────────────────────────────────────
const int N = 5;
int  bufferLecturas[N];
long sumaLecturas = 0;
int  indice       = 0;
bool bufferLleno  = false;

byte ultimoPWM       = 0;
byte ultimaDireccion = 0;
bool fueraDeRango    = false;

// ================================================================
int leerPotFiltrado() {
  int nueva = analogRead(PIN_FEEDBACK);
  sumaLecturas -= bufferLecturas[indice];
  bufferLecturas[indice] = nueva;
  sumaLecturas += nueva;
  indice++;
  if (indice >= N) { indice = 0; bufferLleno = true; }
  return bufferLleno ? (sumaLecturas / N) : (sumaLecturas / indice);
}

byte lecturaToPorcentaje(int lecturaRaw) {
  lecturaRaw = constrain(lecturaRaw, RAW_MIN, RAW_MAX);
  int pct = map(lecturaRaw, RAW_MIN, RAW_MAX, 0, 100);
  return (byte)constrain(pct, 0, 100);
}

bool detectarParoFisico() {
  return (digitalRead(ACT_FLT) == LOW);
}

void enviarFeedbackCAN() {
  if (!canOK) return;
  byte pos = lecturaToPorcentaje(leerPotFiltrado());
  mcp.beginPacket(ID_FB_FRENO);
  mcp.write(pos);
  mcp.endPacket();
}

void actualizarObjetivo() {
  if (masterOffline || modoEmergencia) {
    posicionObjetivo = POS_EMERGENCIA;
  } else {
    posicionObjetivo = frenoCmd;
  }
}

// ── NUEVO v2.7: actualizar relé estroboscópico ─────────────────
void actualizarRele() {
  bool activar = modoEmergencia
              || masterOffline
              || (frenoCmd >= UMBRAL_RELE);
  digitalWrite(PIN_RELE, activar ? HIGH : LOW);
}

void motorOff() {
  analogWrite(PIN_PWM, 0);
  ultimoPWM = 0;
}

// ================================================================
void moverActuadorHaciaObjetivo() {
  if (!bufferLleno) { motorOff(); return; }

  paroFisico = detectarParoFisico();
  int lectura  = leerPotFiltrado();
  unsigned long now = millis();
  bool enRango = (lectura >= RAW_MIN && lectura <= RAW_MAX);

  if (paroFisico) {
    motorOff();
    fueraDeRango  = !enRango;
    tFueraDeRango = 0;
    return;
  }

  if (modoRecuperacion) {
    if (enRango) {
      modoRecuperacion = false;
      errorPot         = false;
      fueraDeRango     = false;
      tFueraDeRango    = 0;
      motorOff();
      return;
    }
    digitalWrite(PIN_DIR, HIGH);
    analogWrite(PIN_PWM, 160);
    ultimoPWM = 160; ultimaDireccion = 1;
    return;
  }

  if (errorPot) {
    motorOff();
    if (modoEmergencia || masterOffline) {
      if (enRango) errorPot = false;
      else         modoRecuperacion = true;
    }
    return;
  }

  if (!enRango) {
    // Objetivo 100%: RAW en o más allá del tope medido = llegado. Soltar motor.
    if (posicionObjetivo >= 100 && lectura >= RAW_MAX) {
      motorOff();
      fueraDeRango  = false;
      tFueraDeRango = 0;
      return;
    }
    if (!fueraDeRango) tFueraDeRango = now;
    fueraDeRango = true;
    if (now - tFueraDeRango > TIMEOUT_FUERA_RANGO_MS) {
      errorPot = true; motorOff(); return;
    }
    if (lectura < RAW_MIN) {
      digitalWrite(PIN_DIR, LOW);  analogWrite(PIN_PWM, 160);
      ultimoPWM = 160; ultimaDireccion = 0;
    } else {
      digitalWrite(PIN_DIR, HIGH); analogWrite(PIN_PWM, 160);
      ultimoPWM = 160; ultimaDireccion = 1;
    }
    return;
  }

  fueraDeRango  = false;
  tFueraDeRango = 0;

  byte posicionActual = lecturaToPorcentaje(lectura);
  int  error = (int)posicionObjetivo - (int)posicionActual;

  if (abs(error) <= TOLERANCIA) {
    motorOff();
    return;
  }

  byte pwmValor = (byte)constrain(
    map(abs(error), TOLERANCIA, 100, 140, 255),
    140, 255);

  if (error > 0) {
    digitalWrite(PIN_DIR, LOW);  ultimaDireccion = 1;
  } else {
    digitalWrite(PIN_DIR, HIGH); ultimaDireccion = 0;
  }

  analogWrite(PIN_PWM, pwmValor);
  ultimoPWM = pwmValor;
}

// ================================================================
void updateOLED() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - FRENOS"));
  display.setTextColor(SSD1306_WHITE);

  if (masterOffline && !paroFisico) {
    display.setCursor(0, 16); display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 28);
    unsigned long ms = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(ms / 1000); display.print('.');
    display.print((ms % 1000) / 100); display.print('s');
    display.setCursor(0, 42); display.print(F("FRENO APLICADO"));
    display.setCursor(0, 52); display.print(F("RELE: ON"));
    display.display(); return;
  }

  if (paroFisico && !modoRecuperacion) {
    display.setTextSize(2);
    display.setCursor(8, 18); display.print(F("PARO FIS"));
    display.setTextSize(1);
    display.setCursor(0, 40); display.print(F("Boton rojo activo"));
    display.setCursor(0, 50);
    if ((millis() / 500) % 2 == 0) display.print(F("(esperando...)"));
  }
  else if (modoRecuperacion) {
    display.setCursor(0, 16); display.print(F("MODO: RECUPERACION"));
    display.setCursor(0, 28); display.print(F("Retrayendo..."));
    display.setCursor(0, 40);
    display.print(F("ACT_FLT: "));
    display.print(paroFisico ? F("FAULT") : F("OK"));
    display.setCursor(0, 50);
    display.print(F("Pot: ")); display.print(leerPotFiltrado());
  }
  else if (modoEmergencia) {
    display.setTextSize(2);
    display.setCursor(10, 20); display.print(F("EMERG!"));
    display.setTextSize(1);
    display.setCursor(0, 44); display.print(F("RELE: ON"));
  }
  else if (fueraDeRango && !errorPot) {
    display.setCursor(0, 18); display.print(F("FUERA RANGO"));
    display.setCursor(0, 28);
    display.print(ultimaDireccion ? F("Extendiendo...") : F("Retrayendo..."));
    unsigned long restante = TIMEOUT_FUERA_RANGO_MS - (millis() - tFueraDeRango);
    display.setCursor(0, 38);
    display.print(F("Timeout: "));
    display.print(restante / 1000); display.print('.');
    display.print((restante % 1000) / 100); display.print('s');
  }
  else {
    byte posActual = lecturaToPorcentaje(leerPotFiltrado());

    display.setCursor(0, 14);
    display.print(F("CMD: ")); display.print(frenoCmd); display.print('%');
    display.setCursor(64, 14);
    display.print(F("POS: ")); display.print(posActual); display.print('%');

    display.setCursor(0, 24);
    display.print(F("OBJ: ")); display.print(posicionObjetivo); display.print('%');
    display.setCursor(64, 24);
    display.print(F("ERR: "));
    int err = (int)posicionObjetivo - (int)posActual;
    if (err > 0) display.print('+');
    display.print(err);

    display.setCursor(0, 34);
    display.print(F("FLT: "));
    display.print(digitalRead(ACT_FLT) == LOW ? F("FAULT") : F("OK"));
    display.setCursor(64, 34);
    display.print(F("PWM: ")); display.print(ultimoPWM);

    display.setCursor(0, 44);
    display.print(F("DIR: "));
    display.print(ultimaDireccion ? F("EXTND") : F("RETR"));
    // NUEVO v2.7: estado del relé en OLED
    display.setCursor(64, 44);
    bool releOn = modoEmergencia || masterOffline || (frenoCmd >= UMBRAL_RELE);
    display.print(releOn ? F("RELE:ON") : F("RELE:--"));
  }

  byte actual = (errorPot && !modoRecuperacion)
                  ? 0 : lecturaToPorcentaje(leerPotFiltrado());
  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(actual, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);
  int targetX = map(posicionObjetivo, 0, 100, 1, 127);
  display.drawLine(targetX, 53, targetX, 64, SSD1306_WHITE);

  if (errorPot && !modoRecuperacion && !paroFisico && !masterOffline) {
    int boxX=14, boxY=18, boxW=100, boxH=28;
    display.fillRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);
    display.drawRect(boxX+2, boxY+2, boxW-4, boxH-4, SSD1306_BLACK);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(boxX+28, boxY+4);  display.print(F("!!ERROR!!"));
    display.setCursor(boxX+6,  boxY+14); display.print(F("Pot fuera rango"));
    display.setCursor(boxX+12, boxY+22);
    if ((millis()/500)%2==0) display.print(F("Desactiva enable"));
    display.setTextColor(SSD1306_WHITE);
  }

  display.display();
}

// ================================================================
void setup() {
  Serial.begin(115200);
  analogWriteFreq(10000);
  analogWriteRange(255);

  pinMode(PIN_PWM,      OUTPUT);
  pinMode(PIN_DIR,      OUTPUT);
  pinMode(ACT_FLT,      INPUT_PULLUP);
  pinMode(PIN_RELE,     OUTPUT);        // ← NUEVO v2.7
  analogWrite(PIN_PWM,  0);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_RELE, LOW);          // ← relé apagado al boot

  for (int i = 0; i < N; i++) {
    bufferLecturas[i] = analogRead(PIN_FEEDBACK);
    sumaLecturas += bufferLecturas[i];
    delay(5);
  }
  bufferLleno = true;

  frenoCmd = 0;
  actualizarObjetivo();

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 14); display.print(F("FRENOS TEST"));
    display.setCursor(10, 26); display.print(F("Rele D4 estrobos."));
    display.setCursor(10, 38); display.print(F("Umbral: 90%"));
    display.setCursor(10, 50); display.print(F("Watchdog 1500ms"));
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

  lastCanRx     = millis();
  masterOffline = true;
  actualizarObjetivo();
  actualizarRele();   // ← relé consistente desde el arranque
}

// ================================================================
void loop() {
  unsigned long now = millis();

  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;
    uint32_t id = mcp.packetId();

    bool idRelevante = (id == ID_CMD_FRENO || id == ID_EMERGENCIA);
    if (idRelevante) {
      lastCanRx = now;
      if (masterOffline) {
        masterOffline = false;
        actualizarObjetivo();
        actualizarRele();   // ← NUEVO v2.7
      }
    }

    if (id == ID_EMERGENCIA && mcp.available()) {
      byte em = mcp.read();
      modoEmergencia = (em == 1);
      actualizarObjetivo();
      actualizarRele();     // ← NUEVO v2.7
      enviarFeedbackCAN();
    }
    else if (id == ID_CMD_FRENO && mcp.available()) {
      byte cmd = mcp.read();
      if (!modoEmergencia) {
        frenoCmd = constrain(cmd, 0, 100);
        actualizarObjetivo();
        actualizarRele();   // ← NUEVO v2.7
      }
      enviarFeedbackCAN();
    }
    else {
      while (mcp.available()) mcp.read();
    }
  }

  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    actualizarObjetivo();
    actualizarRele();       // ← NUEVO v2.7
  }

  moverActuadorHaciaObjetivo();

  static unsigned long lastFeedback = 0;
  if (now - lastFeedback >= 200) {
    lastFeedback = now;
    if (!masterOffline) enviarFeedbackCAN();
  }

  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    if (displayOK) updateOLED();
  }
}