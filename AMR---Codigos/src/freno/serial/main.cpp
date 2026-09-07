/*
 * ================================================================
 *   FEATHER RP2040 SERIAL — SUBSISTEMA FRENOS (CALIBRACIÓN)
 *
 *   Sin CAN. Enviar un valor 0-100 por Serial para mover el pistón
 *   a esa posición (closed-loop). Usar para verificar/actualizar
 *   los límites físicos del actuador cuando cambien.
 *
 *   COMANDOS (Serial Monitor, line ending: Newline, 115200 baud):
 *     0-100  →  Ir a esa posición (%)
 *     s      →  Stop motor
 *     m      →  Marcar lectura actual como RAW_MIN (retraído)
 *     M      →  Marcar lectura actual como RAW_MAX (extendido)
 *     p      →  Imprimir calibración lista para copiar al CAN
 *     h      →  Ayuda
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
//   CALIBRACIÓN DEL ACTUADOR
//   ↓ ESTOS SON LOS ÚNICOS VALORES QUE HAY QUE ACTUALIZAR ↓
//
//   Procedimiento:
//     1. Enviar "0"   → pistón se extiende (freno activo)
//        Anotar el valor RAW que aparece en monitor → ese es RAW_MAX
//     2. Enviar "100" → pistón se retrae (posición normal)
//        Anotar el valor RAW que aparece en monitor → ese es RAW_MIN
//     3. Actualizar los valores aquí y en freno/can/main.cpp
//
//                       ADC leído    Estado físico        Posición
//   RAW_MIN  =  815   ← retraído    (normal/liberado)  = 100 %
//   RAW_MAX  =  868   ← extendido   (freno activo)     =   0 %
//
//   Nota: la escala es invertida (más ADC = más extendido = menos %)
// ================================================================
const int RAW_MIN = 830;   // <-- ACTUALIZAR si cambia el límite retraído 830
const int RAW_MAX = 900;   // <-- ACTUALIZAR si cambia el límite extendido

// ================================================================
//   HARDWARE (no modificar)
// ================================================================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

#define PIN_PWM      6
#define PIN_DIR      9
#define PIN_FEEDBACK A0
#define ACT_FLT      5

// ================================================================
//   CONTROL
// ================================================================
byte posicionObjetivo = 100;
const int TOLERANCIA  = 8;

byte ultimoPWM    = 0;
byte ultimaDir    = 0;
bool motorActivo  = false;
bool motorHabilitado = true;

// Copia en RAM de los límites (se actualiza con m/M sin tocar el flash)
int rawMinActual = RAW_MIN;
int rawMaxActual = RAW_MAX;

// --- Filtro media móvil ---
const int N = 5;
int  bufferLecturas[N];
long sumaLecturas = 0;
int  indice       = 0;
bool bufferLleno  = false;

// ================================================================
int leerPotFiltrado() {
  int nueva = analogRead(PIN_FEEDBACK);
  sumaLecturas -= bufferLecturas[indice];
  bufferLecturas[indice] = nueva;
  sumaLecturas += nueva;
  indice++;
  if (indice >= N) { indice = 0; bufferLleno = true; }
  return bufferLleno ? (sumaLecturas / N) : (sumaLecturas / max(indice, 1));
}

byte lecturaToPorcentaje(int raw) {
  raw = constrain(raw, rawMinActual, rawMaxActual);
  return (byte)constrain(map(raw, rawMaxActual, rawMinActual, 0, 100), 0, 100);
}

void motorStop() {
  analogWrite(PIN_PWM, 0);
  motorActivo = false;
  ultimoPWM   = 0;
}

void moverHaciaObjetivo(int raw) {
  if (!motorHabilitado) { motorStop(); return; }

  byte posActual = lecturaToPorcentaje(raw);
  int  error     = (int)posicionObjetivo - (int)posActual;

  if (abs(error) <= TOLERANCIA) { motorStop(); return; }

  ultimaDir = (error > 0) ? HIGH : LOW;   // HIGH = retraer, LOW = extender
  digitalWrite(PIN_DIR, ultimaDir);

  byte pwm = (byte)constrain(map(abs(error), TOLERANCIA, 100, 140, 255), 140, 255);
  analogWrite(PIN_PWM, pwm);
  motorActivo = true;
  ultimoPWM   = pwm;
}

// ================================================================
void imprimirCalibracion() {
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  CALIBRACION ACTUAL (valores en RAM)"));
  Serial.println(F("  (compilados en el firmware):"));
  Serial.print  (F("    RAW_MIN (retraido/100%) = ")); Serial.println(RAW_MIN);
  Serial.print  (F("    RAW_MAX (extendido/0%)  = ")); Serial.println(RAW_MAX);
  Serial.println(F("  (modificados en esta sesion):"));
  Serial.print  (F("    rawMinActual            = ")); Serial.println(rawMinActual);
  Serial.print  (F("    rawMaxActual            = ")); Serial.println(rawMaxActual);
  Serial.println(F(""));
  Serial.println(F("  -- Copiar en freno/can/main.cpp --"));
  Serial.print  (F("    const int RAW_MIN = ")); Serial.print(rawMinActual); Serial.println(F(";"));
  Serial.print  (F("    const int RAW_MAX = ")); Serial.print(rawMaxActual); Serial.println(F(";"));
  Serial.println(F("  -- Y actualizar en freno/serial/main.cpp --"));
  Serial.print  (F("    const int RAW_MIN = ")); Serial.print(rawMinActual); Serial.println(F(";"));
  Serial.print  (F("    const int RAW_MAX = ")); Serial.print(rawMaxActual); Serial.println(F(";"));
  Serial.println(F("========================================"));
}

void imprimirMenu() {
  Serial.println(F(""));
  Serial.println(F("=== CALIBRACION FRENOS ==="));
  Serial.print  (F("  Valores compilados:  MIN=")); Serial.print(RAW_MIN);
  Serial.print  (F("  MAX=")); Serial.println(RAW_MAX);
  Serial.println(F("  --------------------------"));
  Serial.println(F("  0-100  Mover a posicion (%)"));
  Serial.println(F("  s      Stop motor"));
  Serial.println(F("  m      Marcar RAW_MIN con lectura actual (retraido)"));
  Serial.println(F("  M      Marcar RAW_MAX con lectura actual (extendido)"));
  Serial.println(F("  p      Imprimir calibracion para copiar al codigo CAN"));
  Serial.println(F("  h      Ayuda"));
}

// ================================================================
void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  int raw = leerPotFiltrado();

  if (linea.length() == 1) {
    char c = linea.charAt(0);

    if (c == 's' || c == 'S') {
      motorHabilitado = false;
      motorStop();
      Serial.println(F(">> Stop"));
      return;
    }
    if (c == 'm') {
      rawMinActual = raw;
      motorStop();
      Serial.print(F(">> RAW_MIN actualizado = ")); Serial.println(rawMinActual);
      Serial.println(F(">> (Recuerda pegar el valor en ambos main.cpp cuando termines)"));
      return;
    }
    if (c == 'M') {
      rawMaxActual = raw;
      motorStop();
      Serial.print(F(">> RAW_MAX actualizado = ")); Serial.println(rawMaxActual);
      Serial.println(F(">> (Recuerda pegar el valor en ambos main.cpp cuando termines)"));
      return;
    }
    if (c == 'p' || c == 'P') {
      imprimirCalibracion();
      return;
    }
    if (c == 'h' || c == 'H' || c == '?') {
      imprimirMenu();
      return;
    }
  }

  // Valor numérico → posición objetivo
  bool esNumero = true;
  for (unsigned int i = 0; i < linea.length(); i++) {
    if (!isDigit(linea.charAt(i))) { esNumero = false; break; }
  }

  if (esNumero) {
    int val = constrain(linea.toInt(), 0, 100);
    posicionObjetivo = (byte)val;
    motorHabilitado  = true;
    Serial.print(F(">> Objetivo: ")); Serial.print(posicionObjetivo);
    Serial.println(posicionObjetivo == 0   ? F(" % (extendido/freno)") :
                   posicionObjetivo == 100 ? F(" % (retraido/normal)") : F(" %"));
  } else {
    Serial.println(F("?? Desconocido. Enviar 'h' para ayuda."));
  }
}

// ================================================================
void imprimirEstado(int raw, byte pct) {
  Serial.print(F("RAW:")); Serial.print(raw);
  // Indicador visual si el raw está fuera del rango actual
  if (raw < rawMinActual)      Serial.print(F("<MIN"));
  else if (raw > rawMaxActual) Serial.print(F(">MAX"));
  else                         Serial.print(F("     "));
  Serial.print(F("  PCT:")); Serial.print(pct); Serial.print(F("%"));
  Serial.print(F("  OBJ:")); Serial.print(posicionObjetivo); Serial.print(F("%"));
  Serial.print(F("  "));
  Serial.print(motorActivo ? (ultimaDir == LOW ? F("EXTND") : F("RETR ")) : F("STOP "));
  Serial.print(F(" PWM:")); Serial.print(ultimoPWM);
  Serial.print(F("  FLT:")); Serial.print(digitalRead(ACT_FLT) == LOW ? F("FAULT") : F("OK"));
  Serial.println();
}

// ================================================================
void updateOLED(int raw, byte pct) {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("FRENOS CALIBRACION"));
  display.setTextColor(SSD1306_WHITE);

  // Valor RAW prominente — lo más importante en calibración
  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print(F("RAW:"));
  display.print(raw);

  display.setTextSize(1);

  // Indicador fuera de rango
  if (raw < rawMinActual || raw > rawMaxActual) {
    display.setCursor(100, 14);
    display.print(raw < rawMinActual ? F("<MIN") : F(">MAX"));
  }

  display.setCursor(0, 32);
  display.print(F("PCT:")); display.print(pct);
  display.print(F("%  OBJ:")); display.print(posicionObjetivo); display.print(F("%"));

  display.setCursor(0, 41);
  display.print(F("MIN:")); display.print(rawMinActual);
  display.print(F(" MAX:")); display.print(rawMaxActual);

  display.setCursor(0, 50);
  if (motorActivo)
    display.print(ultimaDir == LOW ? F("EXTND") : F("RETR "));
  else
    display.print(F("STOP "));
  display.print(F(" PWM:")); display.print(ultimoPWM);
  display.print(F(" ")); display.print(digitalRead(ACT_FLT) == LOW ? F("FLT!") : F("OK"));

  display.display();
}

// ================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  analogWriteFreq(10000);
  analogWriteRange(255);

  pinMode(PIN_PWM,  OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(ACT_FLT,  INPUT_PULLUP);
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_DIR, LOW);

  for (int i = 0; i < N; i++) {
    bufferLecturas[i] = analogRead(PIN_FEEDBACK);
    sumaLecturas += bufferLecturas[i];
    delay(2);
  }
  bufferLleno = true;

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(4, 14);
    display.print(F("FRENOS CALIBRACION"));
    display.setCursor(4, 26);
    display.print(F("MIN:")); display.print(RAW_MIN);
    display.print(F(" MAX:")); display.print(RAW_MAX);
    display.setCursor(4, 38);
    display.print(F("Serial 115200"));
    display.display();
    delay(1000);
  }

  imprimirMenu();
  imprimirCalibracion();
}

// ================================================================
void loop() {
  static String inputBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        procesarLinea(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  int  raw = leerPotFiltrado();
  byte pct = lecturaToPorcentaje(raw);

  moverHaciaObjetivo(raw);

  static unsigned long lastSerial = 0;
  if (millis() - lastSerial >= 200) {
    lastSerial = millis();
    imprimirEstado(raw, pct);
  }

  static unsigned long lastOLED = 0;
  if (millis() - lastOLED >= 100) {
    lastOLED = millis();
    if (displayOK) updateOLED(raw, pct);
  }
}
