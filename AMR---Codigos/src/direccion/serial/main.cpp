/*
 * ================================================================
 *   CALIBRACIÓN DIRECCIÓN — Serial
 *   Feather RP2040 + BTS7960
 * ================================================================
 *
 * PINOUT:
 *   R_EN + L_EN  → PIN 5   (enable)
 *   RPWM         → PIN 6   (PWM A)
 *   LPWM         → PIN 9   (PWM B)
 *   Potenciómetro→ PIN A0  (ADC 12 bits, 0-4095)
 *
 * COMANDOS SERIAL (115200 baud):
 *   0-100      → mover a ese porcentaje entre IZQ y DER
 *   L / M / R  → ir a la posición guardada IZQ / MED / DER
 *   SL         → guardar posición actual como IZQUIERDA
 *   SM         → guardar posición actual como MEDIO
 *   SR         → guardar posición actual como DERECHA
 *   s          → parar motor
 *   ?          → imprimir calibración guardada
 *
 * SALIDA CONTINUA (cada 100 ms):
 *   RAW=2453 | L=1728 M=2558 R=3388 | POS=43% | <MED
 *
 * Los puntos L/M/R se guardan en EEPROM y persisten entre reinicios.
 * ================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// ── Pines ──────────────────────────────────────────────────────
#define PIN_EN    5
#define PIN_PWM_A 6
#define PIN_PWM_B 9
#define PIN_POT   A0

// ── EEPROM ─────────────────────────────────────────────────────
#define EEPROM_SIZE  14       // 2(magic) + 4(left) + 4(mid) + 4(right)
#define MAGIC_VAL    0xCA52  // valor nuevo para invalidar EEPROM vieja corrupta
#define ADDR_MAGIC   0
#define ADDR_LEFT    2
#define ADDR_MID     6       // FIX: int = 4 bytes, no 2
#define ADDR_RIGHT   10      // FIX: corrido 4 bytes desde MID

// ── Puntos de calibración (RAW ADC, 0-4095) ────────────────────
int rawLeft  = 1270;   // posición IZQUIERDA  (~1198-1201) new 1270   anterior 1242
int rawMid   = 2022;   // posición MEDIA      (~2004-2008) new 2022   anterior 2005
int rawRight = 2630;   // posición DERECHA    (~2738-2743) new 2630   anterior 2865

// ── Control ────────────────────────────────────────────────────
const int TOLERANCIA_PCT = 2;   // % de error aceptable para "llegó"
int  rawObjetivo = -1;          // -1 = sin objetivo (motor libre)
bool modoStop    = false;

// ── Filtro promedio móvil ──────────────────────────────────────
const int N = 6;
int  buf[N];
long suma  = 0;
int  idx   = 0;
bool lleno = false;

// ── EEPROM ─────────────────────────────────────────────────────
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

void eepromSave() {
  EEPROM.put(ADDR_MAGIC, (uint16_t)MAGIC_VAL);
  EEPROM.put(ADDR_LEFT,  rawLeft);
  EEPROM.put(ADDR_MID,   rawMid);
  EEPROM.put(ADDR_RIGHT, rawRight);
  EEPROM.commit();
}

// ── Lectura filtrada ───────────────────────────────────────────
int leerPot() {
  int v = analogRead(PIN_POT);
  suma -= buf[idx];
  buf[idx] = v;
  suma += v;
  idx++;
  if (idx >= N) { idx = 0; lleno = true; }
  return lleno ? (suma / N) : (suma / idx);
}

// Convierte RAW → % dentro del rango [rawLeft, rawRight]
int rawToPct(int raw) {
  int lo = min(rawLeft, rawRight);
  int hi = max(rawLeft, rawRight);
  raw = constrain(raw, lo, hi);
  return (int)map(raw, rawLeft, rawRight, 0, 100);
}

// Tolerancia en unidades RAW
int tolRaw() {
  return max(abs(rawRight - rawLeft) * TOLERANCIA_PCT / 100, 8);
}

// ── Motor ──────────────────────────────────────────────────────
void motorStop()        { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, 0); }
void motorFwd(byte pwm) { analogWrite(PIN_PWM_B, 0); analogWrite(PIN_PWM_A, pwm); }
void motorRev(byte pwm) { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, pwm); }

void mover() {
  if (modoStop || rawObjetivo < 0) { motorStop(); return; }

  int raw   = leerPot();
  int error = rawObjetivo - raw;

  if (abs(error) <= tolRaw()) { motorStop(); return; }

  int   rango = abs(rawRight - rawLeft);
  byte  pwm   = (byte)constrain(map(abs(error), 0, rango, 135, 195), 0, 255);

  if (error > 0) motorFwd(pwm);
  else           motorRev(pwm);
}

// ── Helpers de impresión ───────────────────────────────────────
void printCalib() {
  Serial.print(F("  IZQ(L)=")); Serial.print(rawLeft);
  Serial.print(F("  MED(M)=")); Serial.print(rawMid);
  Serial.print(F("  DER(R)=")); Serial.println(rawRight);
}

// Etiqueta de posición respecto a los tres puntos
const char* labelPos(int raw) {
  int tol = tolRaw();
  if (abs(raw - rawLeft)  <= tol) return "IZQ";
  if (abs(raw - rawMid)   <= tol) return "MED";
  if (abs(raw - rawRight) <= tol) return "DER";
  // entre puntos
  if (rawLeft < rawRight) {
    if (raw < rawMid) return "<MED";
    else              return "MED>";
  } else {
    if (raw > rawMid) return "<MED";
    else              return "MED>";
  }
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  analogWriteFreq(10000);
  analogWriteRange(255);
  analogReadResolution(12);

  motorStop();
  digitalWrite(PIN_EN, HIGH);

  for (int i = 0; i < N; i++) { buf[i] = analogRead(PIN_POT); suma += buf[i]; delay(2); }
  lleno = true;

  eepromLoad();

  rawObjetivo = leerPot();   // sostener posición actual al arrancar

  Serial.println(F("=== CALIBRACION DIRECCION ==="));
  Serial.println(F("Comandos:"));
  Serial.println(F("  0-100     -> mover a ese %"));
  Serial.println(F("  L/M/R     -> ir a IZQ / MED / DER guardado"));
  Serial.println(F("  SL/SM/SR  -> guardar pos actual como IZQ/MED/DER"));
  Serial.println(F("  s         -> stop"));
  Serial.println(F("  ?         -> mostrar calibracion"));
  Serial.println(F("Calibracion actual:"));
  printCalib();
  Serial.println();
}

// ── Loop ───────────────────────────────────────────────────────
static unsigned long lastPrint = 0;
static String        inputBuf  = "";

void loop() {
  // Lectura de comandos serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuf.trim();
      if (inputBuf.length() > 0) {
        String cmd = inputBuf;
        cmd.toUpperCase();
        inputBuf = "";

        if (cmd == "S") {
          modoStop = true; rawObjetivo = -1;
          Serial.println(F(">> STOP"));

        } else if (cmd == "?") {
          printCalib();

        } else if (cmd == "L") {
          rawObjetivo = rawLeft; modoStop = false;
          Serial.print(F(">> -> IZQUIERDA (RAW=")); Serial.print(rawLeft); Serial.println(')');

        } else if (cmd == "M") {
          rawObjetivo = rawMid; modoStop = false;
          Serial.print(F(">> -> MEDIO     (RAW=")); Serial.print(rawMid); Serial.println(')');

        } else if (cmd == "R") {
          rawObjetivo = rawRight; modoStop = false;
          Serial.print(F(">> -> DERECHA   (RAW=")); Serial.print(rawRight); Serial.println(')');

        } else if (cmd == "SL") {
          rawLeft = leerPot(); eepromSave();
          Serial.print(F(">> [GUARDADO] IZQ = RAW ")); Serial.println(rawLeft);

        } else if (cmd == "SM") {
          rawMid = leerPot(); eepromSave();
          Serial.print(F(">> [GUARDADO] MED = RAW ")); Serial.println(rawMid);

        } else if (cmd == "SR") {
          rawRight = leerPot(); eepromSave();
          Serial.print(F(">> [GUARDADO] DER = RAW ")); Serial.println(rawRight);

        } else {
          int val = cmd.toInt();
          if (val >= 0 && val <= 100) {
            rawObjetivo = (int)map(val, 0, 100, rawLeft, rawRight);
            modoStop = false;
            Serial.print(F(">> Objetivo: ")); Serial.print(val);
            Serial.print(F("% (RAW=")); Serial.print(rawObjetivo); Serial.println(')');
          } else {
            Serial.println(F(">> Invalido. Usa: 0-100 | L M R | SL SM SR | s | ?"));
          }
        }
      } else {
        inputBuf = "";
      }
    } else {
      inputBuf += c;
    }
  }

  mover();

  // Telemetría continua
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    int raw = leerPot();
    int pct = rawToPct(raw);

    Serial.print(F("RAW="));  Serial.print(raw);
    Serial.print(F(" | L=")); Serial.print(rawLeft);
    Serial.print(F(" M="));   Serial.print(rawMid);
    Serial.print(F(" R="));   Serial.print(rawRight);
    Serial.print(F(" | POS=")); Serial.print(pct); Serial.print('%');
    Serial.print(F(" | "));  Serial.println(labelPos(raw));
  }
}
