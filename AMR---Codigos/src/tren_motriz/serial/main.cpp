/*
 * ================================================================
 *   TREN MOTRIZ — Serial (pruebas / diagnóstico)
 *   Feather RP2040
 * ================================================================
 *
 * PINES:
 *   MOTOR_PWM    → A1   (PWM velocidad)
 *   RELAY_ENABLE → 4    (relé de potencia)
 *   MOTOR_DIR_REV→ 11   (HIGH = FWD, LOW = REV)
 *   CONFIG_L1    → 12   (HIGH = transistor ON → 5V a L1)
 *   CONFIG_L2    → 13   (LOW  = transistor OFF → 0V en L2)
 *
 * ENGRANAJE:
 *   HIGH GEAR (marcha alta): L1=HIGH, L2=LOW  ← default
 *   LOW  GEAR (marcha baja): L1=LOW,  L2=HIGH
 *
 * COMANDOS SERIAL (115200):
 *   f        → dirección ADELANTE  (para motor, cambia, queda en 0)
 *   b        → dirección REVERSA   (para motor, cambia, queda en 0)
 *   0-100    → velocidad en %      (relé debe estar ON)
 *   s        → stop  (velocidad = 0, relé permanece)
 *   e        → emergencia (velocidad = 0, relé OFF)
 *   ron      → relé ON
 *   roff     → relé OFF (para motor primero)
 *   hi       → marcha ALTA  (L1=HIGH, L2=LOW)
 *   lo       → marcha BAJA  (L1=LOW,  L2=HIGH)
 *   ?        → imprimir estado
 *
 * SALIDA CONTINUA (200 ms):
 *   VEL=50% | DIR=FWD | RELAY=ON | GEAR=HI | PWM=127
 * ================================================================
 */

#include <Arduino.h>

#define MOTOR_PWM      A1
#define RELAY_ENABLE   4
#define MOTOR_DIR_REV  11
#define CONFIG_L1      12
#define CONFIG_L2      13

// ── Estado ─────────────────────────────────────────────────────
int  velocidad = 0;      // 0-100 %
bool dirFwd    = true;
bool relayOn   = false;
bool highGear  = true;

// ── Motor (llamado cada iteración del loop) ────────────────────
void applyMotor() {
  if (!relayOn || velocidad == 0) {
    analogWrite(MOTOR_PWM, 0);
  } else {
    analogWrite(MOTOR_PWM, map(velocidad, 0, 100, 0, 255));
  }
}

void applyGear() {
  digitalWrite(CONFIG_L1, highGear ? HIGH : LOW);
  digitalWrite(CONFIG_L2, highGear ? LOW  : HIGH);
}

// ── Cambio de dirección — siempre para primero ─────────────────
void setDir(bool fwd) {
  if (dirFwd == fwd) return;
  velocidad = 0;
  analogWrite(MOTOR_PWM, 0);
  delay(150);
  dirFwd = fwd;
  digitalWrite(MOTOR_DIR_REV, dirFwd ? HIGH : LOW);
  // velocidad queda en 0 — enviar la nueva velocidad explícitamente
}

void emergencyStop() {
  velocidad = 0;
  analogWrite(MOTOR_PWM, 0);
  relayOn = false;
  digitalWrite(RELAY_ENABLE, LOW);
  Serial.println(F(">> EMERGENCIA"));
}

void printStatus() {
  Serial.print(F("VEL="));     Serial.print(velocidad); Serial.print('%');
  Serial.print(F(" | DIR="));  Serial.print(dirFwd  ? F("FWD") : F("REV"));
  Serial.print(F(" | RELAY=")); Serial.print(relayOn ? F("ON")  : F("OFF"));
  Serial.print(F(" | GEAR="));  Serial.print(highGear ? F("HI") : F("LO"));
  Serial.print(F(" | PWM="));
  Serial.println(relayOn && velocidad > 0 ? map(velocidad, 0, 100, 0, 255) : 0);
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  pinMode(MOTOR_PWM,     OUTPUT);
  pinMode(RELAY_ENABLE,  OUTPUT);
  pinMode(MOTOR_DIR_REV, OUTPUT);
  pinMode(CONFIG_L1,     OUTPUT);
  pinMode(CONFIG_L2,     OUTPUT);

  analogWriteFreq(10000);
  analogWriteRange(255);

  digitalWrite(MOTOR_DIR_REV, HIGH);  // HIGH = FWD, alineado con dirFwd=true
  digitalWrite(RELAY_ENABLE,  LOW);
  analogWrite(MOTOR_PWM, 0);
  applyGear();

  Serial.println(F("=== TREN MOTRIZ SERIAL ==="));
  Serial.println(F("  f/b      -> adelante/reversa (para y queda en 0)"));
  Serial.println(F("  0-100    -> velocidad %"));
  Serial.println(F("  s        -> stop"));
  Serial.println(F("  e        -> emergencia"));
  Serial.println(F("  ron/roff -> rele ON/OFF"));
  Serial.println(F("  hi/lo    -> marcha alta/baja"));
  Serial.println(F("  ?        -> estado"));
  Serial.println();
}

// ── Loop ───────────────────────────────────────────────────────
static unsigned long lastPrint = 0;
static String        inputBuf  = "";

void loop() {
  // Aplicar estado del motor en cada iteración
  applyMotor();

  // Lectura de comandos
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuf.trim();
      if (inputBuf.length() > 0) {
        String cmd = inputBuf;
        inputBuf = "";
        cmd.toLowerCase();

        if (cmd == "f") {
          setDir(true);
          Serial.println(F(">> ADELANTE — enviar velocidad"));

        } else if (cmd == "b") {
          setDir(false);
          Serial.println(F(">> REVERSA  — enviar velocidad"));

        } else if (cmd == "s") {
          velocidad = 0;
          Serial.println(F(">> STOP"));

        } else if (cmd == "e") {
          emergencyStop();

        } else if (cmd == "ron") {
          relayOn = true;
          digitalWrite(RELAY_ENABLE, HIGH);
          Serial.println(F(">> RELE ON"));

        } else if (cmd == "roff") {
          velocidad = 0;
          relayOn = false;
          digitalWrite(RELAY_ENABLE, LOW);
          Serial.println(F(">> RELE OFF"));

        } else if (cmd == "hi") {
          highGear = true;
          applyGear();
          Serial.println(F(">> MARCHA ALTA"));

        } else if (cmd == "lo") {
          highGear = false;
          applyGear();
          Serial.println(F(">> MARCHA BAJA"));

        } else if (cmd == "?") {
          printStatus();

        } else {
          int val = cmd.toInt();
          if (val > 0 || cmd == "0") {
            val = constrain(val, 0, 100);
            if (!relayOn)
              Serial.println(F(">> AVISO: rele OFF — usa 'ron' primero"));
            velocidad = val;
            Serial.print(F(">> VEL = ")); Serial.print(velocidad); Serial.println('%');
          } else {
            Serial.println(F(">> Invalido. Usa: f b 0-100 s e ron roff hi lo ?"));
          }
        }
      } else {
        inputBuf = "";
      }
    } else {
      inputBuf += c;
    }
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    printStatus();
  }
}
