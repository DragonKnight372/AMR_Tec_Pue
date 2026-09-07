/*
 * ================================================================
 *   SANDBOX TEST — INTÉRPRETE  v2.9
 *   Copia de interprete_can v2.9. Experimentar AQUÍ.
 *   Si falla, flashear interprete_can.
 * ================================================================
 * AMR1 - CONTROL MASTER DASHBOARD PRO v2.9 (TEST)
 * Hardware: Adafruit Feather RP2040 CAN
 * ================================================================
 *
 * Historial de versiones:
 * v2.3 — Freno proporcional CH1. Zona muerta throttle 911-1100.
 *         Throttle/freno excluyentes.
 * v2.4 — Interlock freno-motor (umbral 7%). Tag LOCK en OLED.
 *         Relé activo en freno para liberación rápida.
 * v2.5 — Mutex Core0/Core1: protege vehicle.* contra race
 *         condition. parseSBUS() escribe con mutex. TX y OLED
 *         leen con mutex.
 * v2.6 — CH2 invertido: map(172,1811,0,100) corrige dirección
 *         física del vehículo.
 *         Emergencia separada en dos flags:
 *           hayEmergMotriz = signalLost || !enable
 *             → bloquea tren motriz + frenos (0x210)
 *           hayEmergDir = signalLost SOLO
 *             → bloquea dirección (0x211, ID nuevo)
 *         Cuando CH5=-100: freno=100, motor=0, pero dirección
 *         sigue operativa para maniobrar en frenado.
 * v2.7 — Freno remapeado al rango físico efectivo (0→BRAKE_MAX_EFFECTIVE).
 *         El actuador no frena por encima de 60%; mapear hasta 100
 *         desperdiciaba recorrido de palanca.
 *         Zona de seguridad: valores ≤ UMBRAL_FRENO_BLOQUEO (7%) se
 *         redondean a 0 para evitar bloquear el motor por ruido/holgura.
 * v2.8 — CH3 selector SERIAL/RC (sustituido en v2.9).
 * v2.9 — CH3 recupera deadman. CH4 elige modalidad (-100/0/+100):
 *           -100 SERIAL · 0 RC · +100 AUTO (placeholder, vel 0).
 *         USB: MODE SERIAL | MODE RC | MODE AUTO
 *         Deadman CH3 + CH5 en los tres modos.
 *
 * MAPA CAN:
 *   TX 0x100 → Comando dirección
 *   TX 0x200 → Comando freno proporcional (0-100%)
 *   TX 0x210 → Emergencia motriz (tren + frenos)
 *   TX 0x211 → Emergencia dirección (solo failsafe real)
 *   TX 0x300 → Velocidad
 *   TX 0x310 → Enable relay
 *   TX 0x320 → Reversa
 *   TX 0x330 → High gear
 *   RX 0x101 → Feedback dirección
 *
 * USB SERIAL COMMANDS (115200, Newline). SERIAL mode: CH4=-100 or MODE SERIAL.
 *
 *   MODE SERIAL     USB control
 *   MODE RC         Radio (sticks → CAN)
 *   MODE AUTO       Autonomous placeholder (speed 0)
 *   MODE RADIO      Follow CH4 again
 *   HELP            List IDs
 *
 *   CH4: -100 Serial · 0 RC · +100 Auto
 *   CH3 = deadman    CH5 = enable
 *
 *   Steering 100   0 left · 50 center · 100 right
 *     100 0         Full left
 *     100 50        Center / straight
 *     100 100       Full right
 *     100 25        Slightly left
 *     100 75        Slightly right
 *
 *   Brake 200      0 released · 100 full  (brake > 7% forces speed 0)
 *     200 0         Release brake
 *     200 40        Medium brake
 *     200 100       Full brake
 *
 *   Emergency
 *     210 1         Drivetrain emergency: full brake, motor off
 *     210 0         Clear drivetrain emergency
 *     211 1         Lock steering
 *     211 0         Unlock steering
 *
 *   Speed 300      0 stop · 100 max
 *     300 0         Stop motor
 *     300 15        Slow
 *     300 25        Medium
 *     300 40        Faster
 *     300 100       Maximum
 *
 *   Enable / relay 310
 *     310 1         Powertrain relay on
 *     310 0         Powertrain relay off
 *
 *   Reverse 320     stop motor before changing
 *     320 0         Forward
 *     320 1         Reverse
 *
 *   Gear 330
 *     330 0         Low gear (F-LO)
 *     330 1         High gear (F-HI)
 *
 *   Underscore also works: 100_50 = 100 50
 *   In AUTO, 100 / 300 / ... do not move the vehicle.
 * ================================================================
 */

#include "hardware/gpio.h"
#include "pico/mutex.h"
#include <Adafruit_GFX.h>
#include <Adafruit_MCP2515.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdlib.h>

// ── SBUS ──────────────────────────────────────────────────────
#define SBUS_PIN_RX  1
#define BAUD_SBUS    100000

// ── CAN ───────────────────────────────────────────────────────
#define CAN_BAUDRATE 250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

// ── OLED ──────────────────────────────────────────────────────
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// ── Canales SBUS ──────────────────────────────────────────────
#define CH_THROTTLE  0
#define CH_STEERING  1
#define CH_EMERG     2   // CH3: deadman
#define CH_SELECT    3   // CH4: -100 SERIAL, 0 RC, +100 AUTO
#define CH_ENABLE    4   // CH5: enable LIVE
#define CH_GEAR_REV  6

#define MODO_RC      0
#define MODO_SERIAL  1
#define MODO_AUTO    2

#define CH4_SERIAL_MAX  -50   // ≤ -50  → SERIAL
#define CH4_RC_MIN      -40
#define CH4_RC_MAX       40
#define CH4_AUTO_MIN     50   // ≥ +50  → AUTO

#define SBUS_LOW   500
#define SBUS_HIGH  1500
#define CH7_HIGH   1300
#define CH7_LOW    700

#define THROTTLE_MIN          172
#define THROTTLE_FRENA_TOP    910
#define THROTTLE_DEAD_BOT     911
#define THROTTLE_DEAD_TOP     1100
#define THROTTLE_ACEL_BOT     1101
#define THROTTLE_MAX          1811

#define VEL_MAX_LOW   120
#define VEL_MAX_HIGH  200

#define UMBRAL_FRENO_BLOQUEO  7
#define BRAKE_MIN_EFFECTIVE   40   // actuador físico: empieza a frenar desde 40%

// ── CAN IDs ───────────────────────────────────────────────────
#define CAN_TX_DIR        0x100
#define CAN_TX_BRAKE      0x200
#define CAN_TX_EMERG      0x210   // emergencia motriz (tren + frenos)
#define CAN_TX_EMERG_DIR  0x211   // emergencia dirección (solo failsafe)
#define CAN_TX_VEL        0x300
#define CAN_TX_ENABLE     0x310
#define CAN_TX_REVERSA    0x320
#define CAN_TX_HIGHGEAR   0x330
#define CAN_RX_DIR_FB     0x101

// ── Estado global + mutex ─────────────────────────────────────
struct SBUS_State {
  uint16_t ch[16];
  bool failsafe;
  unsigned long last_update;
  int valorVel;
  int valorDir;
  int valorFreno;
  int valorEnable;
  int valorReversa;
  bool highGear;
  bool motorBloqueado;
};

volatile SBUS_State vehicle;
mutex_t vehicleMutex;

byte           dirFeedback       = 50;
unsigned long  lastDirFeedback   = 0;
unsigned long  cntFeedbackRx     = 0;
bool           dirFeedbackOnline = false;

// ── Modalidad: RC / SERIAL / AUTO ─────────────────────────────
uint8_t modoCtrl = MODO_RC;
bool    modoUSB  = false;     // MODE * ignora CH4 hasta MODE RADIO
bool modoSerial    = false;   // alias: modoCtrl == MODO_SERIAL (compat OLED/TX)
int  serialDir     = 50;
int  serialVel     = 0;       // 0-100 (como va por CAN)
int  serialFreno   = 0;
int  serialEnable  = 0;
int  serialReversa = 0;
bool serialHighGear = false;
bool serialEmergMotriz = false;
bool serialEmergDir    = false;
String serialLine;

// ================================================================
// SBUS DECODING (CORE 1)
// ================================================================
void parseSBUS(uint8_t *packet) {

  uint16_t ch[16];
  ch[0]  = ((packet[1]    | packet[2]  << 8)                    & 0x07FF);
  ch[1]  = ((packet[2]>>3 | packet[3]  << 5)                    & 0x07FF);
  ch[2]  = ((packet[3]>>6 | packet[4]  << 2 | packet[5]  << 10) & 0x07FF);
  ch[3]  = ((packet[5]>>1 | packet[6]  << 7)                    & 0x07FF);
  ch[4]  = ((packet[6]>>4 | packet[7]  << 4)                    & 0x07FF);
  ch[5]  = ((packet[7]>>7 | packet[8]  << 1 | packet[9]  << 9)  & 0x07FF);
  ch[6]  = ((packet[9]>>2 | packet[10] << 6)                    & 0x07FF);
  ch[7]  = ((packet[10]>>5| packet[11] << 3)                    & 0x07FF);
  ch[8]  = ((packet[12]   | packet[13] << 8)                    & 0x07FF);
  ch[9]  = ((packet[13]>>3| packet[14] << 5)                    & 0x07FF);
  ch[10] = ((packet[14]>>6| packet[15] << 2 | packet[16] << 10) & 0x07FF);
  ch[11] = ((packet[16]>>1| packet[17] << 7)                    & 0x07FF);
  ch[12] = ((packet[17]>>4| packet[18] << 4)                    & 0x07FF);
  ch[13] = ((packet[18]>>7| packet[19] << 1 | packet[20] << 9)  & 0x07FF);
  ch[14] = ((packet[20]>>2| packet[21] << 6)                    & 0x07FF);
  ch[15] = ((packet[21]>>5| packet[22] << 3)                    & 0x07FF);

  bool failsafe_local = packet[23] & 0x08;

  // FIX v2.6: CH2 invertido — map 0→100 en lugar de 100→0
  int valorDir_local = map(ch[CH_STEERING], 172, 1811, 0, 100);

  int  valorReversa_local = 0;
  bool highGear_local     = false;
  uint16_t v7 = ch[CH_GEAR_REV];
  if (v7 > CH7_HIGH) {
    valorReversa_local = 1;
    highGear_local     = false;
  } else if (v7 < CH7_LOW) {
    valorReversa_local = 0;
    highGear_local     = true;
  }

  int valorVel_local   = 0;
  int valorFreno_local = 0;
  uint16_t throttle = ch[CH_THROTTLE];

  if (throttle >= THROTTLE_ACEL_BOT) {
    int volMax = (highGear_local && !valorReversa_local)
                   ? VEL_MAX_HIGH : VEL_MAX_LOW;
    valorVel_local   = map(throttle, THROTTLE_ACEL_BOT, THROTTLE_MAX, 50, volMax);
    valorFreno_local = 0;
  } else if (throttle <= THROTTLE_FRENA_TOP) {
    valorVel_local = 0;
    // Mapear palanca al rango 0-100 primero
    int rawBrake = map(throttle, THROTTLE_MIN, THROTTLE_FRENA_TOP, 100, 0);
    // Deadzone: por debajo del umbral de seguridad no frenar
    // Por encima: escalar al rango físico efectivo (BRAKE_MIN_EFFECTIVE→100)
    if (rawBrake <= UMBRAL_FRENO_BLOQUEO) {
      valorFreno_local = 0;
    } else {
      valorFreno_local = map(rawBrake, UMBRAL_FRENO_BLOQUEO, 100, BRAKE_MIN_EFFECTIVE, 100);
    }
  }

  // Deadman: CH3 emergencia o CH5 enable abajo
  bool emerg_ch3  = (ch[CH_EMERG]  > SBUS_HIGH);
  bool enable_ch5 = (ch[CH_ENABLE] > SBUS_HIGH);

  int valorEnable_local = 1;
  if (emerg_ch3 || !enable_ch5) {
    valorFreno_local   = 100;
    valorVel_local     = 0;
    valorReversa_local = 0;
    valorEnable_local  = 0;
  }

  // Interlock freno-motor
  bool motorBloqueado_local = false;
  if (valorFreno_local > UMBRAL_FRENO_BLOQUEO) {
    valorVel_local       = 0;
    motorBloqueado_local = true;
  }

  // Escritura atómica
  mutex_enter_blocking(&vehicleMutex);
  for (int i = 0; i < 16; i++) vehicle.ch[i] = ch[i];
  vehicle.failsafe       = failsafe_local;
  vehicle.valorDir       = valorDir_local;
  vehicle.valorVel       = valorVel_local;
  vehicle.valorFreno     = valorFreno_local;
  vehicle.valorEnable    = valorEnable_local;
  vehicle.valorReversa   = valorReversa_local;
  vehicle.highGear       = highGear_local;
  vehicle.motorBloqueado = motorBloqueado_local;
  if (!failsafe_local) vehicle.last_update = millis();
  mutex_exit(&vehicleMutex);
}

void setup1() {
  Serial1.begin(BAUD_SBUS, SERIAL_8E2);
  gpio_set_inover(SBUS_PIN_RX, 1);
}

void loop1() {
  static uint8_t buf[25];
  static int idx = 0;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (idx == 0 && c != 0x0F) continue;
    buf[idx++] = c;
    if (idx == 25) {
      if (buf[24] == 0x00) parseSBUS(buf);
      idx = 0;
    }
  }
}

// ================================================================
// CAN RX
// ================================================================
void leerCANEntrante() {
  if (!canOK) return;
  while (true) {
    int packetSize = mcp.parsePacket();
    if (packetSize <= 0) break;
    long id = mcp.packetId();
    if (id == CAN_RX_DIR_FB && mcp.available()) {
      byte fb = mcp.read();
      dirFeedback     = constrain(fb, 0, 100);
      lastDirFeedback = millis();
      cntFeedbackRx++;
    } else {
      while (mcp.available()) mcp.read();
    }
  }
  dirFeedbackOnline = (millis() - lastDirFeedback) < 1500;
}

// ================================================================
// CAN TX
// ================================================================
void sendCANTelemetry() {
  if (!canOK) return;

  mutex_enter_blocking(&vehicleMutex);
  int   dir          = vehicle.valorDir;
  int   vel          = vehicle.valorVel;
  int   freno        = vehicle.valorFreno;
  int   enable       = vehicle.valorEnable;
  int   reversa      = vehicle.valorReversa;
  bool  hg           = vehicle.highGear;
  bool  fs           = vehicle.failsafe;
  unsigned long upd  = vehicle.last_update;
  mutex_exit(&vehicleMutex);

  bool radioVivo  = !fs && (upd != 0) && (millis() - upd <= 500);
  bool signalLost = !radioVivo;
  int  enableCh5  = enable;

  byte velCAN = (byte)map(vel, 0, VEL_MAX_HIGH, 0, 100);
  bool hayEmergMotriz = signalLost || !enable;
  bool hayEmergDir    = signalLost;

  if (modoCtrl == MODO_SERIAL) {
    dir     = serialDir;
    velCAN  = (byte)constrain(serialVel, 0, 100);
    freno   = serialFreno;
    reversa = serialReversa;
    hg      = serialHighGear;
    enable  = serialEnable;
    if (radioVivo && !enableCh5) enable = 0;
    hayEmergMotriz = serialEmergMotriz || !enable;
    hayEmergDir    = serialEmergDir;
    if (freno > UMBRAL_FRENO_BLOQUEO) velCAN = 0;
  } else if (modoCtrl == MODO_AUTO) {
    dir     = 50;
    velCAN  = 0;
    freno   = 0;
    reversa = 0;
    hg      = false;
    enable  = 0;
    hayEmergMotriz = signalLost || !enableCh5;
    hayEmergDir    = signalLost;
  }

  mcp.beginPacket(CAN_TX_DIR);
  mcp.write((byte)constrain(dir, 0, 100));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_EMERG);
  mcp.write((byte)(hayEmergMotriz ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_EMERG_DIR);
  mcp.write((byte)(hayEmergDir ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_BRAKE);
  mcp.write((byte)(hayEmergMotriz ? 100 : constrain(freno, 0, 100)));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_VEL);
  mcp.write(hayEmergMotriz ? 0 : velCAN);
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_ENABLE);
  mcp.write((byte)((enable && !hayEmergMotriz) ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_REVERSA);
  mcp.write((byte)(reversa ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_HIGHGEAR);
  mcp.write((byte)(hg ? 1 : 0));
  mcp.endPacket();
}

// ================================================================
// OLED
// ================================================================
void drawBar(int x, int y, int w, int h, int val, int valMax) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = ((w - 2) * val) / valMax;
  fill = constrain(fill, 0, w - 2);
  if (fill > 0)
    display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

void drawDirBar(int x, int y, int w, int h, int cmdDir, int fbDir) {
  const int MID = x + w / 2;
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.drawLine(MID, y, MID, y + h - 1, SSD1306_WHITE);

  int q1 = x + w / 4;
  int q3 = x + (w * 3) / 4;
  display.drawPixel(q1, y, SSD1306_WHITE);
  display.drawPixel(q1, y + h - 1, SSD1306_WHITE);
  display.drawPixel(q3, y, SSD1306_WHITE);
  display.drawPixel(q3, y + h - 1, SSD1306_WHITE);

  int cmdX = x + 1 + ((w - 2) * cmdDir) / 100;
  cmdX = constrain(cmdX, x + 1, x + w - 2);
  display.fillTriangle(cmdX-3, y-5, cmdX+3, y-5, cmdX, y-1, SSD1306_WHITE);

  if (dirFeedbackOnline) {
    int fbX = x + 1 + ((w - 2) * fbDir) / 100;
    fbX = constrain(fbX, x + 1, x + w - 2);
    if (fbX >= MID) display.fillRect(MID, y+2, fbX-MID, h-4, SSD1306_WHITE);
    else            display.fillRect(fbX, y+2, MID-fbX, h-4, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x + w/2 - 2, y + 2);
    display.print('?');
  }
}

void drawTag(int x, int y, const char *txt, bool active) {
  int w = 6 * strlen(txt) + 2;
  if (active) {
    display.fillRect(x, y-1, w, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(x, y-1, w, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(x+1, y+1);
  display.print(txt);
  display.setTextColor(SSD1306_WHITE);
}

void updateOLED() {
  display.clearDisplay();

  mutex_enter_blocking(&vehicleMutex);
  int   cmdDir     = vehicle.valorDir;
  int   cmdVel     = vehicle.valorVel;
  int   cmdFreno   = vehicle.valorFreno;
  int   cmdEnable  = vehicle.valorEnable;
  int   cmdReversa = vehicle.valorReversa;
  bool  hg         = vehicle.highGear;
  bool  bloqueado  = vehicle.motorBloqueado;
  bool  fs         = vehicle.failsafe;
  unsigned long upd = vehicle.last_update;
  mutex_exit(&vehicleMutex);

  bool signalLost = fs || (millis() - upd > 500);

  if (modoCtrl == MODO_SERIAL) {
    cmdDir     = serialDir;
    cmdVel     = map(serialVel, 0, 100, 0, VEL_MAX_HIGH);
    cmdFreno   = serialFreno;
    cmdEnable  = serialEnable;
    cmdReversa = serialReversa;
    hg         = serialHighGear;
    bloqueado  = serialFreno > UMBRAL_FRENO_BLOQUEO;
  } else if (modoCtrl == MODO_AUTO) {
    cmdDir     = 50;
    cmdVel     = 0;
    cmdFreno   = 0;
    cmdEnable  = 0;
    cmdReversa = 0;
    hg         = false;
    bloqueado  = false;
  }

  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1"));

  display.setCursor(40, 2);
  if (modoCtrl == MODO_SERIAL)      display.print(F(" SERIAL  "));
  else if (modoCtrl == MODO_AUTO)   display.print(F("  AUTO   "));
  else if (signalLost)              display.print(F("NO SIGNAL"));
  else if (!cmdEnable)              display.print(F(" FRENO   "));
  else                              display.print(F("  LIVE   "));

  display.setCursor(106, 2);
  if (!canOK)                 display.print(F("CAN-X"));
  else if (dirFeedbackOnline) display.print(F("LINK*"));
  else                        display.print(F("LINK?"));

  display.setTextColor(SSD1306_WHITE);

  if (signalLost && modoCtrl == MODO_RC) {
    display.setTextSize(2);
    display.setCursor(14, 20); display.print(F("TX LOST!"));
    display.setTextSize(1);
    display.setCursor(8, 44);  display.print(F("Sistema en HALT"));
    display.setCursor(8, 54);  display.print(F("Espera senal..."));
    display.display();
    return;
  }

  // VEL
  display.setCursor(0, 14); display.print(F("VEL"));
  drawBar(20, 14, 78, 8, cmdVel, VEL_MAX_HIGH);
  display.setCursor(102, 14);
  int velPct = (cmdVel * 100) / VEL_MAX_HIGH;
  if (velPct < 10) display.print(F("  "));
  else if (velPct < 100) display.print(F(" "));
  display.print(velPct); display.print('%');

  // FRENO
  display.setCursor(0, 24); display.print(F("BRK"));
  drawBar(20, 24, 78, 6, cmdFreno, 100);
  display.setCursor(102, 24);
  if (cmdFreno < 10) display.print(F("  "));
  else if (cmdFreno < 100) display.print(F(" "));
  display.print(cmdFreno); display.print('%');

  // DIR
  display.setCursor(0, 34); display.print(F("DIR"));
  drawDirBar(20, 33, 78, 8, cmdDir, dirFeedback);
  display.setCursor(102, 34);
  if (cmdDir < 10) display.print(F("  "));
  else if (cmdDir < 100) display.print(F(" "));
  display.print(cmdDir);

  // FB
  display.setCursor(0, 44);
  if (dirFeedbackOnline) {
    int err = (int)cmdDir - (int)dirFeedback;
    display.print(F("FB:")); display.print(dirFeedback); display.print('%');
    display.setCursor(48, 44);
    display.print(F("ERR:"));
    if (err > 0) display.print('+');
    display.print(err);
    display.setCursor(102, 44);
    if (abs(err) <= 3)       display.print(F("OK "));
    else if (abs(err) <= 10) display.print(F("..."));
    else                     display.print(F("!!!"));
  } else {
    display.print(F("FB: sin link"));
  }

  display.drawLine(0, 52, 127, 52, SSD1306_WHITE);

  drawTag(0, 54, " EN ", cmdEnable);

  const char *gearTxt;
  bool gearHL;
  if (cmdReversa)       { gearTxt = " REV "; gearHL = true;  }
  else if (hg)          { gearTxt = "F-HI "; gearHL = true;  }
  else                  { gearTxt = "F-LO "; gearHL = false; }
  drawTag(32, 54, gearTxt, gearHL);

  bool brk = cmdFreno > 0;
  const char *brkTxt;
  if (bloqueado) brkTxt = "LOCK";
  else if (brk)  brkTxt = "BRK";
  else           brkTxt = "---";
  drawTag(70, 54, brkTxt, brk);

  display.setCursor(98, 56);
  display.print(F("R"));
  if (cntFeedbackRx % 1000 < 100) display.print('0');
  if (cntFeedbackRx % 1000 < 10)  display.print('0');
  display.print(cntFeedbackRx % 1000);

  display.display();
}

void splashOLED() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawRect(3, 3, 122, 58, SSD1306_WHITE);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 12); display.print(F("AMR1 CTRL"));
  display.setTextSize(1);
  display.setCursor(16, 34); display.print(F("TEST v2.9  3 modos"));
  display.setCursor(8, 48);  display.print(F("CH4: -100/0/+100"));
  display.display();
}

void aplicarInterlockSerial() {
  if (serialFreno > UMBRAL_FRENO_BLOQUEO) serialVel = 0;
}

void sincronizarAliasModo() {
  modoSerial = (modoCtrl == MODO_SERIAL);
}

void entrarModoSerial() {
  mutex_enter_blocking(&vehicleMutex);
  serialDir      = vehicle.valorDir;
  serialReversa  = vehicle.valorReversa;
  serialHighGear = vehicle.highGear;
  serialEnable   = vehicle.valorEnable;
  mutex_exit(&vehicleMutex);
  serialVel = 0;
  serialFreno = 0;
  serialEmergMotriz = false;
  serialEmergDir = false;
  modoCtrl = MODO_SERIAL;
  sincronizarAliasModo();
}

void ponerModo(uint8_t m) {
  if (m == MODO_SERIAL) entrarModoSerial();
  else {
    modoCtrl = m;
    sincronizarAliasModo();
  }
}

void actualizarModoCH4() {
  if (modoUSB) return;
  mutex_enter_blocking(&vehicleMutex);
  uint16_t ch4 = vehicle.ch[CH_SELECT];
  unsigned long upd = vehicle.last_update;
  bool fs = vehicle.failsafe;
  mutex_exit(&vehicleMutex);
  if (fs || upd == 0 || (millis() - upd > 500) || ch4 == 0) return;

  int ch4pct = map((int)ch4, 172, 1811, -100, 100);
  uint8_t nuevo = modoCtrl;
  if (ch4pct <= CH4_SERIAL_MAX)      nuevo = MODO_SERIAL;
  else if (ch4pct >= CH4_AUTO_MIN)   nuevo = MODO_AUTO;
  else if (ch4pct >= CH4_RC_MIN && ch4pct <= CH4_RC_MAX) nuevo = MODO_RC;

  if (nuevo != modoCtrl) ponerModo(nuevo);
}

void procesarLineaSerial(String s) {
  s.trim();
  s.replace('_', ' ');
  s.toUpperCase();
  if (s.length() == 0) return;

  if (s == "MODE SERIAL" || s == "MODO SERIAL") {
    modoUSB = true;
    entrarModoSerial();
    if (serialEnable == 0) serialEnable = 1;
    Serial.println(F("OK MODE SERIAL"));
    return;
  }
  if (s == "MODE SBUS" || s == "MODE RC" || s == "MODO RC") {
    modoUSB = true;
    ponerModo(MODO_RC);
    Serial.println(F("OK MODE RC"));
    return;
  }
  if (s == "MODE AUTO" || s == "MODO AUTO") {
    modoUSB = true;
    ponerModo(MODO_AUTO);
    Serial.println(F("OK MODE AUTO"));
    return;
  }
  if (s == "MODE RADIO" || s == "MODO RADIO") {
    modoUSB = false;
    Serial.println(F("OK MODE RADIO (sigue CH4)"));
    return;
  }
  if (s == "HELP" || s == "H" || s == "?") {
    Serial.println(F("100 DIR  200 BRK  210 EMG  211 EMGDIR"));
    Serial.println(F("300 VEL  310 EN   320 REV  330 HG"));
    Serial.println(F("MODE SERIAL | MODE RC | MODE AUTO | MODE RADIO"));
    return;
  }

  int sp = s.indexOf(' ');
  if (sp < 0) {
    Serial.println(F("ERR formato: ID valor   (ej: 100 50)"));
    return;
  }
  unsigned long id = strtoul(s.substring(0, sp).c_str(), NULL, 16);
  int val = s.substring(sp + 1).toInt();

  if (!modoSerial) {
    Serial.println(F("ERR no SERIAL (CH4=-100 o MODE SERIAL)"));
    return;
  }

  switch (id) {
    case 0x100: serialDir = constrain(val, 0, 100); break;
    case 0x200: serialFreno = constrain(val, 0, 100); break;
    case 0x210: serialEmergMotriz = (val != 0); break;
    case 0x211: serialEmergDir = (val != 0); break;
    case 0x300: serialVel = constrain(val, 0, 100); break;
    case 0x310: serialEnable = (val != 0) ? 1 : 0; break;
    case 0x320: serialReversa = (val != 0) ? 1 : 0; break;
    case 0x330: serialHighGear = (val != 0); break;
    default:
      Serial.println(F("ERR ID desconocido"));
      return;
  }
  aplicarInterlockSerial();
  Serial.print(F("OK 0x"));
  Serial.print(id, HEX);
  Serial.print(' ');
  Serial.println(val);
}

void leerUSB() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        procesarLineaSerial(serialLine);
        serialLine = "";
      }
    } else if (serialLine.length() < 48) {
      serialLine += c;
    }
  }
}

// ================================================================
// DEBUG SERIAL
// ================================================================
void debugChannels() {
  mutex_enter_blocking(&vehicleMutex);
  int  ch0    = vehicle.ch[CH_THROTTLE];
  int  ch6    = vehicle.ch[CH_GEAR_REV];
  int  dir    = vehicle.valorDir;
  int  vel    = vehicle.valorVel;
  int  freno  = vehicle.valorFreno;
  int  enable = vehicle.valorEnable;
  int  rev    = vehicle.valorReversa;
  bool bloq   = vehicle.motorBloqueado;
  bool hg     = vehicle.highGear;
  mutex_exit(&vehicleMutex);

  Serial.print(F("CH1=")); Serial.print(ch0);
  Serial.print(F(" CH7=")); Serial.print(ch6);
  Serial.print(F(" | DIR=")); Serial.print(dir);
  Serial.print(F(" FB=")); Serial.print(dirFeedback);
  Serial.print(dirFeedbackOnline ? F("(ON)") : F("(--)"));
  Serial.print(F(" | VEL=")); Serial.print(vel);
  Serial.print(F(" BRK=")); Serial.print(freno); Serial.print('%');
  if (bloq) Serial.print(F(" [LOCK]"));
  Serial.print(F(" | "));
  if (rev)     Serial.print(F("REV "));
  else if (hg) Serial.print(F("F-HI"));
  else         Serial.print(F("F-LO"));
  Serial.print(F(" | EN=")); Serial.print(enable);
  Serial.print(F(" | "));
  if (modoCtrl == MODO_SERIAL) Serial.print(F("SERIAL"));
  else if (modoCtrl == MODO_AUTO) Serial.print(F("AUTO"));
  else Serial.print(F("RC"));
  Serial.print(F(" | RX=")); Serial.println(cntFeedbackRx);
}

// ================================================================
// SETUP & LOOP (Core 0)
// ================================================================
void setup() {
  Serial.begin(115200);
  mutex_init(&vehicleMutex);

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    splashOLED();
    delay(1500);
  }

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);
  pinMode(PIN_CAN_RESET, OUTPUT);
  digitalWrite(PIN_CAN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_CAN_RESET, HIGH);
  delay(10);

  if (mcp.begin(CAN_BAUDRATE)) canOK = true;

  Serial.println(F("AMR1 interprete_test v2.9"));
  Serial.println(F("CH3=deadman  CH4=-100 SERIAL / 0 RC / +100 AUTO  CH5=enable"));
  Serial.println(F("HELP | MODE SERIAL | MODE RC | MODE AUTO | MODE RADIO"));
}

void loop() {
  static unsigned long lastUI    = 0;
  static unsigned long lastCAN   = 0;
  static unsigned long lastDebug = 0;
  unsigned long now = millis();

  leerUSB();
  actualizarModoCH4();
  leerCANEntrante();

  if (now - lastCAN >= 20) {
    lastCAN = now;
    sendCANTelemetry();
  }

  if (now - lastUI >= 100) {
    lastUI = now;
    if (displayOK) updateOLED();
  }

  if (modoCtrl == MODO_RC && (now - lastDebug >= 500)) {
    lastDebug = now;
    debugChannels();
  }
}