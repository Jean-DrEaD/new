/* =============================================================================
   brWheel_ESP32S3.ino — ESP32-S3 port do Arduino-FFB-wheel (USART3 fork)
   =============================================================================
   PC ←USB HID (FFB/Joystick)→ ESP32-S3 ←Serial1 500kbps→ STM32/GD32 (FOC)
                                     ↑
                               USB CDC (WheelControl config)
   Configuração Arduino IDE:
     Board           : ESP32S3 Dev Module
     USB Mode        : USB-OTG (TinyUSB)
     USB CDC On Boot : Enabled
     Upload Mode     : UART0 / Hardware CDC
     Flash Size      : 4MB ou mais
     Partition Scheme: Default 4MB with spiffs
     CPU Frequency   : 240MHz
     PSRAM           : Disabled

   Upstream: https://github.com/ranenbg/Arduino-FFB-wheel
   ============================================================================= */

// ========================= CONFIG RÁPIDA =========================
#define USE_FIXED_TORQUE_TEST   0     // 0 = FFB real do jogo (1 = torque fixo para teste)
#define FIXED_TORQUE_VALUE      400   // -1000..1000 (comece com 400, suba depois)

#define PRINT_DEBUG_EVERY_MS    500   // debug no Serial (2x por segundo)
#define ENC_LINK_TIMEOUT_MS     200   // timeout do link STM32
// =================================================================

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "USB.h"             // Necessário para forçar o nome do produto
#include "Platform.h"        // ESP32: tipos u8/s32/b8/f32 + PROGMEM stubs
#include "USBAPI.h"          // ESP32: Joystick_, Setup, USBDevice_
#include "ESP32_USB_HID.h"   // ESP32: USB HID Adafruit TinyUSB
#include "ESP32_S3_Pins.h"   // ESP32: mapeamento de pinos Super Mini

#include "StmFrames.h"
#include "Config.h"
#include "ConfigHID.h"
#include "debug.h"
#include "ffb_pro.h"
#include "USBDesc.h"

#ifdef USE_QUADRATURE_ENCODER
#include "QuadEncoder.h"
#endif
#ifdef USE_VNH5019
#include "DualVNH5019MotorShield.h"
#endif
#include <Wire.h>
#ifdef USE_EEPROM
#include <EEPROM.h>
#endif
#ifdef USE_LOAD_CELL
#include <HX711_ADC.h>
#endif
#ifdef USE_LCD
#include <LiquidCrystal_I2C.h>
#endif
#ifdef USE_ADS1015
#include <Adafruit_ADS1015.h>
#endif
#ifdef USE_MCP4725
#include <Adafruit_MCP4725.h>
#endif
#ifdef USE_AS5600
#include "AS5600.h"
#endif

// =================================================================
// STM32 Link — Feedback (RX)
// =================================================================
static bool     gHaveEnc      = false;
#define AUTO_CENTER_SETTLE_MS   4200  // ms de posição estável → dispara auto-center
#define AUTO_CENTER_DEADBAND    3     // ticks de variação tolerada
#define AUTO_CENTER_RESET_MS    3000  // link precisa ficar down >3s para permitir novo auto-center
static int16_t  gAutoCenter_lastPos  = 0;
static uint32_t gAutoCenter_stableMs = 0;
static bool     gAutoCenter_done     = false;
static int32_t  gAutoCenter_sum      = 0;
static int32_t  gAutoCenter_cnt      = 0;
static uint32_t gLinkDownMs          = 0;
static int16_t  gSpeedR      = 0;
static int16_t  gCmd1        = 0;
static int16_t  gCmd2        = 0;
static int16_t  gBatVoltage  = 0;
static int16_t  gBoardTemp   = 0;
static uint32_t gLastEncRxMs = 0;
static float    gEncPos_f  = 0.0f;
static float    gPosOffset = 0.0f;  // offset de centro: subtraído de cada leitura STM32
volatile bool   gResetPosition = false;

static uint16_t stmFbChecksum(const StmEncFrame &f) {
  return (uint16_t)(
    f.start        ^
    (uint16_t)f.cmd1       ^
    (uint16_t)f.cmd2       ^
    (uint16_t)f.speedR_meas ^
    (uint16_t)f.speedL_meas ^
    (uint16_t)f.batVoltage  ^
    (uint16_t)f.boardTemp   ^
    f.cmdLed
  );
}

static void stmEncPoll() {
  static uint8_t buf[sizeof(StmEncFrame)];
  static uint8_t idx = 0;
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    if (idx == 0) { if (b != 0xCD) continue; buf[idx++] = b; continue; }
    if (idx == 1) { if (b != 0xAB) { idx = 0; continue; } buf[idx++] = b; continue; }

    buf[idx++] = b;
    if (idx >= sizeof(StmEncFrame)) {
      idx = 0;
      StmEncFrame f;
      memcpy(&f, buf, sizeof(f));
      if (f.start != 0xABCD) continue;
      if (f.checksum != stmFbChecksum(f)) continue;

      gSpeedR     = f.speedR_meas;
      gCmd1       = f.cmd1;
      gCmd2       = f.cmd2;
      gBatVoltage = f.batVoltage;
      gBoardTemp  = f.boardTemp;
      gLastEncRxMs = millis();
      if (!gHaveEnc) {
        gAutoCenter_lastPos  = f.speedL_meas;
        gAutoCenter_stableMs = millis();
        gAutoCenter_done     = false;
        gAutoCenter_sum      = 0;
        gAutoCenter_cnt      = 0;
      }
      gHaveEnc = true;

      if (!gAutoCenter_done) {
        if (abs((int32_t)f.speedL_meas - (int32_t)gAutoCenter_lastPos) > AUTO_CENTER_DEADBAND) {
          gAutoCenter_lastPos  = f.speedL_meas;
          gAutoCenter_stableMs = millis();
          gAutoCenter_sum      = 0;
          gAutoCenter_cnt      = 0;
        } else {
          // FIX: use snapshot at stable moment, not rolling average.
          // Average accumulated over 4.2s of post-alignment oscillation
          // was giving a biased center. Snapshot is cleaner.
          if ((millis() - gAutoCenter_stableMs) >= AUTO_CENTER_SETTLE_MS) {
            gPosOffset       = (float)f.speedL_meas;  // snapshot, not mean
            gEncPos_f        = 0.0f;
            gAutoCenter_done = true;
          }
        }
      }
      gEncPos_f = (float)f.speedL_meas - gPosOffset;
      gEncPos_f = constrain(gEncPos_f, (float)-ROTATION_MAX, (float)ROTATION_MAX);
    }
  }

  if (gHaveEnc && (millis() - gLastEncRxMs) > ENC_LINK_TIMEOUT_MS) {
    gHaveEnc    = false;
    gSpeedR     = 0;
    gEncPos_f   = 0.0f;
    gLinkDownMs = millis();
  }
  if (!gHaveEnc && gAutoCenter_done && (millis() - gLinkDownMs) > AUTO_CENTER_RESET_MS) {
    gAutoCenter_done = false;
  }
}

// =================================================================
// STM32 Link — Command (TX)
// =================================================================
static uint16_t stmCmdChecksum(const StmCmdFrame &c) {
  return (uint16_t)(c.start ^ (uint16_t)c.steer ^ (uint16_t)c.speed);
}

static void stmSendCmd(int16_t steer, int16_t speed) {
  StmCmdFrame c;
  c.start    = 0xABCD;
  c.steer    = steer;
  c.speed    = speed;
  c.checksum = stmCmdChecksum(c);
  Serial1.write((const uint8_t *)&c, sizeof(c));
}

// =================================================================
// Segurança / filtros de torque
// =================================================================
static const int16_t TORQUE_DEADBAND = 5;
static const int16_t TORQUE_MAX      = 1000;  // FIX: was 850, use full motor range
static const int16_t TORQUE_SLEW     = 300;   // FIX: 300 = 6.7ms to max, balance between snap and overshoot

static int16_t gTorqueOut = 0;
static int16_t applyDeadband(int16_t x, int16_t db) {
  return (x > -db && x < db) ? 0 : x;
}

static int16_t slewLimit(int16_t target, int16_t current, int16_t step) {
  int16_t delta = target - current;
  if (delta >  step) return current + step;
  if (delta < -step) return current - step;
  return target;
}

// =================================================================
// Globals
// =================================================================
fwOpt fwOptions;
s16a accel, clutch, hbrake;
#ifdef USE_SPLITAXIS
s16 combinedAxis;
s16 gasAxis;
#endif
#ifdef USE_XY_SHIFTER
xysh shifter;
#endif
s32a brake;

s32v turn;
s32v axis;
s32v ffbs;
u32 button = 0;

#ifdef USE_ADS1015
Adafruit_ADS1015 ads(0x48);
#endif
#ifdef USE_MCP4725
Adafruit_MCP4725 dac0;
Adafruit_MCP4725 dac1;
#endif

cFFB gFFB;
BRFFB brWheelFFB;

#ifdef AVG_INPUTS
extern s32 analog_inputs[];
u8 asc = 0;
#endif

u32 last_ConfigSerial = 0;
u32 last_refresh      = 0;
u32 now_micros        = 0;
u32 timeDiffConfigSerial = 0;

uint16_t dz, bdz;
uint8_t  last_LC_scaling;

#ifdef USE_LOAD_CELL
HX711_ADC LoadCell(4, 5);
#endif
#ifdef USE_QUADRATURE_ENCODER
cQuadEncoder myEnc;
#endif
#ifdef USE_LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif
#ifdef USE_VNH5019
DualVNH5019MotorShield ms;
void stopIfFault() {
  if (ms.getM1Fault()) { DEBUG_SERIAL.println("M1 fault"); while (1) {} }
  if (ms.getM2Fault()) { DEBUG_SERIAL.println("M2 fault"); while (1) {} }
}
#endif
#ifdef USE_AS5600
AS5600L as5600x(0x36);
#endif

void stmSendCmdKeepalive() { stmSendCmd(0, 0); }

void setup() {
  // --- FORÇA NOME DO DISPOSITIVO (ESP32-S3) ---
  USB.productName("ESP32-S3-FFB-Wheel");
  USB.manufacturerName("FFB-Project");
  USB.begin();
  // --------------------------------------------

  ESP32_HID_Init();

  CONFIG_SERIAL.begin(115200);
  CONFIG_SERIAL.setTimeout(10);

  initESP32ADC();

  Serial1.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX, STM32_UART_TX);
  while (Serial1.available()) Serial1.read();   // flush

  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;
  axis.x    = 0;
  turn.x    = 0;

#ifdef USE_EEPROM
  SetEEPROMConfig();
  LoadEEPROMConfig();
#else
  ROTATION_DEG = 1080;
  CPR          = 4096;
  configGeneralGain   = 100;
  configDamperGain    = 80;   // FIX: was 50
  configFrictionGain  = 80;   // FIX: was 50
  configConstantGain  = 100;
  configPeriodicGain  = 100;
  configSpringGain    = 100;
  configInertiaGain   = 70;   // FIX: was 50
  configCenterGain    = 50;   // FIX: 50 satura aos 410°, deixa headroom p/ endstop aos 540°
  configStopGain      = 100;
  effstate    = 0b00000011;  // FIX: bit0=spring + bit1=damper (damper prevents centering overshoot)
  LC_scaling  = 128;
  pwmstate    = 33;
  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP         = 0.0;
  accel.min  = 0; accel.max  = maxCal;
  brake.min  = 0; brake.max  = maxCal;
  clutch.min = 0; clutch.max = maxCal;
  hbrake.min = 0; hbrake.max = maxCal;
#endif

  if (CPR == 0) CPR = 4096;
  ROTATION_MAX = int32_t(float(CPR) / 360.0f * float(ROTATION_DEG));
  ROTATION_MID = ROTATION_MAX >> 1;

  InitInputs();
  FfbSetDriver(0);
  FfbInitPidState();  // FIX (Bug 2): pre-enable actuators so ACC/games see device ready immediately

  TOP                 = 1000;
  MM_MAX_MOTOR_TORQUE = 1000;
  MM_MIN_MOTOR_TORQUE = 0;

  ffbs.x     = 0;
  gTorqueOut = 0;
  gEncPos_f  = 0.0f;
  dz  = 0;
  bdz = 2047;
  last_LC_scaling = LC_scaling;
  last_refresh    = micros();
}

void loop() {
  now_micros           = micros();
  timeDiffConfigSerial = now_micros - last_ConfigSerial;

  if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
    last_refresh = now_micros;
    stmEncPoll();

    if (gResetPosition) {
      gResetPosition = false;
      gPosOffset += (float)gEncPos_f;
      gEncPos_f   = 0.0f;
      gTorqueOut  = 0;
      stmSendCmd(0, 0);
    }

    turn.x = (int32_t)gEncPos_f;
    axis.x = turn.x;

#if USE_FIXED_TORQUE_TEST
    int32_t torque = FIXED_TORQUE_VALUE;
#else
    ffbs   = gFFB.CalcTorqueCommands(&axis);
    int32_t torque = ffbs.x;
#endif

    torque = constrain(torque, -1000L, 1000L);
    int16_t t16 = (int16_t)torque;
    t16 = applyDeadband(t16, TORQUE_DEADBAND);
    t16 = constrain(t16, (int16_t)-TORQUE_MAX, (int16_t)TORQUE_MAX);
    t16 = slewLimit(t16, gTorqueOut, TORQUE_SLEW);
    gTorqueOut = t16;

    stmSendCmd(0, gTorqueOut);

    int32_t hidPos = 0;
    if (ROTATION_MAX > 0) {
      hidPos =  (int32_t)((float)turn.x / (float)ROTATION_MAX * (float)MID_REPORT_X);
    }
    hidPos = constrain(hidPos, -MID_REPORT_X - 1, MID_REPORT_X);

    {
      auto adcRead4 = [](uint8_t pin, uint8_t flushPin) -> int {
        analogRead(flushPin);
        delayMicroseconds(20);
        int sum = 0;
        for (int _i = 0; _i < 4; _i++) sum += analogRead(pin);
        return sum >> 2;
      };
      int rawBrake  = adcRead4(BRAKE_PIN,  CLUTCH_PIN);
      int rawAccel  = adcRead4(ACCEL_PIN,  BRAKE_PIN);
      int rawClutch = adcRead4(CLUTCH_PIN, ACCEL_PIN);

      int16_t aLo = (accel.min  < accel.max)  ? accel.min  : 0;
      int16_t aHi = (accel.min  < accel.max)  ? accel.max  : maxCal;
      int16_t bLo = (brake.min  < (int32_t)brake.max) ? (int16_t)brake.min : 0;
      int16_t bHi = (brake.min  < (int32_t)brake.max) ? (int16_t)brake.max : (int16_t)maxCal;
      int16_t cLo = (clutch.min < clutch.max) ? clutch.min : 0;
      int16_t cHi = (clutch.min < clutch.max) ? clutch.max : (int16_t)maxCal;

#define ADC_TOP_PCT    94
#define ADC_BOTTOM_PCT  6
      auto softTop = [](int32_t raw, int16_t lo, int16_t hi, int16_t full) -> int16_t {
        int32_t range = (int32_t)(hi - lo);
        int32_t topThr = (int32_t)lo + (range * ADC_TOP_PCT    / 100);
        int32_t botThr = (int32_t)lo + (range * ADC_BOTTOM_PCT / 100);
        if (raw >= topThr) return full;
        if (raw <= botThr) return 0;
        return (int16_t)map(constrain(raw, lo, hi), lo, hi, 0, full);
      };
      accel.val  = softTop(rawAccel,  aLo, aHi, (int16_t)maxCal);
      brake.val  = (int32_t)softTop(rawBrake, bLo, bHi, (int16_t)maxCal);
      clutch.val = softTop(rawClutch, cLo, cHi, (int16_t)maxCal);
      hbrake.val = 0;
    }

    restoreButtonPins();
    button = readInputButtons();

    uint16_t brakeHID = (uint16_t)((uint32_t)brake.val << 4);
    SendInputReport(hidPos + MID_REPORT_X + 1, brakeHID, accel.val, clutch.val, hbrake.val, button);

    // FIX (Bug 4): TinyUSB's sendReport prepends the report_id byte automatically.
    // pidState layout = [reportId | status | effectBlockIndex].
    // Sending &pidState (3 bytes) was feeding pidState.reportId as the "status" byte →
    // Windows always saw status=0 (or 2) → ACTUATORS_ENABLED never seen as set.
    // Correct: skip reportId, send only [status, effectBlockIndex] = 2 bytes.
    extern volatile USB_FFBReport_PIDStatus_Input_Data_t pidState;
    HID_SendReport(2, (uint8_t*)&pidState.status, 2);

#ifdef USE_CONFIGCDC
    if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
      configCDC();
      last_ConfigSerial = now_micros;
    }
#endif

    static uint32_t lastDbg = 0;
    uint32_t nowMs = millis();
    if ((nowMs - lastDbg) >= PRINT_DEBUG_EVERY_MS) {
      lastDbg = nowMs;
      extern volatile uint32_t gFfbRxCount;
      extern volatile bool     gFFB_autoCenter_prev;
      bool ac = gFFB.mAutoCenter;
      if (ac != gFFB_autoCenter_prev) {
        CONFIG_SERIAL.printf("[FFB] mAutoCenter → %s\n", ac ? "TRUE (autocenter)" : "FALSE (game FFB)");
        gFFB_autoCenter_prev = ac;
      }
      CONFIG_SERIAL.print(F("link=")); CONFIG_SERIAL.print(gHaveEnc ? 1 : 0);
      CONFIG_SERIAL.print(F(" pos="));     CONFIG_SERIAL.print((long)turn.x);
      CONFIG_SERIAL.print(F(" torq="));   CONFIG_SERIAL.print(gTorqueOut);
      CONFIG_SERIAL.print(F(" ac="));     CONFIG_SERIAL.print(ac ? 1 : 0);
      CONFIG_SERIAL.print(F(" ffb_rx=")); CONFIG_SERIAL.print(gFfbRxCount);
      CONFIG_SERIAL.print(F(" pid=0x"));  CONFIG_SERIAL.print(pidState.status, HEX);
      CONFIG_SERIAL.print(F(" bat="));    CONFIG_SERIAL.print(gBatVoltage / 100.0f, 1);
      CONFIG_SERIAL.println(F("V"));
    }
  }
}
