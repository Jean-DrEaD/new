/* QuadEncoder.ino — ESP32-S3 port
 * Substitui ISR(INTx_vect) / PIND / EICRA por attachInterrupt() padrão.
 * Com USE_QUADRATURE_ENCODER desabilitado (padrão do projeto), este código
 * compila mas não ocupa interrupções.
 */

#include <Arduino.h>
#include "QuadEncoder.h"
#include "digitalWriteFast.h"

cQuadEncoder gQuadEncoder;
volatile b8  gIndexFound;
volatile u8  gLastState;
volatile s32 gPosition;

#ifdef USE_QUADRATURE_ENCODER
static void IRAM_ATTR encoderISR_A() { gQuadEncoder.Update(); }
static void IRAM_ATTR encoderISR_B() { gQuadEncoder.Update(); }
#endif

#ifdef USE_CENTERBTN
volatile bool cButtonPressed = false;
static void IRAM_ATTR recenterISR() {
#ifdef USE_QUADRATURE_ENCODER
#ifndef USE_ZINDEX
    myEnc.Write(ROTATION_MID);
#endif
#elif defined(USE_AS5600)
    cButtonPressed = true;
#endif
}
#endif

void cQuadEncoder::Init(s32 position, b8 pullups) {
    uint8_t mode = pullups ? INPUT_PULLUP : INPUT;
#ifdef USE_QUADRATURE_ENCODER
    pinMode(QUAD_ENC_PIN_A, mode);
    pinMode(QUAD_ENC_PIN_B, mode);
    gIndexFound = false;
    gPosition   = position;
    gLastState  = 0;
    if (digitalRead(QUAD_ENC_PIN_A)) gLastState |= 1;
    if (digitalRead(QUAD_ENC_PIN_B)) gLastState |= 2;
    attachInterrupt(digitalPinToInterrupt(QUAD_ENC_PIN_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(QUAD_ENC_PIN_B), encoderISR_B, CHANGE);
#endif
#ifdef USE_ZINDEX
    pinMode(QUAD_ENC_PIN_I, mode);
#elif defined(USE_CENTERBTN)
    pinMode(QUAD_ENC_PIN_I, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(QUAD_ENC_PIN_I), recenterISR, RISING);
#endif
}

s32 cQuadEncoder::Read() {
    noInterrupts(); s32 p = gPosition; interrupts(); return p;
}
void cQuadEncoder::Write(s32 pos) {
    noInterrupts(); gPosition = pos; interrupts();
}

static const s8 pos_inc[] = {0,1,-1,2,-1,0,-2,1,1,-2,0,-1,2,-1,1,0};

void cQuadEncoder::Update() {
    uint8_t state = gLastState;
    if (digitalRead(QUAD_ENC_PIN_A)) state |= 4;
    if (digitalRead(QUAD_ENC_PIN_B)) state |= 8;
    gLastState  = (state >> 2);
    gPosition  += pos_inc[state & 0x0F];
    if (gIndexFound) return;
#ifdef USE_ZINDEX
    if (digitalRead(QUAD_ENC_PIN_I)) {
        gIndexFound = true; zIndexFound = true;
        brWheelFFB.state = 1;
        gPosition = ROTATION_MID;
    }
#endif
}
