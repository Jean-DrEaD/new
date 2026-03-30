/* pwm.ino — ESP32-S3 port
 * Substitui registradores AVR (TCCR1A, ICR1, OCR1A/B) pelo driver LEDC
 * do ESP32 (ledcAttach/ledcWrite — API core 3.x).
 */
#include "Config.h"
#include "digitalWriteFast.h"

// LEDC channels
#ifdef USE_TWOFFBAXIS
#endif

static uint8_t  s_pwmBits = 11;
static uint32_t s_pwmFreq = 15600;

#ifndef USE_MCP4725
void PWM16Begin() {
    uint32_t topVal = (TOP > 0) ? TOP : 1;
    s_pwmBits = 1;
    while ((1UL << s_pwmBits) <= topVal && s_pwmBits < 16) s_pwmBits++;
    bool phaseCorrect = bitRead(pwmstate, 0);
    bool rcmMode      = bitRead(pwmstate, 1) && bitRead(pwmstate, 6);
    s_pwmFreq = rcmMode ? 50 : (phaseCorrect ? 7800 : 15600);
    // Core 3.x: ledcAttach() handles channel assignment automatically
}
void PWM16EnableA() { ledcAttach(PWM_PIN_L, s_pwmFreq, s_pwmBits); }
void PWM16EnableB() { ledcAttach(PWM_PIN_R, s_pwmFreq, s_pwmBits); }
#ifdef USE_TWOFFBAXIS
void PWM16EnableC() { ledcAttach(PWM_PIN_U, s_pwmFreq, s_pwmBits); }
void PWM16EnableD() { ledcAttach(PWM_PIN_D, s_pwmFreq, s_pwmBits); }
#endif
inline void PWM16A(uint16_t v) { ledcWrite(PWM_PIN_L, constrain((int)v, 0, (int)TOP)); }
inline void PWM16B(uint16_t v) { ledcWrite(PWM_PIN_R, constrain((int)v, 0, (int)TOP)); }
#ifdef USE_TWOFFBAXIS
inline void PWM16C(uint16_t v) { ledcWrite(PWM_PIN_U, constrain((int)v, 0, (int)TOP)); }
inline void PWM16D(uint16_t v) { ledcWrite(PWM_PIN_D, constrain((int)v, 0, (int)TOP)); }
#endif
#endif // !USE_MCP4725

void blinkFFBclipLED() {
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(FFBCLIP_LED_PIN, HIGH); delay(20);
        digitalWrite(FFBCLIP_LED_PIN, LOW);  delay(20);
    }
}

void activateFFBclipLED(s32 t) {
    float level = 0.01f * configGeneralGain;
    float absT  = (float)abs((long)t);
    float hi    = level * MM_MAX_MOTOR_TORQUE;
    float lo    = 0.9f * hi;
    if (absT >= hi - 1.0f)    digitalWrite(FFBCLIP_LED_PIN, HIGH);
    else if (absT >= lo)      analogWrite(FFBCLIP_LED_PIN, (int)map((long)absT,(long)lo,(long)hi,1,63));
    else                      digitalWrite(FFBCLIP_LED_PIN, LOW);
}

void InitPWM() {
    pinMode(DIR_PIN, OUTPUT);
    TOP = calcTOP(pwmstate);
    MM_MAX_MOTOR_TORQUE = TOP;
    minTorquePP = ((f32)MM_MIN_MOTOR_TORQUE) / ((f32)MM_MAX_MOTOR_TORQUE);
    RCM_min *= RCMscaler(pwmstate);
    RCM_zer *= RCMscaler(pwmstate);
    RCM_max *= RCMscaler(pwmstate);

#ifdef USE_MCP4725
    dac0.begin(0x60); dac1.begin(0x61);
    dac0.setVoltage(0, true, 0); dac1.setVoltage(0, true, 0);
#ifdef USE_TWOFFBAXIS
    pinMode(PWM_PIN_R, OUTPUT);
#endif
#else
    PWM16Begin();
    PWM16A(0); PWM16EnableA();
    PWM16B(0); PWM16EnableB();
#ifdef USE_TWOFFBAXIS
    PWM16C(0); PWM16EnableC();
    PWM16D(0); PWM16EnableD();
#endif
#endif

    pinMode(FFBCLIP_LED_PIN, OUTPUT);
    blinkFFBclipLED();
}

void SetPWM(s32v *torque) {
    if (!torque) return;
    activateFFBclipLED(torque->x);
#ifndef USE_LOAD_CELL
    FFB_bal = (f32)(LC_scaling - 128) / 255.0f;
    if (FFB_bal >= 0) { L_bal = 1.0f - FFB_bal; R_bal = 1.0f; }
    else              { L_bal = 1.0f;            R_bal = 1.0f + FFB_bal; }
#else
    L_bal = 1.0f; R_bal = 1.0f;
#endif

#ifdef USE_MCP4725
    if (bitRead(pwmstate, 7)) {
        if (!bitRead(pwmstate, 6)) {
            if (!bitRead(pwmstate, 5)) {
                if (torque->x > 0) {
                    dac0.setVoltage(0, false, 0);
                    dac1.setVoltage(map(torque->x,0,MM_MAX_MOTOR_TORQUE,(int)(minTorquePP*MM_MAX_MOTOR_TORQUE),(int)(R_bal*MM_MAX_MOTOR_TORQUE)), false, 0);
                } else if (torque->x < 0) {
                    dac0.setVoltage(map(-torque->x,0,MM_MAX_MOTOR_TORQUE,(int)(minTorquePP*MM_MAX_MOTOR_TORQUE),(int)(L_bal*MM_MAX_MOTOR_TORQUE)), false, 0);
                    dac1.setVoltage(0, false, 0);
                } else { dac0.setVoltage(0,false,0); dac1.setVoltage(0,false,0); }
            } else {
                int v = (torque->x>0) ? map(torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE/2+MM_MIN_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE)
                      : (torque->x<0) ? map(-torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE/2-MM_MIN_MOTOR_TORQUE,0)
                      : MM_MAX_MOTOR_TORQUE/2;
                dac0.setVoltage(v,false,0); dac1.setVoltage(MM_MAX_MOTOR_TORQUE/2,false,0);
            }
        } else {
            digitalWrite(DIR_PIN, torque->x>=0 ? HIGH : LOW);
            dac0.setVoltage(map(abs((int)torque->x),0,MM_MAX_MOTOR_TORQUE,(int)(minTorquePP*MM_MAX_MOTOR_TORQUE),MM_MAX_MOTOR_TORQUE),false,0);
            dac1.setVoltage(0,false,0);
        }
    } else {
        int z = (!bitRead(pwmstate,6) && bitRead(pwmstate,5)) ? MM_MAX_MOTOR_TORQUE/2 : 0;
        dac0.setVoltage(z,false,0); dac1.setVoltage(z,false,0);
    }
#else
#ifndef USE_TWOFFBAXIS
    if (!bitRead(pwmstate, 1)) {
        if (!bitRead(pwmstate, 6)) {
            if (torque->x > 0) {
                torque->x = map(torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MIN_MOTOR_TORQUE,(int)(R_bal*MM_MAX_MOTOR_TORQUE));
                PWM16A(0); PWM16B(torque->x); digitalWrite(DIR_PIN, HIGH);
            } else if (torque->x < 0) {
                torque->x = map(-torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MIN_MOTOR_TORQUE,(int)(L_bal*MM_MAX_MOTOR_TORQUE));
                PWM16A(torque->x); PWM16B(0); digitalWrite(DIR_PIN, HIGH);
            } else { PWM16A(0); PWM16B(0); digitalWrite(DIR_PIN, LOW); }
        } else {
            int v = (torque->x>0) ? map(torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE/2+MM_MIN_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE)
                  : (torque->x<0) ? map(-torque->x,0,MM_MAX_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE/2-MM_MIN_MOTOR_TORQUE,0)
                  : MM_MAX_MOTOR_TORQUE/2;
            PWM16A(v); PWM16B(0);
        }
    } else {
        if (!bitRead(pwmstate, 6)) {
            digitalWrite(DIR_PIN, torque->x>=0 ? HIGH : LOW);
            torque->x = map(abs((int)torque->x),0,MM_MAX_MOTOR_TORQUE,MM_MIN_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE);
            PWM16A(torque->x);
        } else {
            if (torque->x>0)      PWM16A(map(torque->x,0,MM_MAX_MOTOR_TORQUE,(int)(RCM_zer*(1.0f+minTorquePP)),(int)RCM_max));
            else if (torque->x<0) PWM16A(map(-torque->x,0,MM_MAX_MOTOR_TORQUE,(int)(RCM_zer*(1.0f-minTorquePP)),(int)RCM_min));
            else                  PWM16A((int)RCM_zer);
        }
        PWM16B((int)RCM_zer);
    }
#endif // !USE_TWOFFBAXIS
#endif // !USE_MCP4725
}
