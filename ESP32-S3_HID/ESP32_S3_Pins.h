/* ESP32_S3_Pins.h
 * Mapeamento de pinos Arduino Pro Micro → ESP32-S3 Super Mini
 * Baseado no diagrama: Arduino FFB Wheel - Firmware FOC (Super Mini ESP32-S3)
 *
 * UART para STM32:
 *   Serial1.begin(500000, SERIAL_8N1, 18, 17)
 *   GPIO 17 = TX → STM32 RX
 *   GPIO 18 = RX ← STM32 TX
 *
 * Pinos analógicos (ADC):
 *   Acelerador   = GPIO 1
 *   Freio        = GPIO 2
 *   Embreagem    = GPIO 3
 *
 * Botões digitais:
 *   Botão 0 = GPIO 5
 *   Botão 1 = GPIO 6
 *   Botão 2 = GPIO 7
 *   Botão 3 = GPIO 8
 *
 * LED:
 *   FFBCLIP_LED  = GPIO 48 (LED onboard do Super Mini)
 *
 * UART STM32 — definido abaixo e usado em brWheel_HID.ino
 */
#ifndef _ESP32_S3_PINS_H_
#define _ESP32_S3_PINS_H_

// ── UART para STM32 ──────────────────────────────────────────────────────────
#define STM32_UART_RX   18   // GPIO 18 ← STM32 TX
#define STM32_UART_TX   17   // GPIO 17 → STM32 RX
#define STM32_UART_BAUD 500000

// ── ADC pedais ───────────────────────────────────────────────────────────────
#undef  ACCEL_PIN
#define ACCEL_PIN   1        // GPIO 1  (A1 no diagrama)
#undef  BRAKE_PIN
#define BRAKE_PIN   2        // GPIO 2
#undef  CLUTCH_PIN
#define CLUTCH_PIN  3        // GPIO 3

// ── LED de clip FFB ──────────────────────────────────────────────────────────
#undef  FFBCLIP_LED_PIN
#define FFBCLIP_LED_PIN  48  // LED onboard Super Mini

// ── Botões ───────────────────────────────────────────────────────────────────
// O Wheel Control foi projetado para o Arduino Pro Micro onde o packing HID é:
//   bits 0-3 do button → campo HAT (não vira botão visível)
//   bit  4 → Button 1 no Wheel Control  = BUTTON4
//   bit  5 → Button 2 no Wheel Control  = BUTTON5
//   bit  6 → Button 3 no Wheel Control  = BUTTON6
//   bit  7 → Button 4 no Wheel Control  = BUTTON7
//
// BUTTON0-3 vão para o campo HAT e devem ser SEMPRE 0 (não pressionados).
// Mapeamos BUTTON0-3 para GPIO 48 (LED onboard, com pullup = sempre HIGH = nunca dispara).
// BUTTON4-7 são os botões físicos GPIO 5, 6, 7, 8.
#undef  BUTTON0
#define BUTTON0  48  // GPIO 48 = LED onboard = sempre HIGH = HAT nunca dispara
#undef  BUTTON1
#define BUTTON1  48
#undef  BUTTON2
#define BUTTON2  48
#undef  BUTTON3
#define BUTTON3  48
// Botões físicos: mapeiam para Button 1-4 no Wheel Control
#undef  BUTTON4
#define BUTTON4  5   // GPIO 5 → Button 1 no Wheel Control
#undef  BUTTON5
#define BUTTON5  6   // GPIO 6 → Button 2 no Wheel Control
#undef  BUTTON6
#define BUTTON6  7   // GPIO 7 → Button 3 no Wheel Control
#undef  BUTTON7
#define BUTTON7  8   // GPIO 8 → Button 4 no Wheel Control (opcional)

// ── PWM output (não usado no modo Serial-DD, mas precisa compilar) ───────────
#undef  PWM_PIN_L
#define PWM_PIN_L   9
#undef  PWM_PIN_R
#define PWM_PIN_R   10
#undef  DIR_PIN
#define DIR_PIN     16

// ── ADC 12-bit (ESP32-S3) ────────────────────────────────────────────────────
// O ESP32-S3 tem ADC de 12-bit (0..4095).
// maxCal=4095 já definido em Config.h via ARDUINO_ARCH_ESP32.
// Chame initESP32ADC() no início de setup() para configurar o hardware.
// ── ESP-IDF GPIO (nível abaixo do Arduino, ignora o mux ADC) ─────────────────
#include <driver/gpio.h>

inline void initESP32ADC() {
    analogReadResolution(12);  // 12-bit: 0..4095
    // Por pino apenas — NAO usar analogSetAttenuation() global
    // (configuraria TODOS GPIO1-10 como ADC, inclusive os pinos de botão)
    analogSetPinAttenuation(ACCEL_PIN,  ADC_11db);  // GPIO1
    analogSetPinAttenuation(BRAKE_PIN,  ADC_11db);  // GPIO2
    analogSetPinAttenuation(CLUTCH_PIN, ADC_11db);  // GPIO3
}

// Chame APÓS analogRead() para restaurar modo digital nos pinos de botão.
// O ADC1 do ESP32-S3 afeta GPIO1-10 (ADC1_CH0-9) ao inicializar.
// gpio_set_direction/gpio_pullup_en operam abaixo do mux ADC e têm prioridade.
inline void restoreButtonPins() {
    const gpio_num_t btnPins[] = {
        (gpio_num_t)BUTTON4, (gpio_num_t)BUTTON5,
        (gpio_num_t)BUTTON6, (gpio_num_t)BUTTON7
    };
    for (int i = 0; i < 4; i++) {
        gpio_set_direction(btnPins[i], GPIO_MODE_INPUT);
        gpio_pullup_en(btnPins[i]);
        gpio_pulldown_dis(btnPins[i]);
    }
}

#endif // _ESP32_S3_PINS_H_
