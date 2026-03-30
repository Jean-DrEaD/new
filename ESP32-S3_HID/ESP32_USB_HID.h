/* ESP32_USB_HID.h — Adafruit TinyUSB HID (Joystick FFB)
 *
 * Cria dispositivo USB COMPOSTO: CDC (Serial/Wheel Control) + HID (joystick FFB).
 * O CDC já é gerenciado pelo core quando CDCOnBoot=Enabled.
 * Este arquivo só adiciona o HID.
 *
 * Library Manager: "Adafruit TinyUSB Library"
 * Tools → USB Mode            → USB-OTG (TinyUSB)
 * Tools → USB CDC On Boot     → Enabled   ← obrigatório para Wheel Control
 */
#ifndef _ESP32_USB_HID_H_
#define _ESP32_USB_HID_H_

#include "Platform.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

extern uint8_t  g_featureReportBuf[64];
extern uint16_t g_featureReportLen;

// Adiciona HID ao dispositivo USB já iniciado pelo core (CDC).
// Chame na PRIMEIRA linha de setup(), antes de Serial.begin().
void ESP32_HID_Init(void);
bool ESP32_HID_Ready(void);

#endif
