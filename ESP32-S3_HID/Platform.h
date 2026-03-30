/* Platform.h — ESP32-S3 port
 * Substitui o Platform.h do core AVR.
 * Define tipos compactos (u8, s32, b8, f32) e neutraliza macros AVR-only.
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>

// ── Tipos curtos usados em todo o projeto ──────────────────────────────────
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef bool      b8;
typedef float     f32;

// ── PROGMEM: ESP32 tem DRAM suficiente, ignoramos ────────────────────────
#ifndef PROGMEM
#  define PROGMEM
#endif
#ifndef pgm_read_byte
#  define pgm_read_byte(addr)  (*(const uint8_t  *)(addr))
#endif
#ifndef pgm_read_word
#  define pgm_read_word(addr)  (*(const uint16_t *)(addr))
#endif
#ifndef pgm_read_dword
#  define pgm_read_dword(addr) (*(const uint32_t *)(addr))
#endif
#ifndef PSTR
#  define PSTR(s) (s)
#endif

// ── Bit helpers usados em ffb.ino / debug.ino ────────────────────────────
#ifndef Bset
#  define Bset(reg, bit) ((reg) |=  (bit))
#endif
#ifndef Bclr
#  define Bclr(reg, bit) ((reg) &= ~(bit))
#endif
#ifndef Btest
#  define Btest(reg, bit) ((reg) & (bit))
#endif

// ── USBCON: sinaliza que USB HID está disponível (TinyUSB) ───────────────
#ifndef USBCON
#  define USBCON
#endif

// ── Constantes HID (normalmente vêm do core AVR) ─────────────────────────
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE  0x21
#define REQUEST_DEVICETOHOST_CLASS_INTERFACE  0xA1
#define HID_GET_REPORT   0x01
#define HID_GET_IDLE     0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT   0x09
#define HID_SET_IDLE     0x0A
#define HID_SET_PROTOCOL 0x0B

#define TRANSFER_PGM     0x80
#define TRANSFER_RELEASE 0x40
#define TRANSFER_ZERO    0x20

#define LEDS_ALL_LEDS 0
#define LEDS_NO_LEDS  0

#endif // _PLATFORM_H_
