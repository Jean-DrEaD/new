/* USBAPI.h — ESP32-S3 port
 * Substitui o USBAPI.h do core AVR.
 * A implementação real está em ESP32_USB_HID.cpp (TinyUSB).
 */
#ifndef __USBAPI__
#define __USBAPI__

#include "Platform.h"
#include <Arduino.h>

#ifdef USBCON

// Estrutura Setup: usada em ffb.ino / HID_Setup callbacks
typedef struct {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint8_t  wValueL;
    uint8_t  wValueH;
    uint16_t wIndex;
    uint16_t wLength;
} Setup;

// Funções USB (implementadas em ESP32_USB_HID.cpp)
void  USB_SendControl(uint8_t flags, const void* data, int len);
int   USB_RecvControl(void* data, int len);
void  HID_SendReport(uint8_t id, const void* data, int len);

// Protótipos dos callbacks FFB (implementados em ffb.ino)

// USBDevice_
class USBDevice_ {
public:
    USBDevice_() {}
    bool configured() { return true; }
    void attach()  {}
    void detach()  {}
    void poll()    {}
    b8  (*HID_Setup_Callback)(Setup& setup) = nullptr;
    void(*HID_ReceiveReport_Callback)(uint8_t*, uint16_t) = nullptr;
};
extern USBDevice_ USBDevice;

// Joystick_
class Joystick_ {
public:
    Joystick_() {}
    void send_16_16_12_12_12_28(uint16_t x, uint16_t y, uint16_t z,
                                uint16_t rx, uint16_t ry, uint32_t buttons);
    void send_8(int8_t x, uint8_t y, uint8_t z, uint8_t buttons);
    void send_16(int16_t x, uint16_t y, uint16_t z, uint8_t buttons);
    void send_16_12_12(int16_t x, uint16_t y, uint16_t z, uint8_t buttons);
    void send_16_16_12(int16_t x, uint16_t y, uint16_t z, uint16_t buttons);
    void send_16_16_10_10_12(int16_t x, uint16_t y, uint16_t z,
                             uint16_t rx, uint32_t buttons);
    void send_16_16_12_12_32(int16_t x, uint16_t y, uint16_t z,
                             uint16_t rx, uint32_t buttons);
    void send_16_10_18(int16_t x, uint16_t y, uint16_t z,
                       uint16_t rx, uint32_t buttons);
    void send_16_8_32(int16_t x, uint16_t y, uint16_t z, uint16_t rx,
                      uint16_t sx, uint16_t sy, uint32_t buttons);
    void send_16_16_12_12_12_12_32(int16_t x, uint16_t y, uint16_t z,
                                   uint16_t rx, uint16_t ry, uint16_t rz,
                                   uint32_t buttons);
};

struct KeyReport { uint8_t modifiers; uint8_t reserved; uint8_t keys[6]; };
class Keyboard_ : public Print {
public:
    Keyboard_() {}
    void begin() {} void end() {}
    size_t write(uint8_t c) { return 0; }
    size_t press(uint8_t k) { return 0; }
    size_t release(uint8_t k) { return 0; }
    void releaseAll() {}
    void sendReport(KeyReport*) {}
};
class Mouse_ {
public:
    Mouse_() {}
    void begin() {} void end() {}
    void click(uint8_t b = 1) {}
    void move(signed char x, signed char y, signed char wheel = 0) {}
    void press(uint8_t b = 1) {}
    void release(uint8_t b = 1) {}
    bool isPressed(uint8_t b = 1) { return false; }
};

extern Joystick_ Joystick;
extern Keyboard_ Keyboard;
extern Mouse_    Mouse;

#endif // USBCON
#endif // __USBAPI__
