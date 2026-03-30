/* initVariant.cpp
 * Define VID:PID antes da inicialização USB do core.
 * As strings USB são definidas em ESP32_HID_Init() (ESP32_USB_HID.cpp)
 * APÓS clearConfiguration() — único ponto confiável no fluxo Adafruit TinyUSB.
 */
#include <Adafruit_TinyUSB.h>

void initVariant(void) {
    // PID 0x4FFC: diferente do 0x4FFB anterior para forçar nova entrada
    // no registro do Windows (sem cache do nome antigo "ESP32S3_DEV").
    TinyUSBDevice.setID(0x1209, 0x4FFC);
}
