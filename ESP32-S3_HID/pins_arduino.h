/* pins_arduino.h — local override in sketch folder.
 * This file is found BEFORE the board variant's pins_arduino.h because
 * the sketch folder (-I sketch/) precedes the variant folder in -I order.
 * Strategy: include the real variant header first, then override USB strings.
 * #include_next searches remaining -I paths after the current file's directory.
 */
#include_next <pins_arduino.h>   // pulls in all standard variant defs

/* Override USB device strings so Adafruit_USBD_Device::clearConfiguration()
 * compiles with our name instead of the board default "ESP32S3_DEV". */
#undef  USB_MANUFACTURER
#define USB_MANUFACTURER "Jean-DrEaD"
#undef  USB_PRODUCT
#define USB_PRODUCT      "ESP32-S3-FFB-Wheel"
