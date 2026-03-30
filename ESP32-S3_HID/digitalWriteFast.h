/* digitalWriteFast.h — shim ESP32
 * A biblioteca digitalWriteFast é AVR-only.
 * No ESP32 mapeamos direto para digitalWrite/pinMode/digitalRead padrão.
 * O ESP32-S3 já é rápido o suficiente; não precisa de atalhos de porta.
 */
#ifndef _DIGITALWRITEFAST_H_
#define _DIGITALWRITEFAST_H_

#include <Arduino.h>

#define digitalWriteFast(pin, val) digitalWrite((pin), (val))
#define digitalReadFast(pin)       digitalRead((pin))
#define pinModeFast(pin, mode)     pinMode((pin), (mode))

#endif // _DIGITALWRITEFAST_H_
