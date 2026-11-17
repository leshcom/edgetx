/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * USB RC OUT — вбудована у прошивку заміна скрипта adis-vcp.lua.
 * Формат пакета описано в usb_rc_out.h.
 */

#include "usb_rc_out.h"

#include "edgetx.h"
#include "hal/usb_driver.h"
#include "os/time.h"

const char STR_USB_RC_OUT[] = "USB RC OUT";

#if defined(STM32) && !defined(SIMU) && defined(USB_SERIAL)

#define RC_OUT_MAGIC_1        0xA5
#define RC_OUT_MAGIC_2        0x5A
#define RC_OUT_PROTO_VERSION  1
#define RC_OUT_LIMIT          1536

static void* _ctx = nullptr;
static uint8_t _sequence = 0;
static uint32_t _lastSendMs = 0;

static uint16_t crc16Ccitt(const uint8_t* data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static inline void putU16LE(uint8_t* p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

void usbRcOutStart()
{
  etx_serial_init params = {};
  params.baudrate = 115200;  // для USB CDC не має значення
  params.encoding = ETX_Encoding_8N1;
  params.direction = ETX_Dir_TX_RX;
  params.polarity = ETX_Pol_Normal;

  _ctx = UsbSerialPort.uart->init(UsbSerialPort.hw_def, &params);
  _sequence = 0;
  _lastSendMs = time_get_ms();

  // Відправляти буфер кожен USB-кадр (1 мс), а не пачками — менше джитера.
  usbSerialSetLowLatency(true);
}

void usbRcOutStop()
{
  void* ctx = _ctx;
  _ctx = nullptr;  // спершу зупиняємо usbRcOutUpdate() у mixer task
  if (ctx && UsbSerialPort.uart->deinit) {
    UsbSerialPort.uart->deinit(ctx);
  }
  usbSerialSetLowLatency(false);
}

void usbRcOutUpdate()
{
  void* ctx = _ctx;
  if (!ctx) return;

  uint32_t now = time_get_ms();
  if ((uint32_t)(now - _lastSendMs) < USB_RC_OUT_PERIOD_MS) return;
  _lastSendMs = now;

  // Якщо програма на ПК не читає порт — пропускаємо пакет, а не блокуємо
  // mixer task. Лічильник sequence при цьому не збільшується.
  if (usbSerialFreeSpace() < USB_RC_OUT_PACKET_SIZE) return;

  uint8_t buf[USB_RC_OUT_PACKET_SIZE];
  uint8_t* p = buf;

  *p++ = RC_OUT_MAGIC_1;
  *p++ = RC_OUT_MAGIC_2;
  *p++ = RC_OUT_PROTO_VERSION;
  *p++ = _sequence;

  for (uint8_t ch = 0; ch < USB_RC_OUT_CHANNELS; ch++) {
    int32_t v = (ch < MAX_OUTPUT_CHANNELS) ? channelOutputs[ch] : 0;
    if (v < -RC_OUT_LIMIT) v = -RC_OUT_LIMIT;
    if (v > RC_OUT_LIMIT) v = RC_OUT_LIMIT;
    putU16LE(p, (uint16_t)(int16_t)v);
    p += 2;
  }

  *p++ = 0;  // flags, зарезервовано

  uint16_t crc = crc16Ccitt(buf, (uint32_t)(p - buf));
  putU16LE(p, crc);

  // У драйвера USB CDC немає sendBuffer, лише sendByte (кладе байт
  // у кільцевий буфер, без блокування).
  auto sendByte = UsbSerialPort.uart->sendByte;
  for (uint8_t i = 0; i < sizeof(buf); i++) sendByte(ctx, buf[i]);
  _sequence++;
}

#else  // симулятор або таргет без USB_SERIAL

void usbRcOutStart() {}
void usbRcOutStop() {}
void usbRcOutUpdate() {}

#endif
