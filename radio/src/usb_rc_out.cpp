/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * USB RC OUT — виходи міксера у вигляді CRSF RC_CHANNELS_PACKED через USB VCP.
 * Формат кадру описано в usb_rc_out.h.
 */

#include "usb_rc_out.h"

#include "edgetx.h"
#include "crc.h"
#include "hal/usb_driver.h"

const char STR_USB_RC_OUT[] = "USB RC OUT";

#if defined(STM32) && !defined(SIMU) && defined(USB_SERIAL)

#define CRSF_ADDR_FLIGHT_CONTROLLER  0xC8
#define CRSF_TYPE_RC_CHANNELS        0x16
#define CRSF_RC_PAYLOAD_SIZE         22
#define CRSF_CH_BITS                 11
#define CRSF_CH_CENTER               992

static void* _ctx = nullptr;
// Допуск: кадр можна відправити трохи раніше дедлайну, щоб тремтіння
// запуску міксера (кілька мкс) не зсувало відправку на цілий цикл.
#define RC_OUT_EARLY_US  500

static uint32_t _nextDueUs = 0;

static uint32_t toCrsf(int32_t out)
{
  int32_t v = CRSF_CH_CENTER + (out * 4) / 5;
  if (v < 0) v = 0;
  if (v > 2 * CRSF_CH_CENTER) v = 2 * CRSF_CH_CENTER;
  return (uint32_t)v;
}

static uint8_t buildFrame(uint8_t* frame)
{
  uint8_t* buf = frame;
  *buf++ = CRSF_ADDR_FLIGHT_CONTROLLER;
  *buf++ = CRSF_RC_PAYLOAD_SIZE + 2;  // type + payload + crc
  uint8_t* crcStart = buf;
  *buf++ = CRSF_TYPE_RC_CHANNELS;

  uint32_t bits = 0;
  uint8_t bitsAvailable = 0;
  for (uint8_t ch = 0; ch < USB_RC_OUT_CHANNELS; ch++) {
    int32_t out = (ch < MAX_OUTPUT_CHANNELS) ? channelOutputs[ch] : 0;
    bits |= toCrsf(out) << bitsAvailable;
    bitsAvailable += CRSF_CH_BITS;
    while (bitsAvailable >= 8) {
      *buf++ = (uint8_t)bits;
      bits >>= 8;
      bitsAvailable -= 8;
    }
  }

  *buf = crc8(crcStart, (uint32_t)(buf - crcStart));
  buf++;
  return (uint8_t)(buf - frame);
}

void usbRcOutStart()
{
  etx_serial_init params = {};
  params.baudrate = 420000;  // для USB CDC не має значення
  params.encoding = ETX_Encoding_8N1;
  params.direction = ETX_Dir_TX_RX;
  params.polarity = ETX_Pol_Normal;

  _ctx = UsbSerialPort.uart->init(UsbSerialPort.hw_def, &params);
  _nextDueUs = timersGetUsTick();

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

  // Розклад із фіксованим кроком: середня частота рівно 250 Гц,
  // незалежно від того, як часто викликається міксер (1, 2 або 4 мс).
  uint32_t now = timersGetUsTick();
  int32_t untilDue = (int32_t)(_nextDueUs - now);
  if (untilDue > RC_OUT_EARLY_US) return;

  _nextDueUs += USB_RC_OUT_PERIOD_US;
  if ((int32_t)(_nextDueUs - now) <= 0) {
    // міксер працює повільніше (модуль з низькою частотою) — не "доганяємо"
    _nextDueUs = now + USB_RC_OUT_PERIOD_US;
  }

  // Якщо програма на ПК не читає порт — пропускаємо кадр, а не блокуємо
  // mixer task.
  if (usbSerialFreeSpace() < USB_RC_OUT_FRAME_SIZE) return;

  uint8_t frame[USB_RC_OUT_FRAME_SIZE];
  uint8_t len = buildFrame(frame);

  // У драйвера USB CDC немає sendBuffer, лише sendByte (кладе байт
  // у кільцевий буфер, без блокування).
  auto sendByte = UsbSerialPort.uart->sendByte;
  for (uint8_t i = 0; i < len; i++) sendByte(ctx, frame[i]);
}

#else  // симулятор або таргет без USB_SERIAL

void usbRcOutStart() {}
void usbRcOutStop() {}
void usbRcOutUpdate() {}

#endif
