/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * USB RC OUT
 * ----------
 * Окремий USB-режим: радіо з'являється на ПК як віртуальний COM-порт (CDC),
 * а прошивка сама, без Lua-скрипта, відправляє виходи міксера CH1..CH16
 * з частотою 50 Гц у тому ж форматі, що й adis-vcp.lua.
 *
 * Пакет, 39 байт:
 *   [0]      0xA5
 *   [1]      0x5A
 *   [2]      версія протоколу = 1
 *   [3]      лічильник послідовності (0..255)
 *   [4..35]  CH1..CH16, int16, little-endian (~ -1024..+1024, макс. ±1536)
 *   [36]     флаги (зарезервовано, 0)
 *   [37..38] CRC16-CCITT (poly 0x1021, init 0xFFFF) по байтах 0..36, LE
 */

#pragma once

#include <stdint.h>

#define USB_RC_OUT_CHANNELS       16
#define USB_RC_OUT_PACKET_SIZE    (4 + USB_RC_OUT_CHANNELS * 2 + 1 + 2)  // 39
#define USB_RC_OUT_PERIOD_MS      20  // 50 Гц

// Пункт меню у спливаючому вікні вибору USB-режиму.
extern const char STR_USB_RC_OUT[];

// Викликається з handleUsbConnection() перед usbStart().
void usbRcOutStart();

// Викликається з handleUsbConnection() після usbStop().
void usbRcOutStop();

// Викликається з mixerTask() після кожного перерахунку міксера.
void usbRcOutUpdate();
