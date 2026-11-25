/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * USB RC OUT
 * ----------
 * Окремий USB-режим: радіо з'являється на ПК як віртуальний COM-порт (CDC),
 * а прошивка відправляє виходи міксера CH1..CH16 стандартними кадрами
 * CRSF RC_CHANNELS_PACKED з частотою 250 Гц — так само, як їх віддає
 * CRSF/ELRS-приймач польотному контролеру.
 *
 * Кадр, 26 байт:
 *   [0]      0xC8  адреса призначення (flight controller)
 *   [1]      24    довжина (type + payload + crc)
 *   [2]      0x16  тип кадру RC_CHANNELS_PACKED
 *   [3..24]  16 каналів по 11 біт, LSB first (22 байти)
 *   [25]     CRC8 DVB-S2 (poly 0xD5) по байтах [2..24]
 *
 * Значення каналу: 992 + out * 4 / 5, обмежене 0..1984
 * (out = вихід міксера -1024..+1024 → 172..1811, як у CRSF-модуля EdgeTX).
 * Перерахунок у мікросекунди на боці приймача: us = (v - 992) * 5 / 8 + 1500.
 */

#pragma once

#include <stdint.h>

#define USB_RC_OUT_CHANNELS       16
#define USB_RC_OUT_FRAME_SIZE     26
// Період відправки: 4000 мкс = 250 Гц (стандартна частота CRSF).
// Міксер EdgeTX без активного радіомодуля рахує кожні 4 мс, тож кожен кадр
// несе свіжі значення. Якщо увімкнено модуль із меншою частотою пакетів,
// міксер синхронізується з ним — і кадри йтимуть з частотою модуля.
#define USB_RC_OUT_PERIOD_US      4000

// Пункт меню у спливаючому вікні вибору USB-режиму.
extern const char STR_USB_RC_OUT[];

// Викликається з handleUsbConnection() перед usbStart().
void usbRcOutStart();

// Викликається з handleUsbConnection() після usbStop().
void usbRcOutStop();

// Викликається з mixerTask() після кожного перерахунку міксера.
void usbRcOutUpdate();
