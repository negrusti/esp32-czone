// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include "czone_firmware_version.h"

#define CZONE_DEVICE_NAME              "ESP32-CZone-Relay"
#define CZONE_DEVICE_PREFERRED_ADDRESS 200
#define CZONE_DEVICE_DIPSWITCH         0x18
// CZone module type (tModuleType). 28 = "Control 1" (a basic output/control
// module, the simpler sibling of COI=31) -- a good fit for an 8-relay board.
// Declared in the CZone status PGN 65284 byte[3].
#define CZONE_DEVICE_MODULE_TYPE       28

// Set to 0 to derive a stable 21-bit N2K identity from the ESP32 factory eFuse MAC.
#define CZONE_DEVICE_IDENTITY_NUMBER   0
// Manufacturer code in the proprietary CZone PGN vendor header. Must stay BEP
// (295) so CZone tooling recognises this module.
#define CZONE_DEVICE_MANUFACTURER_CODE 295
// Manufacturer code reported in the N2K NAME / product information. Set to a
// non-BEP value so the device is not presented as a genuine BEP product in
// generic N2K views; the proprietary PGNs above are unaffected.
#define CZONE_DEVICE_PRODUCT_MANUFACTURER_CODE 999
#define CZONE_DEVICE_FUNCTION          140
#define CZONE_DEVICE_CLASS             30
#define CZONE_DEVICE_INDUSTRY_GROUP    4

#define CZONE_DEVICE_PRODUCT_CODE      0x0841
#define CZONE_DEVICE_N2K_DB_VERSION    1300
#define CZONE_DEVICE_MODEL_ID          "WaveShare Relay Board"
#define CZONE_DEVICE_SOFTWARE_VERSION  CZONE_FIRMWARE_VERSION
#define CZONE_DEVICE_MODEL_VERSION     "ESP32-S3-POE-ETH-8DI-8RO-C"
#define CZONE_DEVICE_MANUFACTURER_INFO "BEP Marine"
