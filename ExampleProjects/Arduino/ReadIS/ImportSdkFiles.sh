#!/bin/bash

# This script copies the SDK subset used by the Arduino ReadIS example.

set -euo pipefail
set -x

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/../../../src"
PRINTF_DIR="${SCRIPT_DIR}/../../../hw-libs/printf"
DEST_DIR="${SCRIPT_DIR}/src/ISsdk"
DEST_CORE_DIR="${DEST_DIR}/core"
DEST_PRINTF_DIR="${SCRIPT_DIR}/src/hw-libs/printf"

SDK_FILES=(
  data_sets.c
  data_sets.h
  ISComm.h
  ISComm.c
  ISConstants.h
  rtk_defines.h
)

CORE_FILES=(
  types.h
  base_port.h
  msg_logger.h
  msg_logger.c
)

PRINTF_FILES=(
  printf.c
  printf.h
)

mkdir -p "${DEST_DIR}" "${DEST_CORE_DIR}" "${DEST_PRINTF_DIR}"

for file in "${SDK_FILES[@]}"; do
  cp "${SDK_DIR}/${file}" "${DEST_DIR}/"
done

for file in "${CORE_FILES[@]}"; do
  cp "${SDK_DIR}/core/${file}" "${DEST_CORE_DIR}/"
done

for file in "${PRINTF_FILES[@]}"; do
  cp "${PRINTF_DIR}/${file}" "${DEST_PRINTF_DIR}/"
done

# Arduino resolves includes relative to the library root. The copied core headers
# live together under src/ISsdk/core, while ISConstants.h lives one directory up.
perl -0pi -e 's|#include "core/types\.h"|#include "types.h"|g; s|#include "core/msg_logger\.h"|#include "msg_logger.h"|g; s|#include "ISConstants\.h"|#include "../ISConstants.h"|g' \
  "${DEST_CORE_DIR}/base_port.h"

perl -0pi -e 's|#include "ISConstants\.h"|#include "../ISConstants.h"|g' \
  "${DEST_CORE_DIR}/msg_logger.h"

perl -0pi -e 's|^#include "luna_data_sets\.h"\n||mg; s|^\s*evb_luna_velocity_control_t wheelController;\n||mg' \
  "${DEST_DIR}/data_sets.h"

awk '
  BEGIN { inserted = 0 }
  /^#else$/ && !inserted {
    print "#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32C3)"
    print "    #define PLATFORM_IS_EMBEDDED 1"
    print "    #define PLATFORM_IS_ARM 0"
    print "    #define CPU_IS_LITTLE_ENDIAN 1"
    print "    #define CPU_IS_BIG_ENDIAN 0"
    inserted = 1
  }
  { print }
' "${DEST_DIR}/ISConstants.h" > "${DEST_DIR}/ISConstants.h.tmp"
mv "${DEST_DIR}/ISConstants.h.tmp" "${DEST_DIR}/ISConstants.h"

perl -0pi -e 's|#include "printf.h"|#include "../hw-libs/printf/printf.h"|g' \
  "${DEST_DIR}/ISConstants.h"

rm -f "${DEST_DIR}/luna_data_sets.h"
