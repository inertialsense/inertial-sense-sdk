/**
 * @file gpx_flash_cfg.h
 * @brief Mapping of DID_GPX_FLASH_CFG (gpx_flash_cfg_t) to the domains that hold its values: CONFIG_SYSTEM,
 *        CONFIG_PORTS, CONFIG_GNSS. A domain whose keys live in several structs is read and written through each of
 *        their mappings.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_GPX_FLASH_CFG_H
#define CBOR_DID_GPX_FLASH_CFG_H

#include <stdbool.h>
#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

void cbor_gpx_flash_cfg_to_config_system(cbor_config_system_t *view, const gpx_flash_cfg_t *src);
bool cbor_gpx_flash_cfg_fits_config_system(const cbor_config_system_t *view);
bool cbor_gpx_flash_cfg_from_config_system(gpx_flash_cfg_t *dst, const cbor_config_system_t *view);

void cbor_gpx_flash_cfg_to_config_ports(cbor_config_ports_t *view, const gpx_flash_cfg_t *src);
bool cbor_gpx_flash_cfg_fits_config_ports(const cbor_config_ports_t *view);
bool cbor_gpx_flash_cfg_from_config_ports(gpx_flash_cfg_t *dst, const cbor_config_ports_t *view);

void cbor_gpx_flash_cfg_to_config_gnss(cbor_config_gnss_t *view, const gpx_flash_cfg_t *src);
bool cbor_gpx_flash_cfg_fits_config_gnss(const cbor_config_gnss_t *view);
bool cbor_gpx_flash_cfg_from_config_gnss(gpx_flash_cfg_t *dst, const cbor_config_gnss_t *view);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_GPX_FLASH_CFG_H */
