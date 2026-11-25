/**
 * @file flash_config.h
 * @brief Mapping of DID_FLASH_CONFIG (nvm_flash_cfg_t) to the domains that hold its values: CONFIG_SYSTEM, CONFIG_INS,
 *        CONFIG_MAG, CONFIG_GROUND_VEHICLE, CONFIG_PORTS, CONFIG_SENSORS, CONFIG_IO, CONFIG_GNSS, STATE. A domain whose
 *        keys live in several structs is read and written through each of their mappings.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_FLASH_CONFIG_H
#define CBOR_DID_FLASH_CONFIG_H

#include <stdbool.h>
#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

void cbor_flash_config_to_config_system(cbor_config_system_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_system(const cbor_config_system_t *view);
bool cbor_flash_config_from_config_system(nvm_flash_cfg_t *dst, const cbor_config_system_t *view);

void cbor_flash_config_to_config_ins(cbor_config_ins_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_ins(const cbor_config_ins_t *view);
bool cbor_flash_config_from_config_ins(nvm_flash_cfg_t *dst, const cbor_config_ins_t *view);

void cbor_flash_config_to_config_mag(cbor_config_mag_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_mag(const cbor_config_mag_t *view);
bool cbor_flash_config_from_config_mag(nvm_flash_cfg_t *dst, const cbor_config_mag_t *view);

void cbor_flash_config_to_config_ground_vehicle(cbor_config_ground_vehicle_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_ground_vehicle(const cbor_config_ground_vehicle_t *view);
bool cbor_flash_config_from_config_ground_vehicle(nvm_flash_cfg_t *dst, const cbor_config_ground_vehicle_t *view);

void cbor_flash_config_to_config_ports(cbor_config_ports_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_ports(const cbor_config_ports_t *view);
bool cbor_flash_config_from_config_ports(nvm_flash_cfg_t *dst, const cbor_config_ports_t *view);

void cbor_flash_config_to_config_sensors(cbor_config_sensors_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_sensors(const cbor_config_sensors_t *view);
bool cbor_flash_config_from_config_sensors(nvm_flash_cfg_t *dst, const cbor_config_sensors_t *view);

void cbor_flash_config_to_config_io(cbor_config_io_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_io(const cbor_config_io_t *view);
bool cbor_flash_config_from_config_io(nvm_flash_cfg_t *dst, const cbor_config_io_t *view);

void cbor_flash_config_to_config_gnss(cbor_config_gnss_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_config_gnss(const cbor_config_gnss_t *view);
bool cbor_flash_config_from_config_gnss(nvm_flash_cfg_t *dst, const cbor_config_gnss_t *view);

void cbor_flash_config_to_state(cbor_state_t *view, const nvm_flash_cfg_t *src);
bool cbor_flash_config_fits_state(const cbor_state_t *view);
bool cbor_flash_config_from_state(nvm_flash_cfg_t *dst, const cbor_state_t *view);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_FLASH_CONFIG_H */
