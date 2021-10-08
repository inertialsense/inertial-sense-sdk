
#ifndef DFU_FILE_H
#define DFU_FILE_H

#include <stdint.h>

#include "dfu.h"
#include "../../../src/uins_types.h"

void dfu_load_file(struct dfu_file *file, enum suffix_req check_suffix, enum prefix_req check_prefix, struct dfu_config* config);
void dfu_store_file(struct dfu_file *file, int write_suffix, int write_prefix);

void dfu_progress_bar(uins_device_context* context, const char *desc, unsigned long long curr, unsigned long long max);

void *dfu_malloc(size_t size);
uint32_t dfu_file_write_crc(int f, uint32_t crc, const void *buf, int size);
void show_suffix_and_prefix(struct dfu_file *file);

#endif /* DFU_FILE_H */
