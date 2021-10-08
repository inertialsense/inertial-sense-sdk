#ifndef DFU_LOAD_H
#define DFU_LOAD_H

#include "../../../src/uins_types.h"

int dfuload_do_upload(uins_device_context* context, struct dfu_config* config, struct dfu_if *dif, int xfer_size, int expected_size, int fd);
int dfuload_do_dnload(uins_device_context* context, struct dfu_config* config, struct dfu_if *dif, int xfer_size, struct dfu_file *file);

#endif /* DFU_LOAD_H */
