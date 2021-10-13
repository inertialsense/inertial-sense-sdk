#ifndef DFU_LOAD_H
#define DFU_LOAD_H

#include "../../../src/uins_types.h"

int dfuload_do_upload(const uins_device_context const * context, struct dfu_config* config, struct dfu_if *dif, int xfer_size, int expected_size, int fd);
int dfuload_do_dnload(const uins_device_context const * context, struct dfu_config* config, struct dfu_if *dif, int xfer_size, struct dfu_file *file);

#endif /* DFU_LOAD_H */
