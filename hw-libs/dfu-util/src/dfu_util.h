#ifndef DFU_UTIL_H
#define DFU_UTIL_H

/* USB string descriptor should contain max 126 UTF-16 characters
 * but 254 would even accommodate a UTF-8 encoding + NUL terminator */
#define MAX_DESC_STR_LEN 254

enum mode {
	MODE_NONE,
	MODE_VERSION,
	MODE_LIST,
	MODE_DETACH,
	MODE_UPLOAD,
	MODE_DOWNLOAD
};

void probe_devices(libusb_context* ctx, struct dfu_config* config);
void disconnect_devices(struct dfu_config* config);
void print_dfu_if(struct dfu_if *);
void list_dfu_interfaces(struct dfu_config* config);

#endif /* DFU_UTIL_H */
