#pragma once

struct dfu_status {
    unsigned char bStatus;
    unsigned int  bwPollTimeout;
    unsigned char bState;
    unsigned char iString;
};

struct dfu_if {
    struct usb_dfu_func_descriptor func_dfu;
    uint16_t quirks;
    uint16_t busnum;
    uint16_t devnum;
    uint16_t vendor;
    uint16_t product;
    uint16_t bcdDevice;
    uint8_t configuration;
    uint8_t interface;
    uint8_t altsetting;
    uint8_t flags;
    uint8_t bMaxPacketSize0;
    char *alt_name;
    char *serial_name;
    libusb_device *dev;
    libusb_device_handle *dev_handle;
    struct dfu_if *next;
    struct memsegment *mem_layout; /* for DfuSe */
};

struct dfu_config {
    int verbose;
    struct dfu_if *dfu_root;
    char *match_path;
    int match_vendor;
    int match_product;
    int match_vendor_dfu;
    int match_product_dfu;
    int match_config_index;
    int match_iface_index;
    int match_iface_alt_index;
    int match_devnum;
    const char *match_iface_alt_name;
    const char *match_serial;
    const char *match_serial_dfu;
    unsigned int last_erased_page;
    unsigned int dfuse_address;
    unsigned int dfuse_address_present;
    unsigned int dfuse_length;
    int dfuse_force;
    int dfuse_leave;
    int dfuse_unprotect;
    int dfuse_mass_erase;
    int dfuse_will_reset;
};

struct dfu_file {
    /* File name */
    const char *name;
    /* Pointer to file loaded into memory */
    uint8_t *firmware;
    /* Different sizes */
    struct {
	off_t total;
	int prefix;
	int suffix;
    } size;
    /* From prefix fields */
    uint32_t lmdfu_address;
    /* From prefix fields */
    uint32_t prefix_type;

    /* From DFU suffix fields */
    uint32_t dwCRC;
    uint16_t bcdDFU;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
};

enum suffix_req {
	NO_SUFFIX,
	NEEDS_SUFFIX,
	MAYBE_SUFFIX
};

enum prefix_req {
	NO_PREFIX,
	NEEDS_PREFIX,
	MAYBE_PREFIX
};

enum prefix_type {
	ZERO_PREFIX,
	LMDFU_PREFIX,
	LPCDFU_UNENCRYPTED_PREFIX
};

struct memsegment {
	unsigned int start;
	unsigned int end;
	int pagesize;
	int memtype;
	struct memsegment *next;
};

