% DFU-SUFFIX(1) dfu-suffix 0.10
% See AUTHORS file in source
% November 2020

# NAME
dfu-suffix - add, check, or remove DFU firmware file suffix

# SYNOPSIS
**dfu-suffix** [*options*] **\--add** *DFU_FILE*\
**dfu-suffix** **\--check** *DFU_FILE*\
**dfu-suffix** **\--delete** *DFU_FILE*\
**dfu-suffix** **\--help**\
**dfu-suffix** **\--version**

# DESCRIPTION
The program **dfu-suffix** can be used to add, check or remove a DFU firmware file suffix,
recommended for safely matching a firmware file and device.

Note that a suffix is recommended by the DFU standard, but not required.
A DFU host tool like dfu-util will recognize the suffix and use it to check
that the device is matching, but not transfer the suffix to the device.

# OPTIONS
-v, \--vid *vendorID*
: Specify USB vendor ID (hexadecimal)

-p, \--pid *productID*
: Specify USB product ID (hexadecimal)

-d, \--did *deviceID*
: Specify USB device ID (hexadecimal)

-S, \--spec *version*
: Specify DFU specification version (hexadecimal)

-h, \--help
: Displays a help message.

-V, \--version
: Displays the software version.

# EXAMPLES
**dfu-suffix** \--vid 0123 \--add firmware.dfu
: Adds a suffix matching vendor 0x0123 and product ID 0x4567.
Since product and device ID are not specified,
they will contain the wildcard value 0xFFFF.

**dfu-suffix** \--check firmware.dfu
: Checks the file firmware.dfu for a valid DFU suffix

**dfu-suffix** \--delete firmware.dfu
: Removes a valid DFU suffix from the file firmware.dfu

# EXIT VALUES
**0**
: Success (also if suffix is missing)

**-64**
: Usage error

# LIMITATIONS
**dfu-suffix** can not tell a broken DFU suffix (e.g. checksum mismatch)
from a non-existing suffix, so only a valid suffix can be removed.

# BUGS
https://sourceforge.net/p/dfu-util/tickets/

# COPYRIGHT
License GPLv2: GNU GPL version 2

# SEE ALSO
**dfu-prefix**(1), **dfu-util**(1)
