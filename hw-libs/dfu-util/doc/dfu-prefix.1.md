% DFU-PREFIX(1) dfu-prefix 0.10
% See AUTHORS file in source
% November 2020

# NAME
dfu-prefix - add, check, or remove special firmware file prefix

# SYNOPSIS
**dfu-prefix** [ **\-s** *address* | **\-L** ] **\--add** *DFU_FILE*\
**dfu-prefix** [ **\-T** | **\-L** ] **\--check** *DFU_FILE*\
**dfu-prefix** [ **\-T** | **\-L** ] **\--delete** *DFU_FILE*\
**dfu-prefix** **\--help**\
**dfu-prefix** **\--version**

# DESCRIPTION
The program **dfu-prefix** can be used to add, check or remove a prefix
used by certain hardware manufacturers. The Stellaris format from TI
and the LPC format from NXP is supported.

Note that a standard DFU firmware file has no concept of a prefix, and
a DFU host tool like dfu-util passes the prefix on to the device as
part of the normal firmware payload.

# OPTIONS
-s, \--stellaris-address *address*
: (in combination with \--add) Add TI Stellaris address prefix to file

-T, \--stellaris
: (in combination with \--delete or \--check) Act on TI Stellaris address prefix of file

-L, \--lpc-prefix
: (in combination with \--add or \--delete or \--check) Use NXP LPC DFU prefix format

-h, \--help
: Displays a help message.

-V, \--version
: Displays the software version.

# EXAMPLES
**dfu-prefix** \--stellaris-address 0x0100 \--add firmware.dfu
: Adds a Stellaris prefix with load address 0x0100

**dfu-prefix** \--stellaris \--check firmware.dfu
: Checks the file firmware.dfu for a Stellaris prefix

**dfu-prefix** \--lpc-prefix \--delete firmware.dfu
: Removes a LPC prefix from the file firmware.dfu

# EXIT VALUES
**0**
: Success (also if prefix is missing)

**-64**
: Usage error

# BUGS
https://sourceforge.net/p/dfu-util/tickets/

# COPYRIGHT
License GPLv2: GNU GPL version 2

# SEE ALSO
**dfu-suffix**(1), **dfu-util**(1)
