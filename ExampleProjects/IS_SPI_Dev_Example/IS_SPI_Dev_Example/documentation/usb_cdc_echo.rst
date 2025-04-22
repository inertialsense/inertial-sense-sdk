============
USB CDC ECHO
============

This demo behaves like a virtual COM port based on USD device CDC. It echoes
back bytes received from the console.

Drivers
-------
* USB Device

Supported Evaluation Kit
------------------------
* SAM D21 Xplained Pro
* SAM DA1 Xplained Pro
* SAM R21 Xplained Pro
* SAM D11 Xplained Pro
* SAM L21 Xplained Pro
* SAM L22 Xplained Pro
* SAM V71 Xplained Ultra
* SAM E70 Xplained
* SAM G55 Xplained Pro
* SAM E54 Xplained Pro

Interface Settings
------------------
* No extra board or wire connection is needed.
* The enumerated serial port does not bridge to any specific USRT hardware but
  simply loopback data, so whatever serial port setting works.

Running the Demo
----------------

1. Download the selected example, or export the example to save the .atzip file.
2. Import .atzip file into Atmel Studio 7, File->Import->Atmel Start Project.
3. Build and flash into supported evaluation board.
4. Connect PC host and TARGET USB plug with a USB cable.
5. Press the RESET button.
6. The serial port is detected by host PC. On Linux the driver will be installed
   automatically. On Windows The INF and CAT file required can be extracted from
   .atzip or found in generated Atmel Studio 7 project folder, relatively at
   ./usb/class/cdc/device.
7. Open the detected/enumerated serial port using a serial port monitor
   (e.g., TeraTerm).
8. Whatever sent from the serial port monitor will be echoed back then.
