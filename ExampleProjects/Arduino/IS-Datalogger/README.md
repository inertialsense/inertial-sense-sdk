# IS-DataLogger

This project is a lightweight, and generally fairly generic serial-data logger that
uses a minimal implementation of the SDK to configure RMC options to stream DIDs over
a connected UART interface on the IMX/GPX (generally Ser2), and then writes the raw
UART data to the SD card, for post-processing. This is similar to collecting PPD logs
through the EvalTool, but instead collects raw data.

This project is intended to be ran on an Adafruit Feather M0 "Adalogger", which has
an integrated SD Card.  It may run on other Arduio devices that have an SD card
configured on the default pins.

Additionally, the Adalogger has an additional GPIO (pin "8") assigned for a second
(green) LED, in addition to a GPIO (pin "7") used for a SD Card detect pin.

## Hookup

Connect the IMX/GPX Ser2 (or any available UART line) to the Arduino/Feather's D0 (TX)
and D1 (RX) pins. Share the ground, and provide power (both the Feather and the IMX are
3.3v, so that should be easy).

Finally, optionally (but _**strongly recommended**_), attach a push-button of your choice between GND
and "A0". This button will start/stop logging, but you can also configure logging to
start automatically.


## Building/Compiling

This is a standard Arduino INO project, and can be opened directly by the Arduino IDE (> 2.x).

**NOTE: Before you do anything else, please be sure to run the `ImportSdkFiles.bat`_(Windows)_ or `ImportSdkFiles.sh`_(Mac/Linux)_ from this directory.
This will copy the necessary header/source files from the parent SDK directory into this directory, so the Arduino IDE
can properly load this as a sketch.**

This project requires on a pair of standard Arduino Libraries, which are both available from
the Arduino IDE's "Library Manager".

 - SD >v1.2.4 (Arduino/SparkFun)
 - ArduinoJson >v7.0.3 (Benoit Blanchon)

After import the project and configuring the libraries, be sure to configure the Board
in the IDE.  If you are using the Adafruit Adalogger, you will need to install the Adafruit
boards package.  You can find these instructions on the Adafruit website; this process is
very well documented there.

After the board package is installed, and configured, connect the board to the PC using
the USB connection, and then "Build and Upload" to the device.

## Preparing the SD Card

At this point, you should find a microSD card, ideally Class 10, and minimum 8GB. 32GB or
64GB sizes are recommended.  The SD card needs to be formatted for FAT16, FAT32.

After formatting, you can create a configuration file (below) to customize the behavior, but
it is not required. You can insert the SD card now, if you want to use the default settings.

## Configuration

IS-DataLogger will read a Json file '_logger.cfg' from the root directory of the SD card.
This simple file defines parameters regarding RMC bit (DIDs to stream) and RMC options
(flags about how to stream those DIDs), as well as maximum file-size, etc.

_logger.cfg:
```json
{
  rmc_bits: '0xc000009001353cf2',
  rmc_options: '0x0',
  max_file_size: 4096000,
  log_on_startup: true,
}
```
`rmc_bits` - defines which DIDs to stream to the logger.
The value here (the default) will stream the same DIDs as the "PPD + Raw IMU" preset found
in the EvalTool.

`rmc_options` - defines which flags/options to apply to the request, these can be found
in the data_sets.h, `RMC_OPTION_*` #defines.

`max_file_size` - defines the maximum size of each log file.  Anytime a log file is exceeds
this size, a new log file will be created with an incremented file name.

`log_on_startup` - determines whether logging will start automatically after power-on.
If `true` logging is started automatically.  If `false` logging must be manually started
by pressing the push-button for 1.5 seconds, or longer (this "long press" is by design,
to prevent accidental start/stop).


## Operation

Operation is pretty simple, as there is a minimal user-interface (LEDs and 1 button).

### Checklist

- microSD card (Class 10), formatted for FAT16 or FAT32.
- Insert microSD card into the SD card slot.
- Provide Power to the Logger (Adalogger).  It may take 2-3 seconds before the logger initializes completely.
- If the SD card is dectect correctly, the GREEN LED will illuminate.  If the green LED does not light up, please remove and re-insert the SD card).
- Once initialized, if IDLE (not logging) the RED LED will blink briefly, once per second.
- Press and Hold the push-button for 2 seconds, then release.
- The RED LED should begin flickering (the LED indicates received data and will "flicker" as data is received). NOTE that the default (no traffic) state of the LED is SOLID, and blinks off as data is received. The more data received, the more the LED will appear to dim.
- When you wish the finish logging, it is recommended that you "Press and Hold" the push-button for 2 seconds, and then release, to disable streaming, and stop logging. Once the logs are properly closed, the red LED will return to its IDLE (blinking) state.

IMPORTANT NOTE: Turning the device off without first stopping the logger risks data loss and (though rare) SD card corruption. Sometimes you can't avoid a power-off, and its generally OK, but we strongly encourage stopping the logger before you power off or remove the SD Card.