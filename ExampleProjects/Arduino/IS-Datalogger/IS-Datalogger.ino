#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 1024

// Enable this if you want to use the USB port for debug/status info... However, the logger will not start until the port is available and connected.
// IE, DO NOT ENABLE THIS if you intent to log in a completely stand-alone way.  For most cases, you shouldn't need this enabled.
// #define DEBUG_LOGGING
#ifdef DEBUG_LOGGING
    #define LOG_DBG(...)    { Serial.printf(__VA_ARGS__); Serial.println(); }
#else
    #define LOG_DBG(...)
#endif


#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#include "ISComm.h"

const int PUSH_BUTTON = A0;
const int LED_RED = LED_BUILTIN;
const int LED_GREEN = 8;
const int SD_CARD_DETECT = 7;
const int SD_CHIP_SELECT = 4;
const int MAX_BUFF_LEN = (SERIAL_RX_BUFFER_SIZE * 10);
const int MAX_LOG_SIZE = 4096000;

const char* log_config = "_logger.cfg";
const char* log_meta = "_last.mta";

int log_num = 0;                // current log number
int file_num = 0;               // current file number (in this log)
uint32_t file_size = 0;         // current size of current file
uint32_t last_report_size = 0;  // number of bytes logged since last report
uint32_t next_report = 0;       // next time to report on the status (over USB)
uint32_t next_led = 0;          // next time to blink the LED
uint32_t next_rmc_send = 0;     // next time the rmc config will be sent

int button_state;               // last known state of the button
uint32_t last_hold = 0;         // time when the button was last pressed
uint32_t last_release = 0;      // time when the button was last released
uint32_t hold_duration = 0;     // duration the button was held

File logFile;                   // the current log file, if any (you can test this, ie: if (logFile) { ... } to know if we should be logging)

struct buffer_s {
  char data[MAX_BUFF_LEN];      // actual data waiting to be written to SD
  size_t len;                   // length of data waiting to be written
  bool pending;                 // if this buffer is waiting to write
} buffer[2] ;
int cur_buff = 0;               // which of our current buffers we are writing to (currently, we don't use this)

struct config_s {
  rmc_t rmc;                    // configured RMC bits/options which are sent to the device when logging is requested
  uint32_t max_file_size;       // the maximum size of each log file before a new one is created
  bool log_on_startup;          // flag indicating if we should log on startup.
} config;

// this is a temporary buffer used when copying data out from the UART into the working buffer (buffer_s.data)
// this could be a local variable, but we don't want to keep putting it on the stack everytime loop() is called
char ser_buf[SERIAL_RX_BUFFER_SIZE];


/**
 * Utility function to format a uint64_t as a hexadecimal string (apparently Arduino doesn't support printf("%llX"))
 */
const char *uint64_to_hexstr(uint64_t v) {
  static char tmp[17];
  sprintf(&tmp[0], "%08X", (uint32_t)((v >> 32) & 0xFFFFFFFF));
  sprintf(&tmp[8], "%08X", (uint32_t)(v & 0xFFFFFFFF));
  return (const char *)tmp;
}

/**
 * Utility function to parse a hexidecimal string into (or upto?) a uint64_t. Because, apparently
 * the JSON standard doesn't support hexadecimal numbers.  Lame, I know.
 */ 
uint64_t getJsonHexParam(JsonDocument& doc, const String& key, uint64_t def = 0) {
  if (doc.containsKey(key)) {
    if (doc[key].is<String>()) {
      String val = doc[key].as<String>();
      return strtoull(val.c_str(), 0, val.startsWith("0x") ? 16 : 10);
    } else {
      return doc[key];
    }
  }

  return def;
}

/**
 * Reads and parses the _logger.cfg JSON file from the SD card, or loads default values
 * if the SD card isn't available, the file doesn't exist, or doesn't contain those parameters.
 */
void readConfig() {
  config.rmc.bits = RMC_PRESET_IMX_PPD;
  config.rmc.options = 0x0;
  config.max_file_size = 4096000;
  config.log_on_startup = false;

  if (!SD.exists(log_config))
    return;

  JsonDocument jsonConfig;
  File cfgFile = SD.open(log_config, FILE_READ);
  if (cfgFile) {
    deserializeJson(jsonConfig, cfgFile.readString());
    config.rmc.bits = getJsonHexParam(jsonConfig, "rmc_bits", 0xc000009001353ce2);
    LOG_DBG(" Configured 'rmc_bits' : 0x%s", uint64_to_hexstr(config.rmc.bits));

    config.rmc.options = getJsonHexParam(jsonConfig, "rmc_options", 0x0);
    LOG_DBG(" Configured 'rmc_options' : 0x%08X", config.rmc.options);

    if (jsonConfig.containsKey("max_file_size")) {
      config.max_file_size = jsonConfig["max_file_size"].as<uint32_t>();
      LOG_DBG(" Configured 'max_file_size' : %lu", config.max_file_size);
    }

    if (jsonConfig.containsKey("log_on_startup")) {
      config.log_on_startup = jsonConfig["log_on_startup"].as<bool>();
      LOG_DBG(" Configured 'log_on_startup' : %s", config.log_on_startup ? "true" : "false");
    }
  }
}

/**
 * Send the DID_RMC with the specified bits/options, to the connected device.
 * This is used to start/stop streaming of requested DIDs to be logged.
 * Sending 0x0 and 0x0 (param defaults) will turn off streaming.
 */
void stream_configure_PPD(uint64_t bits = 0, uint32_t options = 0) {
  is_comm_instance_t commTx = {};
  uint8_t	buf[64] = { 0 };

  rmc_t rmc;
  rmc.bits = bits;
  rmc.options = options;

  int len = is_comm_data_to_buf(buf, sizeof(buf), &commTx, DID_RMC, sizeof(rmc_t), 0, (void*)&rmc);
  Serial1.write(buf, len);
}

/**
 * Returns the number of the last log generated (as read from the _last.mta metadata file).
 * Use writeLogNumber() to update the metadata file, if you change the log number.
 */
int getLastLogNumber() { 
  File logMeta = SD.open(log_meta, FILE_READ);
  if (logMeta)
    log_num = logMeta.parseInt(SKIP_WHITESPACE);
  else
    log_num = 0;
  logMeta.close();
  return log_num;
}

/**
 * Writes/saves the specified log number to the _last.mta metadata file.
 * Use this to record what the current log is, so can persist and increment the log number
 * after a reset, etc.
 */
int writeLogNumber(int log_num) { 
  File logMeta = SD.open(log_meta, O_WRITE | O_CREAT);
  logMeta.print(log_num);
  logMeta.close();
  return log_num;
}

/**
 * Creates a new log directory, and log file (deleting/overwritting if necessary). This will automatically
 * increment the file number, but not the log number. It will create the parent log_#### directory is necessary.
 * This always returns a File object, which if valid, means the system is ready to start accepting and saving
 * data. Use `if (file) { ... }` to test if the file is valid/open and ready for logging.
 */
File openLog() {
  char dirName[16];
  char logName[64];
  
  sprintf(dirName, "log_%04d", log_num);
  sprintf(logName, "%s/%08d.raw", dirName, file_num++);

  if (!SD.exists(dirName))
    SD.mkdir(dirName);

  if (SD.exists(logName))
    SD.remove(logName); // we don't ever append to an existing file... we always delete and start over

  LOG_DBG("Starting new log: %s", logName);

  file_size = 0;
  return SD.open(logName, O_WRITE | O_CREAT);
}

/*
 * Writes data to the current file on the SD card, but only in chunks (sectors) when a sector is ready
 * to be written. If its not ready to write, this copies the data into one of our buffers until it can
 * be written. This intends to prevent unnecessary blocking on the SD card, and attempts to perform
 * writes in an optimized way.  Returns the number of bytes actually written
 */
int writeData(File& file, char *data, size_t len) {

  // first, we need to make sure this chunk of data won't overflow our internal buffer
  if ((buffer[cur_buff].len + len) > MAX_BUFF_LEN) {
    LOG_DBG("TOO MUCH DATA (%d, %d). Possible Buffer Overflow/Data Loss.", buffer[cur_buff].len, len);

    // if so, then we'll force a write (even if its not optimized)
    digitalWrite(LED_GREEN, LOW);
    int written = file.write(buffer[cur_buff].data, buffer[cur_buff].len);
    file_size += written;
    last_report_size += written;
    digitalWrite(LED_GREEN, HIGH);

    // Check to see if we have data remaining in the buffer (did we actually write everything successfully?)
    if (written == buffer[cur_buff].len) {
      buffer[cur_buff].len = 0;
    } else {
      // And, if not, then move the remaining data to the front of the buffer.
      LOG_DBG("%d bytes unsent.", buffer[cur_buff].len - written);
      memmove(buffer[cur_buff].data, &buffer[cur_buff].data[written], written);
      buffer[cur_buff].len -= written;
    }
  }

  // Append our new data to our internal write buffer
  memcpy((void *)&(buffer[cur_buff].data[buffer[cur_buff].len]), (void *)data, len);
  buffer[cur_buff].len += len;

  // check if the SD card is available to write data without blocking
  // and if the buffered data is enough for the full chunk size
  unsigned int chunkSize = file.availableForWrite();
  if (chunkSize && (buffer[cur_buff].len >= chunkSize)) {

    // write to file and blink LED
    digitalWrite(LED_GREEN, LOW);
    int written = file.write(buffer[cur_buff].data, buffer[cur_buff].len);
    file_size += written;
    last_report_size += written;
    digitalWrite(LED_GREEN, HIGH);

    // Check to see if we have data remaining in the buffer (did we actually write everything successfully?)
    if (written == buffer[cur_buff].len) {
      buffer[cur_buff].len = 0;
    } else {
      // And, if not, then move the remaining data to the front of the buffer.
      LOG_DBG("%d bytes unsent.", buffer[cur_buff].len - written);
      memmove(buffer[cur_buff].data, &buffer[cur_buff].data[written], written);
      buffer[cur_buff].len -= written;
    }
    return written;
  }

  return 0;
}

/*
 * writes data to SD, but only in chunks (sectors) when a sector is ready to be written.
 * if its not, this copies the data into one of our buffers until it can be written
 */
int fakeWriteData(File& file, char *data, size_t len) {
  // check if the SD card is available to write data without blocking
  // and if the buffered data is enough for the full chunk size

  // memcpy((void *)&(buffer[cur_buff].data[buffer[cur_buff].len]), (void *)data, len);
  buffer[cur_buff].len += len;

  digitalWrite(LED_GREEN, HIGH);
  file_size += len;
  last_report_size += len;
  digitalWrite(LED_GREEN, LOW);

  buffer[cur_buff].len = 0;
  return len;
}

/**
 * Stop logging; closes the current log file, and deconfigures the target IMX/GPX to stop broadcasting
 */
void stopLogging() {
  logFile.close();
  stream_configure_PPD(0, 0);
  LOG_DBG("Logger stopped.");
}

/**
 * Start logging; Initializes SD card (and verifies a card is installed), reads config and metadata to detemine log number,
 * and RMC bit/options, increments the log number, starts streaming the RMC data, and then open the current log file on
 * the SD card.
 */
bool startLogging() {
  // we can't log if the SD card driver/status isn't happy
  LOG_DBG("Initializing SD card...");
  if (!SD.begin(SD_CHIP_SELECT)) {
    LOG_DBG("Card failed, or not present. Unable to start logging.");
    return false;
  }
  LOG_DBG("Card initialized.");

  // read the config, and increment the last log number
  readConfig();
  log_num = getLastLogNumber();
  writeLogNumber(++log_num);

  // first configure the data stream
  stream_configure_PPD(config.rmc.bits, config.rmc.options);

  // init the things, and off we go.
  file_num = file_size = 0;
  next_led = next_rmc_send = 0;
  next_report = millis() + 5000;
  logFile = openLog();

  LOG_DBG("Logger started.");
  return true;
}

/**
 * Toggle Logging; stops the logger if its already logging, otherwise starts it.
 */
void toggleLogging() {
  if (logFile) {
    stopLogging();
  } else {
    startLogging();
  }
}

/**
 * Simple check if the SD card's detect line is detecting an inserted card
 */
bool isSDCardDetected() {
  return (digitalRead(SD_CARD_DETECT) ? true : false);
}


/**
 * Arduino setup(); configures GPIO and Peripherals (Serial). If SD card is inserted, it will initialize it,
 * and read the config. If configured, it will start logging is able.
 */
void setup() {
  pinMode(LED_RED, OUTPUT);                 // configure the red LED (UART status/power)
  pinMode(LED_GREEN, OUTPUT);               // configure the green LED (SD status)
  pinMode(SD_CARD_DETECT, INPUT_PULLUP);    // configure the SD card detect line
  pinMode(PUSH_BUTTON, INPUT_PULLUP);       // configure the push button

  #ifdef DEBUG_LOGGING
    // Open serial communications and wait for port to open:
    Serial.begin(921600);
    while (!Serial); // wait for serial port to connect. Needed for native USB port only
    LOG_DBG("Initializing...");
  #endif

  Serial1.begin(921600);
  while (!Serial1); // wait for serial port to connect. Needed for native USB port only

  if (isSDCardDetected()) {
    LOG_DBG("SD card detected. Reading config.");
    if (SD.begin(SD_CHIP_SELECT))
      readConfig();
  }

  if (config.log_on_startup)
    startLogging();

  last_hold = millis();
  button_state = digitalRead(PUSH_BUTTON);
  next_report = millis() + 5000;
}


/**
 * Classic Ardiuino loop(). Do the most important stuff here (it's all important).
 */
void loop() {
  uint32_t now = millis();
  digitalWrite(LED_GREEN, isSDCardDetected());

  // check the status of our push-button
  // we only register debounced press events of longer that 1200ms (intentionally long press)
  int cur_button = digitalRead(PUSH_BUTTON);
  if (cur_button == LOW) {
    if (cur_button != button_state) {
      last_hold = now;
    }
  } else {
    if (cur_button != button_state) {
      last_release = now;
      LOG_DBG("Button Released (%dms hold).", now - last_hold);
      if (now - last_hold > 1200) {
        toggleLogging();
      }
    }
  }
  button_state = cur_button;

  // check for available data...
  int bytesToRead = Serial1.available();
  while (bytesToRead) {
    // note that we will loop here as long as data is available.  Reading/buffering all available data is our top priority

    // blink off out RED led, to indicate received data, but only if logging
    // NOTE that a lack of RX data is indicated by SOLID RED (it blink off when received)
    if (logFile)
      digitalWrite(LED_RED, LOW);

    // actually read the data
    int bytesRead = Serial1.readBytes(ser_buf, bytesToRead);
    file_size += bytesRead;
    last_report_size += bytesRead;

    // if we aren't logging, we throw away the data...
    // if we ARE logging, then we'll start buffering/writing to the SD card
    if (logFile) { 
      // but turn back ON our RED led
      digitalWrite(LED_RED, HIGH);
      
      // buffer data (and we'll blink our GREEN led)
      writeData(logFile, ser_buf, bytesRead);

      // if we exceeded our file size, then close this file and start the next one.
      if (file_size > config.max_file_size) {
        logFile.close();
        logFile = openLog();
      }
      next_led = now; // we just blinked the LED; don't do it again... 
    }
    // check if there is any new data, since our last check
    bytesToRead = Serial1.available();
  }

  // This blinks (briefly on) the RED led every second, when we are NOT logging
  if (!logFile && (now > next_led)) {
    if (digitalRead(LED_RED) == LOW) {
      digitalWrite(LED_RED, HIGH);
      next_led = now + 50;
    } else {
      digitalWrite(LED_RED, LOW);
      next_led = now + 950;
    }
  }

  // This provides a status report on the USB's Serial (when enabled) of our status/data rate.
  if (now > next_report) {
    LOG_DBG("%s %lu bytes (%0.1f KB/s)...", (logFile ? "Logged" : "Received"), last_report_size, last_report_size / 5000.0f);
    last_report_size = 0;
    next_report = now + 5000;
  }

  // This triggers a new RMC request to the connected device every 60 seconds (when logging).
  // This generally ensures that we continue to receive the data that we need in the event that
  // something else might deconfigure the RMC bits.
  if (logFile && (now > next_rmc_send)) {
    stream_configure_PPD(config.rmc.bits, config.rmc.options);    
    next_rmc_send = now + 60000;    
  }
}
