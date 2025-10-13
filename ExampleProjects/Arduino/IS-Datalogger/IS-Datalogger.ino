// ===================== Teensy 4.1 — GPS Serial1 -> SD Logger (robusto) =====================
// - Placa: Teensy 4.1
// - GPS por Serial1 (RX1=pin 0, TX1=pin 1). Compartir GND.
// - Guarda TODO lo recibido por Serial1 a la SD interna (BUILTIN_SDCARD, FAT32).
// - Cambios clave:
//    * DEBUG imprime por USB sin bloquear (timeout 2s)
//    * NO usa while(!Serial1) (podía colgar)
//    * Arranca logging al encender (configurable en _logger.cfg)
//    * Buffer + flush por tamaño/tiempo (no solo availableForWrite)
//    * Tipos size_t en escrituras y comprobaciones
//
// Activa/desactiva debug (USB):
#define DEBUG_LOGGING

#ifdef DEBUG_LOGGING
  #define LOGF(...) do { char _b[256]; snprintf(_b, sizeof(_b), __VA_ARGS__); Serial.println(_b); } while (0)
#else
  #define LOGF(...) do {} while (0)
#endif

// ===================== Buffers serie =====================
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 1024

#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

// Ruta típica en tu repo
#include "src/ISsdk/ISComm.h"

// ===================== Pines / HW =====================
const int PIN_BTN    = A0;
const int LED_RED    = LED_BUILTIN;  // 13
const int LED_GREEN  = 8;            // LED opcional (actividad SD)

// ===================== Parámetros =====================
const uint32_t BAUD_USB     = 921600;   // USB serial para imprimir
const uint32_t BAUD_SERIAL1 = 921600;   // GPS (ajústalo si hace falta)

const size_t   MAX_BUFF_LEN  = (SERIAL_RX_BUFFER_SIZE * 10); // 10 KB
const size_t   WRITE_CHUNK   = 4096;       // umbral para escribir a SD
const uint32_t FLUSH_MS      = 100;        // vaciado periódico si no se alcanzó umbral
const uint32_t REPORT_MS     = 5000;       // reporte por USB
const uint32_t RMC_MS        = 60000;      // reenvío de RMC

const char* log_config = "_logger.cfg";
const char* log_meta   = "_last.mta";

// ===================== Estado =====================
int      log_num = 0;
int      file_num = 0;
uint32_t file_size = 0;

uint32_t next_flush   = 0;
uint32_t next_report  = 0;
uint32_t next_rmc     = 0;
uint32_t next_blink   = 0;

uint32_t rx_bytes_period  = 0;  // recibidos por Serial1 (para informes)
uint32_t sd_bytes_period  = 0;  // realmente escritos a SD (para informes)

int      btn_state;
uint32_t press_start = 0;

File logFile;

struct buffer_s {
  char   data[MAX_BUFF_LEN];
  size_t len;
} buffer;

struct config_s {
  rmc_t    rmc;
  uint32_t max_file_size;
  bool     log_on_startup;
} config;

// buffers de trabajo
static char ser_buf[SERIAL_RX_BUFFER_SIZE];

// ===================== Utilidades =====================
static const char *uint64_to_hexstr(uint64_t v) {
  static char tmp[17];
  sprintf(&tmp[0], "%08X", (uint32_t)((v >> 32) & 0xFFFFFFFF));
  sprintf(&tmp[8], "%08X", (uint32_t)(v & 0xFFFFFFFF));
  return (const char *)tmp;
}

// ArduinoJson v6, sin containsKey()
static uint64_t getJsonHexParam(JsonDocument& doc, const char* key, uint64_t def = 0) {
  JsonVariant v = doc[key];
  if (v.isNull()) return def;
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    return strtoull(s, nullptr, (s[0]=='0' && (s[1]=='x' || s[1]=='X')) ? 16 : 10);
  }
  if (v.is<unsigned long long>()) return v.as<unsigned long long>();
  if (v.is<unsigned long>())      return (uint64_t)v.as<unsigned long>();
  if (v.is<uint64_t>())           return v.as<uint64_t>();
  if (v.is<uint32_t>())           return (uint64_t)v.as<uint32_t>();
  return def;
}

// ===================== Config =====================
static void readConfig() {
  // Defaults robustos
  config.rmc.bits        = RMC_PRESET_IMX_PPD_GROUND_VEHICLE;//RMC_PRESET_IMX_PPD; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  config.rmc.options     = 0x0;
  config.max_file_size   = 4096000;     // ~4 MB
  config.log_on_startup  = true;        // <-- Arranca grabando por defecto

  if (!SD.exists(log_config)) return;
  File cfgFile = SD.open(log_config, FILE_READ);
  if (!cfgFile) return;

  DynamicJsonDocument jsonConfig(512);
  DeserializationError err = deserializeJson(jsonConfig, cfgFile);
  cfgFile.close();
  if (err) {
    LOGF("Error JSON cfg: %s", err.c_str());
    return;
  }

  config.rmc.bits    = getJsonHexParam(jsonConfig, "rmc_bits",    config.rmc.bits);
  LOGF("CFG rmc_bits=0x%s", uint64_to_hexstr(config.rmc.bits));

  config.rmc.options = (uint32_t)getJsonHexParam(jsonConfig, "rmc_options", config.rmc.options);
  LOGF("CFG rmc_options=0x%08lX", (unsigned long)config.rmc.options);

  if (jsonConfig["max_file_size"].is<uint32_t>())
    config.max_file_size = jsonConfig["max_file_size"].as<uint32_t>();

  if (jsonConfig["log_on_startup"].is<bool>())
    config.log_on_startup = jsonConfig["log_on_startup"].as<bool>();
}

// ===================== RMC al dispositivo =====================
static void stream_configure_PPD(uint64_t bits = 0, uint32_t options = 0) {
  is_comm_instance_t commTx = {};
  uint8_t buf[64] = { 0 };
  rmc_t rmc; rmc.bits = bits; rmc.options = options;
  int len = is_comm_data_to_buf(buf, sizeof(buf), &commTx, DID_RMC, sizeof(rmc_t), 0, (void*)&rmc);
  Serial1.write(buf, len);
}

// ===================== Metadata SD =====================
static int getLastLogNumber() {
  File f = SD.open(log_meta, FILE_READ);
  if (f) {
    log_num = f.parseInt(SKIP_WHITESPACE);
    f.close();
  } else {
    log_num = 0;
  }
  return log_num;
}

static int writeLogNumber(int n) {
  File f = SD.open(log_meta, FILE_WRITE);
  if (!f) return n;
  f.seek(0);
  f.print(n);
  f.truncate(f.position());
  f.close();
  return n;
}

static File openLog() {
  char dirName[16];
  char logName[64];

  snprintf(dirName, sizeof(dirName), "log_%04d", log_num);
  if (!SD.exists(dirName)) {
    if (!SD.mkdir(dirName)) LOGF("ERROR mkdir %s", dirName);
  }

  snprintf(logName, sizeof(logName), "%s/%08d.raw", dirName, file_num++);
  if (SD.exists(logName)) SD.remove(logName);

  file_size = 0;
  File f = SD.open(logName, FILE_WRITE);
  if (!f) {
    LOGF("ERROR abriendo %s", logName);
  } else {
    LOGF("Grabando en %s", logName);
  }
  return f;
}

// ===================== Escritura bufferizada =====================
static void writeFromBuffer(File& file) {
  if (buffer.len == 0) return;
  digitalWrite(LED_GREEN, LOW);
  size_t written = file.write((const uint8_t*)buffer.data, buffer.len);
  file_size       += (uint32_t)written;
  sd_bytes_period += (uint32_t)written;
  digitalWrite(LED_GREEN, HIGH);

  if (written == buffer.len) {
    buffer.len = 0;
  } else if (written > 0) {
    size_t remain = buffer.len - written;
    memmove(buffer.data, buffer.data + written, remain);
    buffer.len = remain;
  }
}

static void appendToBuffer(const char* data, size_t len) {
  size_t free_space = MAX_BUFF_LEN - buffer.len;
  if (len > free_space) {
    // Si no cabe, primero vacía lo que hay (si hay archivo)
    if (logFile) writeFromBuffer(logFile);
    // recalcula
    free_space = MAX_BUFF_LEN - buffer.len;
    if (len > free_space) {
      // si aún no cabe todo, copia parcial (última defensa)
      len = free_space;
    }
  }
  if (len) {
    memcpy(buffer.data + buffer.len, data, len);
    buffer.len += len;
  }
}

// ===================== Control logging =====================
static bool isLogging() { return (bool)logFile; }

static void stopLogging() {
  if (logFile) {
    writeFromBuffer(logFile);
    logFile.flush();
    logFile.close();
  }
  stream_configure_PPD(0, 0);
  LOGF("Logger detenido.");
}

static bool startLogging() {
  LOGF("Inicializando SD...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    LOGF("ERROR SD.begin(BUILTIN_SDCARD)");
    return false;
  }
  LOGF("SD OK.");

  readConfig();
  log_num = getLastLogNumber();
  writeLogNumber(++log_num);

  // Configura stream del dispositivo (si procede)
  stream_configure_PPD(config.rmc.bits, config.rmc.options);

  file_num = 0;
  file_size = 0;
  buffer.len = 0;

  logFile = openLog();
  if (!logFile) return false;

  const uint32_t now = millis();
  next_flush  = now + FLUSH_MS;
  next_report = now + REPORT_MS;
  next_rmc    = now + 100;     // envía config pronto
  next_blink  = now + 250;

  LOGF("Logger iniciado.");
  return true;
}

static void toggleLogging() {
  if (isLogging()) stopLogging();
  else             startLogging();
}

// Teensy 4.1: no hay pin detect; si SD.begin fue bien, asumimos presente
static bool isSDCardDetected() { return true; }

// ===================== Arduino =====================
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  // USB Serial para mensajes: no bloquear más de 2s
  Serial.begin(BAUD_USB);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* espera corta */ }

  LOGF("\n=== Teensy 4.1 — GPS Serial1 -> SD Logger ===");
  LOGF("USB=%lu bps, Serial1=%lu bps", (unsigned long)BAUD_USB, (unsigned long)BAUD_SERIAL1);

  // Serial1 al GPS
  Serial1.begin(BAUD_SERIAL1);
  // ¡No usar while(!Serial1)!

  // Leer config si SD accesible (no inicia aún)
  if (SD.begin(BUILTIN_SDCARD)) {
    readConfig();
  } else {
    LOGF("Aviso: SD no montó en setup (se reintenta al startLogging).");
  }

  if (config.log_on_startup) {
    startLogging();
  }

  btn_state   = digitalRead(PIN_BTN);
  next_report = millis() + REPORT_MS;
}

void loop() {
  const uint32_t now = millis();

  // LED verde muestra "SD presente" (estático en Teensy 4.1)
  digitalWrite(LED_GREEN, isSDCardDetected());

  // ---- Botón (pulsación larga >1200ms con debounce simple) ----
  int cur = digitalRead(PIN_BTN);
  static uint32_t last_change = 0;
  if (cur != btn_state && (now - last_change) > 30) {
    last_change = now;
    if (cur == LOW) {
      press_start = now;
    } else {
      uint32_t held = now - press_start;
      LOGF("Botón: suelto (%lums).", (unsigned long)held);
      if (held > 1200) toggleLogging();
    }
    btn_state = cur;
  }

  // ---- Lectura Serial1 -> buffer ----
  int avail = Serial1.available();
  while (avail > 0) {
    if (isLogging()) digitalWrite(LED_RED, LOW);

    int toRead = (avail > (int)sizeof(ser_buf)) ? (int)sizeof(ser_buf) : avail;
    int n = Serial1.readBytes(ser_buf, toRead);
    if (n <= 0) break;

    rx_bytes_period += (uint32_t)n;

    if (isLogging()) {
      digitalWrite(LED_RED, HIGH);
      appendToBuffer(ser_buf, (size_t)n);
      // si se alcanzó umbral, escribe ya
      if (buffer.len >= WRITE_CHUNK) {
        writeFromBuffer(logFile);
      }
      // rotación por tamaño
      if (file_size >= config.max_file_size) {
        writeFromBuffer(logFile);
        logFile.flush();
        logFile.close();
        logFile = openLog();
      }
    }

    avail = Serial1.available();
  }

  // ---- Flush por tiempo ----
  if (isLogging() && (int32_t)(now - next_flush) >= 0) {
    writeFromBuffer(logFile);
    logFile.flush();
    next_flush = now + FLUSH_MS;
  }

  // ---- Blink rojo cuando NO se está logueando ----
  if (!isLogging() && (int32_t)(now - next_blink) >= 0) {
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    next_blink = now + 250;
  }

  // ---- Informe periódico ----
  if ((int32_t)(now - next_report) >= 0) {
    const float secs = REPORT_MS / 1000.0f;
    const float kbps_rx = (rx_bytes_period / secs) / 1024.0f;
    const float kbps_sd = (sd_bytes_period / secs) / 1024.0f;
    LOGF("%s RX=%lu B (%.2f KB/s)  SD=%lu B (%.2f KB/s)",
         isLogging() ? "LOG" : "RX",
         (unsigned long)rx_bytes_period, kbps_rx,
         (unsigned long)sd_bytes_period, kbps_sd);
    rx_bytes_period = 0;
    sd_bytes_period = 0;
    next_report = now + REPORT_MS;
  }

  // ---- Reenvío RMC periódico ----
  if (isLogging() && (int32_t)(now - next_rmc) >= 0) {
    stream_configure_PPD(config.rmc.bits, config.rmc.options);
    next_rmc = now + RMC_MS;
  }
}
