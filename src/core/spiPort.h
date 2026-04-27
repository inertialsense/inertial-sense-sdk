#ifndef __IS_SPIPORT_H
#define __IS_SPIPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ISComm.h"
#include "ISConstants.h"

#define MAX_SPI_PORT_NAME_LENGTH    63      // e.g. "/dev/spidev0.0"
#define SPI_PORT_DEFAULT_SPEED_HZ   1000000 // 1 MHz default clock
#define SPI_PORT_DEFAULT_MODE       3       // CPOL=1, CPHA=1
#define SPI_PORT_DEFAULT_BITS       8       // 8 bits per word

struct spi_port_s
{
    // base "implementation" — must be first member
    union {
        base_port_t base;
        comm_port_t comm;
    };

    port_monitor_set_t stats;

    // port name, e.g. "/dev/spidev0.0"
    char name[MAX_SPI_PORT_NAME_LENGTH + 1];

    // OS file descriptor (-1 when closed)
    int fd;

    // SPI configuration (applied on open)
    uint32_t speedHz;       // SPI clock speed in Hz
    uint8_t  mode;          // SPI mode 0-3 (CPOL/CPHA)
    uint8_t  bitsPerWord;   // bits per word (typically 8)
};

typedef struct spi_port_s spi_port_t;
#define SPI_PORT(n) ((spi_port_t*)(n))

/**
 * Initializes a new SPI port struct with the specified name and configuration.
 * Does NOT open the device — call portOpen() separately.
 * @param port     port handle pointing to an allocated spi_port_t
 * @param id       unique port ID
 * @param name     device path, e.g. "/dev/spidev0.0"
 * @param speedHz  SPI clock speed in Hz (0 = use SPI_PORT_DEFAULT_SPEED_HZ)
 * @param mode     SPI mode 0–3 (CPOL/CPHA) (use SPI_PORT_DEFAULT_MODE if unsure)
 * @param flags    port flags (PORT_FLAG__COMM, PORT_FLAG__BLOCKING, etc.)
 */
void spiPortInit(port_handle_t port, int id, const char* name,
                 uint32_t speedHz, uint8_t mode, int flags);

void spiPortDelete(port_handle_t port);

int  spiPortSetSpeed(port_handle_t port, uint32_t speedHz);
int  spiPortSetMode(port_handle_t port, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif // __IS_SPIPORT_H
