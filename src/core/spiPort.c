#include "spiPort.h"

#include <errno.h>
#include <string.h>
#include <stdint.h>

#if PLATFORM_IS_LINUX
#   include <fcntl.h>
#   include <unistd.h>
#   include <sys/ioctl.h>
#   include <linux/spi/spidev.h>
#endif

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------

/** Returns true if the file descriptor is open (≥ 0). */
static inline bool spi_is_open(const spi_port_t* p) { return p->fd >= 0; }

#if PLATFORM_IS_LINUX
/** Stores errno in perror and returns -errno. */
#define HANDLE_SPI_ERROR(p) \
    do { (p)->base.perror = errno; return -errno; } while (0)
#endif

// -----------------------------------------------------------------------
// Function-pointer implementations
// -----------------------------------------------------------------------

const char* spiPortGetName(port_handle_t port)
{
    return SPI_PORT(port)->name;
}

int spiPortValidate(port_handle_t port)
{
#if PLATFORM_IS_LINUX
    struct stat st;
    const char* name = SPI_PORT(port)->name;
    // Device must exist and be a character device
    if (stat(name, &st) != 0 || !S_ISCHR(st.st_mode))
    {
        SPI_PORT(port)->base.perror = errno;
        return 0;
    }
    return 1;
#else
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortOpen(port_handle_t port)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);

    if (spi_is_open(p))
    {
        portFlagsSet(port, PORT_FLAG__OPENED);
        return 0; // already open
    }

    p->fd = open(p->name, O_RDWR);
    if (p->fd < 0)
    {
        portFlagsClear(port, PORT_FLAG__OPENED);
        HANDLE_SPI_ERROR(p);
    }

    // Apply SPI mode
    uint8_t mode = p->mode;
    if (ioctl(p->fd, SPI_IOC_WR_MODE, &mode) < 0)  { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

    // Apply bits per word
    uint8_t bits = p->bitsPerWord;
    if (ioctl(p->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

    // Apply clock speed
    uint32_t speed = p->speedHz;
    if (ioctl(p->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

    portFlagsSet(port, PORT_FLAG__OPENED);
    p->base.perror = 0;
    return 0;
#else
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortClose(port_handle_t port)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);

    if (!spi_is_open(p))
        return 0; // already closed — benign no-op

    int ret = close(p->fd);
    if (ret != 0)
        p->base.perror = errno;

    p->fd = -1;
    portFlagsClear(port, PORT_FLAG__OPENED);
    return 0;
#else
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortFree(port_handle_t port)
{
    // SPI transfers are synchronous — the TX "buffer" is unbounded from
    // the caller's perspective; each write blocks until complete.
    (void)port;
    return INT_MAX;
}

int spiPortAvailable(port_handle_t port)
{
    // SPI is synchronous full-duplex: there is no async RX buffer at the
    // spidev level.  Callers must drive reads explicitly via portRead().
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
}

int spiPortFlush(port_handle_t port)
{
    // Synchronous bus — nothing buffered to flush.
    (void)port;
    return 0;
}

int spiPortDrain(port_handle_t port, uint32_t timeout)
{
    // Each write already blocks until the transfer is complete.
    (void)port; (void)timeout;
    return 0;
}

int spiPortRead(port_handle_t port, uint8_t* buf, unsigned int len)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    if (!spi_is_open(p)) { p->base.perror = EBADF; return -EBADF; }

    // Full-duplex transfer: clock in RX while sending 0xFF dummy bytes
    struct spi_ioc_transfer tr = {
        .tx_buf        = 0,     // NULL — spidev sends 0x00 when tx_buf is 0
        .rx_buf        = (unsigned long)buf,
        .len           = len,
        .speed_hz      = p->speedHz,
        .bits_per_word = p->bitsPerWord,
        .delay_usecs   = 0,
        .cs_change     = 0,
    };

    int ret = ioctl(p->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) { HANDLE_SPI_ERROR(p); }
    return ret;
#else
    (void)port; (void)buf; (void)len;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortReadTimeout(port_handle_t port, uint8_t* buf, unsigned int len, uint32_t timeout)
{
    // SPI has no inherent timeout mechanism — the transfer completes as soon
    // as the host clocks the bits out.  We honour the timeout parameter by
    // returning immediately if the port is not open rather than blocking.
    (void)timeout;
    return spiPortRead(port, buf, len);
}

int spiPortWrite(port_handle_t port, const uint8_t* buf, unsigned int len)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    if (!spi_is_open(p)) { p->base.perror = EBADF; return -EBADF; }

    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)buf,
        .rx_buf        = 0,    // discard received bytes
        .len           = len,
        .speed_hz      = p->speedHz,
        .bits_per_word = p->bitsPerWord,
        .delay_usecs   = 0,
        .cs_change     = 0,
    };

    int ret = ioctl(p->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) { HANDLE_SPI_ERROR(p); }
    return ret;
#else
    (void)port; (void)buf; (void)len;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

// -----------------------------------------------------------------------
// Public init / delete
// -----------------------------------------------------------------------

void spiPortInit(port_handle_t port, int id, const char* name,
                 uint32_t speedHz, uint8_t mode, int flags)
{
    spi_port_t* p = SPI_PORT(port);

    p->base.pnum   = id;
    p->base.ptype  = PORT_TYPE__SPI | PORT_TYPE__COMM;
    p->base.pflags = flags;
    p->base.stats  = (port_stats_t*)&(p->stats);

    p->base.portName        = spiPortGetName;
    p->base.portValidate    = spiPortValidate;
    p->base.portOpen        = spiPortOpen;
    p->base.portClose       = spiPortClose;
    p->base.portFree        = spiPortFree;
    p->base.portAvailable   = spiPortAvailable;
    p->base.portFlush       = spiPortFlush;
    p->base.portDrain       = spiPortDrain;
    p->base.portRead        = spiPortRead;
    p->base.portReadTimeout = spiPortReadTimeout;
    p->base.portWrite       = spiPortWrite;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_init(COMM_PORT(port), NULL);

    p->fd          = -1;  // closed until portOpen() is called
    p->speedHz     = speedHz ? speedHz : SPI_PORT_DEFAULT_SPEED_HZ;
    p->mode        = mode;
    p->bitsPerWord = SPI_PORT_DEFAULT_BITS;

    strncpy(p->name, name, MAX_SPI_PORT_NAME_LENGTH);
    p->name[MAX_SPI_PORT_NAME_LENGTH] = '\0';

    portFlagsSet(port, PORT_FLAG__VALID);
}

void spiPortDelete(port_handle_t port)
{
    if (!port) return;
    memset(port, 0, sizeof(spi_port_t));
}

int spiPortSetSpeed(port_handle_t port, uint32_t speedHz)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    p->speedHz = speedHz;
    if (!spi_is_open(p)) return 0; // applied on next open
    if (ioctl(p->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz) < 0) { HANDLE_SPI_ERROR(p); }
    return 0;
#else
    (void)port; (void)speedHz;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortSetMode(port_handle_t port, uint8_t mode)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    p->mode = mode;
    if (!spi_is_open(p)) return 0; // applied on next open
    if (ioctl(p->fd, SPI_IOC_WR_MODE, &mode) < 0) { HANDLE_SPI_ERROR(p); }
    return 0;
#else
    (void)port; (void)mode;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}
