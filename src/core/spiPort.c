#include "spiPort.h"

#include <errno.h>
#include <string.h>
#include <stdint.h>

#if PLATFORM_IS_LINUX
#   include <fcntl.h>
#   include <unistd.h>
#   include <poll.h>
#   include <sys/ioctl.h>
#   include <sys/stat.h>
#   include <linux/spi/spidev.h>
#endif

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------

static inline bool spi_is_open(const spi_port_t* p) { return p->fd >= 0; }
static inline bool spi_has_dr(const spi_port_t* p)  { return p->drGpioFd >= 0; }

#if PLATFORM_IS_LINUX
#define HANDLE_SPI_ERROR(p) \
    do { (p)->base.perror = errno; return -errno; } while (0)

// Write a string to a sysfs file; returns 0 on success, -errno on failure.
static int sysfs_write(const char* path, const char* val)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -errno;
    int ret = (int)write(fd, val, strlen(val));
    close(fd);
    return (ret < 0) ? -errno : 0;
}

// Close and unexport the DR GPIO if one is configured.
static void dr_gpio_release(spi_port_t* p)
{
    if (p->drGpioFd >= 0)
    {
        close(p->drGpioFd);
        p->drGpioFd = -1;
    }
    if (p->drGpioNum >= 0)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d", p->drGpioNum);
        sysfs_write("/sys/class/gpio/unexport", buf);
        p->drGpioNum = -1;
    }
}
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
        return 0;
    }

    p->fd = open(p->name, O_RDWR);
    if (p->fd < 0) { portFlagsClear(port, PORT_FLAG__OPENED); HANDLE_SPI_ERROR(p); }

    uint8_t mode = p->mode;
    if (ioctl(p->fd, SPI_IOC_WR_MODE, &mode) < 0)             { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

    uint8_t bits = p->bitsPerWord;
    if (ioctl(p->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)    { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

    uint32_t speed = p->speedHz;
    if (ioctl(p->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)    { close(p->fd); p->fd = -1; HANDLE_SPI_ERROR(p); }

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
        return 0;

    int ret = close(p->fd);
    if (ret != 0) p->base.perror = errno;

    p->fd = -1;
    portFlagsClear(port, PORT_FLAG__OPENED);
    // drGpioFd intentionally kept open so the DR pin survives a port re-open
    return 0;
#else
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortFree(port_handle_t port)
{
    (void)port;
    return INT_MAX;
}

int spiPortAvailable(port_handle_t port)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    if (!spi_has_dr(p))
        return PORT_ERROR__NOT_SUPPORTED;

    // Read the current GPIO level from sysfs
    char val = '0';
    lseek(p->drGpioFd, 0, SEEK_SET);
    if (read(p->drGpioFd, &val, 1) < 0) { HANDLE_SPI_ERROR(p); }
    return (val == '1') ? 1 : 0;
#else
    (void)port;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortFlush(port_handle_t port)
{
    (void)port;
    return 0;
}

int spiPortDrain(port_handle_t port, uint32_t timeout)
{
    (void)port; (void)timeout;
    return 0;
}

int spiPortRead(port_handle_t port, uint8_t* buf, unsigned int len)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    if (!spi_is_open(p)) { p->base.perror = EBADF; return -EBADF; }

    struct spi_ioc_transfer tr = {
        .tx_buf        = 0,
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
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);

    if (spi_has_dr(p))
    {
        // Drain any stale edge event so the subsequent poll catches the NEXT rising edge.
        char dummy[2];
        lseek(p->drGpioFd, 0, SEEK_SET);
        read(p->drGpioFd, dummy, sizeof(dummy));

        struct pollfd pfd = { .fd = p->drGpioFd, .events = POLLPRI | POLLERR };
        int rc = poll(&pfd, 1, (int)timeout);
        if (rc < 0)  { HANDLE_SPI_ERROR(p); }
        if (rc == 0) { return 0; }  // timeout — slave has no data

        // Confirm the pin is still high (fast pulses may already be gone)
        char val = '0';
        lseek(p->drGpioFd, 0, SEEK_SET);
        read(p->drGpioFd, &val, 1);
        if (val != '1') return 0;
    }

    return spiPortRead(port, buf, len);
#else
    (void)timeout;
    return spiPortRead(port, buf, len);
#endif
}

int spiPortWrite(port_handle_t port, const uint8_t* buf, unsigned int len)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    if (!spi_is_open(p)) { p->base.perror = EBADF; return -EBADF; }

    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)buf,
        .rx_buf        = 0,
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
// Public init / delete / configuration
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

    p->fd          = -1;
    p->speedHz     = speedHz ? speedHz : SPI_PORT_DEFAULT_SPEED_HZ;
    p->mode        = mode;
    p->bitsPerWord = SPI_PORT_DEFAULT_BITS;
    p->drGpioNum   = -1;
    p->drGpioFd    = -1;

    strncpy(p->name, name, MAX_SPI_PORT_NAME_LENGTH);
    p->name[MAX_SPI_PORT_NAME_LENGTH] = '\0';

    portFlagsSet(port, PORT_FLAG__VALID);
}

void spiPortDelete(port_handle_t port)
{
    if (!port) return;
#if PLATFORM_IS_LINUX
    dr_gpio_release(SPI_PORT(port));
#endif
    memset(port, 0, sizeof(spi_port_t));
}

int spiPortSetSpeed(port_handle_t port, uint32_t speedHz)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);
    p->speedHz = speedHz;
    if (!spi_is_open(p)) return 0;
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
    if (!spi_is_open(p)) return 0;
    if (ioctl(p->fd, SPI_IOC_WR_MODE, &mode) < 0) { HANDLE_SPI_ERROR(p); }
    return 0;
#else
    (void)port; (void)mode;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}

int spiPortSetDataReady(port_handle_t port, int gpioNum)
{
#if PLATFORM_IS_LINUX
    spi_port_t* p = SPI_PORT(port);

    // Release any previously configured DR GPIO
    dr_gpio_release(p);

    if (gpioNum < 0)
        return 0;

    char path[64];
    char numStr[16];
    int  n = snprintf(numStr, sizeof(numStr), "%d", gpioNum);

    // Export the GPIO (ignore EBUSY — already exported is fine)
    int expFd = open("/sys/class/gpio/export", O_WRONLY);
    if (expFd < 0) { HANDLE_SPI_ERROR(p); }
    write(expFd, numStr, n);
    close(expFd);

    // Set direction to input
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpioNum);
    if (sysfs_write(path, "in") < 0) { p->base.perror = errno; return -errno; }

    // Set edge to rising so poll(POLLPRI) fires on the rising edge
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpioNum);
    if (sysfs_write(path, "rising") < 0) { p->base.perror = errno; return -errno; }

    // Open the value file — kept open for the life of the port
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpioNum);
    p->drGpioFd = open(path, O_RDONLY | O_NONBLOCK);
    if (p->drGpioFd < 0) { HANDLE_SPI_ERROR(p); }

    p->drGpioNum = gpioNum;
    return 0;
#else
    (void)port; (void)gpioNum;
    return PORT_ERROR__NOT_SUPPORTED;
#endif
}
