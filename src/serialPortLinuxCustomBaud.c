/**
 * @file serialPortLinuxCustomBaud.c
 * @brief Linux-only helper to set an arbitrary ("custom") serial baud rate via termios2/BOTHER.
 *
 * SN-8239: standard termios Bxxx constants only cover a fixed set of rates and top out at B4000000,
 * so arbitrary rates (and non-standard rates such as 1220000 / 1440000, and rates up to 10 Mbaud)
 * cannot be set with cfsetospeed/cfsetispeed. The Linux termios2 interface with the BOTHER flag lets
 * us program the literal integer rate into c_ispeed/c_ospeed and apply it with ioctl(TCSETS2).
 *
 * This lives in its own translation unit on purpose: the kernel header <asm/termbits.h> (which
 * defines struct termios2, BOTHER, CBAUD, TCGETS2/TCSETS2) conflicts with glibc <termios.h> that
 * serialPortPlatform.c relies on. Keeping this file free of <termios.h> avoids that clash.
 */

#if defined(__linux__)

#include <asm/termbits.h>   // struct termios2, BOTHER, CBAUD, TCGETS2, TCSETS2
#include <sys/ioctl.h>      // ioctl()

/**
 * @brief Set an arbitrary baud rate on an already-open serial fd using termios2/BOTHER.
 * Reads the current termios2 (preserving line flags configured by the caller), replaces only the
 * input/output speed with the literal rate, and applies it. The caller is responsible for having
 * already configured framing/flow/raw-mode via the normal termios path.
 *
 * @param fd The open serial file descriptor.
 * @param baudRate The desired baud rate (bits/sec).
 * @return int 0 on success, -1 on failure (errno is set by the failing ioctl).
 */
int serialPortSetCustomBaudLinux(int fd, int baudRate)
{
    struct termios2 tio;

    if (ioctl(fd, TCGETS2, &tio) != 0)
    {
        return -1;
    }

    // Select "other" (arbitrary) baud and program the literal rate into both directions.
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = (speed_t)baudRate;
    tio.c_ospeed = (speed_t)baudRate;

    if (ioctl(fd, TCSETS2, &tio) != 0)
    {
        return -1;
    }

    return 0;
}

#endif // __linux__
