/**
 * @file IsSerialPort.h 
 * @brief Provides an InertialSense SDK compatible port implementation that wraps the Arduino Serial class.
 *
 * This may not be functional - The constructor relies on a common base class implementation of Serial_ which
 * may or may not exist.  It seems different implementations for different hardware Implement Serial[n] in a
 * variety of ways which are not all derived from a common Serial class.
 *
 * @author Kyle Mallory on 6/9/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_ARDUINO_SERIALPORT_H
#define IS_SDK_ARDUINO_SERIALPORT_H

#include "base_port.h"

class ISArduinoSerialPort : public base_port_t {
private:
    Serial_& m_serial;              // reference to the underlying port implementation
    const String m_portName = "";   // will be assigned in the constructor if valid
    uint32_t m_baud = 921600;       // default baud rate for Inertial Sense devices
    uint8_t m_config = SERIAL_8N1;  // default line protocol for Inertial Sense devices

    /**
     *  These are the core, underlying base_port_t wrappers to allow calling, for example, 'portWrite((port_handle_t)&myStreamPort, data, len)`
     *  and having it work the same as any other port.  This should work because the pointer to the object instance is also a pointer to base_port_t.
     *  This should be a suitable model for implementing most derived types of ports in C++.  Port-type specific functionality like Open() or
     *  Close() which are not available as part of base_port_t should be implemented in the actual class as member functions.
     */

    // return an error (non-zero) if the port handle is invalid
    static int validate(port_handle_t port) {
        if (&((ISArduinoSerialPort*)port)->m_serial == &Serial) return PORT_ERROR__NONE;
#ifdef Serial1
        else if (&((ISArduinoSerialPort*)port)->m_serial == &Serial1) return PORT_ERROR__NONE;
#endif
#ifdef Serial2
        else if (&((ISArduinoSerialPort*)port)->m_serial == &Serial2) return PORT_ERROR__NONE;
#endif
#ifdef Serial3
        else if (&((ISArduinoSerialPort*)port)->m_serial == &Serial3) return PORT_ERROR__NONE;
#endif
#ifdef Serial4
        else if (&((ISArduinoSerialPort*)port)->m_serial == &Serial4) return PORT_ERROR__NONE;
#endif
        return PORT_ERROR__INVALID;
    }

    // Returns the name of the port
    static const char* name(port_handle_t port) {
        return (portIsValid(port) && ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART)) ? ((ISArduinoSerialPort*)port)->m_portName.c_str() : nullptr;
    }

    // returns the number of free bytes in the TX buffer that can safely be written without causing a TX OVERFLOW
    static int free(port_handle_t port) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        return ((ISArduinoSerialPort*)port)->m_serial.availableForWrite();  // call the underlying implementation
    }

    // Returns the number of available bytes in the RX buffer which are available for reading
    static int available(port_handle_t port) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        return ((ISArduinoSerialPort*)port)->m_serial.available();   // call the underlying implementation
    }

    // reads upto len bytes and stores into buf, returning the number of bytes bytes actually read
    static int read(port_handle_t port, uint8_t* buf, unsigned int len) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        return ((ISArduinoSerialPort*)port)->m_serial.readBytes((char *)buf, len);   // call the underlying implementation
    }

    // writes upto len bytes to the port, returning the number of bytes successfully written
    static int write(port_handle_t port, const uint8_t* buf, unsigned int len) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        ((ISArduinoSerialPort*)port)->m_serial.write((const char *)buf, len);   // call the underlying implementation
        return len;
    }

    static int open(port_handle_t port) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        ((ISArduinoSerialPort*)port)->m_serial.begin(((ISArduinoSerialPort*)port)->m_baud, ((ISArduinoSerialPort*)port)->m_config);
        ((ISArduinoSerialPort*)port)->ptype |= PORT_FLAG__OPENED;
        return PORT_ERROR__NONE;
    }

    static int close(port_handle_t port) {
        if (!portIsValid(port) || ((portType(port) & PORT_TYPE__UART) == PORT_TYPE__UART))
            return PORT_ERROR__INVALID;

        ((ISArduinoSerialPort*)port)->m_serial.end();                   // deinitialize the port
        ((ISArduinoSerialPort*)port)->ptype ^= ~PORT_FLAG__OPENED;      // mark the port as closed
        return PORT_ERROR__NONE;
    }

public:
    ISArduinoSerialPort(Serial_& base, uint32_t baud = 921600, uint8_t config = SERIAL_8N1) : m_serial(base) {

        m_serial = base;
        m_baud = baud;
        m_config = config;

        ptype = PORT_TYPE__UART | PORT_TYPE__HDW;
        pflags = 0;
        perror = 0;

        if (validate((port_handle_t)this) == PORT_ERROR__NONE)
            ptype |= PORT_FLAG__VALID;                  // only mark as VALID if it is valid...
        else {
#ifdef Serial1
            if (&base == &Serial1)     pnum = 0xF1, m_portName = "Serial1";  else  // some number and name is identifyable
#endif
#ifdef Serial2
            if (&base == &Serial2)     pnum = 0xF2, m_portName = "Serial2";   else  // some number and name is identifyable
#endif
#ifdef Serial3
            if (&base == &Serial3)     pnum = 0xF3, m_portName = "Serial3";   else  // some number and name is identifyable
#endif
#ifdef Serial4
            if (&base == &Serial4)     pnum = 0xF4, m_portName = "Serial4";   else  // some number and name is identifyable
#endif
            pnum = 0xF0, m_portName = "Serial0";                                    // our default name and pnum

            portValidate = ISArduinoSerialPort::validate;   //!< a function which returns indicates the validity of the port handle
            portName = ISArduinoSerialPort::name;           //!< a function which returns the name associate with this port
            portFree = ISArduinoSerialPort::free;           //!< a function which returns the number of bytes which can safely be written
            portAvailable = ISArduinoSerialPort::available; //!< a function which returns the number of bytes available to be read
            portRead = ISArduinoSerialPort::read;           //!< a function to return copy some number of bytes available for reading into a local buffer (and removed from the ports read buffer)
            portWrite = ISArduinoSerialPort::write;         //!< a function to copy some number of bytes from a local buffer into the ports write buffer (when and how this data is actually "sent" is implementation specific)
            portOpen = ISArduinoSerialPort::open;           //!< a function to initialize the serial port
            portClose = ISArduinoSerialPort::close;         //!< a function to deinitialize the serial port
        }
    }
};

#endif //IS_SDK_ARDUINO_SERIALPORT_H



