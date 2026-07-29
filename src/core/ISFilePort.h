/**
 * @file ISFilePort.h
 * @brief Stream-backed base_port_t implementations: ISStreamPort adapts an arbitrary
 *        %std::istream/%std::ostream pair to the base_port_t interface, and ISFilePort is a
 *        concrete specialization of it that opens a single file for both reading and writing.
 *
 * @author Kyle Mallory on 1/16/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__ISFILEPORT_H
#define IS_CORE__ISFILEPORT_H

#include <fstream>
#include <iostream>
#include <map>

#include "core/types.h"
#include "core/base_port.h"

/**
 * Adapts an arbitrary %std::istream/%std::ostream pair to the base_port_t interface, so that the
 * generic port I/O helpers (portRead(), portWrite(), etc. from base_port.h) can operate on any
 * C++ stream as if it were a hardware port. The port's name/free/available/read/write callbacks
 * are implemented as static member functions that reinterpret the port_handle_t back into an
 * ISStreamPort*, per the base_port_t "vtable" convention.
 */
class ISStreamPort : base_port_t {
private:
    // static std::map<port_handle_t, ISStreamPort> m_streamPortMappings;
    // static const char* name(port_handle_t port) { return m_streamPortMappings.contains(port) ? m_streamPortMappings[port].m_portName.c_str() : nullptr; }
    // static int free(port_handle_t port) { return m_streamPortMappings.contains(port) ? UINT16_MAX : PORT_ERROR__INVALID; }
    // static int available(port_handle_t port) { return m_streamPortMappings.contains(port) ? m_streamPortMappings[port].m_istream.gcount() : PORT_ERROR__INVALID; }

    /**
     *  These are the core, underlying base_port_t wrappers to allow calling, for example, 'portWrite((port_handle_t)&myStreamPort, data, len)`
     *  and having it work the same as any other port.  This should work because the pointer to the object instance is also a pointer to base_port_t.
     *  This should be a suitable model for implementing most derived types of ports in C++.  Port-type specific functionality like Open() or
     *  Close() which are not available as part of base_port_t should be implemented in the actual class as member functions.
     */
    static const char* name(port_handle_t port) { return (port && ((portType(port) & PORT_TYPE__FILE) == PORT_TYPE__FILE)) ? ((ISStreamPort*)port)->m_portName.c_str() : nullptr; }
    static int free(port_handle_t port) { return (port && ((portType(port) & PORT_TYPE__FILE) == PORT_TYPE__FILE)) ? UINT16_MAX : PORT_ERROR__INVALID; }
    static int available(port_handle_t port) { return (port && ((portType(port) & PORT_TYPE__FILE) == PORT_TYPE__FILE)) ? ((ISStreamPort*)port)->m_istream.gcount() : PORT_ERROR__INVALID; }
    static int read(port_handle_t port, uint8_t* buf, unsigned int len) { return (port && ((portType(port) & PORT_TYPE__FILE) == PORT_TYPE__FILE)) ? ((ISStreamPort*)port)->m_istream.readsome((char *)buf, len) : PORT_ERROR__INVALID; }
    static int write(port_handle_t port, const uint8_t* buf, unsigned int len) { if (port && ((portType(port) & PORT_TYPE__FILE) == PORT_TYPE__FILE)) { ((ISStreamPort*)port)->m_ostream.write((const char *)buf, len); return len; } else { return PORT_ERROR__INVALID; } }
    static int logger(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void* userData) {
        // TODO: Not sure what to do here just yet...
        return PORT_ERROR__NOT_SUPPORTED;
    }


public:
    const std::string m_portName;  //!< display name for this port, returned by ISStreamPort::name()
    std::ostream& m_ostream;       //!< the output stream data is written to (portWrite)
    std::istream& m_istream;       //!< the input stream data is read from (portRead)

    /**
     * Constructs a stream-backed port bound to the given input/output streams.
     * @param name display name for this port (returned by ISStreamPort::name())
     * @param in   the input stream to read from
     * @param out  the output stream to write to
     */
    ISStreamPort(const std::string& name, std::istream& in, std::ostream& out) : m_portName(name), m_ostream(out), m_istream(in) {
        pnum = 128; // FIXME unique ID?  maybe an index value or hash?
        ptype = PORT_TYPE__FILE;

        portName = ISStreamPort::name;           //!< a function which returns the name associate with this port
        portFree = ISStreamPort::free;           //!< a function which returns the number of bytes which can safely be written
        portAvailable = ISStreamPort::available; //!< a function which returns the number of bytes available to be read
        portRead = ISStreamPort::read;           //!< a function to return copy some number of bytes available for reading into a local buffer (and removed from the ports read buffer)
        portWrite = ISStreamPort::write;         //!< a function to copy some number of bytes from a local buffer into the ports write buffer (when and how this data is actually "sent" is implementation specific)
    }
};

/**
 * Concrete ISStreamPort that opens a single file for both reading and writing, using the same
 * path for the underlying std::ifstream and std::ofstream. Useful for capturing/replaying raw
 * port traffic to/from disk using the generic base_port_t helpers.
 */
class ISFilePort : ISStreamPort {
    std::ofstream m_out;  //!< output stream opened on the file passed to the constructor
    std::ifstream m_in;   //!< input stream opened on the file passed to the constructor

    /**
     * Opens fname for both reading and writing, and binds the resulting streams to the
     * underlying ISStreamPort.
     * @param fname path of the file to open for read/write access
     */
    ISFilePort(const std::string& fname) : ISStreamPort(fname, m_in, m_out){
        m_out.open(fname);
        m_in.open(fname);
    }
};

#endif //IS_CORE__ISFILEPORT_H
