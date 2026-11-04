/**
 * @file cltool_port_spec.h
 * @brief cltool's handling of the -c port specifier during firmware-update re-discovery (SN-8661).
 *
 * cltool turns its port specifier (a glob, or a comma-separated list of globs) into the regex PortManager::discoverPorts() consumes by
 * way of utils::globToRegex(), which escapes regex metacharacters. That is the step that breaks a udev alias with a dot in it: "/dev/imx5.0"
 * becomes "/dev/imx5\.0", which names no file, so SerialPortFactory::locatePorts() can no longer resolve it to its device node.
 *
 * Nothing else in the tree does this conversion: EvalTool and InertialSense::OpenPorts() hand their pattern or literal path to discoverPorts()
 * unescaped, where locatePorts() resolves a literal alias itself (SN-8575). So the fix is local to cltool: resolve each entry while it is
 * still the literal text the user typed, then let utils::globToRegex() do its normal translation.
 *
 * Header-only so the unit tests can exercise it without linking cltool.
 */
#pragma once

#include <string>

#include "PortFactory.h"

/**
 * Replaces each comma-separated entry of a cltool port specifier that is a udev-style symlink with its canonical device path
 * (SerialPortFactory::resolvePortName(), which only changes a real symlink on Linux and is a no-op elsewhere). Entries that are not
 * symlinks -- "*", "/dev/ttyACM*", URIs, an empty entry -- are left exactly as written.
 * @param specifier the -c value as typed
 * @return the specifier with aliases resolved, ready for utils::globToRegex()
 */
inline std::string cltoolResolvePortAliases(const std::string& specifier)
{
    std::string out;
    for (size_t start = 0; start <= specifier.size(); ) {
        const size_t comma = specifier.find(',', start);
        const std::string entry = specifier.substr(start, (comma == std::string::npos) ? std::string::npos : comma - start);
        if (start > 0)
            out += ',';
        out += entry.empty() ? entry : SerialPortFactory::resolvePortName(entry);
        if (comma == std::string::npos)
            break;
        start = comma + 1;
    }
    return out;
}
