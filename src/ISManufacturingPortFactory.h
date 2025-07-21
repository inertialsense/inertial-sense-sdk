//
// Created by firiusfoxx on 7/3/25.
//

#ifndef IS_SDK__IS_MANUFACTURING_PORT_FACTORY_H
#define IS_SDK__IS_MANUFACTURING_PORT_FACTORY_H

#define IS_MANUFACTURING_PORT_FACTORY_TIME_BETWEEN_QUERIES_MS 150 // How long to wait between sending MDNS queries

#include "PortFactory.h"
#include "core/tcpPort.h"

class ISManufacturingPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
    } portOptions = {};

    static ISManufacturingPortFactory& getInstance() {
        static ISManufacturingPortFactory instance;
        return instance;
    }

    ISManufacturingPortFactory(ISManufacturingPortFactory const &) = delete;
    ISManufacturingPortFactory& operator=(ISManufacturingPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;
    bool releasePort(port_handle_t port) override;
    static void tick();

    static bool validatePortName(const std::string& pName);

    static inline const std::unordered_map<uint16_t, std::string> majorAtlas = {
        {166, "ttyACM"}
    };

private:
    ISManufacturingPortFactory() = default;
    ~ISManufacturingPortFactory() = default;

    typedef struct {
        uint32_t devid;
        uint16_t port;
    } port_t;

    inline static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;

    std::unordered_map<std::string, std::vector<port_t>> getPorts();
    std::pair<std::string, ISManufacturingPortFactory::port_t> parsePortName(const std::string& pName);
};

#endif //IS_SDK__IS_MANUFACTURING_PORT_FACTORY_H
