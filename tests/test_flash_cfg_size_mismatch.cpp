/**
 * @file test_flash_cfg_size_mismatch.cpp
 * @brief Flash-config sync with a device whose nvm_flash_cfg_t is larger than the host's.
 *
 * Fields are only ever appended to nvm_flash_cfg_t, so firmware newer than the host sends a DID_FLASH_CONFIG longer than
 * the host's struct, with every field the host knows at its usual offset. The host keeps the part it knows and syncs on
 * the checksums the device reports in DID_FLASH_CONFIG and DID_SYS_PARAMS, which cover the device's whole struct.
 *
 * The test plays that device on the far end of an SDK bridge test port: its flash config is the host's struct plus
 * EXTRA_BYTES of appended fields, and its checksum covers all of it.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

/** Bytes the device's flash config has beyond the host's nvm_flash_cfg_t */
constexpr size_t EXTRA_BYTES = 12;

/** An IMX whose flash config is EXTRA_BYTES larger than the host's */
class NewerImx
{
public:
    explicit NewerImx(port_handle_t peer) : m_peer(peer)
    {
        memset(m_cfg, 0, sizeof(m_cfg));
        cfg().size = sizeof(m_cfg);
        cfg().ser0BaudRate = 921600;
        m_cfg[sizeof(nvm_flash_cfg_t)] = 30;     // an appended field the host does not know
        updateChecksum();
        is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf), nullptr);
        m_thread = std::thread([this] { run(); });
    }
    ~NewerImx()
    {
        m_stop = true;
        if (m_thread.joinable())
            m_thread.join();
    }

    uint32_t ser0BaudRate()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return cfg().ser0BaudRate;
    }
    uint8_t appendedField()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_cfg[sizeof(nvm_flash_cfg_t)];
    }

private:
    nvm_flash_cfg_t& cfg() { return *(nvm_flash_cfg_t*)m_cfg; }

    void updateChecksum()
    {
        cfg().checksum = 0;
        cfg().checksum = flashChecksum32(m_cfg, sizeof(m_cfg));
    }

    void send(uint16_t did, const void* data, uint16_t size)
    {
        uint8_t buf[2048];
        int n = is_comm_write_to_buf(buf, sizeof(buf), &m_comm, PKT_TYPE_DATA, did, size, 0, data);
        if (n > 0)
            portWrite(m_peer, buf, n);
    }

    void sendSysParams()
    {
        sys_params_t sp = {};
        sp.flashCfgChecksum = cfg().checksum;
        send(DID_SYS_PARAMS, &sp, sizeof(sp));
    }

    void handle(const packet_t& pkt)
    {
        uint8_t type = pkt.hdr.flags & PKT_TYPE_MASK;
        std::lock_guard<std::mutex> lock(m_mutex);
        if (type == PKT_TYPE_GET_DATA && pkt.data.size >= sizeof(uint16_t))
        {
            uint16_t id;
            memcpy(&id, pkt.data.ptr, sizeof(id));
            if (id == DID_SYS_PARAMS)
                sendSysParams();
            else if (id == DID_FLASH_CONFIG)
                send(DID_FLASH_CONFIG, m_cfg, sizeof(m_cfg));
        }
        else if ((type == PKT_TYPE_SET_DATA || type == PKT_TYPE_DATA) && pkt.id == DID_FLASH_CONFIG)
        {   // apply the write to the device's struct and report the checksum over all of it
            uint16_t offset = (pkt.hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) ? pkt.offset : 0;
            if (offset + pkt.data.size <= sizeof(m_cfg))
                memcpy(m_cfg + offset, pkt.data.ptr, pkt.data.size);
            cfg().size = sizeof(m_cfg);
            updateChecksum();
            sendSysParams();
        }
    }

    void run()
    {
        uint8_t buf[512];
        while (!m_stop)
        {
            int n = portRead(m_peer, buf, sizeof(buf));
            for (int i = 0; i < n; i++)
            {
                protocol_type_t ptype = is_comm_parse_byte_timeout(&m_comm, buf[i], current_timeMs());
                if (ptype == _PTYPE_INERTIAL_SENSE_DATA || ptype == _PTYPE_INERTIAL_SENSE_CMD)
                    handle(m_comm.rxPkt);
            }
            if (n <= 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    port_handle_t       m_peer;
    uint8_t             m_cfg[sizeof(nvm_flash_cfg_t) + EXTRA_BYTES];
    is_comm_instance_t  m_comm;
    uint8_t             m_commBuf[4096];
    std::mutex          m_mutex;
    std::atomic<bool>   m_stop{false};
    std::thread         m_thread;
};

/** An IMX in APP, identified by a real response */
std::shared_ptr<ISDevice> makeImx(port_handle_t port)
{
    dev_info_t info = {};
    info.hardwareType = IS_HARDWARE_TYPE_IMX;
    info.hardwareVer[0] = 5;
    info.serialNumber = 62797;
    info.hdwRunState = HDW_STATE_APP;
    info.protocolVer[0] = PROTOCOL_VERSION_CHAR0;
    info.protocolVer[1] = PROTOCOL_VERSION_CHAR1;
    info.protocolVer[2] = PROTOCOL_VERSION_CHAR2;
    info.protocolVer[3] = PROTOCOL_VERSION_CHAR3;
    auto device = std::make_shared<ISDevice>(info, port);
    device->devInfo = info;
    device->hdwId = ENCODE_DEV_INFO_TO_HDW_ID(info);
    device->markDevInfoConfirmed();
    return device;
}

}   // namespace

TEST(flash_cfg_size_mismatch, a_larger_device_flash_config_syncs)
{
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST4_PORT;
    ASSERT_TRUE(portIsOpened(port)) << "bridge TEST4 should be VALID|OPENED after initTestPorts()";
    NewerImx imx((port_handle_t)TEST5_PORT);
    auto device = makeImx(port);

    ASSERT_TRUE(device->WaitForImxFlashCfgSynced(false, 3000)) << "the device's flash config did not sync";
    nvm_flash_cfg_t cfg;
    device->ImxFlashConfig(cfg);
    EXPECT_EQ(cfg.ser0BaudRate, 921600u);
    EXPECT_EQ(cfg.size, sizeof(nvm_flash_cfg_t) + EXTRA_BYTES) << "the device's own size is kept";
}

TEST(flash_cfg_size_mismatch, a_write_to_a_larger_device_flash_config_confirms)
{
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST4_PORT;
    ASSERT_TRUE(portIsOpened(port));
    NewerImx imx((port_handle_t)TEST5_PORT);
    auto device = makeImx(port);

    ASSERT_TRUE(device->WaitForImxFlashCfgSynced(false, 3000));
    nvm_flash_cfg_t cfg;
    device->ImxFlashConfig(cfg);
    cfg.ser0BaudRate = 460800;
    ASSERT_TRUE(device->SetImxFlashCfgAndConfirm(cfg, 3000)) << "the write was not confirmed";
    EXPECT_EQ(imx.ser0BaudRate(), 460800u);
    EXPECT_EQ(imx.appendedField(), 30) << "the write leaves the fields the host does not know alone";

    nvm_flash_cfg_t after;
    ASSERT_TRUE(device->ImxFlashConfig(after, 3000));
    EXPECT_EQ(after.ser0BaudRate, 460800u) << "the host's copy is the device's, re-read";
}
