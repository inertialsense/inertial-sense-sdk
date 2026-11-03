/**
 * @file test_gpx_flash_sync.cpp
 * @brief GPX flash-config sync through an IMX without DID_GPX_DEV_INFO (SN-8867).
 *
 * A host talking to an IMX that has a GPX behind it often never asks for DID_GPX_DEV_INFO (cltool doesn't). The GPX's
 * DID_GPX_STATUS, or a pending GPX upload, is enough to show a GPX is present, so the SDK syncs the GPX's flash config
 * and WaitForGpxFlashCfgSynced() succeeds once the GPX reports the uploaded checksum.
 *
 * The test plays the IMX on the far end of an SDK bridge test port: it answers DID_GPX_STATUS and DID_GPX_FLASH_CFG
 * requests and applies DID_GPX_FLASH_CFG writes the way the IMX forwards them to its GPX. It never sends
 * DID_GPX_DEV_INFO.
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

/** The IMX + GPX at the far end of the bridge, holding the GPX's flash config */
class ImxWithGpx
{
public:
    explicit ImxWithGpx(port_handle_t peer) : m_peer(peer)
    {
        memset(&m_cfg, 0, sizeof(m_cfg));
        m_cfg.size = sizeof(m_cfg);
        m_cfg.ser2BaudRate = 921600;
        m_cfg.checksum = flashChecksum32(&m_cfg, sizeof(m_cfg));
        is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf), nullptr);
        m_thread = std::thread([this] { run(); });
    }
    ~ImxWithGpx()
    {
        m_stop = true;
        if (m_thread.joinable())
            m_thread.join();
    }

    uint32_t ser2BaudRate()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_cfg.ser2BaudRate;
    }

    std::atomic<int>    devInfoRequests{0};     //!< DID_GPX_DEV_INFO requests seen (never answered)

private:
    void send(uint16_t did, const void* data, uint16_t size)
    {
        uint8_t buf[1024];
        int n = is_comm_write_to_buf(buf, sizeof(buf), &m_comm, PKT_TYPE_DATA, did, size, 0, data);
        if (n > 0)
            portWrite(m_peer, buf, n);
    }

    void handle(const packet_t& pkt)
    {
        uint8_t type = pkt.hdr.flags & PKT_TYPE_MASK;
        std::lock_guard<std::mutex> lock(m_mutex);
        if (type == PKT_TYPE_GET_DATA && pkt.data.size >= sizeof(uint16_t))
        {
            uint16_t id;
            memcpy(&id, pkt.data.ptr, sizeof(id));
            if (id == DID_GPX_STATUS)
            {
                gpx_status_t st = {};
                st.flashCfgChecksum = m_cfg.checksum;
                send(DID_GPX_STATUS, &st, sizeof(st));
            }
            else if (id == DID_GPX_FLASH_CFG)
                send(DID_GPX_FLASH_CFG, &m_cfg, sizeof(m_cfg));
            else if (id == DID_GPX_DEV_INFO)
                devInfoRequests++;
        }
        else if ((type == PKT_TYPE_SET_DATA || type == PKT_TYPE_DATA) && pkt.id == DID_GPX_FLASH_CFG)
        {   // the IMX forwards the write; the GPX applies it and reports the new checksum in DID_GPX_STATUS
            uint16_t offset = (pkt.hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) ? pkt.offset : 0;
            if (offset + pkt.data.size <= sizeof(m_cfg))
                memcpy((uint8_t*)&m_cfg + offset, pkt.data.ptr, pkt.data.size);
            m_cfg.checksum = flashChecksum32(&m_cfg, sizeof(m_cfg));
            gpx_status_t st = {};
            st.flashCfgChecksum = m_cfg.checksum;
            send(DID_GPX_STATUS, &st, sizeof(st));
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
    gpx_flash_cfg_t     m_cfg;
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

TEST(gpx_flash_sync, write_through_an_imx_confirms_without_gpx_dev_info)
{
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST4_PORT;
    ASSERT_TRUE(portIsOpened(port)) << "bridge TEST4 should be VALID|OPENED after initTestPorts()";
    ImxWithGpx imx((port_handle_t)TEST5_PORT);
    auto device = makeImx(port);

    ASSERT_TRUE(device->WaitForGpxFlashCfgSynced(false, 3000)) << "the GPX's flash config did not sync";
    gpx_flash_cfg_t cfg;
    device->GpxFlashConfig(cfg);
    EXPECT_EQ(cfg.ser2BaudRate, 921600u);

    cfg.ser2BaudRate = 460800;
    ASSERT_TRUE(device->SetGpxFlashConfig(cfg));
    EXPECT_TRUE(device->WaitForGpxFlashCfgSynced(false, 3000)) << "the write was applied but not confirmed";
    EXPECT_EQ(imx.ser2BaudRate(), 460800u);
    EXPECT_EQ(device->gpxDevInfo.hardwareType, 0) << "precondition: the GPX was never identified";
}
