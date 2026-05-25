/**
 * @file ISLogReaderSugar.cpp
 * @brief Out-of-line implementation of `extractTypedRange<T>` —
 *        D-03 / SN-7894.
 *
 * Walks the segment's raw bytes once via `is_comm_parse_byte`,
 * collecting `(TimeStamp, T)` pairs for matching ISB packets. This
 * lives in a .cpp (rather than the header) so consumers compiling
 * `ISLogReader.h` alone don't pay for `ISComm.h`'s pull-in cost —
 * the header just declares `detail::extractTypedRange`, the .cpp
 * provides it via explicit template instantiation for every
 * `DIDTraits` payload type.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLogReaderSugar.h"

// com_manager.h FIRST — see ISLogReader.cpp's note about the
// extern-C wrap collision in ISFirmwareUpdater.h.
#include "com_manager.h"

#include "ISComm.h"
#include "ISDataMappings.h"
#include "data_sets.h"

#include <cstring>

namespace inertial_sense {
namespace detail {

template <class T>
TypedRange<T> extractTypedRange(const ISLogReader& reader, did_t did) {
    typename TypedRange<T>::sample_t* unused = nullptr;
    (void)unused;

    std::vector<typename TypedRange<T>::sample_t> samples;
    auto bytes = reader.rawBytes();
    if (!bytes.first || bytes.second == 0) {
        return TypedRange<T>{ std::move(samples) };
    }

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    for (std::size_t i = 0; i < bytes.second; ++i) {
        protocol_type_t p = is_comm_parse_byte(&comm, bytes.first[i]);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
            continue;
        }
        if (comm.rxPkt.dataHdr.id != did) continue;
        if (comm.rxPkt.dataHdr.size != sizeof(T)) continue;

        const double tsSec = cISDataMappings::TimestampOrCurrentTime(
            &comm.rxPkt.dataHdr, comm.rxPkt.data.ptr);
        const uint64_t tsMs = static_cast<uint64_t>(tsSec * 1000.0);
        TimeStamp ts{ tsMs, TimeSource::PayloadToW, TimeConfidence::Exact,
                      reader.deviceId() };

        T copy;
        std::memcpy(&copy, comm.rxPkt.data.ptr, sizeof(T));
        samples.emplace_back(ts, copy);
    }
    return TypedRange<T>{ std::move(samples) };
}

// Explicit instantiations — one per `DIDTraits` payload type. Adding a
// new specialization to `DIDTraits.h` requires adding its `type` here
// too. The narrow set is intentional; auto-generation will cover it
// when that lands.
template TypedRange<ins_1_t>          extractTypedRange<ins_1_t>(const ISLogReader&, did_t);
template TypedRange<ins_2_t>          extractTypedRange<ins_2_t>(const ISLogReader&, did_t);
template TypedRange<ins_3_t>          extractTypedRange<ins_3_t>(const ISLogReader&, did_t);
template TypedRange<ins_4_t>          extractTypedRange<ins_4_t>(const ISLogReader&, did_t);
template TypedRange<imu_t>            extractTypedRange<imu_t>          (const ISLogReader&, did_t);
template TypedRange<pimu_t>           extractTypedRange<pimu_t>         (const ISLogReader&, did_t);
template TypedRange<gnss_pos_t>        extractTypedRange<gnss_pos_t>      (const ISLogReader&, did_t);
template TypedRange<gnss_vel_t>        extractTypedRange<gnss_vel_t>      (const ISLogReader&, did_t);
template TypedRange<magnetometer_t>   extractTypedRange<magnetometer_t> (const ISLogReader&, did_t);
template TypedRange<barometer_t>      extractTypedRange<barometer_t>    (const ISLogReader&, did_t);
template TypedRange<dev_info_t>       extractTypedRange<dev_info_t>     (const ISLogReader&, did_t);
template TypedRange<sys_params_t>     extractTypedRange<sys_params_t>   (const ISLogReader&, did_t);

} // namespace detail
} // namespace inertial_sense
