/**
 * @file ISStatusDecode.h
 * @brief Structured, machine-readable decode of status / bitfield / enumerated
 *        device fields (insStatus, hdwStatus, GNSS status, ...).
 *
 * The hand-written `render*` functions in `ISDataMappings.cpp` historically
 * encoded each status field's bit semantics as inline prose (one
 * `std::stringstream` line per set bit / decoded sub-field). That is fine for
 * a human tooltip but useless to a consumer that needs the *structure* — e.g.
 * Logalyzer's status-ribbon chart (SN-7919), which must draw one row per
 * sub-field and color each segment by its decoded value.
 *
 * This header defines a declarative descriptor model for those fields. The
 * descriptor tables are the single structural source of truth: masks, shifts,
 * value->label maps, and error classification live here, referencing the
 * `data_sets.h` enum/macro symbols by name (never transcribed hex). The legacy
 * `render*` functions are refactored to render *from* these tables via
 * `RenderStatusFromDecode`, so their output stays byte-identical (guarded by a
 * round-trip unit test) while the structure becomes queryable.
 *
 * @note D-53 / SN-7919.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_STATUS_DECODE_H
#define IS_STATUS_DECODE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/** @brief Kind of a decoded sub-field within a status value. */
enum class eStatusSubfieldKind : uint8_t
{
    Bit,    //!< Single on/off flag; `mask` selects exactly one bit.
    Enum,   //!< Multi-bit sub-field decoded to a named state via (mask, shift) + a value table.
    Count,  //!< Multi-bit sub-field reported as an integer count via (mask, shift).
};

/** @brief One (value -> label) entry for an `Enum` sub-field. `value` is post-shift. */
struct status_value_label_t
{
    uint32_t    value;        //!< Decoded value AFTER `(raw & mask) >> shift`.
    std::string label;        //!< Clean human label (UI / status-ribbon legend).
    std::string legacyText;   //!< Verbose line the legacy render* emitted; empty -> use `label`.
    bool        isError;      //!< This particular state is an error / fault condition.
};

/** @brief One decodable sub-field of a status field. */
struct status_subfield_t
{
    std::string                       name;                                    //!< Clean row label (status-ribbon row title).
    eStatusSubfieldKind               kind     = eStatusSubfieldKind::Bit;     //!< Bit / Enum / Count.
    uint32_t                          mask     = 0;                            //!< Bit(s) this sub-field occupies in the raw value.
    uint32_t                          shift    = 0;                            //!< Right-shift for Enum/Count extraction; 0 for Bit.
    bool                              isError  = false;                        //!< Bit: this flag is an error. Enum/Count: sub-field is error-bearing (per-value override via `values`).
    uint32_t                          gateMask = 0;                            //!< If non-zero, only decode/emit when `(raw & gateMask) != 0` (hybrid pattern). 0 = always.
    bool                              emitZero = false;                        //!< Count: emit a line even when the extracted value is 0 (e.g. GNSS satellite count). Default: skip zero.
    std::string                       legacyText;                              //!< Bit/Count verbose line for byte-identical render*; a Count line may contain printf conversions (the value is supplied up to twice). Empty -> use `name`.
    std::vector<status_value_label_t> values;                                  //!< Enum only: the value table.
};

/** @brief Structured decode for one named status field. */
struct status_field_decode_t
{
    std::string                    fieldName;   //!< e.g. `"insStatus"`.
    uint32_t                       errorMask;   //!< `(raw & errorMask) != 0` => field is in an error state.
    std::vector<status_subfield_t> subfields;   //!< In legacy emission order.
};

/**
 * @brief Reproduce the legacy multi-line human string for a status value from its decode table.
 *
 * Iterates `dec.subfields` in order, emitting one newline-terminated line per set bit, matching
 * enum value, or non-zero count — exactly as the hand-written render* functions did.
 *
 * @param dec   The field's decode table.
 * @param value The raw status value.
 * @return The rendered multi-line string (may be empty if nothing is set).
 */
std::string RenderStatusFromDecode(const status_field_decode_t& dec, uint32_t value);

/**
 * @brief Look up the structured decode for `(did, fieldName)`.
 *
 * @param did       DID id (accepted for future DID-specific variants; v1 dispatches on field name,
 *                  since shared fields like `insStatus` carry identical semantics across DIDs).
 * @param fieldName Field name, e.g. `"insStatus"`.
 * @return Pointer to the decode table, or `nullptr` when the field has no structured decode.
 */
const status_field_decode_t* GetStatusDecode(uint32_t did, const std::string& fieldName);

/**
 * @brief Convenience: look up a structured decode by field name alone.
 *
 * @param fieldName Field name, e.g. `"insStatus"`.
 * @return Pointer to the decode table, or `nullptr`.
 */
const status_field_decode_t* GetStatusDecodeByField(const std::string& fieldName);

#endif // IS_STATUS_DECODE_H
