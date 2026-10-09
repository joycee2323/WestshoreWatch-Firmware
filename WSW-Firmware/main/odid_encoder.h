#pragma once
#include <stdint.h>
#include "odid_decoder.h"

/* Relay format, advertised two ways (both invisible to shipped apps):
 *   - Pack byte 1 bits 5-7 (shipped TS/Kotlin/Swift read the count as & 0x1F);
 *   - identity advert (0x08FE) fw tag ";rf=<n>".
 * rf absent / 0 / 1 = pre-1.4 relay. rf=2 = this layout:
 *
 *   Westshore relay Pack (FFFA service data after [app code 0x0D][counter]):
 *     [0]      0xF2  (type Pack, protocol 2)
 *     [1]      (WSD_RELAY_FORMAT << 5) | WSD_PACK_MSG_COUNT
 *     [2..26]  Basic ID
 *     [27..51] Location in the pre-1.4 relay layout (what shipped apps decode,
 *              byte for byte identical to 1.3 output; see odid_encode_location_legacy)
 *     [52..76] System (ASTM layout; shipped apps read only bytes 2-9)
 *     [77..101] the drone's own ASTM Location message, byte 0 type nibble set to
 *              WSD_MSG_SPEC_LOCATION (0xE, reserved in F3411) so shipped apps skip it.
 *              Restore type 0x1 to get the drone's original 25 bytes.
 * This is NOT a spec Message Pack (spec has message size 25 at byte 1, count at
 * byte 2, messages from byte 3); spec parsers reject it, as they always have.
 * Handle 0 per-message ODID keeps the pre-1.4 Location layout too. */
#define WSD_RELAY_FORMAT       2
#define WSD_MSG_SPEC_LOCATION  0xE
#define WSD_PACK_MSG_COUNT     4
#define WSD_PACK_PAYLOAD       (2 + 25 * WSD_PACK_MSG_COUNT)   /* 102 */

/* The drone's own Location bytes (loc->raw) when present, else a spec encode
 * mirroring opendroneid encodeLocationMessage(). */
void odid_encode_location(const odid_location_t *loc, uint8_t buf[25]);

/* The Location bytes a pre-1.4 relay emitted for the same received frame:
 * the old (non-spec) decode of the drone's bytes, re-encoded by the old
 * encoder. Byte for byte identical to 1.3 output (test_relay_compat). */
void odid_encode_location_legacy(const odid_location_t *loc, uint8_t buf[25]);

/* System message in the ASTM layout. Operator lat/lon (bytes 2-9, the only
 * bytes shipped apps read) are computed exactly as the pre-1.4 relay did. */
void odid_encode_system(const odid_system_t *sys, uint8_t buf[25]);

/* Assemble the relay Pack above. */
void odid_build_relay_pack(const uint8_t basic_id[25], const odid_location_t *loc,
                           const uint8_t system[25], uint8_t out[WSD_PACK_PAYLOAD]);
