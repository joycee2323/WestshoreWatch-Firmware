#pragma once
#include <stdint.h>
#include "odid_decoder.h"

/* Relay format advertised in the identity advert as "rf=<n>" (ble_relay.c):
 *   rf absent / 1  = pre-1.4 relays: relayed Location used a non-spec byte 1-2
 *                    layout (E/W in byte 1 bit 0, direction/2 + multiplier in
 *                    byte 2) carrying values from the legacy decode.
 *   rf=2           = relayed Location follows ASTM F3411 / opendroneid
 *                    ODID_Location_encoded; it is the drone's own message,
 *                    forwarded byte for byte. */
#define WSD_RELAY_FORMAT 2

/* Write the 25-byte Location message to re-broadcast for `loc`.
 * The drone's own bytes (loc->raw) are forwarded unchanged when present.
 * Otherwise the fields are encoded per opendroneid encodeLocationMessage()
 * (unknown heading -> 361, unknown speed -> 255 m/s). */
void odid_encode_location(const odid_location_t *loc, uint8_t buf[25]);
