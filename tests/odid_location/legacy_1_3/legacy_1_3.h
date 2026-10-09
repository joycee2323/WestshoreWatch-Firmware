#pragma once
#include <stdint.h>
void legacy13_relay(const uint8_t basic_msg[25], const uint8_t loc_msg[25], const uint8_t sys_msg[25],
                    uint8_t pack_out[77], uint8_t handle0_loc_out[25]);
