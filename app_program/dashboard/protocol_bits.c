/**
 * @file protocol_bits.c
 * @brief Bit packing, least significant bit first, for the minimum-bit
 *        frames.  See protocol_bits.h.
 */

#include "protocol_bits.h"

unsigned int protocol_bits_required(uint32_t maximum)
{
    unsigned int bits = 0;
    while (maximum != 0) {
        ++bits;
        maximum >>= 1;
    }
    return bits;
}

static int fits(uint16_t bytes, uint32_t cursor, unsigned int bits)
{
    uint32_t capacity = (uint32_t)bytes * 8u;
    return cursor <= capacity && bits <= capacity - cursor;
}

int protocol_bits_put(uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t value, uint32_t maximum)
{
    unsigned int bits = protocol_bits_required(maximum);
    unsigned int i;
    if (data == 0 || cursor == 0 || value > maximum ||
        !fits(bytes, *cursor, bits)) {
        return 0;
    }
    for (i = 0; i < bits; ++i) {
        uint32_t position = *cursor + i;
        uint8_t mask = (uint8_t)(1u << (position % 8u));
        uint8_t *byte = &data[position / 8u];
        *byte = (uint8_t)((*byte & (uint8_t)~mask) |
                         (((value >> i) & 1u) ? mask : 0u));
    }
    *cursor += bits;
    return 1;
}

int protocol_bits_get(const uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t maximum, uint32_t *value)
{
    unsigned int bits = protocol_bits_required(maximum);
    unsigned int i;
    uint32_t decoded = 0;
    if (data == 0 || cursor == 0 || value == 0 ||
        !fits(bytes, *cursor, bits)) {
        return 0;
    }
    for (i = 0; i < bits; ++i) {
        uint32_t position = *cursor + i;
        decoded |= (uint32_t)((data[position / 8u] >> (position % 8u)) & 1u) << i;
    }
    if (decoded > maximum) {
        return 0;
    }
    *value = decoded;
    *cursor += bits;
    return 1;
}

/*** end of file ***/
