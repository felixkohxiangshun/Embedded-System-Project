/* Host-only tests: independent known vectors plus malformed/boundary inputs. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../../app_program/dashboard/protocol_bits.h"

int main(void)
{
    uint8_t packet[5] = {0};
    uint32_t cursor = 0, value = 99;
    unsigned int i;
    assert(protocol_bits_required(0) == 0);
    assert(protocol_bits_required(4) == 3);
    assert(protocol_bits_required(10) == 4);
    assert(protocol_bits_required(UINT32_MAX) == 32);

    /* Artificial test fields only: 5 (3 bits), 17 (5), 0x123 (9).
     * Wire integer = 5 | (17 << 3) | (0x123 << 8) = 0x01238d. */
    assert(protocol_bits_put(packet, 3, &cursor, 5, 7));
    assert(protocol_bits_put(packet, 3, &cursor, 17, 31));
    assert(protocol_bits_put(packet, 3, &cursor, 0x123, 511));
    assert(cursor == 17 && packet[0] == 0x8d && packet[1] == 0x23 && packet[2] == 1);
    cursor = 0;
    assert(protocol_bits_get(packet, 3, &cursor, 7, &value) && value == 5);
    assert(protocol_bits_get(packet, 3, &cursor, 31, &value) && value == 17);
    assert(protocol_bits_get(packet, 3, &cursor, 511, &value) && value == 0x123);

    cursor = 0;
    assert(!protocol_bits_put(packet, 3, &cursor, 11, 10));
    assert(cursor == 0 && packet[0] == 0x8d);
    packet[0] = 15;
    assert(!protocol_bits_get(packet, 1, &cursor, 10, &value));
    assert(cursor == 0 && value == 0x123);
    cursor = 7;
    assert(!protocol_bits_put(packet, 1, &cursor, 3, 3));
    assert(!protocol_bits_get(packet, 1, &cursor, 3, &value));
    assert(cursor == 7 && packet[0] == 15);
    cursor = UINT32_MAX;
    assert(!protocol_bits_put(packet, 1, &cursor, 0, 1));
    assert(!protocol_bits_get(packet, 1, &cursor, 1, &value));
    cursor = 0;
    assert(protocol_bits_put(packet, 0, &cursor, 0, 0) && cursor == 0);
    assert(protocol_bits_get(packet, 0, &cursor, 0, &value) && value == 0);
    assert(!protocol_bits_put(0, 1, &cursor, 0, 1));
    assert(!protocol_bits_get(packet, 1, 0, 1, &value));
    assert(!protocol_bits_get(packet, 1, &cursor, 1, 0));

    /* Every field width and bit offset, including a full-width 32-bit code.
     * Start with ones: writing zeros must clear old bits. */
    for (i = 1; i <= 32; ++i) {
        unsigned int offset;
        uint32_t maximum = UINT32_MAX >> (32u - i);
        for (offset = 0; offset < 8; ++offset) {
            unsigned int j;
            for (j = 0; j < sizeof(packet); ++j) packet[j] = 0xff;
            cursor = offset;
            assert(protocol_bits_put(packet, 5, &cursor, 0, maximum));
            cursor = offset;
            assert(protocol_bits_get(packet, 5, &cursor, maximum, &value) && value == 0);
            cursor = offset;
            assert(protocol_bits_put(packet, 5, &cursor, maximum, maximum));
            cursor = offset;
            assert(protocol_bits_get(packet, 5, &cursor, maximum, &value) && value == maximum);
        }
    }
    puts("protocol_bits: known vector, invalid inputs, all widths/offsets PASS");
    return 0;
}
