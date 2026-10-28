#ifndef PROTOCOL_BITS_H
#define PROTOCOL_BITS_H

#include <stdint.h>

unsigned int protocol_bits_required(uint32_t maximum);
int protocol_bits_put(uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t value, uint32_t maximum);
int protocol_bits_get(const uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t maximum, uint32_t *value);

#endif
