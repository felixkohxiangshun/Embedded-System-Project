/* Range-based binary fields. Proposed packing convention, not a robot schema. */
#ifndef PROTOCOL_BITS_H
#define PROTOCOL_BITS_H

#include <stdint.h>

/* Fields are unsigned codes 0..maximum. Signed/scaled physical quantities must
 * first be mapped using the agreed schema. Bits advance least-significant bit
 * first within each byte and within each field; no native struct is transmitted.
 * Caller owns the buffer exclusively. Zero it before encoding a new packet so
 * unused trailing bits are canonical zero. Functions return 1 on success, 0 on
 * rejection; rejection changes neither cursor, output value nor buffer.
 * A maximum of zero represents a constant field and consumes zero bits.
 * Length/version/padding/freshness validation belongs to the packet layer.
 */
unsigned int protocol_bits_required(uint32_t maximum);
int protocol_bits_put(uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t value, uint32_t maximum);
int protocol_bits_get(const uint8_t *data, uint16_t bytes, uint32_t *cursor,
                      uint32_t maximum, uint32_t *value);

#endif
