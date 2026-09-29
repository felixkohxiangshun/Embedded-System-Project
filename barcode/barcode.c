#include "barcode.h"
#include "config.h"

// TODO(Vania): placeholder - see Subsystem Statechart 6.1 (Barcode
// Decoding and IR Line Following) in the design review for the
// IDLE -> SCANNING -> BARCODE_DETECTION -> BARCODE_DECODING ->
// COMMAND_OUTPUT state flow this is expected to follow, including the
// ERROR_HANDLING fallback on a decode failure.

void barcode_init(void) {
}

void barcode_on_edge_detected(void) {
    // TODO(Vania): decode logic
}
