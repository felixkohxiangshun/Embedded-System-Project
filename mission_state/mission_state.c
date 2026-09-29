#include "mission_state.h"

// TODO(Felix): placeholder. See the redrawn System-Level Statechart in
// the design review assessment doc for the full state diagram this
// should implement, including which events come from which task.

static mission_state_t current_state = MISSION_INIT;

void mission_state_init(void) {
    current_state = MISSION_INIT;
}

void mission_state_update(void) {
    // TODO(Felix): state machine transitions
}

mission_state_t mission_state_get(void) {
    return current_state;
}
