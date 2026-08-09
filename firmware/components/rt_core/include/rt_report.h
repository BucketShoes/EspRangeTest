// Renders the observation table as text.
//
// This is the M0 deliverable: the rig has to be observable before any radio work can be
// called done, and a serial dump gets there with no UI, no phone and no second board. The
// same numbers later go to the browser over CBOR - this is the reference rendering, and it
// stays pure (snprintf only, no ESP-IDF) so it is host-testable.

#pragma once

#include <stddef.h>

#include "rt_mode.h"
#include "rt_obs.h"

// Writes a NUL-terminated table into buf. Returns the number of characters written,
// excluding the terminator; output is truncated rather than overflowed.
size_t rt_report_render(const rt_obs_table_t *t, const rt_mode_state_t *mode,
                        uint8_t self_node_id, uint32_t now_ms, char *buf, size_t buf_len);
