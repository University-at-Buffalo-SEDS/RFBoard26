#ifndef TIMESYNC_STEP_POLICY_H
#define TIMESYNC_STEP_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* The station's step policy (GroundStation26 absolute_time::may_step), applied
 * where RF leads network time from its GPS fix: a large correction may step
 * the network clock (a new SEDSnet time-base epoch) only outside Armed through
 * Recovery (network FLIGHT_STATE 6..16, Aborted and InFlight included). While
 * inhibited SEDSnet slews any correction at 20 ppm; a state not yet known
 * allows steps, as on the station. */
static inline bool timesync_steps_allowed_for(bool known, uint8_t flight_state)
{
    return !known || flight_state < 6U || flight_state > 16U;
}

#endif
