#ifndef _RACE_AUTOTUNE_H_
#define _RACE_AUTOTUNE_H_

/*
 * Stable wrapper for host-generated parameters.
 * The generated file is intentionally ignored by Git; race-tuner rewrites it.
 */
#include "race_autotune_generated.h"

#if (RACE_THRESHOLD_OFFSET < -40 || RACE_THRESHOLD_OFFSET > 40)
#error "RACE_THRESHOLD_OFFSET outside safe range"
#endif

#if (RACE_STEER_SAMPLE_ROW_FROM_BOTTOM < 8 || RACE_STEER_SAMPLE_ROW_FROM_BOTTOM > 80)
#error "RACE_STEER_SAMPLE_ROW_FROM_BOTTOM outside safe range"
#endif

/* Floating-point limits are enforced by the host-side config validator.
 * The C preprocessor cannot portably compare floating-point constants. */

#endif

