# yeastbasicvelocityfilter - Growth Opportunities

## Improvement Opportunities

### 1. Actually use the filter output
The most critical fix: `src/yeastbasicvelocityfilter.cpp:17-21` should assign from the `step()` return value, not from the original command input.

### 2. Parse configuration from JSON
The constructor should read `acceleration_limit`, `deceleration_limit`, `snap_to_zero_cutoff`, and `dt` from the JSON config parameter instead of using hardcoded zero defaults.

### 3. Filter rotational velocity (omega) too
Either extend `VelocityCommandFilter2D` to handle 3-component twist, or add a separate 1D filter for the omega component.

### 4. Use actual dt from the control loop
Replace the hardcoded `0.01` with a timestep derived from actual elapsed time between calls (similar to how the path follower uses `std::chrono`) or from the configuration.

### 5. Fix the filename typo
Rename `velocitrycommandfilter2d.hpp` to `velocitycommandfilter2d.hpp`.

## Architectural Enhancements

### 6. Make `BasicVelocityFilter` publicly inherit from `DriveFilter`
Change `class BasicVelocityFilter : DriveFilter` to `class BasicVelocityFilter : public DriveFilter` so it can be used polymorphically as intended by the interface.

### 7. Consider a more sophisticated filter model
The current trapezoidal profile (accel/decel limits + snap-to-zero) does not account for:
- Jerk limiting (rate of change of acceleration)
- Direction-dependent limits (different max accel forward vs. lateral)
- Current measured velocity (only the commanded velocity is filtered)

### 8. Separate the 2D velocity filter into its own library
`VelocityCommandFilter2D` is a general-purpose rate limiter with no yeast dependencies. It could be a standalone utility.

## Testing Gaps

### 9. No unit tests
The filter is essentially non-functional (output unused, config all zeros), so tests would immediately reveal these bugs. Needed tests:
- `step()` with acceleration above the limit, verify clamping
- `step()` with deceleration, verify decel limit applies
- Snap-to-zero behavior at low velocities
- `is_zero()` behavior with small but nonzero values
- `unit_vector()` of a zero vector
- Round-trip: set velocity, step toward new target, verify convergence
- `filter()` integration: verify output differs from input when limits apply
