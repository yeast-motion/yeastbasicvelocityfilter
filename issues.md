# yeastbasicvelocityfilter - Issues

## Bugs / Correctness Problems

### 1. `filter()` result does not use the filtered output
In `src/yeastbasicvelocityfilter.cpp:13-17`, the `step()` return value is discarded:
```cpp
filter_object->step(velocity, 0.01); // return value ignored
result_command.velocity.x = command.velocity.x;  // uses original input, not filtered output
result_command.velocity.y = command.velocity.y;
```
The filter is applied but its output is thrown away. The method returns the original unfiltered command. This means the entire filter does nothing.

### 2. Hardcoded timestep of 0.01 seconds
`src/yeastbasicvelocityfilter.cpp:17`: `filter_object->step(velocity, 0.01)` hardcodes a 10ms timestep. The comment says `// NEED TO USE THE CONFIGURATION DT HERE`, confirming this is a known incomplete implementation. If the actual control loop runs at a different rate, acceleration limits will be wrong.

### 3. Constructor ignores the JSON configuration
`src/yeastbasicvelocityfilter.cpp:5-9`: The constructor takes `nlohmann::json` but ignores it (parameter named `/*json*/`). A default `VelocityCommandFilter2D::Configuration` is created with all zeros for `acceleration_limit`, `deceleration_limit`, and `snap_to_zero_cutoff`. With zero acceleration limits, the filter will pass through all velocities unchanged (even if the output were used).

### 4. Omega (rotational velocity) is not filtered
`src/yeastbasicvelocityfilter.cpp:12-15` only filters `x` and `y` velocity components. `command.velocity.omega` passes through unfiltered. This could cause dangerous rotational acceleration spikes.

### 5. `Velocity` default constructor delegates incorrectly
In `velocitrycommandfilter2d.hpp:24-27`:
```cpp
Velocity() { Velocity(0,0); }
```
This creates a temporary `Velocity(0,0)` and immediately destroys it. The default constructor's `this` object retains the member initializer values (which happen to be 0.0 from the in-class initializers). This works by accident but the delegation is a no-op.

### 6. `is_zero()` uses `std::numeric_limits<float>::min()` instead of `epsilon()`
`velocitrycommandfilter2d.hpp:36-37`: `std::numeric_limits<float>::min()` is the smallest positive normalized float (~1.175e-38), not the precision threshold. This effectively makes `is_zero()` only return true for values extremely close to zero (essentially only +0 and -0), making the snap-to-zero logic overly sensitive. Should use `std::numeric_limits<float>::epsilon()` or a small domain-appropriate threshold.

### 7. Filename typo: `velocitrycommandfilter2d.hpp`
`include/yeastbasicvelocityfilter/velocitrycommandfilter2d.hpp` -- "velocitr**y**" should be "velocit**y**".

## Code Smells

### 8. `VelocityCommandFilter2D` is header-only but contains mutable state
The entire `VelocityCommandFilter2D` class (150 lines) is implemented in a header file with no corresponding .cpp. This is fine for template code but unusual for a stateful class and increases compile times for all includers.

### 9. `BasicVelocityFilter` uses private inheritance from `DriveFilter`
`yeastbasicvelocityfilter.hpp:10`: `class BasicVelocityFilter : DriveFilter` (private inheritance). This means it cannot be used polymorphically through a `DriveFilter*` pointer, which defeats the purpose of the interface.

### 10. The `state` parameter in `filter()` is unused
`src/yeastbasicvelocityfilter.cpp:11`: `MotionState /*state*/` is accepted but not used. The filter should ideally consider the current state to make informed filtering decisions (e.g., don't accelerate past physical limits given current velocity).
