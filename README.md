# DhakaSim Update, Crash False Positive Reduction

This repository contains my DhakaSim changes focused on reducing false crash detections while keeping safety events traceable and tunable.

## Why this update exists
The previous crash logic could label normal close driving as a crash, especially in dense mixed traffic where vehicles often move side by side. It also injected random object accidents in a way that made results inconsistent across repeated runs.

My goal was to make a crash detection:
* More realistic in crowded Dhaka-like traffic
* Less prone to false positives
* Deterministic and reproducible
* Easy to tune without rewriting core logic

## What changed in this update
All key updates are in three files:
* `Parameters.java`
* `Strip.java`
* `Vehicle.java`

### 1) Added new tunable safety and lane change parameters
File: `Parameters.java`

I added a small set of parameters to control:
* How do we separate contact vs minor crash vs major crash using relative speed difference
* How much side-to-side overlap is required before we consider it a collision
* How lane change spacing behaves at very low speeds

These make the safety layer adjustable for Dhaka like density without changing code logic each time.

### 2) Removed random object accident injection, made object overlap deterministic
File: `Strip.java`, method `hasGapForObject(Object p).`

Before:
* Overlap with an object could randomly become an accident
* Accident counters and object state could change inside a space check
* Objects could be deleted as a side effect of a gap query

Now:
* Overlap is handled deterministically
* On overlap, the simulator logs an event as contact or crash_minor based on speed
* The function returns false immediately, meaning no gap
* It no longer deletes objects or mutates accident counters inside this feasibility check

### 3) Introduced 2D collision confirmation and severity classification
File: `Strip.java`, new logic via `lateralOverlap(...)` and `registerContactOrCrash(Vehicle v)`

Before:
* Collision checks were mainly front-to-back, which could misclassify side-by-side cases

Now:
* A collision is only considered if vehicles overlap both:
  * Longitudinally, front to back
  * Laterally, side to side, using a tunable overlap fraction
* Severity is classified using relative speed difference:
  * Very small delta v becomes contact
  * Moderate delta v becomes crash_minor
  * Large delta v becomes crash_major
* Events are logged through `segment.logEvent(...)` for later auditing

### 4) Lane change updates, realistic low speed squeezing plus a safety gate
Files: `Parameters.java`, `Strip.java`, `Vehicle.java`

What was happening before:
* Lane change feasibility used one fixed spacing rule at all speeds
* Lane change movement could scan across multiple sideways positions in one call
* There was no consistent final safety gate tied to the new crash logic

What I implemented:
* Speed-aware lane change spacing in `Strip.hasGapForStripChange(...)`
  * At low speeds, the required spacing is reduced but bounded by a minimum
  * At higher speeds, the original spacing is kept
* One-step provisional lane change in:
  * `Vehicle.moveToHigherIndexLane()`
  * `Vehicle.moveToLowerIndexLane()`
  The vehicle tries one sideways shift, checks space and safety, then either commits or reverts immediately
* A safety hook during lane change:
  * `Vehicle.tryLaneChange(...)` consults the safety layer
  * `Vehicle.sweepLaneChangeSafety(...)` checks the target strip using the same crash logic

### 5) Vehicle collision detection now uses the new classifier
File: `Vehicle.java`, method `hasCollided().`

Before:
* `hasCollided()` relied on a simpler collision check

Now:
* `hasCollided()` uses `Strip.registerContactOrCrash(this)`
* Contact does not count as a crash in the final crash statistics
* Crash events are classified and logged

Note:
* Visibility of `hasCollided()` changed from public to package scope in my update, so external package callers may need adjustment.

## New parameters and default values
These are the defaults I introduced in `Parameters.java`.

### Crash severity and overlap
* `ACCIDENT_DELTA_V_CONTACT = 0.5` m/s  
  Below or equal to this, overlap is treated as contact, not a crash
* `ACCIDENT_DELTA_V_MINOR = 1.5` m/s  
  Reserved for finer severity splits, not used in these three files yet
* `ACCIDENT_DELTA_V_MAJOR = 5.0` m/s  
  Above or equal to this, crash_major is logged
* `LATERAL_OVERLAP_FRACTION = 0.40`  
  Minimum side-to-side overlap fraction required to consider collision interaction

### Lane change tuning
* `LANECHANGE_SWEEP_SAMPLES = 1`  
  Minimum is clamped to 1 in code
* `LANECHANGE_LOW_SPEED = 2.0` m/s  
  Below this, low-speed lane change rules apply
* `LANECHANGE_GAP_FACTOR = 0.6`  
  Scales down the normal lane change spacing at low speed
* `LANECHANGE_MIN_GAP = 0.10` m  
  Minimum spacing floor at low speed

## Event logging
Safety events are logged using `segment.logEvent(...)` with labels:
* `contact`
* `crash_minor`
* `crash_major`

Each event includes the simulation step and involved vehicle ids, and a severity proxy based on delta v.

## Known limitations and notes
* `LANECHANGE_SWEEP_SAMPLES` currently repeats the same safety check multiple times unless intermediate positions are explicitly modeled later; it is a hook for future refinement
* Pair event logging can be duplicated if both vehicles register the same interaction in the same step; deduplication can be added later if needed
* If you change the thresholds aggressively, you can shift counts between contact and crash, so calibration should be done using consistent scenarios

## Files changed
* `thesisfinal/Parameters.java`
* `thesisfinal/Strip.java`
* `thesisfinal/Vehicle.java`



## Attribution
DhakaSim baseline code and simulator structure were provided by the original project authors.
This update focuses on safety logic corrections and tuning hooks for Dhaka like traffic.
