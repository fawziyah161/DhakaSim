**Overview:** DhakaSim is a microsimulation of dense, laneless urban corridors with heterogeneous users (cars, rickshaws, bikes, pedestrians). The original build was over-reporting “accidents,” especially in queues and during merges.


**Problem Statement:** Our microsimulator was flagging too many “accidents” in Dhaka-style, laneless, pedestrian-rich traffic. Slow squeezes in queues, gentle side brushes, and mid-merge overlaps were often counted as crashes—even when the relative speeds were tiny and no real harm would occur on the street. 
 
 
**Objectives:**
Distinguish contacts (low-severity, low Δv) from crashes (higher-severity events).
 Prevent spurious lane-change collisions by evaluating the full lateral maneuver.
 Make results calibratable and auditable for planning and policy analysis.



**Methodology:**
Severity-aware event logic: A collision is registered only when bodies overlap in two dimensions (lateral + longitudinal) and the closing speed exceeds a threshold; otherwise, the event is logged as a contact.
Continuous lane-change safety: Lane changes use multi-step safety checks across the lateral path; if any step indicates impending overlap, the maneuver aborts and reverts.
Deterministic pedestrian/object interaction: Replaced random incidents with geometry + speed checks; slow brushes are recorded as contacts, not crashes.
Parameterization and logging: Exposed thresholds (Δv cutoffs, lateral overlap fraction, sweep samples) and added structured logs (contact/crash, lane-change start/abort/commit).



**Technical Contributions :**
Replaced point-based hits with 2D body-overlap + Δv classification (contact / minor/major).
Implemented continuous (swept) lane-change checks to eliminate mid-step artifacts.
Introduced deterministic pedestrian/object conflict handling and separate contact/crash counters.
Made the model tunable and reproducible via parameters and event logs.



**Evaluation & Results :**
Across multiple scenarios (original vs. improved logic), reported “accidents” decrease substantially while major events remain detectable. Lane-change crashes drop and abort rates rise where gaps are unsafe, matching observed behavior on laneless corridors. 
Original mean: 36.0 accidents per run
Improved mean: 14.9 accidents per run
Average absolute reduction: 21.1 accidents per run
Average percent reduction across pairs: 56.8% (median 60.5%, range 45–69.8%)
Overall reduction (sum over all runs): 58.6%




**Implications :**
Credible safety indicators: Reduced over-counting in congestion; analyses reflect severity, not mere proximity.
Fair comparison of low-cost treatments: Supports evaluation of speed management, markings, refuge islands, and curb management without simulator artifacts.
Actionable KPIs: Enables severity-aware metrics (e.g., crashes per 1,000 veh-km; lane-change crash rate per 1,000 attempts; lane-change abort rate; contact-to-crash ratio).
Local calibration: Thresholds can be tuned from short corridor video to align with local driving culture and modal mix.
 
