# Experimental lane warnings with steering assistance on or off

Based on sunnypilot v2026.002.002, commit c440933a65fa2c669a9926c8112ae352d3600f07.

Branch: `l` (lowercase L). Installer: https://installer.comma.ai/TwelveInchPianist/l
The longer branch active-ldw-2026-002-002 is also maintained at this revision.

Lane Departure Warnings, Predictive Lane Warnings (Experimental), and Low-Speed Lane Warnings default on for unset parameters. Existing stored choices are preserved; switches can still be turned off.
The toggle is currently exposed in the comma 3/3X Python settings UI, not the comma four UI or sunnylink remote settings.
The parameter is persistent and included in backups.

## Review fixes

- Either blinker or a model-reported lane-change state other than off suppresses both sides, with a five-second cooldown after both are inactive. This deliberately sacrifices warnings during that interval to avoid opposite-side alerts when lane labels switch.
- A fired latch survives brief invalid inputs, confidence dropouts, speed/lateral-active interruptions and boundary source changes. Partial confirmation, active output pulses and clearance timers are cancelled immediately. State expires after two seconds without a usable observation on that side. Otherwise rearming requires 0.35 m predicted clearance for 0.5 seconds.
- Lane probability enters at 0.65 and can be retained down to 0.50. Boundary geometry and standard deviation checks still apply.
- Unmarked road-edge fallback is disabled by default pending real-data evaluation of roadEdgeStds. There is no new UI setting enabling it. Marked shoulder lines remain eligible.
- Runtime exceptions in the added detector/parameter/clock block are logged once and disable the addition until planner restart. Longitudinal planning continues and stock LDW is selected as fallback. The added detector is not called with its toggle off. Hosts lacking CLOCK_BOOTTIME skip the addition instead of mixing clock domains.

## Validation

58 synthetic tests pass. Two added regression tests reproduced signalled lane-change label-switch alerts and repeated alerts following confidence dropouts on the previous code, and now pass.
Tests include the actual planner main with external/native dependencies stubbed, exception injection, absent BOOTTIME, disabled feature, lane-change suppression, confidence hysteresis, latch retention/expiry and source transitions.

From repository root, run:

```sh
python3 -m unittest discover -s tests -p test_active_ldw.py -v
```

The user reported the prior revision's installer completed downloading/installing. This revision has not been natively built or installed here. No recorded-drive replay or road validation has occurred. There is no prebuilt marker; source installation requires a device build.

## Remaining limitations

Vehicle half-width is an approximate 1.0 m body envelope, not calibrated tire or full footprint tracking. Prediction assumes constant speed and current estimated curvature over the selected 0.5 / 0.8 / 1.1 second horizon. Speed must exceed 2 mph with the default settings, or 31 mph when Low-Speed Lane Warnings is disabled; fresh valid data and calibration are required. Weak or stale detections silently suppress the added warning. Existing higher-priority alerts may supersede it.

A long data absence or explicit detector reset can rearm while still straddling a boundary. Five-second suppression does not prove an unusually long lane change has finished. Latch retention after a boundary-identity change can suppress a genuinely new event temporarily. Unmarked shoulder crossings are not covered by default. Thresholds, false-alarm rate and warning timing need real-drive evaluation; no guarantee is made of a warning on every departure.

Steering, braking, panda safety and driver monitoring code are unchanged. Successful compilation is not driving validation.

## In-car tuning

Settings > Toggles now includes **Low-Speed Lane Warnings (Experimental)** and **Lane Warning Sensitivity**.

Low speed defaults on for unset parameters. When enabled, the added detector operates strictly above 2 mph (3.2 km/h), requiring both LDW master toggles, fresh data, valid calibration, and existing suppression checks. This also covers manual driving while the onroad perception/planning processes are running. Tight turns beyond the existing curvature limit remain unavailable.

| Setting | Prediction horizon | Boundary margin |
| --- | --- | --- |
| Later | 0.5 s | 5 cm |
| Standard (default) | 0.8 s | 10 cm |
| Earlier | 1.1 s | 20 cm |

These are prediction parameters, not guaranteed warning lead times. Earlier can increase false alerts. The 0.15-second confirmation, confidence gates, five-second lane-change suppression, width proxy and disabled road-edge fallback do not change. Longer horizons require correspondingly longer model boundary coverage.

Settings persist and are backed up. The planner polls them every second; changing them does not request a restart. Reconfiguration preserves existing warning latches and signal history and restarts partial confirmation/rearm timers. Malformed sensitivity values fall back to Standard.

Tests cover speed thresholds (including standstill and exactly 2 mph), profile timing order, default equivalence, latch preservation, live planner polling, and actual settings-layout construction/callbacks with rendering dependencies stubbed. Native UI rendering and hardware performance have not been validated here.

The first update introducing these controls requires rebuilding native Params bindings and restarting. Subsequent in-car adjustments do not.

## Steering-independent operation

Predictive Lane Warnings now runs with lateral steering assistance on or off. Enable Enable Lane Departure Warnings, Predictive Lane Warnings, and Low-Speed Lane Warnings for detection strictly above 2 mph. Low-Speed remains an explicit stored setting; turning it off restores the 31 mph threshold. Sensitivity remains adjustable in-car.

The old internal key LdwWhileSteering is retained to preserve existing settings, despite the new UI name.
Only one detector supplies alerts at a time. When predictive LDW is enabled and available, its output replaces stock LDW rather than being ORed with it. This prevents legacy logic from bypassing predictive suppression or re-triggering latched warnings. A runtime fault or host without BOOTTIME falls back to stock LDW, whose original speed/engagement limits apply. Invalid perception simply suppresses predictive output.

Five added regression tests cover both steering states around 2 mph and at higher speeds, manual-mode planner output, detector selection/fallback, and latch continuity across steering-engagement transitions. All 58 synthetic tests pass. This is not a hardware or road-validation result.

## Default-enabled settings

All three LDW switches have registered defaults of true. Manager initializes only unset parameters on startup, so existing saved off values remain off. After updating and rebuilding, existing installations can enable all three once over SSH:

```sh
cd /data/openpilot
python3 -c 'from openpilot.common.params import Params; p = Params(); [p.put_bool(k, True, block=True) for k in ("IsLdwEnabled", "LdwWhileSteering", "LdwLowSpeed")]'
```

This does not force them on at every boot. Sensitivity still defaults to Standard; all suppression and perception checks remain in place.

## Lane-change default

Auto Lane Change by Blinker defaults to Off (`AutoLaneChangeTimer = -1`), including nudge-triggered assisted lane changes. Stored choices are preserved. To turn it off on an existing installation while parked:

```sh
cd /data/openpilot
python3 -c 'from openpilot.common.params import Params; Params().put("AutoLaneChangeTimer", -1, block=True)'
```

The three LDW switches still default on. Ordinary lane centering is unchanged.
