# Experimental lane warnings during active steering

Based on sunnypilot v2026.002.002, commit c440933a65fa2c669a9926c8112ae352d3600f07.

Branch: `l` (lowercase L). Installer: https://installer.comma.ai/TwelveInchPianist/l
The longer branch active-ldw-2026-002-002 is also maintained at this revision.

Enable both Lane Departure Warnings and Lane Warnings During Steering (Experimental).
The toggle is currently exposed in the comma 3/3X Python settings UI, not the comma four UI or sunnylink remote settings.
The parameter is persistent and included in backups.

## Review fixes

- Either blinker or a model-reported lane-change state other than off suppresses both sides, with a five-second cooldown after both are inactive. This deliberately sacrifices warnings during that interval to avoid opposite-side alerts when lane labels switch.
- A fired latch survives brief invalid inputs, confidence dropouts, speed/lateral-active interruptions and boundary source changes. Partial confirmation, active output pulses and clearance timers are cancelled immediately. State expires after two seconds without a usable observation on that side. Otherwise rearming requires 0.35 m predicted clearance for 0.5 seconds.
- Lane probability enters at 0.65 and can be retained down to 0.50. Boundary geometry and standard deviation checks still apply.
- Unmarked road-edge fallback is disabled by default pending real-data evaluation of roadEdgeStds. There is no new UI setting enabling it. Marked shoulder lines remain eligible.
- Runtime exceptions in the added detector/parameter/clock block are logged once and disable the addition until planner restart. Longitudinal planning and stock LDW continue. The added detector is not called with its toggle off. Hosts lacking CLOCK_BOOTTIME skip the addition instead of mixing clock domains.

## Validation

44 synthetic tests pass. Two added regression tests reproduced signalled lane-change label-switch alerts and repeated alerts following confidence dropouts on the previous code, and now pass.
Tests include the actual planner main with external/native dependencies stubbed, exception injection, absent BOOTTIME, disabled feature, lane-change suppression, confidence hysteresis, latch retention/expiry and source transitions.

From repository root, run:

```sh
python3 -m unittest discover -s tests -p test_active_ldw.py -v
```

The user reported the prior revision's installer completed downloading/installing. This revision has not been natively built or installed here. No recorded-drive replay or road validation has occurred. There is no prebuilt marker; source installation requires a device build.

## Remaining limitations

Vehicle half-width is an approximate 1.0 m body envelope, not calibrated tire or full footprint tracking. Prediction assumes constant speed and current estimated curvature over 0.8 seconds. Speed must exceed approximately 31 mph; fresh valid data and calibration are required. Weak or stale detections silently suppress the added warning. Existing higher-priority alerts may supersede it.

A long data absence or explicit detector reset can rearm while still straddling a boundary. Five-second suppression does not prove an unusually long lane change has finished. Latch retention after a boundary-identity change can suppress a genuinely new event temporarily. Unmarked shoulder crossings are not covered by default. Thresholds, false-alarm rate and warning timing need real-drive evaluation; no guarantee is made of a warning on every departure.

Steering, braking, panda safety and driver monitoring code are unchanged. Successful compilation is not driving validation.
