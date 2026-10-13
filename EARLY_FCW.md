# Adjustable forward collision advisory

This adds an experimental warning only. Existing urgent model/planner FCW,
steering, braking, cruise behavior and lane-change defaults are unchanged.
It is not AEB or a parking sensor. A model score is not a measured probability
of successful detection. Road false-positive/missed-warning rates are unknown.

## In-car settings (comma 3/3X)

Settings > Toggles > Forward Collision Warning > Aggressive enables the extra
advisory. The existing AggressiveFCW selection is retained (new installs default
to Default). Three +/- controls below it apply within about one second:

| Setting | Default | Range | Increment |
| --- | --- | --- | --- |
| Lead confidence score | 75% | 55-95% | 5 percentage points |
| Assumed response delay | 2.0 s | 0.4-4.0 s | 0.2 s |
| Forecast horizon | 5.0 s | 2.0-8.0 s | 0.5 s |

The parameters FCWMinConfidence, FCWResponseDelay and FCWForecast are persistent
and backed up. Times are stored in integer tenths of a second. Missing/malformed
values use defaults; out-of-range values are bounded and snapped to the grid.
The 55% floor is above radard's >50% ordinary lead-selection threshold. The 75%
default is an engineering choice, not a validated safe threshold. Settings
changes restart confirmation without immediately rearming an existing episode.

No new comma four/mici or sunnylink editor is included in this 3X-targeted patch.

## Standstill and creep

The advisory runs in Drive with assistance engaged or disengaged, including
zero speed; there is no minimum speed or standstill exclusion. Park, Neutral
and Reverse remain excluded. Two motionless vehicles do not trigger an alarm.
A closing lead (including an observed reversing lead), or initial forward
acceleration toward a close lead, can trigger. It does not detect brake release
itself or hold the brakes. At low speed it requires >0.15 m/s closing speed or
other qualifying acceleration evidence to reject small speed-estimation noise.

Below 5 mph the projected-clearance threshold is 0.6 m instead of 3 m, so being
stopped close to another vehicle does not itself cause a warning. Above that
speed the existing >2 m/s closing / hard lead deceleration qualification remains.
Stable tracking (0.6 s), risk confirmation (0.3 s), fresh data and sufficient
confidence are still required. Those delays can consume much of a tiny gap.

The low-speed radar override in radard may publish modelProb=0. The advisory
can recover camera support only by matching a model lead in distance, lateral
position and speed. It does not bypass the confidence selector for radar-only
objects. Nidec near-field limits and radard's >0.75 m low-speed override gate
remain unchanged. Missing/suppressed objects cannot produce a warning.

## Curves

The old |curvature|*distance^2/2 <= 0.5 m exclusion is removed. The lead is compared
with the model path at the lead's longitudinal coordinate, using the same 1.52 m
radar-to-camera offset as radard and reversing the radar/model lateral sign.
Perpendicular offset must be <=1 m. Ordinary curved paths can now qualify.
It is still a lead-centre gate, not vehicle-width/partial-overlap detection.

The model path must contain a finite, forward-monotonic segment at that distance;
there is no long-distance extrapolation. Missing close-range geometry can use
a bounded constant-curvature fallback within 15 m (helpful at standstill).
The predicted path is not a guarantee of driver intent, especially during lane
changes. Adjacent-lane false positives and delayed/missed curve alerts require
recorded-drive evaluation. The advisory still examines only leadOne.

## Forecast and repeat warnings

Both the current-motion and hypothetical-response forecasts must fall below
the clearance threshold. The hypothetical response applies 2.5 m/s^2 braking
after the selected delay, or stronger observed braking while it is extrapolated.
Observed accelerations are extrapolated for at most 2 s, then coasting is assumed
except hypothetical braking continues to a stop. This bounds eight-second
acceleration errors; it can underestimate sustained acceleration/deceleration.
Forward leads are not extrapolated into reverse; observed reversing leads can
continue reversing. This is not a road-friction or driver-response model.

The horizon is not a warning lead-time guarantee. The detector only accepts
0.3-200 m lead distances and valid geometry; an 8 s horizon does not extend
sensor range. Higher horizons/delays and lower confidence can increase false
positives. There is a hard upper ego-speed bound of 60 m/s for input sanity.

One pulse per risk episode, 10 s cooldown at normal speed, 3 s for creep. Two
seconds of safe observations rearm; holding a steady stopped gap rearms creep.
After >=3 s without eligible observations and expiration of the 10 s cooldown,
the detector can rearm, but must earn fresh confirmation. This avoids an
indefinite latch after losing a lead; it can repeat an alert after long dropouts.
Urgent stock FCW preempts the advisory. Exceptions disable only the extra
advisory until process restart and are logged.

## Verification

Run from repository root:

```sh
python -m pytest tests/test_early_fcw.py tests/test_fcw_tuning.py
```

Tests exercise stopped queues, physically closing creep, initial acceleration,
reversing leads, braking, repeated episodes, confidence thresholds, live setting
updates, stale data, left/right curved paths, adjacent lanes, missing geometry,
low-speed camera/radar matching, bounds, actual UI configuration and the actual
selfdrived advisory hook. The tests run without native openpilot dependencies.
They do not validate camera perception, physical radar performance, native UI
rendering, device builds or warning performance on a recorded/real drive.
