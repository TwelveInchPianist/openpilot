# Experimental in-car FCW tuning

Settings > Toggles > Forward Collision Warning: **Default / Aggressive**.
Default is selected for unset parameters. It preserves existing urgent FCW.
Aggressive adds a distinct `Closing quickly / Check the vehicle ahead` advisory,
with the existing prompt chime and a two-second banner. It does not alter gas,
brakes, steering, stock FCW thresholds, or the factory Honda safety systems.
The control is available in the comma 3/3X Python UI, not mici/four or sunnylink.

`AggressiveFCW` is a persistent, backed-up boolean, polled at 1 Hz. Switching
modes needs no restart after this version has been installed and rebuilt.
The existing LDW, MADS and lane-change defaults remain unchanged.

## Detection and alerting

- Requires valid/fresh radarState, modelV2, controlsState, fresh valid CAN car
  state, calibrated camera, drive gear, and speed strictly above 5 mph.
- Uses primary lead selection from the existing radar/vision association. Lead
  probability must be at least 0.95, lateral offset at most 0.9 m, and the
  estimated constant-curvature lateral displacement at lead distance at most
  0.5 m. This deliberately restricts the advisory to nearly straight approaches.
- Requires 0.6 s track/geometry continuity and 0.3 s continuous risk, on unique
  radar messages with gaps no greater than 0.2 s. Large range/speed jumps and
  radar track-ID changes restart confirmation. Vision-only identity is inferred
  from continuity, not a guaranteed persistent object ID.
- Simulates four seconds in 0.1 s steps, holding estimated lead acceleration
  constant and preventing either vehicle reversing as it stops. Risk requires
  predicted separation below 3 m both with current measured ego acceleration
  and with a hypothetical response after 1.2 s braking at 2.5 m/s^2 (or stronger
  existing braking). Closing speed must exceed 2 m/s or lead deceleration exceed
  1 m/s^2. These are unvalidated design values, not guaranteed reaction times.
- Brake-pedal contact and turn signals do not themselves suppress the advisory.
  Sufficient measured braking can remove the projected conflict. No engagement
  gate is added: manual and assisted driving use the same detector while onroad
  processes run. Dashcam/passive mode is excluded by existing selfdrived logic.
- One advisory per episode. Rearming requires two seconds of continuously valid
  observations predicting at least 6 m separation with current acceleration.
  Invalid data, mode switches and lost leads do not rearm. There is also a ten-
  second minimum interval. This can suppress a new advisory after a different
  threat; existing urgent FCW remains independent. Urgent FCW blocks the new
  event and latches/cools down the advisory. Higher-priority alerts can supersede
  the advisory; no guarantee is made of its presentation or chime.
- Runtime faults log once and disable the new service until selfdrived restarts.
  Source/import/build failures are not covered by that runtime containment.

## Limitations

This is an advisory experiment, not AEB and not a validated collision-warning
system. There is no additional object detector. It can miss stationary objects,
cut-ins, curves, low-confidence leads, and encounters during latch/cooldown.
Range is not a calibrated bumper-to-bumper envelope. Constant acceleration can
overpredict sustained lead braking and underpredict newly developing braking.
The straight-path check is a restrictive approximation, not full swept-footprint
collision checking. Sensor/model latency and confirmation delay reduce usable
warning time. No grip, road-grade, load, or human-response calibration is claimed.
Aggressive does not guarantee earlier warning than either stock FCW path.

## Validation and installation

77 synthetic tests pass, including the 58 existing LDW tests (with added FCW UI
callback assertions), 19 advisory/adapter/integration tests, fault injection,
tracking interruptions, braking, repeated messages, episode rearming, and the
actual selfdrived advisory hook with external dependencies stubbed.

```sh
python3 -m unittest discover -s tests -p 'test_*fcw.py' -v
python3 -m unittest discover -s tests -p test_active_ldw.py -v
```

No native device build, native UI rendering, recorded-drive replay or road test
has been performed here. False positives per driving hour, warning lead time,
and missed-conflict rates have not been measured. Default mode remains the
initial setting until the user selects Aggressive.

The first install requires a full device rebuild and restart because a Params
key and appended custom event enum have been added. Remove the `prebuilt`
marker before rebooting after a clean fast-forward update while parked/offroad.
No new cereal field is added to existing messages and existing enum ordinals
are unchanged; the new event is appended at OnroadEventSP.EventName @24.
