# Experimental lane warnings during active steering

Based on sunnypilot v2026.002.002, commit c440933a65fa2c669a9926c8112ae352d3600f07.

This branch adds an opt-in Lane Warnings During Steering (Experimental) toggle. Enable the existing Lane Departure Warnings toggle as well. The new detector uses current estimated curvature and model boundaries to predict a boundary crossing while lateral control is active, independently of steering saturation. Steering, braking, panda safety and driver monitoring are unchanged.

## Validation status

32 synthetic Python tests pass, including the patched planner with external dependencies stubbed. No complete native build, device installation, recorded-drive replay, or road validation has been performed. This is source for experimental evaluation, not a validated release. Initial installation requires an on-device build; no prebuilt marker is included.

## Limitations

The assumed vehicle half-width is 1.0 m, not a calibrated tire or full vehicle footprint. Prediction uses constant speed and current curvature for 0.8 seconds. Detection requires speed above approximately 31 mph, valid fresh inputs and calibrated perception. Same-side turn signals suppress warnings with a cooldown. A credible lane line takes precedence over a road-edge fallback; an unmarked shoulder may not be detected at its boundary. Invalid or uncertain inputs silently suppress the added warning. Existing alert arbitration can supersede the LDW chime. No guarantee of warning on every departure.

## Installation

Repository: TwelveInchPianist/openpilot
Branch: active-ldw-2026-002-002
Expected custom installer address: https://installer.comma.ai/TwelveInchPianist/active-ldw-2026-002-002

The installer endpoint and complete installation must be verified separately. Keep the official installer URL for your hardware available for recovery. Do not treat successful download or compilation as driving validation.
