"""Synthetic behaviour tests; these do NOT establish on-road warning performance."""
import ast
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'payload' if (ROOT / 'payload').exists() else ROOT
BASE = ROOT / 'base' if (ROOT / 'base').exists() else ROOT
MODULE = SOURCE / 'selfdrive/controls/lib/active_ldw.py'
spec = importlib.util.spec_from_file_location('active_ldw', MODULE)
ldw = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = ldw
spec.loader.exec_module(ldw)


def boundary(offset, slope=0.0, curvature=0.0, kind='line', probability=0.99, std=0.05):
  x = tuple(float(i) for i in range(81))
  def center(v):
    return (1.0 - math.sqrt(1.0 - (curvature*v)**2)) / curvature if curvature else 0.0
  y = tuple(offset + slope*v + center(v) for v in x)
  return ldw.Boundary(x, y, probability, std, kind)


class BehaviourTests(unittest.TestCase):
  def setUp(self):
    self.detector = ldw.ActiveLaneDepartureWarning()
    self.now = 100.0

  def run_frames(self, n=12, **kwargs):
    frames = []
    for _ in range(n):
      self.now += 0.05
      args = dict(now=self.now, frame_time=self.now-0.05, speed=30.0, curvature=0.0,
                  lines=(boundary(-1.8), boundary(1.8)), enabled=True, lateral_active=True)
      args.update(kwargs)
      frames.append(self.detector.update(**args))
    return frames

  def test_centered_straight_no_warning(self):
    self.assertFalse(any(r.left or r.right for r in self.run_frames(100)))

  def test_right_departure_before_crossing_without_torque_saturation(self):
    # Current right-side clearance is +0.39 m; road heading causes a crossing ahead.
    frames = self.run_frames(lines=(boundary(-2.2, -.03), boundary(1.35, -.03)))
    self.assertTrue(any(r.right for r in frames))
    self.assertFalse(any(r.left for r in frames))

  def test_moving_vehicle_warns_before_analytic_boundary_crossing(self):
    # Straight road, actual vehicle heading 0.02 rad right. Generate camera-frame
    # boundaries at each capture time, rather than replaying a frozen scene.
    speed, heading, half_width = 30.0, .02, self.detector.config.half_width
    crossing_time = (1.8 - half_width*math.cos(heading)) / (speed*math.sin(heading))
    first_warning = None
    for i in range(1, 45):
      t = i*.05
      capture_t = t-.05
      ego_y = speed*math.sin(heading)*capture_t
      offset = self.detector.config.camera_offset
      lines = tuple(boundary((side*1.8-ego_y)/math.cos(heading)-offset, -math.tan(heading)) for side in (-1, 1))
      r = self.detector.update(now=100+t, frame_time=100+capture_t, speed=speed, curvature=0,
                               lines=lines, enabled=True, lateral_active=True)
      if r.right and first_warning is None:
        first_warning = t
    self.assertIsNotNone(first_warning)
    self.assertGreater(crossing_time-first_warning, .4)
    self.assertLess(crossing_time-first_warning, 1.0)

  def test_left_departure(self):
    frames = self.run_frames(lines=(boundary(-1.35, .03), boundary(2.2, .03)))
    self.assertTrue(any(r.left for r in frames))
    self.assertFalse(any(r.right for r in frames))

  def test_actual_curvature_predicts_departure_on_straight_road(self):
    self.assertTrue(any(r.right for r in self.run_frames(curvature=.008)))
    self.detector.reset()
    self.assertTrue(any(r.left for r in self.run_frames(curvature=-.008)))

  def test_matching_left_and_right_curves_no_warning(self):
    for curvature in (-.008, .008):
      self.detector.reset()
      frames = self.run_frames(30, curvature=curvature,
                               lines=(boundary(-1.8, curvature=curvature), boundary(1.8, curvature=curvature)))
      self.assertFalse(any(r.left or r.right for r in frames))

  def test_understeering_right_curve_warns_left(self):
    frames = self.run_frames(curvature=0.0,
                             lines=(boundary(-1.8, curvature=.008), boundary(1.8, curvature=.008)))
    self.assertTrue(any(r.left for r in frames))

  def test_close_parallel_line_does_not_chime(self):
    frames = self.run_frames(80, lines=(boundary(-1.8), boundary(1.03)))
    self.assertFalse(any(r.right for r in frames))

  def test_crossed_boundary_warns_even_without_heading_toward_it(self):
    self.assertTrue(any(r.right for r in self.run_frames(lines=(boundary(-2.8), boundary(.90)))))

  def test_transient_one_frame_does_not_warn(self):
    frames = self.run_frames(1, curvature=.008) + self.run_frames(12)
    self.assertFalse(any(r.left or r.right for r in frames))

  def test_same_side_signal_suppresses_and_cooldown_expires(self):
    self.assertFalse(any(r.right for r in self.run_frames(curvature=.008, right_signal=True)))
    self.assertFalse(any(r.right for r in self.run_frames(98, curvature=.008)))
    self.assertTrue(any(r.right for r in self.run_frames(20, curvature=.008)))

  def test_opposite_signal_suppresses_during_lane_change(self):
    self.assertFalse(any(r.right for r in self.run_frames(curvature=.008, left_signal=True)))

  def test_hazards_suppress_both_sides(self):
    self.assertFalse(any(r.left or r.right for r in self.run_frames(curvature=.008, left_signal=True, right_signal=True)))

  def test_disabled_does_not_add_warning(self):
    for options in ({'enabled': False},):
      self.assertFalse(any(r.left or r.right for r in self.run_frames(curvature=.008, **options)))

  def test_speed_gate(self):
    for speed in (0, -1, 31*0.44704, 80):
      self.assertFalse(any(r.left or r.right for r in self.run_frames(speed=speed, curvature=.008)))

  def test_invalid_data_immediately_clears_output_and_confirmation(self):
    self.run_frames(5, curvature=.008)
    self.assertFalse(self.run_frames(1, curvature=.008, valid=False)[0].right)
    self.assertFalse(self.run_frames(1, curvature=.008)[0].right)

  def test_nan_inf_and_extreme_curvature(self):
    for curvature in (math.nan, math.inf, -.1, .1):
      self.assertFalse(any(r.left or r.right for r in self.run_frames(curvature=curvature)))

  def test_stale_and_future_camera_frames(self):
    for frame_time in (self.now-1, self.now+5):
      self.assertFalse(any(r.right for r in self.run_frames(curvature=.008, frame_time=frame_time)))

  def test_duplicate_camera_frames_cannot_accumulate_confirmation(self):
    stamp = self.now
    frames = self.run_frames(4, curvature=.008, frame_time=stamp)
    self.assertFalse(any(r.right for r in frames))

  def test_long_gap_resets_confirmation(self):
    self.run_frames(2, curvature=.008)
    self.now += 1
    self.assertFalse(self.run_frames(1, curvature=.008)[0].right)

  def test_clock_reversal_resets(self):
    self.run_frames(2, curvature=.008)
    self.now -= 5
    self.assertFalse(self.run_frames(1, curvature=.008)[0].right)

  def test_low_confidence_and_malformed_lines(self):
    bad = [None, boundary(1.8, probability=.2), boundary(1.8, std=.8),
           ldw.Boundary((0, 0, 20), (1, 1, 1), .99, .05),
           ldw.Boundary((0, 1, 20), (1, math.nan, 1), .99, .05),
           ldw.Boundary((0, 1, 20), (1, 1), .99, .05),
           ldw.Boundary((0, 1, 2), (1.8, 1.8, 1.8), .99, .05)]
    for b in bad:
      self.detector.reset()
      self.assertFalse(any(r.right for r in self.run_frames(curvature=.008, lines=(boundary(-1.8), b))))

  def test_marked_shoulder_line_precedes_distant_road_edge(self):
    frames = self.run_frames(curvature=.008, edges=(boundary(-6, kind='edge'), boundary(6, kind='edge')))
    self.assertTrue(any(r.right for r in frames))
    self.assertEqual(frames[-1].right_source, 'line')

  def test_road_edge_fallback_and_uncertainty_gate(self):
    self.detector = ldw.ActiveLaneDepartureWarning(ldw.Config(road_edge_fallback=True))
    frames = self.run_frames(curvature=.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge')))
    self.assertTrue(any(r.right for r in frames))
    self.assertEqual(frames[-1].right_source, 'edge')
    self.detector.reset()
    frames = self.run_frames(curvature=.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge', std=.6)))
    self.assertFalse(any(r.right for r in frames))

  def test_latch_one_pulse_then_rearm_after_clearance(self):
    frames = self.run_frames(80, curvature=.008)
    rising = sum(r.right and (i == 0 or not frames[i-1].right) for i, r in enumerate(frames))
    self.assertEqual(rising, 1)
    self.run_frames(20)
    self.assertTrue(any(r.right for r in self.run_frames(curvature=.008)))

  def test_line_identity_jump_restarts_confirmation(self):
    self.run_frames(2, curvature=.008)
    self.assertFalse(self.run_frames(1, curvature=.008, lines=(boundary(-1.8), boundary(.8)))[0].right)

  def test_signalled_lane_change_line_reassignment(self):
    before = self.run_frames(20, left_signal=True, lines=(boundary(-.2), boundary(3.4)))
    after = self.run_frames(20, left_signal=True, lines=(boundary(-3.4), boundary(.2)))
    self.assertFalse(any(r.left or r.right for r in before + after))

  def test_confidence_dropout_does_not_retrigger_latched_warning(self):
    self.run_frames(80, lines=(boundary(-2.8), boundary(.9)))
    for _ in range(4):
      self.run_frames(1, lines=(boundary(-2.8), boundary(.9, probability=.4)))
      self.assertFalse(any(r.right for r in self.run_frames(20, lines=(boundary(-2.8), boundary(.9)))))

  def test_latch_survives_brief_invalid_speed_and_lateral_dropouts(self):
    for fault in ({'valid': False}, {'speed': 10}, {'lateral_active': False}):
      self.detector.reset()
      self.run_frames(80, curvature=.008)
      self.run_frames(1, curvature=.008, **fault)
      self.assertFalse(any(r.right for r in self.run_frames(30, curvature=.008)))

  def test_latch_survives_gap_but_expires_after_long_absence(self):
    self.run_frames(80, curvature=.008)
    self.now += .5
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008)))
    self.now += 3
    self.assertTrue(any(r.right for r in self.run_frames(20, curvature=.008)))

  def test_latch_survives_source_switch(self):
    self.detector = ldw.ActiveLaneDepartureWarning(ldw.Config(road_edge_fallback=True))
    self.run_frames(80, curvature=.008)
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008,
      lines=(None, None), edges=(None, boundary(1.8, kind='edge')))))
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008)))

  def test_lane_probability_hysteresis(self):
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008,
      lines=(None, boundary(1.8, probability=.6)))))
    self.run_frames(1, curvature=.008)
    self.assertTrue(any(r.right for r in self.run_frames(20, curvature=.008,
      lines=(None, boundary(1.8, probability=.6)))))

  def test_default_road_edge_fallback_disabled(self):
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008,
      lines=(None, None), edges=(None, boundary(1.8, kind='edge')))))

  def test_model_lane_change_suppresses_both_and_cooldown(self):
    self.assertFalse(any(r.left or r.right for r in self.run_frames(20, curvature=.008, lane_change_active=True)))
    self.assertFalse(any(r.left or r.right for r in self.run_frames(98, curvature=.008)))
    self.assertTrue(any(r.right for r in self.run_frames(20, curvature=.008)))

  def test_envelope_width_affects_warning(self):
    lines = (boundary(-1.8), boundary(1.20, -.013))
    narrow = ldw.ActiveLaneDepartureWarning(ldw.Config(half_width=.8))
    wide = ldw.ActiveLaneDepartureWarning(ldw.Config(half_width=1.1))
    seen = [False, False]
    for i in range(10):
      for j, detector in enumerate((narrow, wide)):
        now = 200 + i*.05
        seen[j] |= detector.update(now=now, frame_time=now-.05, speed=25, curvature=0,
                                   lines=lines, enabled=True, lateral_active=True).right
    self.assertEqual(seen, [False, True])


def make_sm():
  class SM(dict):
    def all_checks(self, services):
      return all(self.checks.get(s, True) for s in services)
  lines = [boundary(-5), boundary(-1.8), boundary(1.8), boundary(5)]
  model = NS(timestampEof=int(99.95e9), laneLines=[NS(x=b.x, y=b.y) for b in lines],
             laneLineProbs=[.99]*4, laneLineStds=[.05]*4,
             roadEdges=[], roadEdgeStds=[], meta=NS(desirePrediction=[0]*8, laneChangeState=0))
  sm = SM(modelV2=model, carState=NS(vEgo=30, leftBlinker=False, rightBlinker=False),
          carControl=NS(latActive=True), controlsState=NS(curvature=.008),
          liveParameters=NS(valid=True), liveCalibration=NS(calStatus=1))
  sm.checks = {}
  sm.logMonoTime = {name: int(100e9) for name in sm}
  return sm


class AdapterTests(unittest.TestCase):
  def test_camera_boottime_offset_after_suspend(self):
    sm, detector, seen = make_sm(), ldw.ActiveLaneDepartureWarning(), False
    for i in range(12):
      now = 100 + i*.05
      sm['modelV2'].timestampEof = int((now+300-.05)*1e9)
      sm.logMonoTime = {name: int(now*1e9) for name in sm}
      seen |= ldw.update_active_ldw(detector, sm, now, True, 1, camera_now=now+300).right
    self.assertTrue(seen)

  def test_adapter_uses_actual_curvature_with_zero_model_desire(self):
    sm, detector, seen = make_sm(), ldw.ActiveLaneDepartureWarning(), False
    for i in range(12):
      now = 100 + i*.05
      sm['modelV2'].timestampEof = int((now-.05)*1e9)
      sm.logMonoTime = {name: int(now*1e9) for name in sm}
      seen |= ldw.update_active_ldw(detector, sm, now, True, 1).right
    self.assertTrue(seen)

  def test_calibration_invalid_parameters_and_stale_controls_suppress(self):
    for fault in ('calibration', 'parameters', 'stale', 'frequency'):
      sm, detector = make_sm(), ldw.ActiveLaneDepartureWarning()
      if fault == 'calibration': sm['liveCalibration'].calStatus = 0
      if fault == 'parameters': sm['liveParameters'].valid = False
      if fault == 'stale': sm.logMonoTime['controlsState'] = int(90e9)
      if fault == 'frequency': sm.checks['modelV2'] = False
      self.assertFalse(ldw.update_active_ldw(detector, sm, 100, True, 1).right)

  def test_adapter_passes_lane_change_state(self):
    sm, detector = make_sm(), ldw.ActiveLaneDepartureWarning()
    sm['modelV2'].meta.laneChangeState = 2
    for i in range(30):
      now = 100+i*.05
      sm['modelV2'].timestampEof = int((now-.05)*1e9)
      sm.logMonoTime = {name: int(now*1e9) for name in sm}
      result = ldw.update_active_ldw(detector, sm, now, True, 1)
      self.assertFalse(result.left or result.right)

  def test_missing_model_arrays_become_unavailable(self):
    self.assertIsNone(ldw.boundary_from_model(NS(), 1))
    self.assertIsNone(ldw.boundary_from_model(make_sm()['modelV2'], 0, True))

  def run_planner(self, fault=False, clock=True, enabled=True, low_speed=False, speed=30, sensitivity=1, curvature=.008, lateral_active=True, legacy_warning=False):
    # Execute the actual patched main, replacing only external processes and native imports.
    class StopLoop(Exception): pass
    sm, sent, errors, calls, planned = make_sm(), [], [], [], []
    def adapter(*args, **kwargs):
      calls.append(1)
      if fault: raise RuntimeError('injected detector fault')
      return ldw.update_active_ldw(*args, **kwargs)
    sm['selfdriveStateSP'] = NS(buttonsReleaseToggle=0)
    sm['carControl'].latActive = lateral_active
    sm['carState'].vEgo = speed
    sm['controlsState'].curvature = curvature
    sm.frame, sm.updated = 0, {'modelV2': True}
    def update():
      if sm.frame >= 30: raise StopLoop()
      sm.frame += 1
      now = 100 + sm.frame*.05
      sm['modelV2'].timestampEof = int((now-.05)*1e9)
      sm.logMonoTime = {name: int(now*1e9) for name in sm}
    sm.update = update
    class Params:
      def get(self, key, *args, **kwargs):
        return sensitivity if key == 'LdwSensitivity' else b''
      def get_bool(self, key):
        if key == 'LdwLowSpeed': return low_speed and sm.frame >= 10
        return enabled and key in ('LdwWhileSteering', 'IsLdwEnabled')
    class Planner:
      def __init__(self, *args): self.sla = NS(update_buttons=lambda *a: None)
      def update(self, *args): pass
      def publish(self, *args): planned.append(1)
    # Load unmodified release LDW with its genuine logic, without cereal dependencies.
    tree = ast.parse((BASE/'selfdrive/controls/lib/ldw.py').read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    legacy = {'log': NS(Desire=NS(laneChangeLeft=3, laneChangeRight=4)), 'DT_CTRL': .01,
              'CV': NS(MPH_TO_MS=.44704)}
    exec(compile(tree, '<legacy ldw>', 'exec'), legacy)
    if legacy_warning:
      class AlwaysWarn:
        left, right = True, True
        def update(self, *args): pass
      legacy['LaneDepartureWarning'] = AlwaysWarn
    env = dict(time=NS(monotonic=lambda: 100+sm.frame*.05, CLOCK_BOOTTIME=7,
                       clock_gettime=lambda clock: 100+sm.frame*.05), car=NS(CarParams=object),
               custom=NS(CarParamsSP=object), log=NS(LiveCalibrationData=NS(Status=NS(calibrated=1))),
               config_realtime_process=lambda *a: None, Priority=NS(CTRL_LOW=1),
               cloudlog=NS(info=lambda *a: None, exception=lambda *a: errors.append(a)), Params=Params, get_gps_location_service=lambda p: 'gpsLocation',
               LaneDepartureWarning=legacy['LaneDepartureWarning'], ActiveLaneDepartureWarning=ldw.ActiveLaneDepartureWarning,
               update_active_ldw=adapter, Result=ldw.Result, config_from_settings=ldw.config_from_settings, LongitudinalPlanner=Planner,
               messaging=NS(log_from_bytes=lambda *a: NS(brand='honda'),
                            PubMaster=lambda *a: NS(send=lambda name, msg: sent.append(msg)),
                            SubMaster=lambda *a, **kw: sm,
                            new_message=lambda *a: NS(driverAssistance=NS())), __name__='test')
    if not clock: del env['time'].CLOCK_BOOTTIME
    source = ast.parse((SOURCE/'selfdrive/controls/plannerd.py').read_text())
    source.body = [n for n in source.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    exec(compile(source, '<patched plannerd>', 'exec'), env)
    with self.assertRaises(StopLoop): env['main']()
    self.assertEqual(len(planned), 30)
    return sent, errors, calls

  def test_real_plannerd_main_publishes_advisory(self):
    sent, errors, calls = self.run_planner()
    self.assertFalse(errors)
    self.assertTrue(any(m.driverAssistance.rightLaneDeparture for m in sent))
    self.assertTrue(all(m.valid for m in sent))
    self.assertFalse(any(m.driverAssistance.leftLaneDeparture for m in sent))


class PlannerIsolationTests(unittest.TestCase):
  run_planner = AdapterTests.run_planner
  def test_detector_exception_does_not_stop_planner(self):
    sent, errors, calls = self.run_planner(fault=True)
    self.assertEqual(len(calls), 1)
    self.assertEqual(len(errors), 1)
    self.assertEqual(len(sent), 30)
    self.assertFalse(any(m.driverAssistance.leftLaneDeparture or m.driverAssistance.rightLaneDeparture for m in sent))

  def test_missing_boottime_suppresses_added_detector(self):
    sent, errors, calls = self.run_planner(clock=False)
    self.assertEqual(len(sent), 30)
    self.assertFalse(errors or calls)

  def test_toggle_off_skips_detector(self):
    sent, errors, calls = self.run_planner(enabled=False, fault=True)
    self.assertEqual(len(sent), 30)
    self.assertFalse(errors or calls)


class SettingsTests(unittest.TestCase):
  setUp = BehaviourTests.setUp
  run_frames = BehaviourTests.run_frames
  run_planner = AdapterTests.run_planner

  def test_defaults_match_original_config(self):
    self.assertEqual(ldw.config_from_settings(), ldw.Config())

  def test_invalid_sensitivity_falls_back_to_standard(self):
    for value in (None, b'2', '2', -1, 3, 999, True, float('nan')):
      self.assertEqual(ldw.config_from_settings(sensitivity=value), ldw.Config())

  def test_low_speed_on_off_and_standstill(self):
    for enabled in (False, True):
      for mph in (0, .5, 2, 2.1, 15, 30, 32):
        self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(enabled))
        result = self.run_frames(20, speed=mph*.44704, lines=(boundary(-2.8), boundary(.9)))
        self.assertEqual(any(r.right for r in result), mph > (2 if enabled else 31))

  def test_sensitivity_orders_warning_on_approach(self):
    first = []
    for profile in range(3):
      self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(sensitivity=profile))
      fired = None
      for i in range(40):
        offset = 2.0-i*.025
        if self.run_frames(1, lines=(boundary(-2.8, -.02), boundary(offset, -.02)))[0].right and fired is None:
          fired = i
      self.assertIsNotNone(fired)
      first.append(fired)
    self.assertGreater(first[0], first[1])
    self.assertGreater(first[1], first[2])

  def test_tuning_keeps_latch_and_signal_history(self):
    self.run_frames(80, curvature=.008)
    self.detector.configure(ldw.config_from_settings(True, 2))
    self.assertTrue(self.detector.sides[1].latched)
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008)))
    self.run_frames(1, left_signal=True)
    stamp = self.detector.last_signal[:]
    self.detector.configure(ldw.config_from_settings(False, 0))
    self.assertEqual(stamp, self.detector.last_signal)
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008)))

  def test_repeated_same_settings_do_not_reset_confirmation(self):
    seen = False
    for _ in range(20):
      self.detector.configure(ldw.config_from_settings())
      seen |= self.run_frames(1, curvature=.008)[0].right
    self.assertTrue(seen)

  def test_planner_picks_up_low_speed_setting_without_restart(self):
    # A curved trajectory at 25 mph supplies risk; setting changes at frame 10,
    # the 1-second poll picks it up around frame 21, and the same loop warns.
    sent, errors, calls = self.run_planner(low_speed=True, speed=25*.44704, curvature=.025)
    self.assertFalse(errors)
    self.assertFalse(any(m.driverAssistance.rightLaneDeparture for m in sent[:20]))
    self.assertTrue(any(m.driverAssistance.rightLaneDeparture for m in sent[20:]))

  def test_new_python_files_parse(self):
    for file in ('selfdrive/controls/lib/active_ldw.py', 'selfdrive/controls/plannerd.py',
                 'selfdrive/ui/layouts/settings/toggles.py'):
      ast.parse((SOURCE/file).read_text())


class TunerUITests(unittest.TestCase):
  def test_controls_construct_and_persist_settings(self):
    values = {'LdwSensitivity': 99, 'LongitudinalPersonality': 1}
    class Params:
      def get(self, key, **kw): return values.get(key)
      def get_bool(self, key): return bool(values.get(key, False))
      def put(self, key, value, **kw): values[key] = value
      def put_bool(self, key, value, **kw): values[key] = value
    class Widget:
      def __init__(self): pass
    class Action:
      def set_enabled(self, value): self.enabled = value
      def set_state(self, value): self.state = value
      def get_state(self): return getattr(self, 'state', False)
      def set_selected_button(self, value): self.selected_index = value
    class Item:
      def __init__(self, title, description, *args, **kw):
        self.title, self.description = title, description
        self.action_item = Action()
        self.callback = kw.get('callback')
        self.action_item.selected_index = kw.get('selected_index')
      def set_description(self, value): self.description = value
      def set_icon(self, value): self.icon = value
    tree = ast.parse((SOURCE/'selfdrive/ui/layouts/settings/toggles.py').read_text())
    # Execute description table and actual layout class, stubbing rendering only.
    tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef) or
                 isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DESCRIPTIONS' for t in n.targets)]
    env = dict(Params=Params, Widget=Widget, UnknownKeyName=KeyError, sensitivity_index=ldw.sensitivity_index,
               toggle_item=Item, multiple_button_item=Item, tr=lambda s:s, tr_noop=lambda s:s,
               Scroller=lambda *a, **k: NS(), ui_state=NS(add_engaged_transition_callback=lambda cb:None))
    exec(compile(tree, '<settings ui>', 'exec'), env)
    ui = env['TogglesLayout']()
    self.assertIn('LdwLowSpeed', ui._toggles)
    self.assertEqual(ui._ldw_sensitivity_setting.action_item.selected_index, 1)
    ui._ldw_sensitivity_setting.callback(2)
    self.assertEqual(values['LdwSensitivity'], 2)
    ui._toggles['LdwLowSpeed'].callback(True)
    self.assertTrue(values['LdwLowSpeed'])
    self.assertNotIn('OnroadCycleRequested', values)
    self.assertEqual(ui._fcw_sensitivity_setting.action_item.selected_index, 0)
    ui._fcw_sensitivity_setting.callback(1)
    self.assertIs(values['AggressiveFCW'], True)
    ui._fcw_sensitivity_setting.callback(0)
    self.assertIs(values['AggressiveFCW'], False)
    ui._fcw_sensitivity_setting.callback(99)
    self.assertIs(values['AggressiveFCW'], False)


class AllSteeringStatesTests(unittest.TestCase):
  setUp = BehaviourTests.setUp
  run_frames = BehaviourTests.run_frames
  run_planner = AdapterTests.run_planner

  def test_above_two_mph_in_both_steering_states(self):
    for lateral in (False, True):
      for mph in (0, 2, 2.1, 10, 30, 60):
        self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(True))
        frames = self.run_frames(20, speed=mph*.44704, lateral_active=lateral,
                                 lines=(boundary(-2.8), boundary(.9)))
        self.assertEqual(any(r.right for r in frames), mph > 2)

  def test_planner_manual_mode_low_speed_warning(self):
    sent, errors, calls = self.run_planner(low_speed=True, speed=25*.44704, curvature=.025, lateral_active=False)
    self.assertFalse(errors)
    self.assertTrue(any(m.driverAssistance.rightLaneDeparture for m in sent[20:]))

  def test_stock_does_not_bypass_predictive_suppression(self):
    sent, errors, calls = self.run_planner(speed=0, lateral_active=False, legacy_warning=True)
    self.assertFalse(errors)
    self.assertFalse(any(m.driverAssistance.leftLaneDeparture or m.driverAssistance.rightLaneDeparture for m in sent))

  def test_stock_fallback_when_predictive_disabled_or_faulted(self):
    for options in ({'enabled': False}, {'fault': True}, {'clock': False}):
      sent, errors, calls = self.run_planner(legacy_warning=True, **options)
      self.assertTrue(all(m.driverAssistance.leftLaneDeparture and m.driverAssistance.rightLaneDeparture for m in sent))

  def test_steering_transition_does_not_rearm_latch(self):
    self.run_frames(80, curvature=.008)
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008, lateral_active=False)))
    self.assertFalse(any(r.right for r in self.run_frames(20, curvature=.008, lateral_active=True)))


if __name__ == '__main__':
  unittest.main(verbosity=2)
