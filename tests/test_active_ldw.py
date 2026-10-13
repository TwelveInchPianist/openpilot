import pytest
from typing import Any
from test_early_fcw import fcw

"""Synthetic behavior tests; these do NOT establish on-road warning performance."""
import ast
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

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
    return (1.0 - math.sqrt(1.0 - (curvature * v) ** 2)) / curvature if curvature else 0.0

  y = tuple(offset + slope * v + center(v) for v in x)
  return ldw.Boundary(x, y, probability, std, kind)


class TestBehaviourTests:
  def setUp(self):
    self.detector = ldw.ActiveLaneDepartureWarning()
    self.now = 100.0

  def run_frames(self, n=12, **kwargs):
    frames = []
    for _ in range(n):
      self.now += 0.05
      args = dict(now=self.now, frame_time=self.now - 0.05, speed=30.0, curvature=0.0, lines=(boundary(-1.8), boundary(1.8)), enabled=True, lateral_active=True)
      args.update(kwargs)
      frames.append(self.detector.update(**args))
    return frames

  def test_centered_straight_no_warning(self):
    assert not any(r.left or r.right for r in self.run_frames(100))

  def test_right_departure_before_crossing_without_torque_saturation(self):
    frames = self.run_frames(lines=(boundary(-2.2, -0.03), boundary(1.35, -0.03)))
    assert any(r.right for r in frames)
    assert not any(r.left for r in frames)

  def test_moving_vehicle_warns_before_analytic_boundary_crossing(self):
    speed, heading, half_width = (30.0, 0.02, self.detector.config.half_width)
    crossing_time = (1.8 - half_width * math.cos(heading)) / (speed * math.sin(heading))
    first_warning = None
    for i in range(1, 45):
      t = i * 0.05
      capture_t = t - 0.05
      ego_y = speed * math.sin(heading) * capture_t
      offset = self.detector.config.camera_offset
      lines = tuple(boundary((side * 1.8 - ego_y) / math.cos(heading) - offset, -math.tan(heading)) for side in (-1, 1))
      r = self.detector.update(now=100 + t, frame_time=100 + capture_t, speed=speed, curvature=0, lines=lines, enabled=True, lateral_active=True)
      if r.right and first_warning is None:
        first_warning = t
    assert first_warning is not None
    assert crossing_time - first_warning > 0.4
    assert crossing_time - first_warning < 1.0

  def test_left_departure(self):
    frames = self.run_frames(lines=(boundary(-1.35, 0.03), boundary(2.2, 0.03)))
    assert any(r.left for r in frames)
    assert not any(r.right for r in frames)

  def test_actual_curvature_predicts_departure_on_straight_road(self):
    assert any(r.right for r in self.run_frames(curvature=0.008))
    self.detector.reset()
    assert any(r.left for r in self.run_frames(curvature=-0.008))

  def test_matching_left_and_right_curves_no_warning(self):
    for curvature in (-0.008, 0.008):
      self.detector.reset()
      frames = self.run_frames(30, curvature=curvature, lines=(boundary(-1.8, curvature=curvature), boundary(1.8, curvature=curvature)))
      assert not any(r.left or r.right for r in frames)

  def test_understeering_right_curve_warns_left(self):
    frames = self.run_frames(curvature=0.0, lines=(boundary(-1.8, curvature=0.008), boundary(1.8, curvature=0.008)))
    assert any(r.left for r in frames)

  def test_close_parallel_line_does_not_chime(self):
    frames = self.run_frames(80, lines=(boundary(-1.8), boundary(1.03)))
    assert not any(r.right for r in frames)

  def test_crossed_boundary_warns_even_without_heading_toward_it(self):
    assert any(r.right for r in self.run_frames(lines=(boundary(-2.8), boundary(0.9))))

  def test_transient_one_frame_does_not_warn(self):
    frames = self.run_frames(1, curvature=0.008) + self.run_frames(12)
    assert not any(r.left or r.right for r in frames)

  def test_same_side_signal_suppresses_and_cooldown_expires(self):
    assert not any(r.right for r in self.run_frames(curvature=0.008, right_signal=True))
    assert not any(r.right for r in self.run_frames(98, curvature=0.008))
    assert any(r.right for r in self.run_frames(20, curvature=0.008))

  def test_opposite_signal_suppresses_during_lane_change(self):
    assert not any(r.right for r in self.run_frames(curvature=0.008, left_signal=True))

  def test_hazards_suppress_both_sides(self):
    assert not any(r.left or r.right for r in self.run_frames(curvature=0.008, left_signal=True, right_signal=True))

  def test_disabled_does_not_add_warning(self):
    for options in ({'enabled': False},):
      assert not any(r.left or r.right for r in self.run_frames(curvature=0.008, **options))

  def test_speed_gate(self):
    for speed in (0, -1, 31 * 0.44704, 80):
      assert not any(r.left or r.right for r in self.run_frames(speed=speed, curvature=0.008))

  def test_invalid_data_immediately_clears_output_and_confirmation(self):
    self.run_frames(5, curvature=0.008)
    assert not self.run_frames(1, curvature=0.008, valid=False)[0].right
    assert not self.run_frames(1, curvature=0.008)[0].right

  def test_nan_inf_and_extreme_curvature(self):
    for curvature in (math.nan, math.inf, -0.1, 0.1):
      assert not any(r.left or r.right for r in self.run_frames(curvature=curvature))

  def test_stale_and_future_camera_frames(self):
    for frame_time in (self.now - 1, self.now + 5):
      assert not any(r.right for r in self.run_frames(curvature=0.008, frame_time=frame_time))

  def test_duplicate_camera_frames_cannot_accumulate_confirmation(self):
    stamp = self.now
    frames = self.run_frames(4, curvature=0.008, frame_time=stamp)
    assert not any(r.right for r in frames)

  def test_long_gap_resets_confirmation(self):
    self.run_frames(2, curvature=0.008)
    self.now += 1
    assert not self.run_frames(1, curvature=0.008)[0].right

  def test_clock_reversal_resets(self):
    self.run_frames(2, curvature=0.008)
    self.now -= 5
    assert not self.run_frames(1, curvature=0.008)[0].right

  def test_low_confidence_and_malformed_lines(self):
    bad = [
      None,
      boundary(1.8, probability=0.2),
      boundary(1.8, std=0.8),
      ldw.Boundary((0, 0, 20), (1, 1, 1), 0.99, 0.05),
      ldw.Boundary((0, 1, 20), (1, math.nan, 1), 0.99, 0.05),
      ldw.Boundary((0, 1, 20), (1, 1), 0.99, 0.05),
      ldw.Boundary((0, 1, 2), (1.8, 1.8, 1.8), 0.99, 0.05),
    ]
    for b in bad:
      self.detector.reset()
      assert not any(r.right for r in self.run_frames(curvature=0.008, lines=(boundary(-1.8), b)))

  def test_marked_shoulder_line_precedes_distant_road_edge(self):
    frames = self.run_frames(curvature=0.008, edges=(boundary(-6, kind='edge'), boundary(6, kind='edge')))
    assert any(r.right for r in frames)
    assert frames[-1].right_source == 'line'

  def test_road_edge_fallback_and_uncertainty_gate(self):
    self.detector = ldw.ActiveLaneDepartureWarning(ldw.Config(road_edge_fallback=True))
    frames = self.run_frames(curvature=0.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge')))
    assert any(r.right for r in frames)
    assert frames[-1].right_source == 'edge'
    self.detector.reset()
    frames = self.run_frames(curvature=0.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge', std=0.6)))
    assert not any(r.right for r in frames)

  def test_latch_one_pulse_then_rearm_after_clearance(self):
    frames = self.run_frames(80, curvature=0.008)
    rising = sum((r.right and (i == 0 or not frames[i - 1].right) for i, r in enumerate(frames)))
    assert rising == 1
    self.run_frames(20)
    assert any(r.right for r in self.run_frames(curvature=0.008))

  def test_line_identity_jump_restarts_confirmation(self):
    self.run_frames(2, curvature=0.008)
    assert not self.run_frames(1, curvature=0.008, lines=(boundary(-1.8), boundary(0.8)))[0].right

  def test_signaled_lane_change_line_reassignment(self):
    before = self.run_frames(20, left_signal=True, lines=(boundary(-0.2), boundary(3.4)))
    after = self.run_frames(20, left_signal=True, lines=(boundary(-3.4), boundary(0.2)))
    assert not any(r.left or r.right for r in before + after)

  def test_confidence_dropout_does_not_retrigger_latched_warning(self):
    self.run_frames(80, lines=(boundary(-2.8), boundary(0.9)))
    for _ in range(4):
      self.run_frames(1, lines=(boundary(-2.8), boundary(0.9, probability=0.4)))
      assert not any(r.right for r in self.run_frames(20, lines=(boundary(-2.8), boundary(0.9))))

  def test_latch_survives_brief_invalid_speed_and_lateral_dropouts(self):
    for fault in ({'valid': False}, {'speed': 10}, {'lateral_active': False}):
      self.detector.reset()
      self.run_frames(80, curvature=0.008)
      self.run_frames(1, curvature=0.008, **fault)
      assert not any(r.right for r in self.run_frames(30, curvature=0.008))

  def test_latch_survives_gap_but_expires_after_long_absence(self):
    self.run_frames(80, curvature=0.008)
    self.now += 0.5
    assert not any(r.right for r in self.run_frames(20, curvature=0.008))
    self.now += 3
    assert any(r.right for r in self.run_frames(20, curvature=0.008))

  def test_latch_survives_source_switch(self):
    self.detector = ldw.ActiveLaneDepartureWarning(ldw.Config(road_edge_fallback=True))
    self.run_frames(80, curvature=0.008)
    assert not any(r.right for r in self.run_frames(20, curvature=0.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge'))))
    assert not any(r.right for r in self.run_frames(20, curvature=0.008))

  def test_lane_probability_hysteresis(self):
    assert not any(r.right for r in self.run_frames(20, curvature=0.008, lines=(None, boundary(1.8, probability=0.6))))
    self.run_frames(1, curvature=0.008)
    assert any(r.right for r in self.run_frames(20, curvature=0.008, lines=(None, boundary(1.8, probability=0.6))))

  def test_default_road_edge_fallback_disabled(self):
    assert not any(r.right for r in self.run_frames(20, curvature=0.008, lines=(None, None), edges=(None, boundary(1.8, kind='edge'))))

  def test_model_lane_change_suppresses_both_and_cooldown(self):
    assert not any(r.left or r.right for r in self.run_frames(20, curvature=0.008, lane_change_active=True))
    assert not any(r.left or r.right for r in self.run_frames(98, curvature=0.008))
    assert any(r.right for r in self.run_frames(20, curvature=0.008))

  def test_envelope_width_affects_warning(self):
    lines = (boundary(-1.8), boundary(1.2, -0.013))
    narrow = ldw.ActiveLaneDepartureWarning(ldw.Config(half_width=0.8))
    wide = ldw.ActiveLaneDepartureWarning(ldw.Config(half_width=1.1))
    seen = [False, False]
    for i in range(10):
      for j, detector in enumerate((narrow, wide)):
        now = 200 + i * 0.05
        seen[j] |= detector.update(now=now, frame_time=now - 0.05, speed=25, curvature=0, lines=lines, enabled=True, lateral_active=True).right
    assert seen == [False, True]

  def setup_method(self):
    self.setUp()


def make_sm():

  class SM(dict):
    def all_checks(self, services):
      return all(self.checks.get(s, True) for s in services)

  lines = [boundary(-5), boundary(-1.8), boundary(1.8), boundary(5)]
  model = NS(
    timestampEof=int(99950000000.0),
    laneLines=[NS(x=b.x, y=b.y) for b in lines],
    laneLineProbs=[0.99] * 4,
    laneLineStds=[0.05] * 4,
    roadEdges=[],
    roadEdgeStds=[],
    meta=NS(desirePrediction=[0] * 8, laneChangeState=0),
  )
  sm = SM(
    modelV2=model,
    carState=NS(vEgo=30, leftBlinker=False, rightBlinker=False),
    carControl=NS(latActive=True),
    controlsState=NS(curvature=0.008),
    liveParameters=NS(valid=True),
    liveCalibration=NS(calStatus=1),
  )
  sm.checks = {}
  sm.logMonoTime = {name: int(100000000000.0) for name in sm}
  return sm


class TestAdapterTests:
  def test_camera_boottime_offset_after_suspend(self):
    sm, detector, seen = (make_sm(), ldw.ActiveLaneDepartureWarning(), False)
    for i in range(12):
      now = 100 + i * 0.05
      sm['modelV2'].timestampEof = int((now + 300 - 0.05) * 1000000000.0)
      sm.logMonoTime = {name: int(now * 1000000000.0) for name in sm}
      seen |= ldw.update_active_ldw(detector, sm, now, True, 1, camera_now=now + 300).right
    assert seen

  def test_adapter_uses_actual_curvature_with_zero_model_desire(self):
    sm, detector, seen = (make_sm(), ldw.ActiveLaneDepartureWarning(), False)
    for i in range(12):
      now = 100 + i * 0.05
      sm['modelV2'].timestampEof = int((now - 0.05) * 1000000000.0)
      sm.logMonoTime = {name: int(now * 1000000000.0) for name in sm}
      seen |= ldw.update_active_ldw(detector, sm, now, True, 1).right
    assert seen

  def test_calibration_invalid_parameters_and_stale_controls_suppress(self):
    for fault in ('calibration', 'parameters', 'stale', 'frequency'):
      sm, detector = (make_sm(), ldw.ActiveLaneDepartureWarning())
      if fault == 'calibration':
        sm['liveCalibration'].calStatus = 0
      if fault == 'parameters':
        sm['liveParameters'].valid = False
      if fault == 'stale':
        sm.logMonoTime['controlsState'] = int(90000000000.0)
      if fault == 'frequency':
        sm.checks['modelV2'] = False
      assert not ldw.update_active_ldw(detector, sm, 100, True, 1).right

  def test_adapter_passes_lane_change_state(self):
    sm, detector = (make_sm(), ldw.ActiveLaneDepartureWarning())
    sm['modelV2'].meta.laneChangeState = 2
    for i in range(30):
      now = 100 + i * 0.05
      sm['modelV2'].timestampEof = int((now - 0.05) * 1000000000.0)
      sm.logMonoTime = {name: int(now * 1000000000.0) for name in sm}
      result = ldw.update_active_ldw(detector, sm, now, True, 1)
      assert not (result.left or result.right)

  def test_missing_model_arrays_become_unavailable(self):
    assert ldw.boundary_from_model(NS(), 1) is None
    assert ldw.boundary_from_model(make_sm()['modelV2'], 0, True) is None

  def run_planner(
    self, fault=False, clock=True, enabled=True, low_speed=False, speed=30, sensitivity=1, curvature=0.008, lateral_active=True, legacy_warning=False
  ):

    class StopLoop(Exception):
      pass

    sm, sent, errors, calls, planned = (make_sm(), [], [], [], [])

    def adapter(*args, **kwargs):
      calls.append(1)
      if fault:
        raise RuntimeError('injected detector fault')
      return ldw.update_active_ldw(*args, **kwargs)

    sm['selfdriveStateSP'] = NS(buttonsReleaseToggle=0)
    sm['carControl'].latActive = lateral_active
    sm['carState'].vEgo = speed
    sm['controlsState'].curvature = curvature
    sm.frame, sm.updated = (0, {'modelV2': True})

    def update():
      if sm.frame >= 30:
        raise StopLoop()
      sm.frame += 1
      now = 100 + sm.frame * 0.05
      sm['modelV2'].timestampEof = int((now - 0.05) * 1000000000.0)
      sm.logMonoTime = {name: int(now * 1000000000.0) for name in sm}

    sm.update = update

    class Params:
      def get(self, key, *args, **kwargs):
        return sensitivity if key == 'LdwSensitivity' else b''

      def get_bool(self, key):
        if key == 'LdwLowSpeed':
          return low_speed and sm.frame >= 10
        return enabled and key in ('LdwWhileSteering', 'IsLdwEnabled')

    class Planner:
      def __init__(self, *args):
        self.sla = NS(update_buttons=lambda *a: None)

      def update(self, *args):
        pass

      def publish(self, *args):
        planned.append(1)

    tree = ast.parse((BASE / 'selfdrive/controls/lib/ldw.py').read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    legacy = {'log': NS(Desire=NS(laneChangeLeft=3, laneChangeRight=4)), 'DT_CTRL': 0.01, 'CV': NS(MPH_TO_MS=0.44704)}
    exec(compile(tree, '<legacy ldw>', 'exec'), legacy)
    if legacy_warning:

      class AlwaysWarn:
        left, right = (True, True)

        def update(self, *args):
          pass

      legacy['LaneDepartureWarning'] = AlwaysWarn
    env: dict[str, Any] = dict(
      time=NS(monotonic=lambda: 100 + sm.frame * 0.05, CLOCK_BOOTTIME=7, clock_gettime=lambda clock: 100 + sm.frame * 0.05),
      car=NS(CarParams=object),
      custom=NS(CarParamsSP=object),
      log=NS(LiveCalibrationData=NS(Status=NS(calibrated=1))),
      config_realtime_process=lambda *a: None,
      Priority=NS(CTRL_LOW=1),
      cloudlog=NS(info=lambda *a: None, exception=lambda *a: errors.append(a)),
      Params=Params,
      get_gps_location_service=lambda p: 'gpsLocation',
      LaneDepartureWarning=legacy['LaneDepartureWarning'],
      ActiveLaneDepartureWarning=ldw.ActiveLaneDepartureWarning,
      update_active_ldw=adapter,
      Result=ldw.Result,
      config_from_settings=ldw.config_from_settings,
      LongitudinalPlanner=Planner,
      messaging=NS(
        log_from_bytes=lambda *a: NS(brand='honda'),
        PubMaster=lambda *a: NS(send=lambda name, msg: sent.append(msg)),
        SubMaster=lambda *a, **kw: sm,
        new_message=lambda *a: NS(driverAssistance=NS()),
      ),
      __name__='test',
    )
    if not clock:
      del env['time'].CLOCK_BOOTTIME
    source = ast.parse((SOURCE / 'selfdrive/controls/plannerd.py').read_text())
    source.body = [n for n in source.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    exec(compile(source, '<patched plannerd>', 'exec'), env)
    with pytest.raises(StopLoop):
      env['main']()
    assert len(planned) == 30
    return (sent, errors, calls)

  def test_real_plannerd_main_publishes_advisory(self):
    sent, errors, calls = self.run_planner()
    assert not errors
    assert any(m.driverAssistance.rightLaneDeparture for m in sent)
    assert all(m.valid for m in sent)
    assert not any(m.driverAssistance.leftLaneDeparture for m in sent)


class TestPlannerIsolationTests:
  run_planner = TestAdapterTests.run_planner

  def test_detector_exception_does_not_stop_planner(self):
    sent, errors, calls = self.run_planner(fault=True)
    assert len(calls) == 1
    assert len(errors) == 1
    assert len(sent) == 30
    assert not any(m.driverAssistance.leftLaneDeparture or m.driverAssistance.rightLaneDeparture for m in sent)

  def test_missing_boottime_suppresses_added_detector(self):
    sent, errors, calls = self.run_planner(clock=False)
    assert len(sent) == 30
    assert not (errors or calls)

  def test_toggle_off_skips_detector(self):
    sent, errors, calls = self.run_planner(enabled=False, fault=True)
    assert len(sent) == 30
    assert not (errors or calls)


class TestSettingsTests:
  setUp = TestBehaviourTests.setUp
  run_frames = TestBehaviourTests.run_frames
  run_planner = TestAdapterTests.run_planner

  def test_defaults_match_original_config(self):
    assert ldw.config_from_settings() == ldw.Config()

  def test_invalid_sensitivity_falls_back_to_standard(self):
    for value in (None, b'2', '2', -1, 3, 999, True, float('nan')):
      assert ldw.config_from_settings(sensitivity=value) == ldw.Config()

  def test_low_speed_on_off_and_standstill(self):
    for enabled in (False, True):
      for mph in (0, 0.5, 2, 2.1, 15, 30, 32):
        self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(enabled))
        result = self.run_frames(20, speed=mph * 0.44704, lines=(boundary(-2.8), boundary(0.9)))
        assert any(r.right for r in result) == (mph > (2 if enabled else 31))

  def test_sensitivity_orders_warning_on_approach(self):
    first = []
    for profile in range(3):
      self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(sensitivity=profile))
      fired = None
      for i in range(40):
        offset = 2.0 - i * 0.025
        if self.run_frames(1, lines=(boundary(-2.8, -0.02), boundary(offset, -0.02)))[0].right and fired is None:
          fired = i
      assert fired is not None
      first.append(fired)
    assert first[0] > first[1]
    assert first[1] > first[2]

  def test_tuning_keeps_latch_and_signal_history(self):
    self.run_frames(80, curvature=0.008)
    self.detector.configure(ldw.config_from_settings(True, 2))
    assert self.detector.sides[1].latched
    assert not any(r.right for r in self.run_frames(20, curvature=0.008))
    self.run_frames(1, left_signal=True)
    stamp = self.detector.last_signal[:]
    self.detector.configure(ldw.config_from_settings(False, 0))
    assert stamp == self.detector.last_signal
    assert not any(r.right for r in self.run_frames(20, curvature=0.008))

  def test_repeated_same_settings_do_not_reset_confirmation(self):
    seen = False
    for _ in range(20):
      self.detector.configure(ldw.config_from_settings())
      seen |= self.run_frames(1, curvature=0.008)[0].right
    assert seen

  def test_planner_picks_up_low_speed_setting_without_restart(self):
    sent, errors, calls = self.run_planner(low_speed=True, speed=25 * 0.44704, curvature=0.025)
    assert not errors
    assert not any(m.driverAssistance.rightLaneDeparture for m in sent[:20])
    assert any(m.driverAssistance.rightLaneDeparture for m in sent[20:])

  def test_new_python_files_parse(self):
    for file in ('selfdrive/controls/lib/active_ldw.py', 'selfdrive/controls/plannerd.py', 'selfdrive/ui/layouts/settings/toggles.py'):
      ast.parse((SOURCE / file).read_text())

  def setup_method(self):
    self.setUp()


class TestTunerUITests:
  def test_controls_construct_and_persist_settings(self):
    values = {'LdwSensitivity': 99, 'LongitudinalPersonality': 1}

    class Params:
      def get(self, key, **kw):
        return values.get(key)

      def get_bool(self, key):
        return bool(values.get(key, False))

      def put(self, key, value, **kw):
        values[key] = value

      def put_bool(self, key, value, **kw):
        values[key] = value

    class Widget:
      def __init__(self):
        pass

    class Action:
      def set_enabled(self, value):
        self.enabled = value

      def set_state(self, value):
        self.state = value

      def get_state(self):
        return getattr(self, 'state', False)

      def set_selected_button(self, value):
        self.selected_index = value

    class Item:
      def __init__(self, title, description, *args, **kw):
        self.title, self.description = (title, description)
        self.action_item = Action()
        self.callback = kw.get('callback')
        self.action_item.selected_index = kw.get('selected_index')

      def set_description(self, value):
        self.description = value

      def set_icon(self, value):
        self.icon = value

    tree = ast.parse((SOURCE / 'selfdrive/ui/layouts/settings/toggles.py').read_text())
    tree.body = [
      n
      for n in tree.body
      if isinstance(n, ast.ClassDef) or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DESCRIPTIONS' for t in n.targets))
    ]
    env: dict[str, Any] = dict(
      Params=Params,
      Widget=Widget,
      UnknownKeyName=KeyError,
      sensitivity_index=ldw.sensitivity_index,
      SETTING_LIMITS=fcw.SETTING_LIMITS,
      setting_value=fcw.setting_value,
      option_item_sp=Item,
      toggle_item=Item,
      multiple_button_item=Item,
      tr=lambda s: s,
      tr_noop=lambda s: s,
      Scroller=lambda *a, **k: NS(),
      ui_state=NS(add_engaged_transition_callback=lambda cb: None),
    )
    exec(compile(tree, '<settings ui>', 'exec'), env)
    ui = env['TogglesLayout']()
    assert 'LdwLowSpeed' in ui._toggles
    assert ui._ldw_sensitivity_setting.action_item.selected_index == 1
    ui._ldw_sensitivity_setting.callback(2)
    assert values['LdwSensitivity'] == 2
    ui._toggles['LdwLowSpeed'].callback(True)
    assert values['LdwLowSpeed']
    assert 'OnroadCycleRequested' not in values
    assert ui._fcw_sensitivity_setting.action_item.selected_index == 0
    ui._fcw_sensitivity_setting.callback(1)
    assert values['AggressiveFCW'] is True
    ui._fcw_sensitivity_setting.callback(0)
    assert values['AggressiveFCW'] is False
    ui._fcw_sensitivity_setting.callback(99)
    assert values['AggressiveFCW'] is False


class TestAllSteeringStatesTests:
  setUp = TestBehaviourTests.setUp
  run_frames = TestBehaviourTests.run_frames
  run_planner = TestAdapterTests.run_planner

  def test_above_two_mph_in_both_steering_states(self):
    for lateral in (False, True):
      for mph in (0, 2, 2.1, 10, 30, 60):
        self.detector = ldw.ActiveLaneDepartureWarning(ldw.config_from_settings(True))
        frames = self.run_frames(20, speed=mph * 0.44704, lateral_active=lateral, lines=(boundary(-2.8), boundary(0.9)))
        assert any(r.right for r in frames) == (mph > 2)

  def test_planner_manual_mode_low_speed_warning(self):
    sent, errors, calls = self.run_planner(low_speed=True, speed=25 * 0.44704, curvature=0.025, lateral_active=False)
    assert not errors
    assert any(m.driverAssistance.rightLaneDeparture for m in sent[20:])

  def test_stock_does_not_bypass_predictive_suppression(self):
    sent, errors, calls = self.run_planner(speed=0, lateral_active=False, legacy_warning=True)
    assert not errors
    assert not any(m.driverAssistance.leftLaneDeparture or m.driverAssistance.rightLaneDeparture for m in sent)

  def test_stock_fallback_when_predictive_disabled_or_faulted(self):
    for options in ({'enabled': False}, {'fault': True}, {'clock': False}):
      sent, errors, calls = self.run_planner(legacy_warning=True, **options)
      assert all(m.driverAssistance.leftLaneDeparture and m.driverAssistance.rightLaneDeparture for m in sent)

  def test_steering_transition_does_not_rearm_latch(self):
    self.run_frames(80, curvature=0.008)
    assert not any(r.right for r in self.run_frames(20, curvature=0.008, lateral_active=False))
    assert not any(r.right for r in self.run_frames(20, curvature=0.008, lateral_active=True))

  def setup_method(self):
    self.setUp()
