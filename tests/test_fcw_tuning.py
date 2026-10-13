import pytest

"""Synthetic behavior/integration tests; no claim of road detection validation."""
import ast
import math
from types import SimpleNamespace as NS
import test_early_fcw as base

fcw, SOURCE = (base.fcw, base.SOURCE)


class TestTuningTests:
  def test_parameter_bounds_defaults_and_grid(self):
    for key, (lo, hi, step, default) in fcw.SETTING_LIMITS.items():
      for bad in (None, 'bad', float('nan'), float('inf')):
        assert fcw.setting_value(key, bad) == default
      assert fcw.setting_value(key, -999) == lo
      assert fcw.setting_value(key, 999) == hi
      for v in range(lo, hi + 1, step):
        assert fcw.setting_value(key, str(v)) == v

  def test_longer_delay_and_horizon_are_monotone_for_stationary_target(self):

    def gap(delay, horizon):
      return fcw.minimum_gap(150, 33.528, 0, 0, 0, True, delay, horizon)

    assert gap(2, 5) < gap(1.2, 5)
    assert gap(2, 8) < gap(2, 5)
    assert fcw.minimum_gap(3, 0, 0, 0, -5, False, 2, 8) > 2.99

  def test_acceleration_does_not_continue_for_eight_seconds(self):
    assert fcw.minimum_gap(100, 0, 2, 0, 0, False, 2, 8) == pytest.approx(72.0, abs=1e-07, rel=0)

  def test_curve_both_directions_and_adjacent_lane(self):
    xs = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0]
    for sign in (-1, 1):
      ys = [sign * x * x / 2000 for x in xs]
      gap = 100 - fcw.RADAR_TO_CAMERA
      assert fcw.path_offset(gap, -ys[5], sign * 0.001, xs, ys) == pytest.approx(0.0, abs=1e-07, rel=0)
      assert abs(fcw.path_offset(gap, -ys[5] + 3.5, sign * 0.001, xs, ys)) > 3.0
      assert abs(fcw.path_offset(gap, 0, sign * 0.001, xs, ys)) > 4.0

  def test_no_long_range_path_extrapolation(self):
    assert fcw.path_offset(100, 0, 0, [0, 20], [0, 0]) is None
    assert fcw.path_offset(100, 0, 0, [0, 100], [0, math.nan]) is None
    assert fcw.path_offset(100, 0, 0, [100, 0], [0, 0]) is None
    assert fcw.path_offset(100, 0, 0) is None
    assert fcw.path_offset(3, 0, 0) == pytest.approx(0.0, abs=1e-07, rel=0)

  def test_low_speed_override_needs_matching_camera_evidence(self):
    lead = NS(modelProb=0.0, radar=True, dRel=3.0, yRel=0.0, vLead=0.0)
    vision = NS(prob=0.8, x=[4.52], y=[0.0], v=[0.0])
    model = NS(leadsV3=[vision])
    assert fcw.lead_probability(lead, model, 0) == 0.8
    vision.y = [3.5]
    assert fcw.lead_probability(lead, model, 0) == 0.0
    vision.y = [0.0]
    vision.x = [40.0]
    assert fcw.lead_probability(lead, model, 0) == 0.0
    vision.x = [4.52]
    vision.v = [10.0]
    assert fcw.lead_probability(lead, model, 0) == 0.0
    assert fcw.lead_probability(lead, NS(), 0) == 0.0

  def test_registered_params_and_actual_ui_configuration(self):
    header = (SOURCE / 'common/params_keys.h').read_text()
    ui = ast.parse((SOURCE / 'selfdrive/ui/layouts/settings/toggles.py').read_text())
    for key, (_, _, _, default) in fcw.SETTING_LIMITS.items():
      assert '"' + key + '", {PERSISTENT | BACKUP, INT, "' + str(default) + '"}' in header
    cls = next(n for n in ui.body if isinstance(n, ast.ClassDef) and n.name == 'TogglesLayout')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    first = next((i for i, n in enumerate(init.body) if 'self._fcw_tuning = {}' == ast.unparse(n)))
    last = next((i for i, n in enumerate(init.body) if 'self._toggles = {}' == ast.unparse(n)))
    values = dict.fromkeys(fcw.SETTING_LIMITS, 'bad')
    params = NS(get=lambda k, **kw: values[k], put=lambda k, v, **kw: values.update({k: v}), get_bool=lambda k: True)
    obj = NS(_params=params)
    env = dict(self=obj, SETTING_LIMITS=fcw.SETTING_LIMITS, setting_value=fcw.setting_value, tr=lambda s: s, option_item_sp=lambda **kw: NS(**kw))
    exec(compile(ast.Module(body=init.body[first:last], type_ignores=[]), '<actual UI>', 'exec'), env)
    assert len(obj._fcw_tuning) == 4
    for k, w in obj._fcw_tuning.items():
      lo, hi, step, default = fcw.SETTING_LIMITS[k]
      assert (w.min_value, w.max_value, w.value_change_step) == (lo, hi, step)
      assert values[k] == default
      assert w.enabled()
    assert obj._fcw_tuning['FCWResponseDelay'].label_callback(20) == '2.0 s'
    assert obj._fcw_tuning['FCWMinConfidence'].label_callback(75) == '75%'


class TestAllSpeedTests:
  setUp = base.TestDetectorTests.setUp
  frame = base.TestDetectorTests.frame
  run_frames = base.TestDetectorTests.run_frames

  def test_stopped_close_queue_is_silent(self):
    assert not any(self.run_frames(400, speed=0, lead_speed=0, gap=1.0))

  def test_creep_from_standstill_with_physically_closing_gap(self):
    self.run_frames(40, speed=0, lead_speed=0, gap=1.5)
    alerts = [self.frame(speed=0.5, lead_speed=0, gap=1.5 - i * 0.025) for i in range(30)]
    assert any(alerts)

  def test_zero_speed_reversing_lead(self):
    assert any(self.run_frames(30, speed=0, lead_speed=-0.5, gap=1.0))

  def test_zero_speed_initial_acceleration(self):
    assert any(self.run_frames(30, speed=0, acceleration=0.5, lead_speed=0, gap=0.75))

  def test_tiny_closing_noise_is_silent(self):
    assert not any(self.run_frames(100, speed=0.05, lead_speed=0, gap=1.0))

  def test_creep_with_sufficient_braking_is_silent(self):
    assert not any(self.run_frames(50, speed=0.5, acceleration=-1, lead_speed=0, gap=1.0))

  def test_confidence_selector_changes_eligibility(self):
    assert any(self.run_frames(probability=0.8))
    self.setUp()
    assert not any(self.run_frames(probability=0.8, settings=fcw.FCWSettings(confidence=85)))
    self.setUp()
    assert any(self.run_frames(probability=0.55, settings=fcw.FCWSettings(confidence=55)))

  def test_second_creep_episode_after_stopped_interval(self):
    assert any(self.run_frames(30, speed=0.5, lead_speed=0, gap=0.9))
    self.run_frames(80, speed=0, lead_speed=0, gap=1.0)
    assert any(self.run_frames(30, speed=0.5, lead_speed=0, gap=0.9))

  def test_curve_gate_uses_path_instead_of_raw_yrel(self):
    assert any(self.run_frames(gap=60, lead_speed=0, lateral=-4, curvature=0.002, path_lateral=0))
    self.setUp()
    assert not any(self.run_frames(gap=60, lead_speed=0, lateral=0, curvature=0.002, path_lateral=-4))

  def setup_method(self):
    self.setUp()


class TestFullServiceTests:
  setUp = base.TestServiceTests.setUp
  frame = base.TestServiceTests.frame

  def test_standstill_flag_does_not_block_rollback_warning(self):
    self.cs.standstill = True
    self.cs.vEgo = 0.0
    lead = self.sm['radarState'].leadOne
    lead.dRel = 1.0
    lead.vLead = -0.5
    frames = [self.frame() for _ in range(30)]
    assert any(frames)
    assert not self.errors

  def test_radar_override_confirmation_reaches_detector(self):
    self.cs.vEgo = 0.5
    lead = self.sm['radarState'].leadOne
    lead.modelProb = 0.0
    lead.dRel = 0.9
    lead.vLead = 0.0
    self.sm['modelV2'].leadsV3 = [NS(prob=0.8, x=[2.52], y=[0.0], v=[0.0])]
    frames = [self.frame() for _ in range(30)]
    assert any(frames)

  def test_live_numeric_settings_apply(self):
    self.sm['radarState'].leadOne.modelProb = 0.8
    self.values['FCWMinConfidence'] = 95
    frames = [self.frame() for _ in range(30)]
    assert not any(frames)
    self.values.update(FCWMinConfidence=75, FCWResponseDelay=24, FCWForecast=80)
    frames = [self.frame() for _ in range(50)]
    assert any(frames)
    assert self.s.settings.response_delay == 2.4
    assert self.s.settings.forecast == 8.0

  def test_missing_long_range_path_disables_advisory(self):
    self.sm['modelV2'] = NS()
    frames = [self.frame() for _ in range(50)]
    assert not any(frames)
    assert not self.errors

  def setup_method(self):
    self.setUp()
