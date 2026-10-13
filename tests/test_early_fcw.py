import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'payload' if (ROOT / 'payload').exists() else ROOT
spec = importlib.util.spec_from_file_location('early_fcw', SOURCE / 'selfdrive/controls/lib/early_fcw.py')
fcw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fcw)


class TestDetectorTests:
  def setUp(self):
    self.d = fcw.EarlyForwardWarning()
    self.t = 10.0

  def frame(self, **kw):
    self.t += 0.05
    args = dict(
      valid=True,
      urgent=False,
      speed=25.0,
      acceleration=0.0,
      gap=25.0,
      lead_speed=15.0,
      lead_accel=0.0,
      probability=0.99,
      lateral=0.0,
      curvature=0.0,
      track=(True, 1),
      path_lateral=0.0,
    )
    args.update(kw)
    return self.d.update(self.t, self.t, **args)

  def run_frames(self, n=25, **kw):
    return [self.frame(**kw) for _ in range(n)]

  def test_closing_encounter_warns_after_stable_confirmation(self):
    alerts = [self.frame(gap=30 - i * 0.5) for i in range(30)]
    assert not any(alerts[:12])
    assert any(alerts)

  def test_equal_speed_close_following_is_not_collision_alarm(self):
    assert not any(self.run_frames(gap=8, lead_speed=25))

  def test_opening_gap_is_quiet(self):
    assert not any(self.run_frames(gap=10, lead_speed=30))

  def test_sufficient_braking_quiet_but_light_braking_warns(self):
    assert not any(self.run_frames(acceleration=-4))
    assert any(self.run_frames(acceleration=-0.5))

  def test_lead_braking_at_initially_equal_speed(self):
    assert any(self.run_frames(gap=20, lead_speed=25, lead_accel=-4))

  def test_stationary_high_confidence_lead(self):
    assert any(self.run_frames(gap=45, lead_speed=0))

  def test_invalid_envelopes_suppress(self):
    for kw in (
      {'speed': -1},
      {'probability': 0.74},
      {'path_lateral': 1.2},
      {'path_lateral': None},
      {'gap': 0},
      {'lead_speed': -6},
      {'lead_accel': -10},
      {'gap': float('nan')},
      {'valid': False},
      {'acceleration': float('inf')},
    ):
      self.setUp()
      assert not any(self.run_frames(**kw))

  def test_track_switch_and_geometry_jumps_restart_confirmation(self):
    for i in range(100):
      assert not self.frame(track=(True, i // 5))
    self.setUp()
    for i in range(100):
      assert not self.frame(gap=15 if i % 2 else 25)

  def test_one_pulse_per_episode(self):
    alerts = self.run_frames(300)
    starts = sum((v and (i == 0 or not alerts[i - 1]) for i, v in enumerate(alerts)))
    assert starts == 1

  def test_dropouts_and_mode_pause_do_not_rearm(self):
    assert any(self.run_frames())
    self.run_frames(20, probability=0.5)
    self.d.pause()
    assert not any(self.run_frames(100))

  def test_long_absence_does_not_latch_forever(self):
    assert any(self.run_frames())
    self.run_frames(220, probability=0.5)
    assert any(self.run_frames(30))

  def test_safe_observations_rearm_after_cooldown(self):
    assert any(self.run_frames())
    self.run_frames(210, gap=80, lead_speed=25)
    assert any(self.run_frames())

  def test_urgent_warning_preempts_and_latches(self):
    assert not self.frame(urgent=True)
    assert not any(self.run_frames(300))

  def test_repeated_stale_and_reversed_timestamps(self):
    args = dict(
      valid=True, urgent=False, speed=25, acceleration=0, gap=25, lead_speed=15, lead_accel=0, probability=0.99, lateral=0, curvature=0, track=1, path_lateral=0
    )
    for i in range(30):
      assert not self.d.update(10 + i * 0.01, 10, **args)
    assert not self.d.update(11, 9, **args)
    assert not self.d.update(8, 8, **args)
    assert not self.d.update(8.05, 9, **args)

  def setup_method(self):
    self.setUp()


class TestServiceTests:
  def setUp(self):
    self.s = fcw.EarlyFCWService()
    self.enabled = True
    self.errors = []
    self.values = {k: v[3] for k, v in fcw.SETTING_LIMITS.items()}
    self.params = NS(get_bool=lambda key: self.enabled, get=lambda key, **kw: self.values[key])
    self.cs = NS(canValid=True, standstill=False, gearShifter=4, vEgo=25.0, aEgo=0.0, brakePressed=False)

    class SM(dict):
      def all_checks(self, services):
        return self.valid

    self.sm = SM(
      radarState=NS(
        leadOne=NS(status=True, dRel=25.0, vLead=15.0, aLeadK=0.0, modelProb=0.99, yRel=0.0, radar=True, radarTrackId=1), radarErrors=NS(to_dict=dict)
      ),
      controlsState=NS(curvature=0.0),
      liveCalibration=NS(calStatus=1),
      modelV2=NS(position=NS(x=[0.0, 50.0, 100.0, 200.0], y=[0.0, 0.0, 0.0, 0.0])),
    )
    self.sm.valid = True
    self.t = 10.0

  def frame(self, stale=None, urgent=False):
    self.t += 0.05
    self.sm.logMonoTime = {s: int(self.t * 1000000000.0) for s in self.sm}
    if stale:
      self.sm.logMonoTime[stale] -= int(1000000000.0)
    return self.s.update(self.sm, self.cs, self.t, self.params, self.t, urgent, 4, 1, NS(exception=self.errors.append))

  def test_default_and_live_setting(self):
    self.enabled = False
    assert not any(self.frame() for _ in range(30))
    self.enabled = True
    frames = [self.frame() for _ in range(50)]
    assert any(frames)
    self.enabled = False
    assert not any([self.frame() for _ in range(50)][25:])

  def test_stale_services_and_invalid_calibration(self):
    for service in ('radarState', 'modelV2', 'controlsState'):
      self.setUp()
      assert not any(self.frame(stale=service) for _ in range(30))
    self.sm['liveCalibration'].calStatus = 0
    assert not any(self.frame() for _ in range(30))

  def test_reverse_and_invalid_can(self):
    self.cs.gearShifter = 2
    assert not any(self.frame() for _ in range(30))
    self.cs.gearShifter = 4
    self.cs.canValid = False
    assert not any(self.frame() for _ in range(30))

  def test_brake_switch_does_not_suppress_insufficient_braking(self):
    self.cs.brakePressed = True
    self.cs.aEgo = -0.5
    frames = [self.frame() for _ in range(30)]
    assert any(frames)

  def test_exception_disables_only_addition_and_logs_once(self):
    del self.sm['radarState'].leadOne
    assert not any(self.frame() for _ in range(30))
    assert self.s.faulted
    assert len(self.errors) == 1

  def test_actual_selfdrived_advisory_hook_manual_and_urgent(self):
    tree = ast.parse((SOURCE / 'selfdrive/selfdrived/selfdrived.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_events')
    hook = next(n for n in method.body if isinstance(n, ast.If) and 'self.early_fcw.update' in ast.unparse(n.test))
    code = compile(ast.Module(body=[hook], type_ignores=[]), '<fcw hook>', 'exec')
    captured = []
    obj = NS(
      CP=NS(notCar=False),
      early_fcw=NS(update=lambda *a: captured.append(a) or not a[5]),
      sm=self.sm,
      early_fcw_car_time=self.t,
      params=self.params,
      events_sp=NS(add=captured.append),
      enabled=False,
    )
    env = dict(
      self=obj,
      CS=self.cs,
      time=NS(monotonic=lambda: self.t),
      planner_fcw=False,
      model_fcw=False,
      car=NS(CarState=NS(GearShifter=NS(drive=4))),
      log=NS(LiveCalibrationData=NS(Status=NS(calibrated=1))),
      cloudlog=None,
      custom=NS(OnroadEventSP=NS(EventName=NS(earlyForwardWarning=24))),
    )
    exec(code, env)
    assert captured[-1] == 24
    env['model_fcw'] = True
    captured.clear()
    exec(code, env)
    assert len(captured) == 1

  def setup_method(self):
    self.setUp()
