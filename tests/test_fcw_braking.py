"""Physical trajectory regressions, not road-performance validation."""
import pytest
import test_early_fcw as base

fcw = base.fcw


class TestBrakingForecast:
  setUp = base.TestDetectorTests.setUp
  frame = base.TestDetectorTests.frame
  run_frames = base.TestDetectorTests.run_frames

  def setup_method(self):
    self.setUp()

  def trajectory(self, *, speed, acceleration, lead_speed, lead_accel, gap, seconds, **kw):
    alerts = []
    for _ in range(round(seconds / .05)):
      alerts.append(self.frame(speed=speed, acceleration=acceleration if speed else 0,
                               lead_speed=lead_speed, lead_accel=lead_accel if lead_speed else 0, gap=gap, **kw))
      next_speed = max(0., speed + acceleration * .05)
      next_lead = max(0., lead_speed + lead_accel * .05)
      gap += ((lead_speed + next_lead) - (speed + next_speed)) * .025
      speed, lead_speed = next_speed, next_lead
    return alerts, gap

  def test_firm_braking_to_stationary_queue_keeps_five_meter_clearance(self):
    # 20 m/s at -3.5 m/s^2 stops in 57.14 m. Old response forecast loses
    # braking after two seconds, falsely consuming the remaining five meters.
    gap = 20 ** 2 / (2 * 3.5) + 5
    assert fcw.minimum_gap(gap, 20, -3.5, 0, 0, True, horizon=8) < 3
    alerts, final_gap = self.trajectory(speed=20, acceleration=-3.5, lead_speed=0, lead_accel=0,
                                      gap=gap, seconds=7, settings=fcw.FCWSettings(forecast=80))
    assert final_gap == pytest.approx(5, abs=.01)
    assert not any(alerts)

  def test_braking_tail_matches_analytic_stopping_distance(self):
    gap = 70
    for response in (False, True):
      result = fcw.minimum_gap(gap, 20, -3.5, 0, 0, response, horizon=8, ego_braking=-3.5)
      assert result == pytest.approx(gap - 20 ** 2 / 7, abs=.01)

  def test_inadequate_sustained_braking_still_warns(self):
    alerts, _ = self.trajectory(speed=20, acceleration=-1.5, lead_speed=0, lead_accel=0,
                               gap=45, seconds=1.5)
    assert any(alerts)

  def test_releasing_brakes_removes_credit(self):
    self.run_frames(20, speed=20, acceleration=-3.5, gap=65, lead_speed=0)
    assert self.d.braking_history
    alerts, _ = self.trajectory(speed=20, acceleration=0, gap=35, lead_speed=0, lead_accel=0, seconds=1)
    assert any(alerts)

  def test_transient_deceleration_and_track_change_do_not_persist(self):
    self.frame(acceleration=-7, lead_accel=-7)
    self.run_frames(12, acceleration=0, lead_accel=0)
    assert all(row[1:] == (0, 0) for row in self.d.braking_history)
    self.frame(track=(True, 2), acceleration=-3, lead_accel=-3)
    assert len(self.d.braking_history) == 1
    self.frame(valid=False)
    assert not self.d.braking_history

  def test_sustained_lead_braking_warns_before_two_second_coast_forecast(self):
    assert fcw.minimum_gap(25, 20, 0, 20, -3, True) > 3
    assert fcw.minimum_gap(25, 20, 0, 20, -3, True, lead_braking=-3) < 3
    alerts, _ = self.trajectory(speed=20, acceleration=0, lead_speed=20, lead_accel=-3,
                               gap=25, seconds=.9)
    assert any(alerts)

  def test_lead_braking_to_stop_does_not_invent_reverse(self):
    assert fcw.minimum_gap(10, 0, 0, 2, -3, False, horizon=8, lead_braking=-3) == 10

  def test_ordinary_creep_then_stop_is_quiet(self):
    alerts, gap = self.trajectory(speed=1, acceleration=0, lead_speed=0, lead_accel=0, gap=3.5, seconds=1)
    braking, _ = self.trajectory(speed=1, acceleration=-1, lead_speed=0, lead_accel=0, gap=gap, seconds=2)
    assert not any(alerts + braking)

  def test_unattended_creep_still_warns_before_contact(self):
    alerts, gap = self.trajectory(speed=.5, acceleration=0, lead_speed=0, lead_accel=0, gap=2, seconds=3)
    assert gap > 0
    assert any(alerts)

  def test_creep_setting_is_independent_and_blended(self):
    s = fcw.FCWSettings()
    assert s.delay_at_speed(0) == .8
    assert s.delay_at_speed(5 * .44704) == pytest.approx(.8)
    assert s.delay_at_speed(7.5 * .44704) == pytest.approx(1.4)
    assert s.delay_at_speed(10 * .44704) == pytest.approx(2.)
    assert s.delay_at_speed(30) == 2.
    s = fcw.FCWSettings(creep_response_delay=20)
    assert s.delay_at_speed(.5) == 2.
    assert s.response_delay == 2.


class TestCreepSettingService:
  setUp = base.TestServiceTests.setUp
  frame = base.TestServiceTests.frame

  def setup_method(self):
    self.setUp()

  def test_live_creep_setting_and_brake_switch_do_not_disable_risk(self):
    self.values['FCWCreepResponseDelay'] = 16
    self.cs.brakePressed = True
    frames = [self.frame() for _ in range(30)]
    assert any(frames)
    assert self.s.settings.creep_response_delay == 1.6
    self.values['FCWCreepResponseDelay'] = 6
    for _ in range(25):
      self.frame()
    assert self.s.settings.creep_response_delay == .6
    assert self.s.settings.response_delay == 2.
