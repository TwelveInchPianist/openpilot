"""Synthetic behavior/integration tests; no claim of road detection validation."""
import ast
import math
import unittest
from types import SimpleNamespace as NS

import test_early_fcw as base

fcw, SOURCE = base.fcw, base.SOURCE


class TuningTests(unittest.TestCase):
  def test_parameter_bounds_defaults_and_grid(self):
    for key, (lo, hi, step, default) in fcw.SETTING_LIMITS.items():
      for bad in (None, 'bad', float('nan'), float('inf')):
        self.assertEqual(fcw.setting_value(key, bad), default)
      self.assertEqual(fcw.setting_value(key, -999), lo)
      self.assertEqual(fcw.setting_value(key, 999), hi)
      for v in range(lo, hi + 1, step):
        self.assertEqual(fcw.setting_value(key, str(v)), v)

  def test_longer_delay_and_horizon_are_monotone_for_stationary_target(self):
    def gap(delay, horizon):
      return fcw.minimum_gap(150, 33.528, 0, 0, 0, True, delay, horizon)
    self.assertLess(gap(2, 5), gap(1.2, 5))
    self.assertLess(gap(2, 8), gap(2, 5))
    self.assertGreater(fcw.minimum_gap(3, 0, 0, 0, -5, False, 2, 8), 2.99)

  def test_acceleration_does_not_continue_for_eight_seconds(self):
    # 2 m/s^2 for 2 seconds then coasting: total distance 4 + 6*4 = 28 m.
    self.assertAlmostEqual(fcw.minimum_gap(100, 0, 2, 0, 0, False, 2, 8), 72.)

  def test_curve_both_directions_and_adjacent_lane(self):
    xs=[0., 20., 40., 60., 80., 100., 120.]
    for sign in (-1, 1):
      ys=[sign * x*x/2000 for x in xs]
      # gap is measured from front; model coordinates include 1.52 m offset.
      gap=100-fcw.RADAR_TO_CAMERA
      self.assertAlmostEqual(fcw.path_offset(gap, -ys[5], sign*.001, xs, ys), 0.)
      self.assertGreater(abs(fcw.path_offset(gap, -ys[5]+3.5, sign*.001, xs, ys)), 3.)
      self.assertGreater(abs(fcw.path_offset(gap, 0, sign*.001, xs, ys)), 4.)

  def test_no_long_range_path_extrapolation(self):
    self.assertIsNone(fcw.path_offset(100, 0, 0, [0,20], [0,0]))
    self.assertIsNone(fcw.path_offset(100, 0, 0, [0,100], [0,math.nan]))
    self.assertIsNone(fcw.path_offset(100, 0, 0, [100,0], [0,0]))
    self.assertIsNone(fcw.path_offset(100, 0, 0))
    self.assertAlmostEqual(fcw.path_offset(3, 0, 0), 0.)

  def test_low_speed_override_needs_matching_camera_evidence(self):
    lead=NS(modelProb=0.,radar=True,dRel=3.,yRel=0.,vLead=0.)
    vision=NS(prob=.8,x=[4.52],y=[0.],v=[0.])
    model=NS(leadsV3=[vision])
    self.assertEqual(fcw.lead_probability(lead,model,0), .8)
    vision.y=[3.5]
    self.assertEqual(fcw.lead_probability(lead,model,0), 0.)
    vision.y=[0.]; vision.x=[40.]
    self.assertEqual(fcw.lead_probability(lead,model,0), 0.)
    vision.x=[4.52]; vision.v=[10.]
    self.assertEqual(fcw.lead_probability(lead,model,0), 0.)
    self.assertEqual(fcw.lead_probability(lead,NS(),0), 0.)

  def test_registered_params_and_actual_ui_configuration(self):
    header=(SOURCE/'common/params_keys.h').read_text()
    ui=ast.parse((SOURCE/'selfdrive/ui/layouts/settings/toggles.py').read_text())
    for key, (_,_,_,default) in fcw.SETTING_LIMITS.items():
      self.assertIn('"'+key+'", {PERSISTENT | BACKUP, INT, "'+str(default)+'"}', header)
    cls=next(n for n in ui.body if isinstance(n, ast.ClassDef) and n.name=='TogglesLayout')
    init=next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='__init__')
    # Execute the real tuning-widget initialization using a headless widget factory.
    first=next(i for i,n in enumerate(init.body) if 'self._fcw_tuning = {}' == ast.unparse(n))
    last=next(i for i,n in enumerate(init.body) if 'self._toggles = {}' == ast.unparse(n))
    values={k:'bad' for k in fcw.SETTING_LIMITS}
    params=NS(get=lambda k,**kw:values[k],put=lambda k,v,**kw:values.update({k:v}),get_bool=lambda k:True)
    obj=NS(_params=params)
    env=dict(self=obj,SETTING_LIMITS=fcw.SETTING_LIMITS,setting_value=fcw.setting_value,tr=lambda s:s,
             option_item_sp=lambda **kw:NS(**kw))
    exec(compile(ast.Module(body=init.body[first:last],type_ignores=[]),'<actual UI>','exec'),env)
    self.assertEqual(len(obj._fcw_tuning),3)
    for k,w in obj._fcw_tuning.items():
      lo,hi,step,default=fcw.SETTING_LIMITS[k]
      self.assertEqual((w.min_value,w.max_value,w.value_change_step),(lo,hi,step))
      self.assertEqual(values[k],default)
      self.assertTrue(w.enabled())
    self.assertEqual(obj._fcw_tuning['FCWResponseDelay'].label_callback(20),'2.0 s')
    self.assertEqual(obj._fcw_tuning['FCWMinConfidence'].label_callback(75),'75%')


class AllSpeedTests(unittest.TestCase):
  setUp = base.DetectorTests.setUp
  frame = base.DetectorTests.frame
  run_frames = base.DetectorTests.run_frames
  def test_stopped_close_queue_is_silent(self):
    self.assertFalse(any(self.run_frames(400,speed=0,lead_speed=0,gap=1.)))

  def test_creep_from_standstill_with_physically_closing_gap(self):
    # Establish stationary lead, then roll at 0.5 m/s (about 1.1 mph).
    self.run_frames(40,speed=0,lead_speed=0,gap=1.5)
    alerts=[self.frame(speed=.5,lead_speed=0,gap=1.5-i*.025) for i in range(30)]
    self.assertTrue(any(alerts))

  def test_zero_speed_reversing_lead(self):
    self.assertTrue(any(self.run_frames(30,speed=0,lead_speed=-.5,gap=1.)))

  def test_zero_speed_initial_acceleration(self):
    self.assertTrue(any(self.run_frames(30,speed=0,acceleration=.5,lead_speed=0,gap=1.)))

  def test_tiny_closing_noise_is_silent(self):
    self.assertFalse(any(self.run_frames(100,speed=.05,lead_speed=0,gap=1.)))

  def test_creep_with_sufficient_braking_is_silent(self):
    self.assertFalse(any(self.run_frames(50,speed=.5,acceleration=-1,lead_speed=0,gap=1.)))

  def test_confidence_selector_changes_eligibility(self):
    self.assertTrue(any(self.run_frames(probability=.8)))
    self.setUp()
    self.assertFalse(any(self.run_frames(probability=.8,settings=fcw.FCWSettings(confidence=85))))
    self.setUp()
    self.assertTrue(any(self.run_frames(probability=.55,settings=fcw.FCWSettings(confidence=55))))

  def test_second_creep_episode_after_stopped_interval(self):
    self.assertTrue(any(self.run_frames(30,speed=.5,lead_speed=0,gap=1.)))
    self.run_frames(80,speed=0,lead_speed=0,gap=1.)
    self.assertTrue(any(self.run_frames(30,speed=.5,lead_speed=0,gap=1.)))

  def test_curve_gate_uses_path_instead_of_raw_yrel(self):
    self.assertTrue(any(self.run_frames(gap=60,lead_speed=0,lateral=-4,curvature=.002,path_lateral=0)))
    self.setUp()
    self.assertFalse(any(self.run_frames(gap=60,lead_speed=0,lateral=0,curvature=.002,path_lateral=-4)))


class FullServiceTests(unittest.TestCase):
  setUp = base.ServiceTests.setUp
  frame = base.ServiceTests.frame
  def test_standstill_flag_does_not_block_rollback_warning(self):
    self.cs.standstill=True; self.cs.vEgo=0.
    lead=self.sm['radarState'].leadOne
    lead.dRel=1.; lead.vLead=-.5
    self.assertTrue(any([self.frame() for _ in range(30)]))
    self.assertFalse(self.errors)

  def test_radar_override_confirmation_reaches_detector(self):
    self.cs.vEgo=.5
    lead=self.sm['radarState'].leadOne
    lead.modelProb=0.; lead.dRel=1.; lead.vLead=0.
    self.sm['modelV2'].leadsV3=[NS(prob=.8,x=[2.52],y=[0.],v=[0.])]
    self.assertTrue(any([self.frame() for _ in range(30)]))

  def test_live_numeric_settings_apply(self):
    self.sm['radarState'].leadOne.modelProb=.8
    self.values['FCWMinConfidence']=95
    self.assertFalse(any([self.frame() for _ in range(30)]))
    self.values.update(FCWMinConfidence=75,FCWResponseDelay=24,FCWForecast=80)
    self.assertTrue(any([self.frame() for _ in range(50)]))
    self.assertEqual(self.s.settings.response_delay,2.4)
    self.assertEqual(self.s.settings.forecast,8.)

  def test_missing_long_range_path_disables_advisory(self):
    self.sm['modelV2']=NS()
    self.assertFalse(any([self.frame() for _ in range(50)]))
    self.assertFalse(self.errors)


if __name__=='__main__':
  unittest.main()
