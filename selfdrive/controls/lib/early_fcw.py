"""Experimental, warning-only FCW. No actuator or stock urgent FCW changes.

Settings and geometric gates are engineering starting points, not validated
false-positive rates or guaranteed detection/response times.
"""
import math


# Integer storage avoids rounding drift in the on-device +/- controls.
SETTING_LIMITS = {
  "FCWMinConfidence": (55, 95, 5, 75),
  "FCWResponseDelay": (4, 40, 2, 20),  # tenths of a second
  "FCWForecast": (20, 80, 5, 50),     # tenths of a second
}
RADAR_TO_CAMERA = 1.52  # matches radard's lead coordinate conversion


def setting_value(key, value):
  low, high, step, default = SETTING_LIMITS[key]
  try:
    value = float(value)
    if not math.isfinite(value):
      return default
    return max(low, min(high, low + int((value - low) / step + .5) * step))
  except (TypeError, ValueError, OverflowError):
    return default


class FCWSettings:
  def __init__(self, confidence=75, response_delay=20, forecast=50):
    self.probability = setting_value("FCWMinConfidence", confidence) / 100.
    self.response_delay = setting_value("FCWResponseDelay", response_delay) / 10.
    self.forecast = setting_value("FCWForecast", forecast) / 10.


def minimum_gap(gap, ego_speed, ego_accel, lead_speed, lead_accel,
                comfortable_response, response_delay=2., horizon=5.):
  """Integrate signed relative motion; don't invent indefinite acceleration.

Observed acceleration is extrapolated for at most two seconds. Thereafter
vehicles coast, except hypothetical response braking continues to a stop.
A forward/stationary lead is never extrapolated into reverse. An observed
reversing lead may continue reversing. No claim of friction/driver calibration.
"""
  closest = gap
  reversing = lead_speed < 0
  for step in range(round(horizon * 10)):
    t = step * .1
    a = ego_accel if t < 2. else 0.
    if comfortable_response and t + 1e-9 >= response_delay:
      a = min(a, -2.5)
    next_ego = max(0., ego_speed + a * .1)
    next_lead = lead_speed + (lead_accel if t < 2. else 0.) * .1
    next_lead = min(0., next_lead) if reversing else max(0., next_lead)
    gap += ((lead_speed + next_lead) - (ego_speed + next_ego)) * .05
    closest = min(closest, gap)
    ego_speed, lead_speed = next_ego, next_lead
  return closest


def path_offset(gap, lateral, curvature, path_x=(), path_y=()):
  """Signed perpendicular lead offset from the model path, in meters.

Model x/y: camera reference, right positive. Radar lead: front reference,
left positive. Never extrapolate the model path. For close creep encounters
only, fall back to a constant-curvature arc if the stationary model path
does not extend to the lead. Long-range missing paths disable this advisory.
"""
  if not all(math.isfinite(v) for v in (gap, lateral, curvature)):
    return None
  x = gap + RADAR_TO_CAMERA
  if len(path_x) == len(path_y) and 2 <= len(path_x) <= 100:
    if all(math.isfinite(v) for v in (*path_x, *path_y)) and all(b > a for a, b in zip(path_x, path_x[1:], strict=False)):
      for i in range(1, len(path_x)):
        if path_x[i-1] <= x <= path_x[i]:
          dx = path_x[i] - path_x[i-1]
          dy = path_y[i] - path_y[i-1]
          # Reject degenerate/sideways path segments, not ordinary road curves.
          if abs(dy / dx) > 1.:
            return None
          y = path_y[i-1] + (x - path_x[i-1]) * dy / dx
          return (-lateral - y) / math.sqrt(1. + (dy / dx)**2)
  if 0 < gap <= 15 and abs(curvature) <= .1 and abs(curvature * gap) < .8:
    # Circle through the vehicle origin; k=0 is the straight-line limit.
    y = curvature * gap * gap / (1 + math.sqrt(1 - (curvature * gap)**2))
    slope = curvature * gap / math.sqrt(1 - (curvature * gap)**2)
    return (-lateral - y) / math.sqrt(1 + slope * slope)
  return None


def lead_probability(lead, model, speed):
  """Recover vision support lost by radard's low-speed radar override.

That override sets modelProb=0 even for a camera-confirmed object. Require a
matching camera lead; this does NOT enable unconfirmed radar-only warnings.
"""
  probability = lead.modelProb
  if probability == 0 and lead.radar and speed < 4.:
    for vision in getattr(model, "leadsV3", ()):
      if not (len(vision.x) and len(vision.y) and len(vision.v)):
        continue
      values = (vision.prob, vision.x[0], vision.y[0], vision.v[0])
      if not all(math.isfinite(v) for v in values):
        continue
      if (abs(vision.x[0] - RADAR_TO_CAMERA - lead.dRel) <= max(1., .15 * lead.dRel) and
          abs(-vision.y[0] - lead.yRel) <= .75 and
          abs(vision.v[0] - lead.vLead) <= 2.):
        probability = max(probability, vision.prob)
  return probability


class EarlyForwardWarning:
  def __init__(self):
    self.last_time = None
    self.last_stamp = None
    self.last_eligible = None
    self.previous = None
    self.stable_since = None
    self.risk_since = None
    self.clear_since = None
    self.latched = False
    self.last_fire = -math.inf
    self.pulse_until = -math.inf

  def pause(self):
    self.previous = None
    self.stable_since = self.risk_since = self.clear_since = None
    self.pulse_until = -math.inf
    return False

  def update(self, now, stamp, *, valid, urgent, speed, acceleration, gap,
             lead_speed, lead_accel, probability, lateral, curvature, track,
             path_lateral=None, settings=None):
    settings = settings or FCWSettings()
    values = (now, stamp, speed, acceleration, gap, lead_speed, lead_accel,
              probability, lateral, curvature)
    if not all(math.isfinite(v) for v in values):
      return self.pause()
    if self.last_time is not None and now < self.last_time:
      self.last_stamp = self.last_eligible = None
      self.last_fire = now
      self.last_time = now
      return self.pause()
    self.last_time = now
    if urgent:
      self.latched = True
      self.last_fire = now
      return self.pause()
    if not valid or not (0 <= now - stamp <= .25):
      return self.pause()
    if self.last_stamp is not None and stamp <= self.last_stamp:
      if stamp < self.last_stamp:
        return self.pause()
      return now < self.pulse_until
    dt = None if self.last_stamp is None else stamp - self.last_stamp
    self.last_stamp = stamp
    if path_lateral is None:
      return self.pause()
    if not (0 <= speed < 60 and -8 <= acceleration <= 4 and
            .3 < gap < 200 and -5 <= lead_speed < 70 and -8 <= lead_accel <= 4 and
            settings.probability <= probability <= 1 and
            math.isfinite(path_lateral) and abs(path_lateral) <= 1.):
      return self.pause()

    if self.last_eligible is not None and stamp - self.last_eligible >= 3. and now - self.last_fire >= 10:
      self.latched = False
    self.last_eligible = stamp
    previous = self.previous
    continuous = (previous is not None and dt is not None and 0 < dt <= .2 and
                  track == previous[0] and abs(lead_speed - previous[2]) <= 2 and
                  abs(gap - (previous[1] + (previous[2] - previous[3]) * dt)) <= 2)
    if not continuous:
      self.pause()
      self.stable_since = stamp
    self.previous = (track, gap, lead_speed, speed)

    closing = speed - lead_speed
    low_speed = speed < 5 * .44704
    # Near a stopped lead, a 3 m buffer would beep during ordinary close queues.
    clearance = .6 if low_speed else 3.
    actual_gap = minimum_gap(gap, speed, acceleration, lead_speed, lead_accel, False,
                             settings.response_delay, settings.forecast)
    response_gap = minimum_gap(gap, speed, acceleration, lead_speed, lead_accel, True,
                               settings.response_delay, settings.forecast)
    moving_risk = closing > (.15 if low_speed else 2.) or lead_accel < -1 or (low_speed and acceleration > .2)
    risk = moving_risk and actual_gap < clearance and response_gap < clearance
    if not risk:
      self.risk_since = None
      self.pulse_until = -math.inf
      safe = actual_gap >= (1.2 if low_speed else 6.)
      if low_speed and closing <= .05 and acceleration <= .1 and lead_accel >= -.1:
        safe = True  # stopped/holding gap rearms the next creep episode
      if safe:
        if self.clear_since is None:
          self.clear_since = stamp
        if stamp - self.clear_since >= 2:
          self.latched = False
      else:
        self.clear_since = None
      return False

    self.clear_since = None
    if self.risk_since is None:
      self.risk_since = stamp
    if (not self.latched and stamp - self.stable_since >= .6 and
        stamp - self.risk_since >= .3 and now - self.last_fire >= (3 if low_speed else 10)):
      self.latched = True
      self.last_fire = now
      self.pulse_until = now + .2
    return now < self.pulse_until


class EarlyFCWService:
  """Read-only adapter; faults disable only the advisory until restart."""
  def __init__(self):
    self.detector = EarlyForwardWarning()
    self.enabled = False
    self.faulted = False
    self.next_read = 0.
    self.settings = FCWSettings()

  def update(self, sm, cs, car_time, params, now, urgent, drive_gear, calibrated, logger):
    if self.faulted:
      return False
    try:
      if now >= self.next_read:
        enabled = params.get_bool("AggressiveFCW")
        values = [setting_value(k, params.get(k, return_default=True)) for k in SETTING_LIMITS]
        settings = FCWSettings(*values)
        if enabled != self.enabled or vars(settings) != vars(self.settings):
          self.detector.pause()  # new settings must earn fresh confirmation
        self.enabled, self.settings = enabled, settings
        self.next_read = now + 1.
      if not self.enabled:
        return self.detector.pause()
      services = ['radarState', 'modelV2', 'controlsState', 'liveCalibration']
      valid = (sm.all_checks(services) and cs.canValid and
               cs.gearShifter == drive_gear and 0 <= now - car_time <= .25 and
               sm['liveCalibration'].calStatus == calibrated and
               all(0 <= now - sm.logMonoTime[s] * 1e-9 <= .25 for s in services[:3]) and
               not any(sm['radarState'].radarErrors.to_dict().values()))
      lead = sm['radarState'].leadOne
      model = sm['modelV2']
      position = getattr(model, 'position', None)
      curvature = sm['controlsState'].curvature
      offset = path_offset(lead.dRel, lead.yRel, curvature,
                           tuple(position.x) if position is not None else (),
                           tuple(position.y) if position is not None else ())
      return self.detector.update(
        now, sm.logMonoTime['radarState'] * 1e-9, valid=valid and lead.status, urgent=urgent,
        speed=cs.vEgo, acceleration=cs.aEgo, gap=lead.dRel,
        lead_speed=lead.vLead, lead_accel=lead.aLeadK, probability=lead_probability(lead, model, cs.vEgo),
        lateral=lead.yRel, curvature=curvature, path_lateral=offset,
        track=(lead.radar, lead.radarTrackId), settings=self.settings)
    except Exception:
      self.faulted = True
      logger.exception("Early FCW advisory disabled until restart after exception")
      return False
