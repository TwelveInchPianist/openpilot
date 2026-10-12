"""Experimental warning-only FCW advisory. No actuator or stock FCW changes.

Deliberately restricted to stable primary leads on approximately straight paths.
Thresholds are engineering starting points, not validated warning performance.
"""
import math


def minimum_gap(gap, ego_speed, ego_accel, lead_speed, lead_accel, comfortable_response):
  """Four-second constant-acceleration forecast, with speeds clamped at zero.

The hypothetical response coasts/currently brakes for 1.2 s, then brakes at
2.5 m/s^2 or keeps stronger existing braking. Lead acceleration is held constant.
This intentionally does not claim calibrated human response or friction modeling.
  """
  closest = gap
  for step in range(40):
    a = min(ego_accel, -2.5) if comfortable_response and step >= 12 else ego_accel
    next_ego = max(0., ego_speed + a * .1)
    next_lead = max(0., lead_speed + lead_accel * .1)
    gap += ((lead_speed + next_lead) - (ego_speed + next_ego)) * .05
    closest = min(closest, gap)
    ego_speed, lead_speed = next_ego, next_lead
  return closest


class EarlyForwardWarning:
  def __init__(self):
    self.last_time = None
    self.last_stamp = None
    self.previous = None
    self.stable_since = None
    self.risk_since = None
    self.clear_since = None
    self.latched = False
    self.last_fire = -math.inf
    self.pulse_until = -math.inf

  def pause(self):
    # Invalid inputs and mode changes must not rearm an ongoing episode.
    self.previous = None
    self.stable_since = self.risk_since = self.clear_since = None
    self.pulse_until = -math.inf
    return False

  def update(self, now, stamp, *, valid, urgent, speed, acceleration, gap,
             lead_speed, lead_accel, probability, lateral, curvature, track):
    values = (now, stamp, speed, acceleration, gap, lead_speed, lead_accel,
              probability, lateral, curvature)
    if not all(math.isfinite(v) for v in values):
      return self.pause()
    if self.last_time is not None and now < self.last_time:
      self.last_stamp = None
      self.last_fire = now  # cooldown in the new clock domain
      self.last_time = now
      return self.pause()
    self.last_time = now
    if urgent:
      self.latched = True
      self.last_fire = now
      return self.pause()
    if not valid or not (0 <= now - stamp <= .25):
      return self.pause()
    # No replayed/reversed radar samples may advance confirmation.
    if self.last_stamp is not None and stamp <= self.last_stamp:
      if stamp < self.last_stamp:
        return self.pause()
      return now < self.pulse_until
    dt = None if self.last_stamp is None else stamp - self.last_stamp
    self.last_stamp = stamp
    if not (5 * .44704 < speed < 60 and -8 <= acceleration <= 4 and
            3 < gap < 150 and 0 <= lead_speed < 70 and -6 <= lead_accel <= 3 and
            .95 <= probability <= 1 and abs(lateral) <= .9 and
            abs(curvature) * gap * gap / 2 <= .5):
      return self.pause()

    previous = self.previous
    continuous = (previous is not None and dt is not None and 0 < dt <= .2 and
                  track == previous[0] and abs(lead_speed - previous[2]) <= 2 and
                  abs(gap - (previous[1] + (previous[2] - previous[3]) * dt)) <= 2)
    if not continuous:
      self.pause()
      self.stable_since = stamp
    self.previous = (track, gap, lead_speed, speed)

    closing = speed - lead_speed
    actual_gap = minimum_gap(gap, speed, acceleration, lead_speed, lead_accel, False)
    response_gap = minimum_gap(gap, speed, acceleration, lead_speed, lead_accel, True)
    risk = (closing > 2 or lead_accel < -1) and actual_gap < 3 and response_gap < 3
    if not risk:
      self.risk_since = None
      self.pulse_until = -math.inf
      # A real, continuously observed safe trajectory rearms; loss of lead does not.
      if actual_gap >= 6:
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
        stamp - self.risk_since >= .3 and now - self.last_fire >= 10):
      self.latched = True
      self.last_fire = now
      self.pulse_until = now + .2
    return now < self.pulse_until


class EarlyFCWService:
  """Read-only adapter; runtime faults disable only this advisory until restart."""
  def __init__(self):
    self.detector = EarlyForwardWarning()
    self.enabled = False
    self.faulted = False
    self.next_read = 0.

  def update(self, sm, cs, car_time, params, now, urgent, drive_gear, calibrated, logger):
    if self.faulted:
      return False
    try:
      if now >= self.next_read:
        self.enabled = params.get_bool("AggressiveFCW")
        self.next_read = now + 1.
      if not self.enabled:
        return self.detector.pause()
      services = ['radarState', 'modelV2', 'controlsState', 'liveCalibration']
      valid = (sm.all_checks(services) and cs.canValid and not cs.standstill and
               cs.gearShifter == drive_gear and 0 <= now - car_time <= .25 and
               sm['liveCalibration'].calStatus == calibrated and
               all(0 <= now - sm.logMonoTime[s] * 1e-9 <= .25 for s in services[:3]) and
               not any(sm['radarState'].radarErrors.to_dict().values()))
      lead = sm['radarState'].leadOne
      valid = valid and lead.status
      # Primary lead selection comes from the existing radar/vision association.
      # Radar track ID plus geometry continuity is used; vision-only IDs are -1.
      return self.detector.update(
        now, sm.logMonoTime['radarState'] * 1e-9, valid=valid, urgent=urgent,
        speed=cs.vEgo, acceleration=cs.aEgo, gap=lead.dRel,
        lead_speed=lead.vLead, lead_accel=lead.aLeadK, probability=lead.modelProb,
        lateral=lead.yRel, curvature=sm['controlsState'].curvature,
        track=(lead.radar, lead.radarTrackId))
    except Exception:
      self.faulted = True
      logger.exception("Early FCW advisory disabled until restart after exception")
      return False
