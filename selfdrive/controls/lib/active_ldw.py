"""Experimental advisory LDW during lateral control; no actuator commands.

Coordinates follow modelV2/controlsState: x forward, y and curvature right-positive.
The envelope is a configurable body-width proxy, not calibrated tire tracking.
All distances are metres, speeds m/s, times monotonic seconds, curvature 1/metre.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Config:
  min_speed: float = 31.0 * 0.44704
  half_width: float = 1.0  # Approximate Odyssey body half-width, excluding mirrors. NOT measured.
  camera_offset: float = 0.04  # Preserve this release's LDW coordinate convention.
  horizon: float = 0.8
  step: float = 0.05
  margin: float = 0.10
  min_approach: float = 0.05
  confirm_time: float = 0.15
  signal_cooldown: float = 5.0
  pulse_time: float = 0.30  # Existing event manager supplies the 3-second audible/display alert.
  rearm_clearance: float = 0.35
  rearm_time: float = 0.5
  max_age: float = 0.25
  max_gap: float = 0.20
  latch_hold_time: float = 2.0
  min_lane_probability: float = 0.65
  retain_lane_probability: float = 0.50
  max_lane_std: float = 0.30
  max_edge_std: float = 0.20
  road_edge_fallback: bool = False

  def __post_init__(self):
    numeric = [v for v in self.__dict__.values() if type(v) is not bool]
    if not all(math.isfinite(v) for v in numeric):
      raise ValueError("configuration must be finite")
    if not (0.7 <= self.half_width <= 1.3 and 0 < self.step <= self.horizon <= 1.5):
      raise ValueError("invalid envelope or prediction horizon")
    if not (0 < self.confirm_time <= 0.5 and self.margin < self.rearm_clearance):
      raise ValueError("invalid confirmation or rearm thresholds")


# Later / Standard / Earlier. Confirmation and confidence gates stay unchanged.
SENSITIVITY_PROFILES = ((0.5, 0.05), (0.8, 0.10), (1.1, 0.20))


def sensitivity_index(value):
  # Params returns an int; reject malformed/out-of-range values to Standard.
  return value if type(value) is int and 0 <= value < len(SENSITIVITY_PROFILES) else 1


def config_from_settings(low_speed=False, sensitivity=1):
  horizon, margin = SENSITIVITY_PROFILES[sensitivity_index(sensitivity)]
  return Config(min_speed=(2.0 if low_speed else 31.0) * 0.44704,
                horizon=horizon, margin=margin)


@dataclass(frozen=True)
class Boundary:
  x: tuple[float, ...]
  y: tuple[float, ...]
  probability: float
  std: float
  kind: str = "line"


@dataclass(frozen=True)
class Result:
  left: bool = False
  right: bool = False
  left_source: str = "unavailable"
  right_source: str = "unavailable"
  reason: str = "inactive"


@dataclass
class _Side:
  candidate_since: float | None = None
  fired_at: float | None = None
  clear_since: float | None = None
  latched: bool = False
  last_near: float | None = None
  source: str | None = None
  last_valid: float | None = None


class ActiveLaneDepartureWarning:
  def __init__(self, config: Config | None = None):
    self.config = config or Config()
    self.reset()

  def configure(self, config):
    if config == self.config:
      return
    self.config = config
    # Preserve fired latches, signal cooldowns and observation history when tuning.
    # Restart partial confirmation/rearm timers; never replay an interrupted pulse.
    for state in self.sides:
      state.candidate_since = state.clear_since = state.fired_at = None
    self.result = Result(reason="settings changed")

  def reset(self):
    self.sides = [_Side(), _Side()]
    self.last_signal = [-math.inf, -math.inf]
    self.last_frame_time = None
    self.last_camera_timestamp = None
    self.last_now = None
    self.result = Result()

  def _pause_side(self, i):
    state = self.sides[i]
    if (state.last_valid is None or self.last_now is None or
        self.last_now - state.last_valid > self.config.latch_hold_time):
      state = self.sides[i] = _Side()
    # Cancel confirmation, clearance timer and pulse, but retain a recent latch.
    state.candidate_since = state.clear_since = state.fired_at = None
    return state

  def _invalid(self, reason):
    for i in range(2):
      self._pause_side(i)
    self.result = Result(reason=reason)
    return self.result

  def _usable(self, b, retain_line=False):
    c = self.config
    if b is None or b.kind not in ("line", "edge"):
      return False
    if len(b.x) != len(b.y) or not 3 <= len(b.x) <= 256:
      return False
    if not all(math.isfinite(v) for v in (*b.x, *b.y, b.probability, b.std)):
      return False
    if b.x[0] > 0 or b.x[0] < -5 or any(v <= u for u, v in zip(b.x, b.x[1:])):
      return False
    if not 0 <= b.probability <= 1 or b.std < 0:
      return False
    if b.kind == "line":
      threshold = c.retain_lane_probability if retain_line else c.min_lane_probability
      return b.probability >= threshold and b.std <= c.max_lane_std
    return c.road_edge_fallback and b.std <= c.max_edge_std

  @staticmethod
  def _interpolate(b, x):
    if not b.x[0] <= x <= b.x[-1]:
      return None  # Never extrapolate beyond model coverage.
    j = max(0, min(bisect_right(b.x, x) - 1, len(b.x) - 2))
    slope = (b.y[j + 1] - b.y[j]) / (b.x[j + 1] - b.x[j])
    if abs(slope) > 1.0:
      return None
    return b.y[j] + slope * (x - b.x[j]), slope

  def _clearances(self, boundary, side, speed, curvature, age):
    c = self.config
    gaps = []
    for i in range(math.ceil(c.horizon / c.step) + 1):
      t = age + min(i * c.step, c.horizon)
      distance = speed * t
      heading = curvature * distance
      if abs(curvature) < 1e-7:
        x, y = distance, 0.0
      else:
        x = math.sin(heading) / curvature
        y = 2.0 * math.sin(heading / 2.0)**2 / curvature
      # Swept lateral envelope at the camera-referenced cross-section. No axle claim.
      x_edge = x - side * c.half_width * math.sin(heading)
      y_edge = y + side * c.half_width * math.cos(heading) - c.camera_offset
      point = self._interpolate(boundary, x_edge)
      if point is None:
        return None
      lane_y, slope = point
      gaps.append(side * (lane_y - y_edge) / math.sqrt(1.0 + slope*slope))
    return gaps[0], min(gaps)

  def update(self, *, now, frame_time, speed, curvature, lines, edges=(None, None),
             left_signal=False, right_signal=False, lane_change_active=False,
             enabled=False, lateral_active=False, valid=True):
    c = self.config
    if not all(math.isfinite(v) for v in (now, frame_time, speed, curvature)):
      return self._invalid("nonfinite input")
    if self.last_now is not None and now <= self.last_now:
      self.reset()
      self.last_now = now
      return self._invalid("clock discontinuity")
    gap = self.last_now is not None and now - self.last_now > c.max_gap
    self.last_now = now
    if gap:
      self._invalid("frame gap")
    if left_signal or right_signal or lane_change_active:
      self.last_signal = [now, now]
    if not enabled or not lateral_active:
      return self._invalid("inactive")
    age = now - frame_time
    if not valid or not -0.005 <= age <= c.max_age:
      return self._invalid("invalid or stale input")
    age = max(age, 0.0)  # Permit sub-millisecond timestamp conversion/clock skew.
    if self.last_frame_time is not None and frame_time <= self.last_frame_time:
      return self._invalid("duplicate or reversed camera frame")
    self.last_frame_time = frame_time
    if speed <= c.min_speed:
      return self._invalid("below speed threshold")
    if speed > 70 or abs(curvature) > 0.03:
      return self._invalid("outside prediction envelope")
    if len(lines) != 2 or len(edges) != 2:
      return self._invalid("malformed boundary set")

    warnings, sources = [False, False], ["unavailable", "unavailable"]
    for i, side in enumerate((-1, 1)):
      if now - self.last_signal[i] < c.signal_cooldown:
        self._pause_side(i)
        sources[i] = "signal suppressed"
        continue
      state = self.sides[i]
      if state.last_valid is not None and now - state.last_valid > c.latch_hold_time:
        state = self.sides[i] = _Side()
      candidates = [b for b in (lines[i], edges[i]) if self._usable(b, state.source == "line")]
      # A credible lane/shoulder stripe takes precedence over the outer road edge.
      if not candidates:
        self._pause_side(i)
        continue
      boundary = candidates[0]
      near = self._interpolate(boundary, 0.0)
      if near is None or not -0.25 <= side * (near[0] + c.camera_offset) <= 6.0:
        self._pause_side(i)
        continue
      clearance = self._clearances(boundary, side, speed, curvature, age)
      if clearance is None:
        self._pause_side(i)
        continue
      current, minimum = clearance
      sources[i] = boundary.kind
      state = self.sides[i]
      if state.source != boundary.kind or (state.last_near is not None and abs(near[0] - state.last_near) > 0.7):
        state = self._pause_side(i)
      state.last_valid = now
      state.last_near, state.source = near[0], boundary.kind

      risk = current <= 0.0 or (minimum <= c.margin and current - minimum >= c.min_approach)
      if state.latched:
        if minimum >= c.rearm_clearance:
          if state.clear_since is None:
            state.clear_since = now
          if now - state.clear_since >= c.rearm_time:
            state.latched, state.fired_at, state.candidate_since = False, None, None
        else:
          state.clear_since = None
        warnings[i] = state.fired_at is not None and now - state.fired_at < c.pulse_time
      elif risk:
        if state.candidate_since is None:
          state.candidate_since = now
        if now - state.candidate_since >= c.confirm_time:
          state.latched, state.fired_at, state.clear_since = True, now, None
          warnings[i] = True
      else:
        state.candidate_since = None
    self.result = Result(*warnings, *sources, reason="monitoring")
    return self.result


def boundary_from_model(model, index, edge=False):
  """Convert a native modelV2 boundary; incomplete model outputs become unavailable."""
  try:
    points = model.roadEdges[index] if edge else model.laneLines[index]
    std = model.roadEdgeStds[index] if edge else model.laneLineStds[index]
    probability = 1.0 if edge else model.laneLineProbs[index]
    return Boundary(tuple(points.x), tuple(points.y), float(probability), float(std), "edge" if edge else "line")
  except (AttributeError, IndexError, TypeError, ValueError):
    return None


def update_active_ldw(detector, sm, now, enabled, calibrated_status, camera_now=None):
  """Adapter for this release's SubMaster. Call only for a new modelV2 message.

  Invalid/missing inputs suppress this additional warning; stock LDW is separate.
  Calibration and controlsState are required; desiredCurvature is never used.
  """
  model, cs, cc = sm['modelV2'], sm['carState'], sm['carControl']
  # Tici camera timestamps use CLOCK_BOOTTIME; Python messages use MONOTONIC.
  # These diverge after suspend. Translate capture age instead of mixing clocks.
  camera_now = now if camera_now is None else camera_now
  raw_frame_time = model.timestampEof * 1e-9
  fresh_frame = detector.last_camera_timestamp is None or raw_frame_time > detector.last_camera_timestamp
  detector.last_camera_timestamp = raw_frame_time
  frame_time = now - (camera_now - raw_frame_time)
  required = ['modelV2', 'carState', 'carControl', 'controlsState', 'liveParameters', 'liveCalibration']
  valid = sm.all_checks(required) and fresh_frame
  valid = valid and sm['liveParameters'].valid and sm['liveCalibration'].calStatus == calibrated_status
  valid = valid and all(-0.005 <= now - sm.logMonoTime[name] * 1e-9 <= detector.config.max_age
                        for name in ('modelV2', 'carState', 'carControl', 'controlsState'))
  return detector.update(now=now, frame_time=frame_time,
                         speed=cs.vEgo, curvature=sm['controlsState'].curvature,
                         lines=(boundary_from_model(model, 1), boundary_from_model(model, 2)),
                         edges=(boundary_from_model(model, 0, True), boundary_from_model(model, 1, True)),
                         left_signal=cs.leftBlinker, right_signal=cs.rightBlinker,
                         lane_change_active=model.meta.laneChangeState != 0,
                         enabled=enabled, lateral_active=cc.latActive, valid=valid)
