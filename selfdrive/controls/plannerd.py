#!/usr/bin/env python3
import time

from cereal import car, custom, log
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.active_ldw import ActiveLaneDepartureWarning, Result, config_from_settings, update_active_ldw
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
import cereal.messaging as messaging


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  cloudlog.info("plannerd is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("plannerd got CarParamsSP")

  gps_location_service = get_gps_location_service(params)
  ignore_services = ["liveMapDataSP", gps_location_service]

  ldw = LaneDepartureWarning()
  active_ldw = ActiveLaneDepartureWarning()
  active_ldw_enabled = False
  active_ldw_faulted = False
  next_ldw_param_read = 0.0
  longitudinal_planner = LongitudinalPlanner(CP, CP_SP)
  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'longitudinalPlanSP'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState',
                            'liveMapDataSP', 'carStateSP', 'selfdriveStateSP', 'liveCalibration', gps_location_service],
                           poll='modelV2', ignore_alive=ignore_services, ignore_avg_freq=ignore_services, ignore_valid=ignore_services)

  while True:
    sm.update()
    longitudinal_planner.sla.update_buttons(sm['selfdriveStateSP'].buttonsReleaseToggle)
    if sm.updated['modelV2']:
      longitudinal_planner.update(sm)
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      active_departure = Result()
      if not active_ldw_faulted:
        try:
          now = time.monotonic()
          if now >= next_ldw_param_read:
            active_ldw_enabled = params.get_bool("LdwWhileSteering") and params.get_bool("IsLdwEnabled")
            active_ldw.configure(config_from_settings(params.get_bool("LdwLowSpeed"),
                                                      params.get("LdwSensitivity", return_default=True)))
            next_ldw_param_read = now + 1.0
          if active_ldw_enabled:
            # Do not silently mix MONOTONIC with BOOTTIME on unsupported hosts.
            if hasattr(time, "CLOCK_BOOTTIME"):
              active_departure = update_active_ldw(active_ldw, sm, now, True,
                                                   log.LiveCalibrationData.Status.calibrated,
                                                   camera_now=time.clock_gettime(time.CLOCK_BOOTTIME))
          else:
            active_ldw.reset()
        except Exception:
          # An advisory fault must not terminate longitudinal planning. Log once
          # and disable the added detector until this process restarts.
          active_ldw_faulted = True
          cloudlog.exception("Active LDW disabled until restart after exception")
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks(['carState', 'carControl', 'modelV2', 'liveParameters'])
      # Use one detector at a time: stock manual-driving LDW must not bypass
      # predictive suppression or retrigger an already-latched departure.
      use_predictive_ldw = active_ldw_enabled and not active_ldw_faulted and hasattr(time, "CLOCK_BOOTTIME")
      msg.driverAssistance.leftLaneDeparture = active_departure.left if use_predictive_ldw else ldw.left
      msg.driverAssistance.rightLaneDeparture = active_departure.right if use_predictive_ldw else ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
