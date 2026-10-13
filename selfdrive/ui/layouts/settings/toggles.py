from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.controls.lib.active_ldw import sensitivity_index
from openpilot.selfdrive.controls.lib.early_fcw import SETTING_LIMITS, setting_value
from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state

if gui_app.sunnypilot_ui():
  from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp as toggle_item
  from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp as multiple_button_item

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# Description constants
DESCRIPTIONS = {
  "OpenpilotEnabledToggle": tr_noop(
    "Use the sunnypilot system for adaptive cruise control and lane keep driver assistance. " +
    "Your attention is required at all times to use this feature."
  ),
  "DisengageOnAccelerator": tr_noop("When enabled, pressing the accelerator pedal will disengage sunnypilot."),
  "LongitudinalPersonality": tr_noop(
    "Standard is recommended. In aggressive mode, sunnypilot will follow lead cars closer and be more aggressive with the gas and brake. " +
    "In relaxed mode sunnypilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with " +
    "your steering wheel distance button."
  ),
  "IsLdwEnabled": tr_noop(
    "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line " +
    "without a turn signal activated. Stock warnings operate above 31 mph (50 km/h); enable Predictive Lane Warnings and Low-Speed Lane Warnings below for coverage above 2 mph."
  ),
  "LdwWhileSteering": tr_noop(
    "Experimental audible departure warnings with steering assistance on or off. Normally above 31 mph; the low-speed option lowers this to 2 mph. " +
    "Requires Enable Lane Departure Warnings. Predicts approaches to detected lane or shoulder lines; " +
    "Unmarked road-edge fallback is disabled. Either signal or an active lane change suppresses both sides, plus 5 seconds. " +
    "Unvalidated timing and approximate Odyssey width: warnings can be missed or false. " +
    "Does not change steering. Low-confidence or stale detections cannot provide warnings."
  ),
  "LdwLowSpeed": tr_noop(
    "Enable predictive warnings above 2 mph (3.2 km/h), whether steering assistance is on or off. " +
    "Requires both lane-warning toggles above. No warnings at standstill. " +
    "Parking areas and intersections can cause false alerts; tight turns may be outside the prediction range."
  ),
  "AlwaysOnDM": tr_noop("Enable driver monitoring even when sunnypilot is not engaged."),
  'RecordFront': tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
  "IsMetric": tr_noop("Display speed in km/h instead of mph."),
  "RecordAudio": tr_noop("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
}


class TogglesLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = False  # self._params.get_bool("IsReleaseBranch")

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      "OpenpilotEnabledToggle": (
        lambda: tr("Enable sunnypilot"),
        DESCRIPTIONS["OpenpilotEnabledToggle"],
        "chffr_wheel.png",
        True,
      ),
      "ExperimentalMode": (
        lambda: tr("Experimental Mode"),
        "",
        "experimental_white.png",
        False,
      ),
      "DisengageOnAccelerator": (
        lambda: tr("Disengage on Accelerator Pedal"),
        DESCRIPTIONS["DisengageOnAccelerator"],
        "disengage_on_accelerator.png",
        False,
      ),
      "IsLdwEnabled": (
        lambda: tr("Enable Lane Departure Warnings"),
        DESCRIPTIONS["IsLdwEnabled"],
        "warning.png",
        False,
      ),
      "LdwWhileSteering": (
        lambda: tr("Predictive Lane Warnings (Experimental)"),
        DESCRIPTIONS["LdwWhileSteering"],
        "warning.png",
        False,
      ),
      "LdwLowSpeed": (
        lambda: tr("Low-Speed Lane Warnings (Experimental)"),
        DESCRIPTIONS["LdwLowSpeed"],
        "warning.png",
        False,
      ),
      "AlwaysOnDM": (
        lambda: tr("Always-On Driver Monitoring"),
        DESCRIPTIONS["AlwaysOnDM"],
        "monitoring.png",
        False,
      ),
      "RecordFront": (
        lambda: tr("Record and Upload Driver Camera"),
        DESCRIPTIONS["RecordFront"],
        "monitoring.png",
        True,
      ),
      "RecordAudio": (
        lambda: tr("Record and Upload Microphone Audio"),
        DESCRIPTIONS["RecordAudio"],
        "microphone.png",
        True,
      ),
      "IsMetric": (
        lambda: tr("Use Metric System"),
        DESCRIPTIONS["IsMetric"],
        "metric.png",
        False,
      ),
    }

    self._long_personality_setting = multiple_button_item(
      lambda: tr("Driving Personality"),
      lambda: tr(DESCRIPTIONS["LongitudinalPersonality"]),
      buttons=[lambda: tr("Aggressive"), lambda: tr("Standard"), lambda: tr("Relaxed")],
      button_width=300,
      callback=self._set_longitudinal_personality,
      selected_index=self._params.get("LongitudinalPersonality", return_default=True),
      icon="speed_limit.png"
    )

    self._ldw_sensitivity_setting = multiple_button_item(
      lambda: tr("Lane Warning Sensitivity"),
      lambda: tr("Applies to predictive warnings with steering assistance on or off. Later: 0.5 s look-ahead / 5 cm margin. " +
                 "Standard: 0.8 s / 10 cm. Earlier: 1.1 s / 20 cm; may produce more false alerts. " +
                 "These are prediction settings, not guaranteed warning lead times. " +
                 "Signal suppression and confidence checks are unchanged. Applies within a second; no reboot."),
      buttons=[lambda: tr("Later"), lambda: tr("Standard"), lambda: tr("Earlier")],
      selected_index=sensitivity_index(self._params.get("LdwSensitivity", return_default=True)),
      button_width=300,
      callback=self._set_ldw_sensitivity,
      icon="warning.png",
    )

    self._fcw_sensitivity_setting = multiple_button_item(
      lambda: tr("Forward Collision Warning"),
      lambda: tr("Default preserves existing FCW. Aggressive adds adjustable warnings in Drive, including stopped and creeping traffic, with assistance on or off. " +
                 "A stopped car with no closing motion does not continuously alarm. Requires a supported detected lead; this is not a parking sensor. " +
                 "No braking changes. May miss threats or give false alerts; warning timing is unvalidated. " +
                 "Existing urgent FCW remains enabled in both modes. Applies within a second; no reboot."),
      buttons=[lambda: tr("Default"), lambda: tr("Aggressive")],
      selected_index=int(self._params.get_bool("AggressiveFCW")),
      button_width=300,
      callback=self._set_fcw_sensitivity,
      icon="warning.png",
    )

    self._fcw_tuning = {}
    tuning_labels = {
      "FCWMinConfidence": ("FCW Lead Confidence", "Lower admits less-certain leads and may increase false alerts. "
                           "55-95 in 5-point steps; default 75. This is a model score, not detection reliability."),
      "FCWResponseDelay": ("FCW Response Delay", "Assumed delay before braking. Higher can warn earlier. "
                           "0.4-4.0 seconds in 0.2-second steps; default 2.0 seconds."),
      "FCWForecast": ("FCW Forecast", "Prediction horizon, not guaranteed warning time. "
                      "2-8 seconds in 0.5-second steps; default 5 seconds. Longer forecasts may increase false alerts."),
    }
    for key, (low, high, step, default) in SETTING_LIMITS.items():
      # Repair malformed external values before the numeric widget reads them.
      value = setting_value(key, self._params.get(key, return_default=True))
      self._params.put(key, value, block=True)
      title, description = tuning_labels[key]
      self._fcw_tuning[key] = option_item_sp(
        title=lambda t=title: tr(t), param=key, min_value=low, max_value=high,
        value_change_step=step, description=lambda d=description: tr(d) + " " + tr("Applies within a second; no reboot."),
        enabled=lambda: self._params.get_bool("AggressiveFCW"), label_width=210,
        label_callback=(lambda v: f"{v}%") if key == "FCWMinConfidence" else (lambda v: f"{v / 10:.1f} s"),
      )

    self._toggles = {}
    self._locked_toggles = set()
    for param, (title, desc, icon, needs_restart) in self._toggle_defs.items():
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      # Make description callable for live translation
      additional_desc = ""
      if needs_restart and not locked:
        additional_desc = tr("Changing this setting will restart sunnypilot if the car is powered on.")
      toggle.set_description(lambda og_desc=toggle.description, add_desc=additional_desc: tr(og_desc) + (" " + tr(add_desc) if add_desc else ""))

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

      if param == "LdwLowSpeed":
        self._toggles["LdwSensitivity"] = self._ldw_sensitivity_setting
        self._toggles["AggressiveFCW"] = self._fcw_sensitivity_setting
        self._toggles.update(self._fcw_tuning)

      # insert longitudinal personality after NDOG toggle
      if param == "DisengageOnAccelerator":
        self._toggles["LongitudinalPersonality"] = self._long_personality_setting

    self._update_experimental_mode_icon()
    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._long_personality_setting.action_item.set_selected_button(personality)
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    e2e_description = tr(
      "sunnypilot defaults to driving in chill mode. Experimental mode enables alpha-level features that aren't ready for chill mode. " +
      "Experimental features are listed below:<br>" +
      "<h4>End-to-End Longitudinal Control</h4><br>" +
      "Let the driving model control the gas and brakes. sunnypilot will drive as it thinks a human would, including stopping for red lights and stop signs. " +
      "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; " +
      "mistakes should be expected.<br>" +
      "<h4>New Driving Visualization</h4><br>" +
      "The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. " +
      "The Experimental mode logo will also be shown in the top right corner."
    )

    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._toggles["ExperimentalMode"].action_item.set_enabled(True)
        self._toggles["ExperimentalMode"].set_description(e2e_description)
        self._long_personality_setting.action_item.set_enabled(True)
      else:
        # no long for now
        self._toggles["ExperimentalMode"].action_item.set_enabled(False)
        self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._long_personality_setting.action_item.set_enabled(False)
        self._params.remove("ExperimentalMode")

        unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.")

        long_desc = unavailable + " " + tr("sunnypilot longitudinal control may come in a future update.")
        if ui_state.CP.alphaLongitudinalAvailable:
          if self._is_release:
            long_desc = unavailable + " " + tr("An alpha version of sunnypilot longitudinal control can be tested, along with " +
                                               "Experimental mode, on non-release branches.")
          else:
            long_desc = tr("Enable the sunnypilot longitudinal control (alpha) toggle to allow Experimental mode.")

        self._toggles["ExperimentalMode"].set_description("<b>" + long_desc + "</b><br><br>" + e2e_description)
    else:
      self._toggles["ExperimentalMode"].set_description(e2e_description)

    self._update_experimental_mode_icon()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    self._fcw_sensitivity_setting.action_item.set_selected_button(int(self._params.get_bool("AggressiveFCW")))
    for key, item in self._fcw_tuning.items():
      item.action_item.current_value = setting_value(key, self._params.get(key, return_default=True))
    self._ldw_sensitivity_setting.action_item.set_selected_button(
      sensitivity_index(self._params.get("LdwSensitivity", return_default=True)))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

  def _render(self, rect):
    self._scroller.render(rect)

  def _update_experimental_mode_icon(self):
    icon = "experimental.png" if self._toggles["ExperimentalMode"].action_item.get_state() else "experimental_white.png"
    self._toggles["ExperimentalMode"].set_icon(icon)

  def _handle_experimental_mode_toggle(self, state: bool):
    confirmed = self._params.get_bool("ExperimentalModeConfirmed")
    if state and not confirmed:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("ExperimentalMode", True, block=True)
          self._params.put_bool("ExperimentalModeConfirmed", True, block=True)
        else:
          self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._update_experimental_mode_icon()

      # show confirmation dialog
      content = (f"<h1>{self._toggles['ExperimentalMode'].title}</h1><br>" +
                 f"<p>{self._toggles['ExperimentalMode'].description}</p>")
      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)
    else:
      self._update_experimental_mode_icon()
      self._params.put_bool("ExperimentalMode", state, block=True)

  def _toggle_callback(self, state: bool, param: str):
    if param == "ExperimentalMode":
      self._handle_experimental_mode_toggle(state)
      return

    self._params.put_bool(param, state, block=True)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True, block=True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index, block=True)

  def _set_ldw_sensitivity(self, button_index: int):
    self._params.put("LdwSensitivity", sensitivity_index(button_index), block=True)

  def _set_fcw_sensitivity(self, button_index: int):
    self._params.put_bool("AggressiveFCW", button_index == 1, block=True)

