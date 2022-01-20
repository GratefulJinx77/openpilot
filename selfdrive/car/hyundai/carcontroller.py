from cereal import car, log
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_acc_commands, create_acc_opt, create_frt_radar_opt
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR
from opendbc.can.packer import CANPacker
import cereal.messaging as messaging
from common.params import Params
from selfdrive.controls.lib.speed_limit_controller import SpeedLimitController

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
SpeedLimitControlState = log.LongitudinalPlan.SpeedLimitControlState


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert in [VisualAlert.steerRequired, VisualAlert.ldw])

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.speed_limit_controller = SpeedLimitController()
    self.sm = messaging.SubMaster(['liveMapData', 'longitudinalPlan'])

    self.signal_last = 0.
    self.disengage_blink = 0.
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.sl_force_active_timer = 0
    self.slc_state = 0
    self.speed_limit_osm = 0.
    self.speed_limit_offset_osm = 0.
    self.switching_to_hda_timer = 0.
    self.switching_to_scc_timer = 0.
    self.update_speed_limit_state = 0
    self.speed_limit_change_applied = True
    self.speed_limit_current = 0.
    self.speed_limit_offsetted_prev = 0.
    self.last_spam_resume_frame = 0
    self.last_spam_set_frame = 0
    #self.speed_diff_prev = 0.

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart, lead_visible):
    # Steering Torque
    new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    cur_time = frame * DT_CTRL
    if CS.leftBlinkerOn or CS.rightBlinkerOn:
      self.signal_last = cur_time

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = enabled and not CS.out.steerWarning and CS.out.vEgo >= CS.CP.minSteerSpeed and\
                  (CS.lfaEnabled or CS.accMainEnabled) and ((CS.automaticLaneChange and not CS.belowLaneChangeSpeed) or
                  ((not ((cur_time - self.signal_last) < 1) or not CS.belowLaneChangeSpeed) and not
                  (CS.leftBlinkerOn or CS.rightBlinkerOn)))

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []

    # show LFA "white_wheel" and LKAS "White car + lanes" when disengageFromBrakes
    disengage_from_brakes = (CS.lfaEnabled or CS.accMainEnabled) and not lkas_active

    # show LFA "white_wheel" and LKAS "White car + lanes" when belowLaneChangeSpeed and (leftBlinkerOn or rightBlinkerOn)
    below_lane_change_speed = (CS.lfaEnabled or CS.accMainEnabled) and CS.belowLaneChangeSpeed and\
                              (CS.leftBlinkerOn or CS.rightBlinkerOn)

    if not (disengage_from_brakes or below_lane_change_speed):
      self.disengage_blink = cur_time

    disengage_blinking_icon = (disengage_from_brakes or below_lane_change_speed) and not\
                              ((cur_time - self.disengage_blink) > 1)

    speed_limit_control_enabled = Params().get_bool("SpeedLimitControl")
    speed_limit_perc_offset = Params().get_bool("SpeedLimitPercOffset")
    last_speed_limit_sign_tap = Params().get_bool("LastSpeedLimitSignTap")
    stock_long_speed_limit_control_enabled = Params().get_bool("StockLongSpeedLimitControl")

    speed_limit_offsetted = int(self.get_speed_limit_osm(CS) + (self.get_speed_limit_offset_osm(CS) if speed_limit_perc_offset else 0.))

    if last_speed_limit_sign_tap:
      self.sl_force_active_timer = cur_time

    sl_force_active = speed_limit_control_enabled and (cur_time < self.sl_force_active_timer + 2.0)
    sl_inactive = not sl_force_active and (not speed_limit_control_enabled or (True if self.get_slc_state() == 0 else False))
    sl_temp_inactive = not sl_force_active and (speed_limit_control_enabled and (True if self.get_slc_state() == 1 else False))
    slc_active = not sl_inactive and not sl_temp_inactive

    set_speed_non_sl = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)

    if not slc_active:
      self.switching_to_hda_timer = cur_time

    switching_to_hda = slc_active and CS.out.cruiseState.enabled and not ((cur_time - self.switching_to_hda_timer) > 0.5)

    speed_limit_changed = True if self.speed_limit_offsetted_prev != speed_limit_offsetted else False
    self.speed_limit_offsetted_prev = speed_limit_offsetted

    print("speed_limit_osm = " + str(self.get_speed_limit_osm(CS)))
    print("speed_limit_offset = " + str(self.get_speed_limit_offset_osm(CS)))
    print("speed_limit_offsetted = " + str(speed_limit_offsetted))
    print("slc_state = " + str(self.get_slc_state()))
    print("sl_force_active = " + str(sl_force_active))
    print("sl_inactive = " + str(sl_inactive))
    print("sl_temp_inactive = " + str(sl_temp_inactive))
    print("slc_active = " + str(slc_active))
    print("self.update_speed_limit_state = " + str(self.get_update_state()))

    #if slc_active and CS.out.cruiseState.enabled:
      #self.switching_to_scc_timer = cur_time

    #switching_to_scc = CS.out.cruiseState.enabled and not ((cur_time - self.switching_to_scc_timer) > 0.5)

    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.openpilotLongitudinalControl:
      if (frame % 100) == 0:
        can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   lkas_active, disengage_from_brakes, below_lane_change_speed, disengage_blinking_icon,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning))

    if self.get_speed_limit_osm(CS) != self.speed_limit_current:
      if stock_long_speed_limit_control_enabled:
        self.speed_limit_change_applied = False
    self.speed_limit_current = self.get_speed_limit_osm(CS)
    speed_limit_offsetted_new = int(self.speed_limit_current + self.get_speed_limit_offset_osm(CS))

    print("self.speed_limit_change_applied BEFORE SPAM: " + str(self.speed_limit_change_applied))

    if not CS.CP.openpilotLongitudinalControl:
      if (CS.out.cruiseState.enabled) and not self.speed_limit_change_applied and not pcm_cancel_cmd and not CS.out.cruiseState.standstill:
        set_speed_current = int(float(CS.out.cruiseState.speed) * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH))
        speed_diff = int(speed_limit_offsetted_new - set_speed_current)

        if speed_diff > 0:
          if (frame - self.last_spam_resume_frame) * DT_CTRL > 0.1:
            can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
            self.last_spam_resume_frame = frame
        else:
          if (frame - self.last_spam_set_frame) * DT_CTRL > 0.1:
            can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL)] * 25)
            self.last_spam_set_frame = frame
        if speed_diff == 0:
          self.speed_limit_change_applied = True

        #self.speed_diff_prev = speed_diff

      if CS.out.gasPressed or CS.out.brakePressed or CS.out.cruiseButtons == 4:
        if stock_long_speed_limit_control_enabled:
          self.speed_limit_change_applied = False

      if pcm_cancel_cmd:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
        if stock_long_speed_limit_control_enabled:
          self.speed_limit_change_applied = False
      elif CS.out.cruiseState.standstill:
        # send resume at a max freq of 10Hz
        if (frame - self.last_resume_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
          if stock_long_speed_limit_control_enabled:
            self.speed_limit_change_applied = False
          self.last_resume_frame = frame

    #speed_calc = True if not CS.CP.openpilotLongitudinalControl and self.speed_diff_prev == 0 else False
    #slc_applied = False if not CS.CP.openpilotLongitudinalControl and self.speed_limit_change_applied == False else True

    if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
      #lead_visible = False
      accel = actuators.accel if enabled and CS.out.cruiseState.enabled else 0

      jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)

      if accel < 0:
        accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])

      accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      stopping = (actuators.longControlState == LongCtrlState.stopping)
      set_speed_in_units = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
      can_sends.extend(create_acc_commands(self.packer, enabled and CS.out.cruiseState.enabled, accel, jerk, int(frame / 2), lead_visible, set_speed_in_units, speed_limit_offsetted, slc_active, stopping))

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021,
                                                   CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_CEED, CAR.KIA_SELTOS, CAR.KONA_EV,
                                                   CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.SANTA_FE_2022,
                                                   CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.GENESIS_G70_2020]:
      can_sends.append(create_lfahda_mfc(self.packer, CS.out.cruiseState.enabled, lkas_active, disengage_from_brakes, below_lane_change_speed, disengage_blinking_icon, speed_limit_offsetted, slc_active, switching_to_hda, set_speed_non_sl, speed_limit_changed, speed_limit_offsetted_new))

    # 5 Hz ACC options
    if frame % 20 == 0 and CS.CP.openpilotLongitudinalControl:
      can_sends.extend(create_acc_opt(self.packer))

    # 2 Hz front radar options
    if frame % 50 == 0 and CS.CP.openpilotLongitudinalControl:
      can_sends.append(create_frt_radar_opt(self.packer))

    return can_sends

  def get_slc_state(self):
    self.sm.update(0)
    self.slc_state = self.sm['longitudinalPlan'].speedLimitControlState
    return self.slc_state

  def get_speed_limit_osm(self, CS):
    self.sm.update(0)
    self.speed_limit_osm = float(self.sm['longitudinalPlan'].speedLimit if self.sm['longitudinalPlan'].speedLimit is not None else 0.0) * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
    return self.speed_limit_osm

  def get_speed_limit_offset_osm(self, CS):
    self.sm.update(0)
    self.speed_limit_offset_osm = float(self.sm['longitudinalPlan'].speedLimitOffset) * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
    return self.speed_limit_offset_osm