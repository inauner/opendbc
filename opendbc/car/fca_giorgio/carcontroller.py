from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_giorgio import fca_giorgiocan
from opendbc.car.fca_giorgio.values import CanBus, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CANBUS = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.lat_active_last = False
    self.lka_active = False
    self.request_started_ns = None
    self.reset_started_ns = None
    self.standby_started_ns = None
    self.frame = 0

  def update_lka_request(self, lat_active, CS, now_nanos):
    fault = CS.out.steerFaultPermanent or CS.out.steerFaultTemporary
    if self.lka_active:
      # Count the request itself, including zero torque while awaiting acknowledgment.
      # Torque reversals, missing acknowledgment and brief disengagements cannot extend it.
      if not lat_active or fault or now_nanos - self.request_started_ns >= self.CCP.LKA_MAX_REQUEST_NS:
        self.lka_active = False
        self.request_started_ns = None
        self.reset_started_ns = now_nanos
        self.standby_started_ns = None

    if not self.lka_active:
      if CS.lka_status == 0 and not fault:
        if self.standby_started_ns is None:
          self.standby_started_ns = now_nanos
      else:
        self.standby_started_ns = None

      # First activation requires standby. Subsequent activations require a complete
      # inactive interval AND continuous fault-free standby, including on re-engage.
      reset_complete = self.reset_started_ns is None or (
        now_nanos - self.reset_started_ns >= self.CCP.LKA_RESET_NS and
        self.standby_started_ns is not None and now_nanos - self.standby_started_ns >= self.CCP.LKA_RESET_NS)
      if lat_active and not fault and CS.lka_status == 0 and reset_complete:
        self.lka_active = True
        self.request_started_ns = now_nanos
    return self.lka_active

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      was_lka_active = self.lka_active
      lka_active = self.update_lka_request(CC.latActive, CS, now_nanos)

      # Only command non-zero torque after the EPS rack has formally acknowledged (lka_status == 2)
      if lka_active and was_lka_active and CS.lka_status == 2:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        request_age = now_nanos - self.request_started_ns
        if request_age >= self.CCP.LKA_RAMP_DOWN_NS:
          # Ignore renewed demand and sign reversals once shutdown has started.
          magnitude = max(0, abs(self.apply_torque_last) - self.CCP.LKA_RAMP_DELTA)
          new_torque = magnitude if self.apply_torque_last >= 0 else -magnitude
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
        if request_age >= self.CCP.LKA_ZERO_TORQUE_NS:
          # Keep the request active at zero before dropping it at the hard deadline.
          # A stalled control loop must never extend the request to finish a ramp.
          apply_torque = 0
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND", apply_torque, lka_active))
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND_2", apply_torque * 4, lka_active))

    # **** HUD Controls ***************************************************** #

    request_changed = self.lka_active != self.lat_active_last
    if self.frame % self.CCP.HUD_2_STEP == 0 or request_changed:
      can_sends.append(fca_giorgiocan.create_lka_hud_2_control(self.packer_pt, self.CANBUS.pt, self.lka_active))
    if self.frame % self.CCP.HUD_3_STEP == 0 or request_changed:
      can_sends.append(fca_giorgiocan.create_lka_hud_3_control(self.packer_pt, self.CANBUS.pt, self.lka_active))
    self.lat_active_last = self.lka_active

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
