#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

class TestFcaGiorgio_Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (0x117, 0x1F6, 0x547, 0x5A2)}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 150

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  TX_MSGS = [[0x117, 0], [0x1F6, 0], [0x547, 0], [0x5A2, 0]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x117, 0x1F6, 0x547, 0x5A2]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("fca_giorgio")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fcaGiorgio, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"CANCEL": cancel, "RESUME": resume}
    return self.packer.make_can_msg_safety("ACC_BUTTON", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": enable}
    return self.packer.make_can_msg_safety("ACC_2", 2, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  def _user_gas_msg(self, gas):
   values = {"ACCEL_PEDAL": gas}
   return self.packer.make_can_msg_safety("ENGINE_1", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")

  def _paired_msg(self, secondary, torque, steer_req=1, counter=0):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req, "COUNTER": counter}
    return self.packer.make_can_msg_safety("LKA_COMMAND_2" if secondary else "LKA_COMMAND", 0, values)

  def test_secondary_requires_checked_primary(self):
    for sign in (-1, 1):
      self.setUp()
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._paired_msg(True, sign * 16)))
      self.assertTrue(self._tx(self._paired_msg(False, sign * 4)))
      self.assertTrue(self._tx(self._paired_msg(True, sign * 16)))
      self.assertFalse(self._tx(self._paired_msg(True, sign * 16)))

  def test_secondary_rejects_mismatch(self):
    for torque, req, counter in ((1200, 1, 0), (-16, 1, 0), (16, 0, 0), (16, 1, 1), (1201, 1, 0), (-1201, 1, 0)):
      with self.subTest(torque=torque, req=req, counter=counter):
        self.setUp()
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(self._paired_msg(False, 4)))
        self.assertFalse(self._tx(self._paired_msg(True, torque, req, counter)))

  def test_secondary_rejects_failed_primary(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._paired_msg(False, 4)))
    self.assertFalse(self._tx(self._paired_msg(False, 300)))
    self.assertFalse(self._tx(self._paired_msg(True, 16)))
    self.assertFalse(self._tx(self._paired_msg(True, 1200)))

  def test_secondary_timeout(self):
    for delay in (0, 20000, 20001):
      for start in (0, 0xFFFFFF00):
        with self.subTest(delay=delay, start=start):
          self.setUp()
          self.safety.set_controls_allowed(True)
          self.safety.set_timer(start)
          self.assertTrue(self._tx(self._paired_msg(False, 4)))
          self.safety.set_timer((start + delay) & 0xFFFFFFFF)
          self.assertEqual(self._tx(self._paired_msg(True, 16)), delay <= 20000)

  def test_secondary_disengagement_and_reset(self):
    for reset in (False, True):
      self.setUp()
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._paired_msg(False, 4)))
      if reset:
        self.safety.set_safety_hooks(CarParams.SafetyModel.fcaGiorgio, 0)
        self.safety.set_controls_allowed(True)
      else:
        self.safety.set_controls_allowed(False)
      self.assertFalse(self._tx(self._paired_msg(True, 16)))
      self.assertTrue(self._tx(self._paired_msg(True, 0, 0)))

  def test_radar_message_is_forwarded(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x4AE), 0)
    self.assertFalse(self._tx(common.make_msg(0, 0x4AE)))
    self.assertTrue(self._rx(common.make_msg(0, 0x4AE)))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_controller_sequence(self):
    from opendbc.car import structs
    from opendbc.car.fca_giorgio.carcontroller import CarController
    from opendbc.car.fca_giorgio.carstate import CarState
    from opendbc.car.fca_giorgio.interface import CarInterface
    from opendbc.car.fca_giorgio.values import CAR, DBC

    cp = CarInterface.get_non_essential_params(CAR.RAM_PROMASTER)
    controller = CarController(DBC[CAR.RAM_PROMASTER], cp)
    state = CarState(cp)
    control = structs.CarControl()
    self.safety.set_controls_allowed(True)
    # Exercise acknowledgment, saturation, reversal, driver override, and disengagement.
    for frame in range(500):
      control.latActive = frame < 450
      control.actuators.torque = 1.0 if frame < 200 else -1.0
      state.lka_status = 2 if 10 <= frame < 400 else 0
      driver_torque = 250 if 350 <= frame < 400 else 0
      state.out.steeringTorque = driver_torque
      self.safety.set_timer(frame * 10000)
      self.assertTrue(self._rx(self._torque_driver_msg(driver_torque)))
      if not control.latActive:
        self.safety.set_controls_allowed(False)
      _, messages = controller.update(control.as_reader(), state, frame * 10_000_000)
      for addr, dat, bus in messages:
        self.assertTrue(self._tx(libsafety_py.make_CANPacket(addr, bus, dat)), f'{frame=} {addr=:#x}')


if __name__ == "__main__":
  unittest.main()
