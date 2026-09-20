import unittest
from collections import defaultdict
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.fca_giorgio.carcontroller import CarController
from opendbc.car.fca_giorgio.carstate import CarState
from opendbc.car.fca_giorgio.interface import CarInterface
from opendbc.car.fca_giorgio.values import CAR, DBC


class TestFcaGiorgioLateral(unittest.TestCase):
  def setUp(self):
    self.cp = CarInterface.get_non_essential_params(CAR.RAM_PROMASTER)
    self.controller = CarController(DBC[CAR.RAM_PROMASTER], self.cp)
    self.state = CarState(self.cp)
    self.control = structs.CarControl(latActive=True)
    self.control.actuators.torque = 1.0
    self.parser = CANParser('fca_giorgio', [('LKA_COMMAND', 100), ('LKA_COMMAND_2', 100)], 0)

  def update(self):
    frame = self.controller.frame
    output, messages = self.controller.update(self.control.as_reader(), self.state, frame * 10_000_000)
    self.parser.update([(frame * 10_000_000, messages)])
    primary = self.parser.vl['LKA_COMMAND']
    secondary = self.parser.vl['LKA_COMMAND_2']
    self.assertEqual(secondary['LKA_TORQUE'], primary['LKA_TORQUE'] * 4)
    self.assertEqual(secondary['COUNTER'], primary['COUNTER'])
    self.assertEqual(secondary['LKA_ACTIVE'], primary['LKA_ACTIVE'])
    return output, messages, primary, secondary

  def test_zero_torque_until_acknowledgment(self):
    for status in (0, 1, 3):
      self.state.lka_status = status
      output, _, primary, _ = self.update()
      self.assertEqual(output.torqueOutputCan, 0)
      self.assertEqual(primary['LKA_ACTIVE'], 1)
    self.state.lka_status = 2
    output, _, _, _ = self.update()
    self.assertEqual(output.torqueOutputCan, 4)

  def test_acknowledgment_loss_and_disengagement(self):
    self.state.lka_status = 2
    for _ in range(5):
      self.update()
    self.state.lka_status = 0
    output, _, primary, _ = self.update()
    self.assertEqual(output.torqueOutputCan, 0)
    self.assertEqual(primary['LKA_ACTIVE'], 1)
    self.state.lka_status = 2
    output, _, _, _ = self.update()
    self.assertEqual(output.torqueOutputCan, 4)
    self.control.latActive = False
    output, _, primary, _ = self.update()
    self.assertEqual(output.torqueOutputCan, 0)
    self.assertEqual(primary['LKA_ACTIVE'], 0)

  def test_limits_in_both_directions(self):
    for sign in (-1, 1):
      self.setUp()
      self.state.lka_status = 2
      self.control.actuators.torque = sign
      for frame in range(100):
        output, _, _, _ = self.update()
        self.assertEqual(output.torqueOutputCan, sign * min((frame + 1) * 4, 300))

  def test_hud_on_unscheduled_activation(self):
    self.control.latActive = False
    self.update()
    self.control.latActive = True
    _, messages, _, _ = self.update()
    self.assertIn(0x5A2, [addr for addr, _, _ in messages])

  def test_driver_override_both_directions(self):
    parsers = {bus: SimpleNamespace(vl=defaultdict(lambda: defaultdict(float))) for bus in (Bus.pt, Bus.cam)}
    for torque in (-81, -80, 0, 80, 81):
      parsers[Bus.pt].vl['EPS_2']['DRIVER_TORQUE'] = torque
      state = self.state.update(parsers)
      self.assertEqual(state.steeringPressed, abs(torque) > 80, torque)

  def test_stock_command_fixtures(self):
    # Camera-bus frames from the 2022 reference route, decoded and repacked exactly.
    packer = CANPacker('fca_giorgio')
    fixtures = (
      ('LKA_COMMAND', '803006be', {'LKA_TORQUE': 1, 'LKA_ACTIVE': 1, 'COUNTER': 6}),
      ('LKA_COMMAND_2', '806806ba', {'LKA_TORQUE': 6, 'LKA_ACTIVE': 1, 'COUNTER': 6}),
    )
    for name, expected, values in fixtures:
      self.assertEqual(packer.make_can_msg(name, 0, values)[1], bytes.fromhex(expected))

  def test_report_rejects_bad_checksum(self):
    from examples.promaster_lateral_report import decode
    from opendbc.can.dbc import DBC as CanDBC

    dbc = CanDBC('fca_giorgio')
    data = bytes.fromhex('803006be')
    self.assertEqual(decode(dbc, 0x1F6, data)['LKA_TORQUE'], 1)
    self.assertIsNone(decode(dbc, 0x1F6, data[:-1]))
    self.assertIsNone(decode(dbc, 0x1F6, data[:-1] + bytes([data[-1] ^ 1])))


if __name__ == '__main__':
  unittest.main()
