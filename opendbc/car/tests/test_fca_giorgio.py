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
    self.parser = CANParser('fca_giorgio', [('LKA_COMMAND', 100), ('LKA_COMMAND_2', 100), ('LKA_HUD_2', 4), ('LKA_HUD_3', 1)], 0)

  def update(self, now_nanos=None):
    frame = self.controller.frame
    if now_nanos is None:
      now_nanos = frame * 10_000_000
    output, messages = self.controller.update(self.control.as_reader(), self.state, now_nanos)
    self.parser.update([(frame * 10_000_000, messages)])
    primary = self.parser.vl['LKA_COMMAND']
    secondary = self.parser.vl['LKA_COMMAND_2']
    self.assertEqual(secondary['LKA_TORQUE'], primary['LKA_TORQUE'] * 4)
    self.assertEqual(secondary['COUNTER'], primary['COUNTER'])
    self.assertEqual(secondary['LKA_ACTIVE'], primary['LKA_ACTIVE'])
    return output, messages, primary, secondary

  def test_request_deadline_includes_waiting_for_ack(self):
    for acknowledged in (False, True):
      self.setUp()
      self.update(10_000_000_000)
      self.state.lka_status = 2 if acknowledged else 0
      _, _, primary, _ = self.update(13_499_999_999)
      self.assertEqual(primary['LKA_ACTIVE'], 1)
      output, messages, primary, _ = self.update(13_500_000_000)
      self.assertEqual(primary['LKA_ACTIVE'], 0)
      self.assertEqual(output.torqueOutputCan, 0)
      # Both HUDs follow the falling edge, even off their regular schedules.
      self.assertTrue({0x547, 0x5A2}.issubset({addr for addr, _, _ in messages}))
      self.assertEqual(self.parser.vl['LKA_HUD_2']['LKA_ACTIVE'], 0)
      self.assertEqual(self.parser.vl['LKA_HUD_3']['LKA_ACTIVE'], 0)

  def test_reset_requires_full_standby_and_new_ack(self):
    self.update(0)
    self.state.lka_status = 2
    self.update(3_500_000_000)
    for status in (2, 1, 3):
      self.state.lka_status = status
      output, _, primary, _ = self.update(6_000_000_000)
      self.assertEqual(primary['LKA_ACTIVE'], 0)
      self.assertEqual(output.torqueOutputCan, 0)
    self.state.lka_status = 0
    self.update(6_010_000_000)
    _, _, primary, _ = self.update(8_009_999_999)
    self.assertEqual(primary['LKA_ACTIVE'], 0)
    output, _, primary, _ = self.update(8_010_000_000)
    self.assertEqual(primary['LKA_ACTIVE'], 1)
    self.assertEqual(output.torqueOutputCan, 0)
    self.state.lka_status = 2
    output, _, _, _ = self.update(8_020_000_000)
    self.assertEqual(output.torqueOutputCan, 4)

  def test_disengagement_cannot_bypass_reset(self):
    self.update(0)
    self.control.latActive = False
    self.update(500_000_000)
    self.control.latActive = True
    _, _, primary, _ = self.update(510_000_000)
    self.assertEqual(primary['LKA_ACTIVE'], 0)
    _, _, primary, _ = self.update(2_499_999_999)
    self.assertEqual(primary['LKA_ACTIVE'], 0)
    _, _, primary, _ = self.update(2_500_000_000)
    self.assertEqual(primary['LKA_ACTIVE'], 1)

  def test_fault_or_nonstandby_restarts_reset(self):
    for fault_field in ('steerFaultPermanent', 'steerFaultTemporary', None):
      self.setUp()
      self.update(0)
      self.update(3_500_000_000)
      if fault_field:
        setattr(self.state.out, fault_field, True)
      else:
        self.state.lka_status = 2
      self.update(5_000_000_000)
      if fault_field:
        setattr(self.state.out, fault_field, False)
      self.state.lka_status = 0
      self.update(5_010_000_000)
      _, _, primary, _ = self.update(7_009_999_999)
      self.assertEqual(primary['LKA_ACTIVE'], 0)
      _, _, primary, _ = self.update(7_010_000_000)
      self.assertEqual(primary['LKA_ACTIVE'], 1)

  def test_fault_drops_request_immediately(self):
    self.update()
    self.state.lka_status = 2
    self.update()
    self.state.out.steerFaultPermanent = True
    output, _, primary, _ = self.update()
    self.assertEqual(primary['LKA_ACTIVE'], 0)
    self.assertEqual(output.torqueOutputCan, 0)

  def test_stale_ack_at_start_cannot_command_torque(self):
    self.state.lka_status = 2
    output, _, primary, _ = self.update()
    self.assertEqual(primary['LKA_ACTIVE'], 0)
    self.assertEqual(output.torqueOutputCan, 0)

  def test_multiple_cycles_keep_pairs_counters_and_checksum(self):
    active = []
    last_counter = None
    # Simulated EPS acknowledges a request one frame later, and drops to standby
    # one frame after withdrawal. Include reversals and zero-demand intervals.
    for frame in range(1800):
      self.state.lka_status = 2 if self.controller.lka_active else 0
      self.control.actuators.torque = (0, -1, 1)[(frame // 50) % 3]
      output, messages, primary, _ = self.update()
      active.append(primary['LKA_ACTIVE'])
      self.assertLessEqual(abs(output.torqueOutputCan), 34)
      if last_counter is not None:
        self.assertEqual(primary['COUNTER'], (last_counter + 1) % 16)
      last_counter = primary['COUNTER']
      # Independent c15e87dc CRC formulation: init 0, poly 0x1D, final XOR 0xF1.
      for addr, data, _ in messages:
        if addr in (0x1F6, 0x117):
          crc = 0
          for byte in data[:-1]:
            crc ^= byte
            for _ in range(8):
              crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
          self.assertEqual(data[-1], crc ^ 0xF1)
    transitions = [i for i in range(1, len(active)) if active[i] != active[i-1]]
    self.assertEqual(transitions, [350, 551, 901, 1102, 1452, 1653])

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
    self.update()
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
      self.update()
      self.state.lka_status = 2
      self.control.actuators.torque = sign
      for frame in range(100):
        output, _, _, _ = self.update()
        self.assertEqual(output.torqueOutputCan, sign * min((frame + 1) * 4, 34))

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
