#!/usr/bin/env python3
"""Read-only ProMaster steering audit of local rlog files; requires openpilot's cereal.

Run from an environment with this opendbc checkout and openpilot on PYTHONPATH:
  python -m examples.promaster_lateral_report /path/to/route/*/rlog.zst
Prints JSON only. Does not connect to panda, transmit CAN, or change parameters.
"""
import argparse
import bz2
import json
from collections import Counter, defaultdict
from pathlib import Path

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

ADDRESSES = (0x106, 0x117, 0x1F6, 0x547, 0x5A2)


def decode(dbc, addr, dat):
  msg = dbc.addr_to_msg[addr]
  if len(dat) != msg.size:
    return None
  values = {}
  for name, sig in msg.sigs.items():
    raw = get_raw_value(dat, sig)
    if sig.calc_checksum is not None and raw != sig.calc_checksum(addr, sig, bytearray(dat)):
      return None
    if sig.is_signed and raw & (1 << (sig.size - 1)):
      raw -= 1 << sig.size
    values[name] = raw * sig.factor + sig.offset
  return values


def report(events):
  dbc = DBC('fca_giorgio')
  counts, invalid, lat_active = Counter(), Counter(), Counter()
  distributions = defaultdict(Counter)
  last, pending = {}, {}
  pair_residuals = defaultdict(Counter)
  ack_ms = defaultdict(list)
  configs = []
  first, end = None, None
  for event in events:
    kind = event.which()
    t = event.logMonoTime
    if kind == 'carParams':
      cp = event.carParams
      config = {'fingerprint': cp.carFingerprint, 'dashcamOnly': cp.dashcamOnly,
                'safetyModels': [str(c.safetyModel) for c in cp.safetyConfigs]}
      if config not in configs:
        configs.append(config)
    elif kind == 'carControl':
      lat_active[str(event.carControl.latActive)] += 1
    elif kind in ('can', 'sendcan'):
      first = t if first is None else min(first, t)
      end = t if end is None else max(end, t)
      for frame in getattr(event, kind):
        addr, bus, dat = frame.address, frame.src, bytes(frame.dat)
        if addr not in (*ADDRESSES, 0x4AE):
          continue
        key = kind, bus, addr
        label = f'{kind}:bus{bus}:{addr:#x}'
        counts[label] += 1
        if addr == 0x4AE:
          continue
        values = decode(dbc, addr, dat)
        if values is None:
          invalid[label] += 1
          continue
        for name in ('LKA_STATUS', 'LKA_FAULT', 'LKA_ACTIVE', 'LKA_TORQUE', 'DRIVER_TORQUE', 'NEW_SIGNAL_1'):
          if name in values:
            distributions[f'{label}:{name}'][values[name]] += 1
        # Compare matched-counter command pairs, regardless of arrival order.
        if addr in (0x117, 0x1F6):
          other = last.get((kind, bus, 0x117 if addr == 0x1F6 else 0x1F6))
          if other and 0 <= t - other[0] <= 5_000_000 and values['COUNTER'] == other[1]['COUNTER']:
            primary, secondary = (values, other[1]) if addr == 0x1F6 else (other[1], values)
            pair_residuals[f'{kind}:bus{bus}'][secondary['LKA_TORQUE'] - 4 * primary['LKA_TORQUE']] += 1
        # Stock requests are observed on camera bus 2; generated requests use sendcan bus 0.
        if addr == 0x1F6 and ((kind, bus) in (('can', 2), ('sendcan', 0))):
          previous = last.get(key)
          eps = last.get(('can', 0, 0x106))
          if not values['LKA_ACTIVE']:
            pending.pop(kind, None)
          elif previous and not previous[1]['LKA_ACTIVE'] and eps and eps[1]['LKA_STATUS'] == 0:
            pending[kind] = t
        if key == ('can', 0, 0x106) and values['LKA_STATUS'] == 2:
          for source, start in list(pending.items()):
            if 0 <= t - start <= 1_000_000_000:
              ack_ms[source].append(round((t - start) / 1e6, 3))
            pending.pop(source)
        last[key] = t, values
  return {
    'can_span_seconds': (end - first) / 1e9 if first is not None else 0,
    'carParams': configs,
    'carControl_latActive': dict(lat_active),
    'frame_counts': dict(counts),
    'invalid_length_or_checksum': dict(invalid),
    'signals': {k: dict(v) if len(v) <= 8 else {'min': min(v), 'max': max(v), 'samples': sum(v.values())}
                for k, v in distributions.items()},
    'secondary_minus_four_times_primary': {k: dict(v) for k, v in pair_residuals.items()},
    'fresh_request_to_eps_ack_ms': dict(ack_ms),
  }


def read_events(paths):
  from cereal import log
  import zstandard

  for path in paths:
    data = Path(path).read_bytes()
    if data.startswith(b'BZh'):
      data = bz2.decompress(data)
    elif data.startswith(b'\x28\xb5\x2f\xfd'):
      with zstandard.ZstdDecompressor().stream_reader(data) as reader:
        data = reader.read()
    yield from log.Event.read_multiple_bytes(data)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('logs', nargs='+', help='Local full rlogs in segment order (qlogs omit raw CAN)')
  parser.add_argument('--output', type=Path, help='Write JSON to this file instead of stdout')
  args = parser.parse_args()
  result = json.dumps(report(read_events(args.logs)), indent=2) + '\n'
  if args.output:
    args.output.write_text(result)
  else:
    print(result, end='')
