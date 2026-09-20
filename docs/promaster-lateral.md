# ProMaster lateral development: stock reference and remaining blocker

## Baseline

This branch starts at inauner/opendbc `22b85a3cace6650dc8abd554dc93d8b85853a5b7`,
the commit pinned by inauner/promaster. Its parent development comes from
belm0's `promaster_0.11.0` branch (`64a7f7e` before the three inauner commits).
The existing inauner handshake edit is behaviorally equivalent to belm0's:
request activation immediately, send zero torque until EPS status is 2.
Increasing the torque limit is not a demonstrated solution to missing acknowledgment.

## Evidence from the supplied 2022 stock-LKAS reference

All six full rlogs were examined, spanning 345.18 seconds of CAN.
The recorded software is comma openpilot 0.11.1, release-tizi; CarParams is MOCK,
dashcamOnly, with noOutput safety. All 33,936 carControl samples have latActive
false. This is a factory-LKAS reference, not an openpilot actuation test.

| Observation | Result |
| --- | --- |
| Camera-bus primary command `0x1F6` | 34,512 frames, approximately 100 Hz |
| Camera-bus secondary command `0x117` | 34,511 frames, approximately 100 Hz |
| Camera-bus primary torque range | -68 through +34, in DBC raw command units |
| Camera-bus secondary torque range | -269 through +137 |
| Matched-counter secondary minus four times primary | Only 0, 1, 2, or 3 |
| Fresh request followed by EPS acknowledgment | 22 observed events, 9.657–30.433 ms |
| EPS status | 0 (standby) or 2 (active) |
| EPS fault bit | Zero in all 34,519 bus-0 EPS frames |
| Driver torque range | -387 through +414 |
| `0x547` HUD message | Approximately 4 Hz |
| `0x4AE` | 3,836 frames on bus 1; none on buses 0 or 2 |
| Checked steering/EPS/HUD frame length or checksum errors | None |

The command relation is consistent with primary = floor(secondary / 4).
The controller's exact secondary = primary * 4 fits that encoding but omits
the finer two bits of resolution. There is no evidence here that omitting
those bits prevents activation.

Bus 0 contains a startup command outlier (-1024/-2048); those values are not
the factory camera-bus operating range and must not be used to set limits.
The optional NEW_SIGNAL_1 bits vary in the stock route; their meaning remains
unproven. These findings describe one LKAS-equipped 2022 vehicle and do not
establish compatibility with every 2022+ EPS firmware or ACC-only vehicle.

## Changes and validation

- Recognize driver override in both torque directions, using the existing
  allowance of 80. Previously negative torque never set steeringPressed.
- Remove radar address 0x4AE from the steering transmit and relay-check lists.
  It is forwarded from bus 2 if present, rather than suppressed or treated as
  a steering ECU. This aligns safety with belm0's earlier DBC cleanup; it is
  not a demonstrated cause of missing torque in this route.
- Require each nonzero secondary command to match one accepted primary's
  torque times four and counter, with its request bit set and controls allowed.
  Consume the authorization once, expire it after 20 ms, and reset it on
  safety initialization. The previous bounds-only check allowed secondary
  commands to bypass rate and driver-torque limits.
- The 20 ms deadline is between the two outgoing software safety checks;
  it is not an EPS acknowledgment timeout. The controller emits both in one
  update. Zero secondary torque remains allowed for shutdown.
- Preserve the existing torque maximum (300), rate limits (4/update), EPS
  acknowledgment gate, and controller message timing.
- Add controller, stock-byte fixture, paired-message safety, timeout/wrap,
  reset, forwarding, and 500-frame controller-to-safety regression coverage.

Tests execute against a separately compiled libsafety shared library.
They do not flash panda or transmit CAN. A software test pass does not
demonstrate EPS acceptance or establish safe physical steering authority.

## Reproduce the read-only report

Use full rlogs, not qlogs. Have this opendbc checkout and openpilot's cereal
available on PYTHONPATH, and pass local files in segment order:

```sh
python -m examples.promaster_lateral_report \
  --output promaster-report.json /path/to/route/0/rlog.zst /path/to/route/1/rlog.zst
```

The script verifies available DBC checksums before decoding, summarizes
stock and sendcan messages separately, and reports acknowledgment latency.
It does not access hardware or change device parameters. Raw logs, access
URLs, VINs, and account credentials are not included in this repository.

## What is still needed to establish lateral control

An engagement log from the target ProMaster with the actual experimental
checkout is needed. Record its EPS firmware, factory LKAS equipment, harness
and CAN topology, software commit, and observed symptom. Determine the first
failed step from that log:

1. Vehicle recognized as RAM_PROMASTER and the intended safety model selected.
2. Cruise engagement and openpilot latActive actually become true.
3. Both steering requests reach sendcan and are accepted by panda safety.
4. EPS status moves from 0 to 2; inspect fault/status and HUD ordering if it does not.
5. After acknowledgment, nonzero paired torque commands appear and EPS/steering
   response is measured.

The stock reference proves none of steps 1–5 for openpilot on the target van.
Do not remove the acknowledgment gate or raise limits to compensate for a
failure earlier in this chain. Existing unvalidated areas also include RX
checksum/counter enforcement in safety, unknown stock signal bits, vehicle
geometry/tuning, and applicability to EPS variants. No live installation,
panda flash, or vehicle torque test was performed for this branch.
