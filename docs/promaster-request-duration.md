# ProMaster: bound the steering request before the observed EPS fault

This is a conservative development mitigation, not a demonstrated EPS protocol
fix. It is based on PR #1 merge `26cfc43c0b197a99f61106dfef6fafd2461179a5`.
Do not repeat the existing approximately 8.8-second faulting build.

## Independent examination of the actuation route

Both full rlogs, segments 0 and 1 of
`06dc1eb54e298062/00000005--2c0b456823`, were decoded using the exact tested
opendbc DBC and checksum implementation. There are 81,822 and 22,613 events,
respectively, ending 61.300049 and 77.900702 seconds after the first event.
Both initData records identify openpilot
`158241f5ac597ab74a57d4bc2f477f3eb0c19020`; that commit's opendbc gitlink is
`33d3ccec57446387a666599ca0094c39e08067e0`. The latter is the tested tree,
not the later pass-through experiment or PR #1's tree.

| Event | Seconds after first log event |
| --- | ---: |
| carControl.latActive becomes true | 29.549556 |
| First active generated pair (zero torque) | 29.555646 |
| EPS status 0 -> 2, fault remains 0 | 29.584328 |
| EPS status 2 -> 0 and fault 0 -> 1 | 38.424565 |
| carControl.latActive becomes false | 38.429738 |
| Generated pair becomes inactive | 38.435872 |
| EPS fault bit returns to 0 in segment 1 | 77.574113 |

The EPS fault precedes openpilot withdrawal. It occurs 8.868918 seconds after
the first active command, or 8.840236 seconds after acknowledgment. Only one
engagement/fault event is present in these two segments. The tester's report
of about six repeats cannot be independently timed from this route.
The eventual fault-bit clear does not establish which vehicle action cleared it.

All 7,129 sendcan pairs have matching active bits/counters and exactly 4:1
torque. Each command stream has zero counter discontinuities. All 14,242
returned panda frames have payloads present in sendcan; there are eight fewer
returned frames per address than sent over the whole recording. All 28,500
command frames checked (sent and returned) and 7,775 bus-0 EPS frames have valid
checksums. The maximum interval between generated pairs during activation is
11.390 ms; there is no missing-message gap near the fault.

Primary torque spans -58..+116; secondary spans -232..+464. The last primary
before the fault is +49 at 38.415507 s; driver torque at the fault is -186.
The +116 maximum occurred earlier, at 36.876 s. In the final 425 ms,
primary commands span +29..+97 and driver torque -250..-106. Opposing driver
torque is therefore a real confounder, not an excluded hypothesis. The reported
roughly 1 Hz oscillation is not a precise frequency measurement: a short
30.0–38.4 s, linearly detrended, Hann-window spectral check gives strongest
0.5–3 Hz components near 1.90 Hz for command, 1.43 Hz for driver torque, and
1.55 Hz for steering angle. This is consistent with substantial oscillation,
but does not establish its cause.

## Code comparison and likely trigger

The tested controller requests continuously for all of latActive, including
zero-demand periods and the initial wait for status 2. Neither torque reversal
nor driver override drops the request. PR #1's acknowledgment implementation
has equivalent behavior. The outer openpilot control/car code has no changes
between the tested commit and the integration base that add a request deadline.

The tested tree has a one-sided steeringPressed check (positive torque only).
Its torque controller freezes the integrator when steeringPressed is true;
negative driver opposition can therefore affect the control-loop behavior.
PR #1 already corrects this to absolute driver torque and adds the paired
secondary-message checks. Both improvements are retained here.

The tested checksum uses CRC-8 polynomial 0x1D, init 0, final XOR 0xF1 for
0x1F6/0x117. PR #1's existing Chrysler checksum is byte-equivalent for these
four-byte messages: none of the 28,500 logged command checksums differs.
Do not replace it with the default 0x0A final XOR. A new regression verifies
every generated steering frame across multiple cycles against the explicit
0xF1 formulation, in addition to the stock-byte fixtures.

A maximum continuous-request watchdog is the leading working hypothesis:
the EPS first accepts control, then faults after a long unbroken request;
the available stock-reference analysis has only roughly 0.8–3.4 second bursts.
This favors a duration/accumulation trigger over a checksum, counter, initial
handshake, missing-message, or simple instantaneous peak-torque explanation.
It does **not** distinguish a request watchdog from accumulated torque,
oscillation, or driver-opposition protection. One recorded failure and no EPS
diagnostic trouble code cannot prove an exact 8.8-second protocol limit.

## Mitigation behavior

- Limit the request to 3.5 seconds of monotonic time from its first active pair,
  including acknowledgment wait, zero torque, reversals, and lost acknowledgment.
- At expiry or disengagement, send both messages inactive with zero torque.
  Continue transmitting at the normal cadence with synchronized rolling counters.
- Require at least 2 seconds of inactive commands **and** 2 continuous seconds
  of status 0 with neither temporary nor permanent steering fault before a new
  request. A nonzero/unknown status or fault restarts the standby observation.
  A quick disengage/re-engage cannot bypass the reset interval.
- Startup also requires status 0. The first request has zero torque; nonzero
  torque requires a subsequent status-2 acknowledgment. The torque limiter
  restarts from zero. A fault immediately withdraws the request.
- Both factory HUD messages follow request transitions, including reset.
- Lower the Python and panda absolute limits from 300 to 34 primary units,
  and the panda secondary bound from 1,200 to 136. A symmetric 34 is within
  both sides of the stock-observed -68..+34 range. That range is an observation,
  not proof of a safe torque calibration for every vehicle.
- Keep rates 4/update, driver allowance 80, multiplier 3, and realtime delta
  150 unchanged. Keep PR #1's one-use matched-primary, counter, and 20 ms pair
  authorization checks. No safety check is disabled and no limit is increased.

The 3.5/2-second timing is a conservative experiment, not a stock-derived reset
protocol. This controller automatically starts another handshake if latActive
is still true after reset eligibility. **There is no steering assist during
the reset interval**, and it can last longer if EPS does not return to standby.
Openpilot's high-level latActive can remain true during that interval; the
factory request/HUD state and actuator output go inactive/zero. This is not
continuous lane-centering availability. The duration policy lives in the car
controller; panda retains torque/pair enforcement, not an independent duration
watchdog. No EPS DTC is cleared or masked.

## Next vehicle test

Use this exact candidate with matching panda safety firmware, following the
existing controlled-test handoff. First use short 3–4 second engagements,
preferably cancel at 3 seconds. At 3.5 seconds the controller withdraws the
request even if the engagement has not been canceled. Cancel before any
automatic re-request, and verify the full rlog before extending the test:

1. Both requests and torques return to zero, with valid checksums/counters.
2. EPS returns cleanly from 2 to 0 with LKA_FAULT remaining zero throughout.
3. The next separate short engagement gets a fresh acknowledgment before torque.
4. Driver override, cancellation, and panda TX acceptance remain correct; record
   steering oscillation and driver opposition rather than increasing torque.

Any recurrence of the EPS fault ends that test; preserve logs and EPS DTCs if
available. Do not repeat the old 8.8-second faulting build. Short clean releases
would support this mitigation, but do not by themselves prove the watchdog
hypothesis or authorize longer continuous steering. Changing both duration and
torque prioritizes the next test's conservatism over isolating one variable.

## Software validation

Controller tests cover exact deadline boundaries, waiting for acknowledgment,
stale initial acknowledgment, fault withdrawal, delayed/unknown standby,
reset interruption, brief disengagement, re-acknowledgment, and three complete
request/reset cycles with reversals and zero torque. They verify paired bits,
4:1 torque, uninterrupted counters, and explicit 0xF1 checksums. A 1,800-frame
controller-to-compiled-safety sequence covers resets, saturation, reversal,
driver override, and disengagement.

The generic realtime-limit test assumes max torque exceeds realtime delta.
With the new 34-unit cap, even a full 68-unit reversal is below delta 150;
the ProMaster test instead verifies the tighter absolute bound both before
and after the realtime timer expires. The production realtime check remains.

Targeted controller, safety, and CAN checksum suites: 74 passed, 4 skipped,
389 subtests passed. Ruff passed for all modified Python files. These are
software tests, not a complete device build, firmware flash, HIL result, or
vehicle validation of the reset protocol.
