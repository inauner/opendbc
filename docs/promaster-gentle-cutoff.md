# ProMaster gradual cutoff candidate

The September 24 logs from openpilot 60c3b50 show one clean automatic
cutoff and one fault immediately after cutoff. In the clean case torque was
already zero for about 0.22 seconds; in the faulting case primary torque
changed from 11 to zero as the request fell. This supports testing a gradual
shutdown, but does not prove the EPS diagnostic rule.

The request still has a 3.5-second monotonic deadline, including acknowledgment
wait. At 3.0 seconds, disregard new torque demand and reduce the last primary
command toward zero by at most 2 units per normal 10 ms update. From the
34-unit cap this takes 170 ms. At 3.2 seconds force zero torque while retaining
the active request until 3.5 seconds, providing about 300 ms of zero torque.
Deadlines are evaluated on controller updates. A stalled loop may require an
abrupt zero command; it must never extend the deadline to finish the ramp.

Faults, acknowledgment loss and manual disengagement retain immediate zero
torque behavior. Faults and disengagement drop the request immediately.
Driver limits remain applied during the ramp. The paired secondary remains
exactly primary x4, with unchanged checksums, synchronized counters and panda
pair authorization. No safety limits or torque caps are increased.

Reset still requires at least two seconds inactive and two seconds continuous
fault-free EPS status 0. Restart begins with a zero-torque request and requires
EPS status 2 before torque resumes. There is no assist during reset, even if
the high-level engagement remains active. Sustained cyclic operation remains
unvalidated; this is not uninterrupted steering assistance.

Vehicle validation stages: first record a single automatic cutoff and cancel
before restart; inspect both full-rlog segments for accepted pairs, zero hold,
EPS status 0 and no fault. Only after reviewing that log, test one automatic
restart. Review again before repeated cycles. Stop on faults or unexpected
steering, and do not repeat the original 8.8-second faulting build.

Unit tests cover both torque signs, renewed/reversed demand during shutdown,
the zero hold, delayed updates, immediate cancellation/fault handling, and the
existing reset/acknowledgment/counter/checksum behavior. Software tests cannot
validate the EPS's physical response.
