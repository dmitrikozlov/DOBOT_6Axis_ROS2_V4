# Test plan: keep commands queued ahead in the ServoJ queue

## Why

Dobot (2026-10-20): ServoJ is a queued command; commands are executed one after
another, each over its `t`; the ServoJ queue is 32 deep. The controller follows
its queue, not our send times. With `t` equal to the send period the queue sits
at empty, so any lateness in arrival can leave it with nothing to execute.

That would explain three effects measured on 2026-10-15/16
(`results/20261015_servoj_ripple`, `results/20261016_servoj_tshort`):

| Effect | Queue explanation (to test) |
|---|---|
| Target speed collapses to 0 in one packet ("knocks"): ~1 move in 50 with `t` = period, ~1 in 10 with `t` shorter | the queue runs dry for a cycle |
| A command 10 ms late followed by one on schedule: target speed dips, then surges 30 %; alarm 225 at `t` = 28 ms | dry queue, then two commands close together |
| Ripple that varies run to run | how close to empty the queue runs |

**Hypothesis:** sending the first N commands at once and then keeping the
period keeps N commands queued ahead. The queue never runs dry, lateness under
N × `t` has no effect, and the knocks and surges disappear. Cost: N × `t` of
extra lag (40 ms for N = 2 at 50 Hz).

## Step 0: check the queue in existing recordings (no robot time)

Each ServoJ reply carries the command's queue ID (`0,{13571},ServoJ(...)`), and
the realtime feed reports `current_command_id`. For every recorded run:

1. Queue depth over time = last ID sent so far − `current_command_id`.
2. Confirm `current_command_id` follows the ServoJ IDs (one step per `t`).
3. At each knock (target collapse) and each late-send surge: what was the
   depth just before? Expected: 0.

If knocks happen with commands still queued, the queue model does not explain
them, and Steps 2–3 change (see Decisions).

## Step 1: script support

`dobot_proto.StreamConfig` gets `prefill=N`: at the start, send commands
0 … N back to back (each still after the previous reply), then command k at
start + (k − N) × period. Each command still carries its own trajectory point
(index timing). The run summary adds queue depth (min, median, max) from the
reply IDs and `current_command_id`, and the knock count. A unit check with
`fake_controller.py` confirms the send schedule.

## Step 2: buffer comparison (~10 min)

J1 out-and-back sweep, 25° at 20°/s (as before), 50 Hz, `t` = 20 ms,
aheadtime 100, gain 500, SpeedFactor 10, index timing. Interleaved, 20 runs
each:

| Config | prefill |
|---|---|
| q0 | 0 (today's driver) |
| q1 | 1 |
| q2 | 2 |
| q3 | 3 |

Measure per config: ripple (as defined in the Dobot question), knocks, lag
(target and actual), queue depth, alarms.

## Step 3: lateness with a buffer (~5 min)

Same move, two commands held back per run (at 0.9 s and 2.4 s), next command
on schedule (the pattern that raised alarm 225):

| Config | prefill | Delay |
|---|---|---|
| L0_10 | 0 | +10 ms |
| L2_10 | 2 | +10 ms |
| L2_30 | 2 | +30 ms |
| L2_50 | 2 | +50 ms (longer than the buffer: expect a dip) |

3 runs each. L0_10 may raise alarm 225; the script stops at the first alarm.

## Step 4 (if Step 2 helps): Dobot's 33 Hz

q0 and q2 at 33.3 Hz, `t` = 30 ms, 20 runs each.

## Before running

- drover-arm and drover-harvester stopped (port 29999 takes one client).
- The arm at a start pose where J1 ±25° is clear in the **new mount**, chosen
  on site (the collision scene in `task_params.yaml` still describes the old
  mount).
- If this is the controller's first ServoJ since it booted, expect one alarm
  225 on the first run (see `docs/dobot_servoj_first_stream.md` in drover);
  clear it and restart the script.
- Keep the safety TCP limit (250 mm/s).

## Decisions

- **Knocks gone and late-send surges gone with prefill 2:** add
  `servo_prefill` to the driver's TrajectoryExecutor (parameter in drover's
  `arm_driver.yaml`), keep `t` = period. Revisit the late-send rule
  (`kLateSend`): with a buffer, a late send no longer needs to move the
  schedule.
- **Prefill 1 enough:** prefer it (20 ms less lag).
- **No change in knocks:** the queue does not explain them; send the Step 0/2
  data to Dobot with the follow-up.
- **Queue depth drifts over a move** (commands arriving faster or slower than
  `t` is executed): measure the controller's execution period against `t` and
  report it to Dobot.

## Step 0 result (2026-10-20)

Data: all runs in `results/20261015_servoj_ripple` and
`results/20261016_servoj_tshort` that have `commands.csv` (about 440; injected
delays and pipelined runs excluded from the hold table). Depth = last queue ID
replied − `current_command_id` in each realtime packet.

1. **No command ever waits.** In every setting the depth is 0 or 1 (2 once);
   each new ServoJ becomes `current_command_id` a median 10 ms (one or two
   controller cycles) after its reply, p99 15 ms, whatever `t` is.
2. **`t` longer than the period does not back up.** With period 20 ms and
   `t` = 30 ms (27 runs) the depth also never exceeds 1. If each ServoJ ran for
   its full `t` in turn, that setting would gain one queued command every 60 ms
   (tens of commands and seconds of lag by the end of a move); the measured lag
   was only ~10 ms longer than at `t` = 20 ms.
3. **Knocks are not a dry queue.** How long each command stayed current, in
   controller cycles (8.2 ms):

   | Setting | 2 cycles | 3 cycles | 4 cycles | 5+ cycles | knocks in |
   |---|---|---|---|---|---|
   | period 20, t 20.0 | 16227 | 12591 | 78 | 3 | 2 × 3 cycles |
   | period 20, t 18.0 | 2862 | 2222 | 13 | 2 | 1 × 2, 2 × 3 cycles |
   | period 20, t 16.0 | 695 | 545 | 1 | 0 | none |
   | period 30, t 30.0 | 7 | 2433 | 3794 | 179 | 1 × 4 cycles |
   | period 30, t 26–28 | 2 | 1435 | 2278 | 114 | 3 × 3, 4 × 4 cycles |

   Commands are routinely held longer than `t` (e.g. 545 times 25 ms at
   `t` = 16 ms) without a knock, and the knocks happen in ordinary holds.

**Conclusion.** Whatever Dobot means by "queued", the controller takes up each
new ServoJ within a cycle or two and does not run the previous one for its
full `t` first. A buffer of commands sent ahead (Steps 1–4) would make the
controller aim ahead, not hold a reserve, so Steps 1–4 are not run. The knocks
remain unexplained: ask Dobot, with this table and the knock recordings
(`p30_t280_04`, `t260_05/06`, `t270_01/03/04`, `t280_04`, `t180_07/11/18`,
`t200_23`, `p20_ah50_04`, `p30_ah100_09`).
