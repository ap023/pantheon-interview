# TODO / tracking

Lightweight tracker for known gaps and backlog items surfaced during the
build. Not a substitute for DESIGN.md — this is "what's left to do,"
DESIGN.md is "why it's shaped this way." Update as things get fixed or
new gaps get found; delete finished items rather than leaving them
checked off forever.

## Known gaps / bugs

- [ ] A part consumed by a cell that doesn't finish `done` (over-takt,
      logic fault, safety violation, or a refusal) is scrapped, not
      retried or held for the next tick — `Line.tick()` pops it off the
      upstream buffer the moment a cycle is attempted, regardless of
      outcome, and only pushes it downstream on success. DESIGN.md
      section 5's failure table doesn't actually specify a part's fate
      for most of these rows ("scrapped or retried" is only stated for
      task-outcome failures) — this was a build-time choice to keep
      `Line.tick()` simple, not something section 5 mandated. Worth a
      real decision later: over-takt in particular reads more like "still
      in progress, ran out of time" than "destroyed," and a version of
      `Cell` that could resume a cycle across tick boundaries (rather
      than always finishing or timing out within one `run_cycle` call)
      would change what "consumed" even means here.

## Backlog (not built yet)

- [ ] `Cell.recalibrate()` — no way to clear a `calibration_stale`
      refusal at all right now. Obstruction has `obstruct()` /
      `clear_failure()`; calibration has the age-tracking and the
      lookahead refusal, but nothing that resets `_calibrated_at`, so
      once a cell trips `calibration_stale` it refuses every cycle
      forever. DESIGN.md section 6 already reasoned through this as a
      manual injected event (`recalibrate(cell_id)` — fresh offset,
      reset timestamp), same posture as every other injected fault in
      this build; just needs implementing. Section 6 also flags what's
      still genuinely open even once that exists: who's authorized to
      trigger it, whether recalibration should take simulated time
      during which the cell is unavailable, and how a recalibration
      event should be reflected in records of cycles that ran under the
      calibration it replaced.
- [ ] Multi-waypoint cycles (reach -> grasp -> move -> release) instead
      of a single target_qpos per `run_cycle` call.
- [ ] Torque safety-violation check. Velocity is now wired in
      (`hardware_limits.max_joint_velocity`, checked every physics step);
      torque/force isn't — MuJoCo already physically clamps applied
      force to `actuator_forcerange` internally, so this would be lower
      priority than velocity was, but it's not an explicit detected
      check the way velocity now is.
- [ ] No site-configured (softer-than-physical) position limit exists.
      The logic-fault check added this session compares the target
      against the model's *true* joint range only
      (`hardware_limits.actuated_position_range`) — there's no config
      field for a tighter, deployment-specific bound.
- [ ] `Line.tick()` (runtime/line.py) now reads `Cell.done` at the tick
      boundary as the real over-takt signal, per DESIGN.md section 3 —
      but `run_cycle` still also decides over-takt internally by
      comparing its own step count to budget, and `TickResult.over_takt`
      currently reads `record.reason == "over_takt"` (set by that same
      internal decision) rather than purely off `done`. The two happen
      to always agree today because they're driven by the same
      `self.done = reached` line inside `run_cycle`, but the redundancy
      itself hasn't actually collapsed — `run_cycle` would need to stop
      deciding over-takt on its own for that to be true, which is a
      bigger change (see the `Cell` resume-across-ticks note above) than
      building the Line alone justified.
- [ ] `task_input/` and `commands/` file-polling (task overrides and
      fault injection, DESIGN.md sections 1b/1c). `Cell.obstruct()` /
      `clear_failure()` exist now as direct method calls but aren't
      wired to the `commands/` channel yet.
- [ ] Site/per-unit layering for `config.py` (per-cell) and
      `line_config.py` (per-edge/site) — both are single-layer
      (fleet/line defaults only) stubs right now. Two separate
      precedence chains, since they're two separate config surfaces
      (DESIGN.md section 1a) — `config.resolve(cell_id)` and
      `line_config.resolve(edge_id)` already take the right key for it,
      just don't merge anything on top of the defaults layer yet.

## Low priority

- [ ] Verify `hardware_limits.py`'s max joint velocity numbers against
      the actual manufacturer datasheets (Franka Emika Panda, Universal
      Robots UR5e) rather than the commonly-cited community values
      currently used. Fine as a placeholder for sim purposes; only
      matters before anything safety-real depends on them.
- [ ] Randomize the Part (once it exists) — varying box size/pose/target
      per release instead of a fixed one, to exercise more of the
      controller/success-criteria surface. Very low priority, purely a
      testing-variety nice-to-have, not needed for correctness.
- [ ] Literal "two grippers" (take-home spec, Phase 2). Right now Panda
      brings its own bundled gripper (not Robotiq 2F-85) and UR5e has
      none attached. DESIGN.md section 1b already reasoned through the
      options (Robotiq 2F-85 on both, or a scripted stand-in for the
      second) but neither was implemented. Deliberately deprioritized:
      the exercise is about the software around the arms, not what's
      inside the gripper black box — revisit later if time allows.
