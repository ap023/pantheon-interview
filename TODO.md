# TODO / tracking

Lightweight tracker for known gaps and backlog items surfaced during the
build. Not a substitute for DESIGN.md — this is "what's left to do,"
DESIGN.md is "why it's shaped this way." Update as things get fixed or
new gaps get found; delete finished items rather than leaving them
checked off forever.

## Known gaps / bugs

(none open right now)

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
- [ ] Line should read `Cell.done` at shared tick boundaries as the
      actual source of truth for over-takt, per DESIGN.md section 3.
      `done` now exists as real state (false at cycle start, true on
      completion), but `run_cycle` still also decides over-takt
      internally by comparing its own step count to budget — that's
      redundant with `done` and should collapse once the Line exists to
      be the one reading it.
- [ ] Line: takt clock, topology, variant registry, part release
      (DESIGN.md section 1). Includes a `Part` record — bare id only,
      passed along the chain of cells as it moves through the line —
      that the Line releases at each takt tick. Decided: no physically
      simulated box for now, just the id; today's "success" stays pure
      joint-space convergence, nothing checks whether a simulated object
      was actually grasped or moved. `Buffer` (runtime/buffer.py) is
      already built standalone and usable in tests, but nothing owns or
      wires one per edge yet — that's still the Line's job.
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
