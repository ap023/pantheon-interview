# TODO / tracking

Lightweight tracker for known gaps and backlog items surfaced during the
build. Not a substitute for DESIGN.md — this is "what's left to do,"
DESIGN.md is "why it's shaped this way." Update as things get fixed or
new gaps get found; delete finished items rather than leaving them
checked off forever.

## Known gaps / bugs

- [ ] Config isn't validated up front. `Cell.__init__` calls
      `_resolve_config()` unguarded (no try/except), so a broken
      `config/fleet_defaults.yaml` (malformed YAML, a missing required
      field) crashes construction itself rather than becoming a refusal.
      `run_cycle` re-resolves config every cycle wrapped in a broad
      try/except, so a config that only goes bad *after* a Cell already
      exists (live-edited mid-shift) fares better for some failure modes
      (malformed YAML -> clean `config_unresolved` refusal) but not
      others (a missing field read outside that try/except, e.g.
      `takt_s`, still crashes `run_cycle` itself). Verified against real
      `Cell`/`config.resolve()` — see `config/test_fixtures/README.md`
      for the full breakdown and drop-in broken configs to reproduce
      each case. Not fixed here: out of scope while only known-good
      configs are being passed in.

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
- [ ] DESIGN.md section 5's "sensor dropout (mid-cycle)" row — no
      simulated sensor exists that can actually drop out while a cycle
      is running. `declared_sensors` is a static frozenset checked once,
      pre-cycle; there's no dynamic sensor state to interrupt mid-cycle
      the way a kill or velocity spike can. Deliberately not attempted
      alongside the two rows below (logic fault systematic, task outcome
      clear) — this one needs a genuinely new stateful mechanism (a
      live-mutable sensor set plus a per-physics-step in-loop check,
      same shape as `kill`) that wasn't worth rushing under time
      pressure without real testing.
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
- [ ] `commands/` doesn't support a "change the Line's active variant"
      command while running — `line.set_variant(...)` is a Python
      method, no `commands/set_variant_{value}.json`-style file channel
      exists to call it live. This is the reason DESIGN.md section 5's
      "capability mismatch" / "sensor gone (pre-cycle)" refusal rows
      aren't currently live-triggerable against a running
      `line_runner` — both depend on which variant a part carries, and
      that's Line-wide, not something the per-cell `task_input/` inbox
      can already reach. Everything else needed to demonstrate it exists
      (`variants.py`'s registry, the sensor/capability checks in
      `Cell.run_cycle`) — just no way to flip the active variant from a
      second terminal without editing Python directly.
- [ ] `commands/` isn't a strict oldest-first FIFO queue across multiple
      pending commands the way DESIGN.md originally discussed — `kill`,
      `obstruct`, and `clear_failure` are each independently
      check-and-consumed per cell per tick (`runtime/commands.py`), with
      no explicit ordering guarantee if more than one lands between
      polls. `recalibrate` still doesn't exist as a command at all
      (blocked on `Cell.recalibrate()` itself not existing — see the
      calibration item above).
- [ ] Site/per-unit layering for `line_config.py` (per-edge/site) — still
      a single-layer (line defaults only) stub. `config.py` (per-cell)
      got its full fleet -> site -> per-unit chain built (DESIGN.md
      section 2: field-by-field merge, provenance tags, tie-value rule)
      — see `config/site_overrides/`, `config/per_unit/`. `line_config`
      is the same shape of gap, just not done yet: `resolve(edge_id)`
      already takes the right key, just doesn't merge anything on top of
      line defaults.

## Low priority

- [ ] Real tendon/gripper control. `controller.controlled_actuator_ids`
      now excludes tendon-transmitted actuators (e.g. Panda's real
      bundled gripper) from what `Cell` addresses at all, rather than
      mis-addressing them (see resolved bug note in git history) — so
      the gripper slot is simply untouched, not actually controllable.
      Real gripper control needs its own transmission-aware path
      (`actuator_trntype`-branching), not an extension of the
      joint-only controller.
- [ ] `Cell`'s demo `control_gain_kp=4.0` (config/fleet_defaults.yaml) is
      tuned against the synthetic 2-joint test fixture, not real robot
      dynamics — passing `robot_name="panda"` or `"ur5e"` to a `Cell`
      wrapping the real menagerie models (now safe to do at all, since
      the tendon-addressing crash is fixed) trips the velocity safety
      check on the very first cycle, because the proportional step spikes
      joint velocity past the datasheet limit before it can settle. Seen
      when trying to enable the velocity check in
      `multi_hardware_test.py`; left disabled there and in
      `line_runner.py` (both omit `robot_name` for the real-hardware
      cells) rather than tuned, since real gain tuning is out of scope for
      this build.
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
