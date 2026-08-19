# DESIGN.md
## Phase 1: Design Note
### 1. Pieces, ownership, and boundaries
```
Line (the loop)
├── Takt clock — advances time, releases parts at fixed interval
│
├── Topology — the ordered sequence of stations
│     owned by: Line
│     shape: a linear chain — station 1 → station 2 → ... → station N
│       → sink. A Cell only knows its own upstream buffer and
│       downstream buffer — it is handed references by the Line, not a
│       position/index in the sequence, so it doesn't need to know its
│       own place in the chain. (A graph-shaped topology, with
│       converging/diverging stations, was considered — see section 6
│       for why it's deferred rather than built.)
│
├── Buffers (one per gap between stations, each sized N)
│     owned by: Line, not by either neighboring Cell
│     read by: the Cell on each side (upstream = am I starved,
│               downstream = am I blocked)
│
├── Variant / Task Spec registry — which variant is currently in force
│     switched by: Line, at changeover events
│     read by: every Cell, every cycle
│
├── Part — released by Line at each takt tick; carries an id and a
│     target variant; flows through buffers along the topology;
│     operated on by whichever Cell it is currently in front of
│
└── Cells (one per station)
      │
      ├── Hardware — arm model, gripper, declared sensor set (static-ish)
      │
      ├── Calibration — value + age
      │     checked by: Cell, every cycle (expiry check)
      │
      ├── Config view — resolved read from Config resolver
      │     re-read by: Cell, every cycle, not just at startup
      │
      ├── Task Instruction channel — the concrete command handed to
      │     the controller for this cycle
      │     default: derived automatically from Part + active Variant
      │       (normal operation — pick up this part, standard grasp)
      │     override: an explicit instruction polled from an external
      │       source (a file, in this build — see section 1b), checked
      │       at cycle start and optionally at each physics step, used
      │       for manual/automated testing to hand the controller a
      │       deliberately adversarial target (out-of-range joint
      │       angle, excessive velocity, a target that can't complete
      │       within takt). Distinct from Config (describes what the
      │       cell *is*, changes rarely) and from Variant (describes
      │       *capability requirements*, not a concrete motion) — this
      │       is the actual value fed into controller(sim_data, target).
      │       An adversarial instruction flows through the same
      │       execution path as a normal one, so it's caught by the
      │       same mid-cycle checks (joint limits → logic fault,
      │       velocity → safety violation, duration → over-takt)
      │       rather than needing special-cased handling.
      │
      ├── Readiness check — runs before every cycle, using:
      │     calibration age, declared sensors, current variant vs. Cell's
      │     capability, current takt status
      │     outputs: proceed → run cycle, or refuse → Refusal record
      │
      └── Cycle execution — one attempt against the part in front of it
            outputs: Cycle record (outcome: success / failure / refusal)
Config resolver (separate from Line, separate from Cell)
│
├── Fleet defaults
├── Site overrides
└── Per-unit corrections
      resolved per cell, precedence = later layer wins, field by field
      each resolved value carries provenance (which layer set it)
      read by: each Cell, every cycle
```

**Boundary summary:**

- **Line owns** anything shared across stations: the clock, the topology, the buffers, the current variant, and part release. No Cell holds a reference to another Cell — they only touch shared state through the Line, and only know their own buffer endpoints, not the shape of the line around them.
- **Config resolver owns** the three-layer precedence and provenance. It has no knowledge of MuJoCo or hardware — Cell just asks it for a resolved view.
- **Cell owns** its own hardware description, calibration, readiness decision, and cycle execution. It does not own the buffer, the variant, or the config layers — it only reads from them. It also owns resolving its per-cycle Task Instruction — derived by default, overridable for testing — which is what actually gets handed to the controller, separate from Config and from Variant.
- **Cycle/Refusal records** are the output artifact of a Cell running (or refusing) a cycle — they get written out for the dashboard and for later reproducibility, not held long-term by Cell.

### 1a. Buffer semantics and topology validation

**Buffers, not `done` flags, are the source of truth for starved/blocked.** `done` is a single-cycle timing signal (did *this* cycle finish within takt), reset every cycle. It has no memory and doesn't reflect accumulated inventory. All cells run on the same shared takt clock, so cycles start in lockstep — a buffer doesn't let a downstream cell get ahead of an upstream neighbor's cycle timing. What it does provide is tolerance across *ticks*: if an upstream cell refuses or fails on a given tick, a downstream cell isn't necessarily starved on the very next tick, because inventory already banked from earlier successful ticks can cover it. This only works if buffer size is greater than 1 — at size 1, there's zero cushion, and any single missed tick upstream immediately starves the downstream cell on the next tick. Starved/blocked have to be read from actual buffer occupancy, not derived from `done`.

**Buffer size default: 1, configurable.** This is a deliberate scope choice, not an oversight — it means the line has no built-in tolerance for a single missed tick between neighbors by default, which is the simplest version of the mechanism and still satisfies every required behavior (starved, blocked, propagation). Sites or specific edges can be configured with a larger size if more slack between mismatched cells is wanted; the default just doesn't assume it.

**Why both `done` and buffers are needed — they measure different axes.** Buffers answer an *inventory* question: is there a part available to work on, and is there room to place my output. This drives starved/blocked. `done` answers a *timing* question: did this cycle finish before the clock moved to the next tick. This drives over-takt. Neither substitutes for the other — a cell can have a full upstream buffer (not starved) and still miss `done` because its own cycle is running slow (over-takt is about speed, not inventory). Conversely a cell can flip `done` to `true` on time every single cycle and still sit mostly idle because its upstream buffer keeps running empty (starved is about inventory, not speed). The Line reads buffer occupancy to determine starved/blocked per cell, and separately reads each cell's `done` flag at tick boundaries to determine over-takt per cell — two independent checks, not one derived from the other.

**Readiness rule.** A cell attempts a cycle on a given tick only if its upstream buffer currently holds a part. This is a per-cell decision, not a line-wide one: different cells can be in different readiness states on the same tick, and there is no single "is the system ready" flag for the line as a whole.

**Blocked is resolved downstream-first within a tick — build-time addendum.** Originally, both starved and blocked were meant to be read off one shared pre-tick snapshot, taken before any cell ran that tick. Built and tested that way first, then found the consequence empirically: at the default buffer_size=1, it produces a permanent one-tick "ping-pong" bubble between any two neighbors — an upstream cell can never see its output slot as freed by a downstream neighbor's pop *within the same tick*, only on the tick after, so two adjacent cells can never both run on the same tick, forever, even under otherwise-perfect conditions. Fixed by processing cells downstream-first within `tick()` and reading `blocked` live off actual buffer occupancy at the moment each cell is checked, rather than off the pre-tick snapshot — starved stays snapshotted exactly as before, since it's about a cell's own upstream, not a neighbor's action this same tick. This is the same evaluation order a synchronous hardware pipeline uses to get full throughput out of capacity-1 stages (the read on one side of a register and the write on the other commit on the same clock edge, not one tick apart) — not a new mechanism, just resolving the existing one in dependency order (last cell's state depends on nothing produced this tick; each cell upstream of it depends only on cells already resolved). Confirmed against a 3-cell line at buffer_size=1: every part in flight now advances exactly one buffer per tick, and no cell sits out a tick it didn't need to.

**Two config surfaces, not one.** Section 2's fleet -> site -> per-unit resolver is keyed by `cell_id` — it resolves settings that describe one cell (takt, tolerance, gain, calibration threshold). Buffer size doesn't fit that shape at all: it's a property of an *edge* between two stations, owned by the Line (as stated above), not by either cell on either side of it. So there are two separate config surfaces, each with their own defaults file and (eventually) their own precedence chain: `config/fleet_defaults.yaml` for per-cell settings, resolved by `runtime/config.py`; `config/line_defaults.yaml` for Line/topology settings like buffer size, resolved by `runtime/line_config.py`. This was originally left as an open question and has now been decided this way — build-time addendum, not a change to the original design.

**Preflight topology validator.** Runtime readiness checks answer "can this cell run this cycle." A separate, one-time check at shift start answers "is the line as configured even sound" — before any part is released, walk the chain and fail startup if:

- Any buffer has size ≤ 0 (can never hold a part — line can't run)
- Any cell is missing an upstream or downstream buffer reference (a broken link in the chain)
- The chain doesn't terminate at a defined **sink** (the line's completion/output stage)

This validator runs once, before the shift starts, and is a hard stop — a bad topology fails at startup, not partway through a run.

### 1b. Takt, hardware picks, and what's not being built

**Takt as a config field.** Takt isn't hardcoded — it's a field resolved through the same fleet defaults → site override → per-unit correction chain as everything else in section 2, so a site running faster or slower than the fleet norm can override it without touching code. Fleet default: **4 seconds**. Short enough that over-takt shows up as a real, demonstrable failure mode when a cell runs slow (rather than every cycle trivially finishing in time), long enough for a scripted pick-and-place motion — reach, close gripper, move, release — to complete under normal conditions in sim.

**First two arm/gripper combinations: Panda + Robotiq 2F-85, and UR5e + Robotiq 2F-85.** Chosen specifically for actuator-count contrast — Panda (`nu`=8, 7-DOF collaborative arm with redundant reach) and UR5e (`nu`=6, standard 6-axis industrial arm, no redundancy) have the widest `nu` gap among the arms in the model table. That gap is the actual point: Phase 2 requires proving the same orchestration code runs unmodified across cells with different actuator counts, and the widest gap available is the strongest version of that proof — a design that secretly assumed a fixed control-vector shape would fail loudest here, not on a narrower pair like Panda/iiwa14. Robotiq 2F-85 is the only real gripper model given, so it's used on both; if "two grippers" needs to be literal rather than "two arms sharing one real gripper," the second gripper is a minimal scripted stand-in (binary open/close, no MuJoCo model backing it) rather than a second real model that doesn't exist in the provided set.

**What is not being built:** see section 6 for the full list with reasoning. In summary — recalibration as anything beyond a manual injected event, mid-cycle abort on calibration/sensor loss (handled instead by pre-cycle lookahead plus a next-cycle fallback), a separate inter-station transit timer (folded into takt), and non-linear (converging/diverging) topology. Each of these was reasoned through, not skipped by omission — the tradeoff in every case is build time against a 5-hour budget, prioritized toward Phase 3's fault injection and dashboard over generality the spec doesn't require.

**Task instruction and fault injection channels.** Both reuse the same file-polling pattern already established for config (section 2) rather than inventing a new IPC mechanism:

- `task_input/task_{cell_id}.json` — an explicit per-cycle instruction override, polled by that cell at cycle start (and optionally per physics step for continuous updates). Written by a separate script/terminal for manual or automated adversarial testing.
- `commands/{command}_{cell_id}.json` — an injected fault (kill, drop a sensor, force calibration stale), polled by the Line/Cell once per tick and consumed (deleted) once applied.

Both directories are just files a second terminal process writes to and the main loop polls — no sockets, no shared process state, consistent with how config edits already take effect without a push mechanism.

### 1c. Execution mode and obstruction clearing

**Manual vs. automatic tick advance.** The takt clock's job is unchanged either way — it's still the thing that decides "has this tick elapsed, check every cell's `done` flag now." What changes is only the *trigger* that advances it: in automatic mode, a background timer fires every takt-interval; in manual mode, a keypress (Enter) on the clock's terminal fires it instead. Every downstream consequence — starved/blocked, over-takt, readiness checks, cycle records — is identical in both modes, since they only ever react to "a tick happened," not to *how* it happened. This means one clock interface with a pluggable trigger source, not two separate Line implementations. Manual mode is the natural default to build first (simpler, and makes demoing/debugging deterministic — you control exactly when the next tick lands relative to a task-input override you just wrote); automatic mode with a real timer, and a CLI flag to choose between them, is scoped as a fast-follow, not a blocker for anything else.

**Clearing an obstructing failure — three strategies, chosen per deployment or per run.** Section 5's "task outcome (obstructing)" row says the default response is a human clearing it. Two configurable alternatives are worth having:

- **Auto-clear.** The obstruction is removed programmatically the instant it's detected — useful for unattended demo runs or when you don't want a stuck sim during testing. The failure is still logged as obstructing; only the *response* changes, from paging a human to a scripted removal. This is a toggle on the failure-response behavior, not a change to the failure taxonomy itself.
- **Manual clear, explicit command.** The operator types something like `clear_failure {cell_id}` on the command terminal — this reuses the same `commands/` polling channel as fault injection (section 1b), just as a command that removes an obstruction rather than creates a fault. This is the version that models a real shift supervisor actually walking over and clearing the line, and it's the one worth building first since it exercises the same file-polling mechanism you're already building for fault injection — no new machinery, just a new command type.
- **No clear (default/current behavior).** The cell stays halted until one of the above happens. This remains the baseline if neither auto-clear nor manual-clear is configured.

### 1d. Hardware limits: model vs. datasheet

Not every physical limit lives in the same place, and the difference matters for where the number is allowed to come from:

- **Position range and torque/force range are encoded in the MJCF itself** (`jnt_range`, `actuator_forcerange` — confirmed present in both the Panda and UR5e menagerie models). These are read live from the loaded `MjModel`, never duplicated as a second hardcoded copy that could drift out of sync with the model.
- **Max joint velocity is not encoded in either MJCF at all** — checked directly, no velocity field exists anywhere in either model. It only exists as a manufacturer datasheet fact, so it's hardcoded per robot name in a dedicated `hardware_limits` module, separate from both the Config resolver and the model-derived limits above.

Both kinds are **Hardware** (section 1: "arm model, gripper, declared sensor set — static-ish"), not **Config** — a robot's top joint speed doesn't change because it's deployed at a different site, so it has no business going through the fleet/site/per-unit precedence chain. This is a build-time addendum, not a change to the original design.

### 2. Config precedence rule and provenance

**Precedence:** fleet defaults < site overrides < per-unit corrections, applied field by field — not whole-file replacement. A per-unit correction on one field does not blow away a site override on a different field.

**Provenance shape:** each resolved field is a pair, not a bare value:

```
{ value: 4.2, source: "site_override" }
```

This lets a reader print the full resolved config alongside a provenance column for every field, without re-deriving where any value came from.

**Where each layer physically lives.** One file per fleet default set, one file per site, one file per specific cell:

```
config/
├── fleet_defaults.yaml          # one file, applies to every cell
├── site_overrides/
│   ├── site_a.yaml               # one file per site
│   └── site_b.yaml
└── per_unit/
    ├── cell_003.yaml              # one file per specific cell
    └── cell_007.yaml
```

A site or per-unit file only needs to contain the field(s) being overridden — not a full restated config — since precedence is applied field by field. The resolver loads fleet defaults, then this cell's site file (if one exists), then this cell's per-unit file (if one exists), merging in that order and tagging each field's source by whichever file last set it.

**Persistence, not one-shot.** "Per-unit" describes *scope* (applies to this one specific unit, as opposed to fleet-wide or site-wide) — it does not mean the correction expires after a single read. A per-unit file persists and keeps applying every cycle until a person edits or removes it, exactly like the fleet-defaults and site-override layers above it. Because the resolver already re-reads and re-merges all three layers fresh every cycle (section 3), a file edited on disk mid-shift takes effect on the next cycle automatically — no push/notification mechanism is needed, and no layer needs special-case expiry behavior to support live edits.

**Tie values don't change precedence.** If a site override and a per-unit correction set the same field to the *same* value, the resolved source still reports `"per_unit_correction"`, not `"site_override"` — provenance reports which layer's decision currently governs the field, not whether that decision happened to produce a different number. This matters because if the site override later changes, a cell with a matching-but-independent per-unit correction should not drift with it; it stays pinned at its per-unit value. Reporting the higher layer whenever values coincidentally match would hide that pinning from a reader.

### 3. What a cell confirms before every cycle

Every cycle, before attempting work, a Cell:

1. Re-resolves its config view (not cached from startup). **This step can itself fail** — a malformed file (broken JSON/YAML) or a structurally invalid value (wrong type, nonsensical value like negative calibration age) at any of the three layers means resolution cannot complete. This fails loud, not silent: the cell does not fall back to a stale or partial config. It refuses this cycle and every cycle after, until the broken file is fixed — see the classification table below (Refusal — config unresolved). This check runs before any of the checks that follow, since none of them are meaningful without a resolved config to check.
2. Checks calibration will not expire during this cycle — not just "is it expired now," but "will `age + takt` cross the expiry threshold before this cycle is expected to finish." If so, refuse now rather than start a cycle likely to go stale mid-run. This is the primary calibration check; see section 6 for the fallback that covers what this lookahead can't predict (an unexpected over-takt overrun, or calibration forced stale by injected fault after the check already cleared it).
3. Checks all sensors required by the current variant are present in its declared sensor set
4. Checks the current variant is within its capability (reach, grasp)

All four checks run fresh every cycle. Failing any of them produces a refusal, not an exception — the cell does not attempt the cycle.

**Takt is not a cell self-check.** A cell can't determine in isolation whether the line's beat is being held — that's a comparison against the Line's clock, which the Line owns. Instead, each Cell exposes a `done` flag: set to `false` at the start of every cycle, set to `true` when the cycle completes. The Line's takt clock reads this flag for each cell at every tick — `true` means takt held, `false` means an over-takt event, which the Line records against that cell. This is a Failure (the cycle was attempted, just too slow), not a Refusal — see the classification table below.

### 4. Cycle record contents

To be reproducible six months later, each Cycle record contains:

- Cell id, cycle id, timestamp
- Part id, variant, task-spec version
- **Software versions**: orchestrator version (the runtime code running Cell/Line/Config — a git commit hash or tagged release is enough) and task/controller model version (whatever the controller is at the time — even for a scripted stub, this matters once it gets replaced or tuned, and matters more if a real policy model is ever swapped in). Without this, a record from six months ago is only reproducible against *today's* code — if the orchestrator or controller logic changed since, the same inputs could legitimately produce a different outcome now, and there'd be no way to tell that from the record alone.
- Resolved config at run time — stored as a hash pointing to a separately-stored resolved-config snapshot, so identical configs across many cycles are not duplicated
- Calibration value and age at run time
- Sensors actually present at run time
- Outcome: success / failure / refusal, plus the specific reason if not success
- Duration vs. takt, so over-takt is derivable later

**Reproducibility approach:** config resolution happens live every cycle (fleet defaults → site overrides → per-unit corrections, merged fresh). Reproducibility does not come from being able to time-travel the Config resolver itself — overrides are stored as current mutable state, not versioned history. Instead, reproducibility comes entirely from freezing the resolved result into each Cycle record at the moment the cycle ran, referenced by hash. This is a deliberate scope tradeoff: it is simpler to build, at the cost of not being able to ask "what was the override set to on a specific past date" independent of a cycle record that ran during that window.

### 5. Failure classification

The reasons a cell can fail to comply are not the same, and shouldn't be handled the same way:

| Class | Example | Triggers | Escalation | Who's notified |
|---|---|---|---|---|
| Refusal — config unresolved | Malformed JSON/YAML, or a structurally invalid value (wrong type, nonsensical field) at any of the three config layers feeding this cell | Cell fails loud, not silent — no fallback to stale/partial config. Refuses every cycle, stays out of rotation, until the broken file is fixed | Immediate — every occurrence, no threshold | Engineering/config-tooling if the broken file is fleet-level (broad blast radius); site owner or maintenance if it's a site/per-unit file (localized) |
| Refusal — stale state | Calibration expired, or predicted to expire within this cycle | Cell pulls itself out of rotation, waits for recalibration | Immediate — every occurrence | Maintenance / calibration owner |
| Refusal — capability mismatch | Variant needs a grasp this cell lacks | Cell refuses this part only, stays in rotation for compatible variants | Logged only — not an anomaly, it's routing information | Line planner (routing/staffing issue, not a fault) |
| Refusal — sensor gone (pre-cycle) | Required sensor missing at the pre-cycle check, before the cycle starts | Cell pulls out if that sensor is required by the current variant | Immediate — every occurrence | Maintenance |
| Failure — logic fault (transient) | Isolated controller error/timeout, one-off joint-limit violation | Log, cycle marked failed; cell retries next part normally | Logged only, unless it recurs | Engineering, if recurrence crosses the systematic threshold below |
| Failure — logic fault (systematic) | Same cell fails the same variant N cycles in a row (e.g. N=3) | Cell stops attempting that variant; other variants unaffected | Immediate once threshold crossed — retrying further won't fix a bad task spec | Engineering — task spec/controller is broken for this variant, not a hardware incident |
| Failure — sensor dropout (mid-cycle) | Sensor required by the controller disappears while a cycle is already running | If the controller can't complete without it, the attempt fails (logic-fault-style); if not needed for the rest of that cycle, it finishes and gets caught as a normal pre-cycle refusal next cycle | Immediate if it breaks an in-flight attempt; logged only if the cycle finished clean | Maintenance |
| Failure — in-cycle fault | Cell dies mid-cycle | Abort, log partial state, part's fate (scrap/retry) decided | Immediate — every occurrence | Shift supervisor |
| Failure — over takt | `done` not `true` by the tick boundary | Logged, cell stays in rotation but flagged as the line's bottleneck | Logged only, unless it becomes the line's sustained constraint | Shift supervisor / line balancing |
| Failure — safety violation | Velocity/torque exceeds a defined threshold, or unintended collision/contact force | Hard stop, cell pulled out immediately, line likely blocks if no bypass | Immediate — every occurrence, no threshold | Safety owner immediately, then maintenance |
| Failure — task outcome (clear) | Part dropped or misplaced, but lands outside the cell's workspace/buffer bounds | Logged, part scrapped or retried | Logged only | No immediate human required |
| Failure — task outcome (obstructing) | Part dropped or misplaced *inside* the cell's workspace or a buffer, physically blocking the next cycle | Cell halts itself (can't safely proceed), functions like a blocked station until cleared | Immediate — every occurrence | Shift supervisor paged to clear it |

Refusals are decisions a cell makes *before* attempting a cycle — recorded outcomes, not exceptions. Failures are outcomes of cycles that were actually attempted. The transient/systematic split on logic faults and the mid-cycle sensor dropout row both follow the same underlying rule: an isolated, unpredictable incident is logged and let go, while anything that recurs on the same cell/variant combination or that breaks safety escalates immediately rather than waiting for a pattern.

**Note on task-outcome split:** "clear" vs. "obstructing" is distinguished by whether the dropped/misplaced part's final position falls inside the cell's reachable workspace or a buffer zone — a check you define. Which side counts as needing a human is itself a configurable judgment call that could reasonably differ by deployment; it's documented here as an assumption, not a universal rule.

### 6. Out of scope / future work

**Summary.**

*Not building, in short:* automatic/scheduled recalibration (manual injected event only); mid-cycle abort on calibration or sensor loss (pre-cycle lookahead catches most cases, next-cycle refusal is the fallback for the rest); a separate inter-station transit timer (folded into takt); non-linear topology — converging/diverging stations (design reasoned through, deferred for build time).

*Build order and priorities:* Phase 2 first — one cell running cycles end to end (config resolution, readiness checks, refusals, a real Cycle record), then a second cell with different hardware (Panda + UR5e) through the same code, proving the abstraction actually holds before anything else is built on top of it. Then Phase 3 — multiple stations on a shared Line, fault injection, and the dashboard reading real records. Phase 4 (optional extension) only if time remains. Priority order within that: correctness of the readiness/refusal/failure boundary matters more than breadth of hardware or variants — a small, provably correct fleet beats a large one with fuzzy edges, since the whole exercise is about the software around the arms, not the arms themselves.

*Test apparatus.* Beyond the injected faults Phase 3 explicitly asks for (cell killed mid-cycle, calibration expiring under a running station, sensor dropping out, a cell slowing past takt), there should be a deliberate pass through the edge cases this document has already surfaced — calibration expiring exactly at a cycle boundary, a converging-style buffer contention if that ever gets built, config overlap where site and per-unit set the same field to the same value, over-takt triggering exactly at the tick boundary rather than after it. A **randomized test generator** — one that injects faults at random ticks/cells and asserts the resulting state (buffer levels, refusal reasons, records) is internally consistent — would be the strongest version of this, since hand-picked test cases only ever cover the edges you already thought of. That's flagged as likely out of scope for the time budget, not because it isn't valuable, but because hand-picked edge-case tests covering the known failure classes are a reasonable, cheaper substitute for a 5-hour build.

Design questions that are real but deliberately not being resolved for this build — flagged here rather than silently skipped, so the gap is visible rather than discovered later.

- **Recalibration mechanism.** Calibration *expiry* is handled (age is derived live from a last-calibrated timestamp, checked every cycle), but *re*calibration — the action that clears that expiry — is not designed in depth. For this build, recalibration is treated as a manual, injected event: a `recalibrate(cell_id)` call that sets a fresh offset and resets the timestamp, consistent with how every other fault in this project is injected by hand rather than arising on its own. Not designed here: who or what is authorized to trigger it, whether it should be automatic/scheduled (e.g., after N cycles or elapsed time) rather than manual, whether recalibration itself takes simulated time during which the cell is unavailable, and how a recalibration event should be reflected in the records of cycles that ran under the calibration it replaced (this last point overlaps with the Phase 4 calibration-lifecycle extension, if that's the one picked).
- **Mid-cycle calibration expiry — fallback only.** The pre-cycle lookahead check (section 3) is the primary defense and should catch calibration going stale in the ordinary case. It cannot catch every case: an over-takt overrun stretching a cycle longer than predicted, or calibration forced stale by an injected fault after the check already cleared it (the Phase 3 requirement to inject "calibration expiring under a running station"). For those, no mid-cycle abort is built — the in-flight cycle is allowed to finish, and the expiry is caught on the *next* cycle's pre-cycle check as a normal Refusal — stale state. The Cycle record for the cycle that ran through the expiry still captures calibration age at cycle *start* (per section 4), so the record stays honestly reproducible even though it reflects a cycle that went stale partway through.
- **Inter-station transit time.** Time to move a part from a downstream buffer into the next station's cell is assumed to be zero / folded into that station's own takt, not modeled as a separate timer. Takt is defined as the inter-arrival interval, which by definition already has to account for however long a part takes to get from one station to the next — adding a second clock per station (buffer→cell transit, distinct from cycle takt) would add real complexity (a third buffer state, effectively — "in transit" vs. "waiting" vs. "consumed") for something the spec doesn't ask to be modeled separately. Buffer-to-cell handoff is treated as instantaneous: a completed part appears in the downstream buffer, and the next station picks it up at its next cycle start.
- **Non-linear topology (converging/diverging stations).** The spec describes the line as a single sequential loop throughout — a linear chain is what's actually asked for, and what's built here. A graph-shaped topology (two cells feeding one shared downstream buffer, or one cell fanning out to several) was considered, since real mixed-model lines sometimes do have multi-arm stations feeding a shared fixture. It's deliberately not built, but the design was worked through far enough to be confident it would extend cleanly rather than requiring a rework:
  - **Topology as a graph, not a sequence.** Each cell would be handed references to its own upstream buffer(s) and downstream buffer(s) directly by the Line, rather than a position/index — so a cell never needs to know the shape of the line around it, linear or not. This is what would let converging/diverging layouts be added without touching Cell code.
  - **Starved generalizes to "any."** A cell with multiple upstream buffers would be starved if *any* one of them is empty — it needs a part from every input to run a cycle, so one empty input stops the cycle even if the others are full.
  - **Blocked propagates to all upstream neighbors, not just one.** If a converging cell runs slow relative to its inputs, all of its upstream buffers fill up, and every cell feeding it goes blocked at once. This doesn't need new state machinery — it's the same blocked/starved mechanism from the linear case, just applied per-edge instead of assuming exactly one edge per direction.
  - **Buffers remain the source of truth over `done`, for the same reason as the linear case.** `done` is still just a single-cycle timing signal with no memory; buffer occupancy is still what encodes accumulated inventory and tolerance to a missed tick. None of that reasoning changes just because a cell has more than one input.
  - **The preflight validator would need two more checks.** Beyond the linear checks (buffer size > 0, chain terminates at a sink), a graph topology would also need: every cell reachable from a part-release point (unreachable = starves forever), and no cycle in the graph (a loop would let a part circulate forever instead of reaching the sink).
  - **One question was left genuinely open and not resolved:** contention. If two upstream cells finish on the same tick but the shared downstream buffer only has room for one more part, there's no rule yet for which one gets in and what happens to the other (retry next tick, treat it as backpressure, some priority order). This would need a real answer before a converging topology could actually run, not just be representable.

  This is flagged here, with the reasoning kept rather than just the conclusion, so the decision to leave it out is a stated tradeoff against a 5-hour budget better spent on Phase 3's fault injection and dashboard — not a gap that was never noticed.
- **Per-cycle overhead / throughput ratio.** A derived metric worth having but not built here: for a given cell (or the whole line), what fraction of takt-windows resulted in a completed cycle — `completed cycles / total takt-windows elapsed`, capped at 1. This is essentially the manufacturing-standard idea of line efficiency (OEE-style), expressed simply. A few things worth deciding if this gets built:
  - **Denominator choice matters.** "Total takt-windows elapsed" (every tick, whether or not a cycle was even attempted) is different from "total cycles attempted" (excludes starved ticks). The first captures true idle time including starvation; the second only captures failure/refusal rate among attempts. Both are legitimate, they answer different questions, and the metric should probably expose both rather than picking one silently.
  - **Resettable over configurable windows.** Per-shift, per-hour, or a rolling window — not just a single lifetime ratio, since a cell's efficiency an hour ago and its efficiency in the last five minutes tell you different things (a rolling window would surface degrading performance faster than a lifetime average, which gets diluted over a long shift).
  - **Nothing new to collect — this is a computation over existing Cycle records.** Every input this metric needs (outcome, timestamp, which cell) is already in the Cycle record from section 4. It's a dashboard-layer aggregation, not a new data-collection mechanism, which makes it a natural Phase 3/4 addition rather than something that touches Cell or Line at all.
- **Manual buffer tampering.** Not designed here: what happens if a human physically removes a broken part from the line and places it into an arbitrary buffer that wasn't expecting it — e.g. dropping it into a buffer two stations downstream of where it failed, rather than where it belongs in the topology. Right now buffers are only ever written to by the cell immediately upstream of them, as part of normal cycle completion; there's no modeled path for an external, out-of-band insertion. If this were built, it raises questions the current design doesn't answer: does the inserted part need a valid id/variant to be readable at all, or does an ungoverned insertion count as a new failure class of its own (a part with unknown provenance entering a buffer)? Does the receiving cell's readiness check need a new check — "is this part one I actually expect" — or does it just process whatever's next in the buffer, unaware anything unusual happened? This would likely need its own manual command (reusing the `commands/` channel from section 1c, in the same spirit as manual clear), but the semantics of what a cell should do with an unexpected part aren't resolved and would need real thought before building.

## Phase 2: What Was Built, and Why

One cell running cycles end to end, then a second cell with different hardware through the same code — the phase-2 brief.

**Built:**
- `Cell` (`runtime/cell.py`) owns exactly what section 1's boundary summary says it should: hardware description, calibration, readiness decision, cycle execution. Constructed from an MJCF path plus a declared sensor set and capable-variant set. Resets to the model's `home` keyframe rather than a zeroed pose — Panda's zero pose is nonphysical (joint4's zero sits outside its own range, joint6's zero folds the hand into the forearm), verified empirically to wedge on self-collision otherwise.
- `ProportionalController` (`runtime/controller.py`) — one control loop that runs unmodified against Panda (`nu`=8) and UR5e (`nu`=6), the actuator-count gap the pair was picked for (section 1b). It addresses actuators through `actuator_trnid -> jnt_qposadr`/`jnt_dofadr` rather than assuming qpos/ctrl share an index order, and explicitly excludes tendon-transmitted actuators (Panda's real gripper) instead of mis-addressing them — `controlled_actuator_ids()`/`require_joint_transmission()` fail loud rather than silently aliasing an actuator onto the wrong joint.
- The config resolver (`runtime/config.py`) — the full fleet → site → per-unit chain from section 2: field-by-field merge, provenance tag per field, tie-value rule. `config/fleet_defaults.yaml`, `config/site_overrides/site_a.yaml`, `config/per_unit/cell_panda_001.yaml` are real example layers (README shows printing a resolved config with provenance for `cell_panda_001`).
- `runtime/hardware_limits.py` — position/torque range read live off the loaded `MjModel` (`jnt_range`, `actuator_forcerange`); max joint velocity hardcoded per robot name, since no MJCF exposes it at all (section 1d).
- The readiness checks inside `run_cycle`, run in the order section 3 specifies — config resolution (which can itself fail → refusal), calibration lookahead, sensor presence, capability match — each one a refusal, never an exception.
- Cycle records (`runtime/records.py`) carrying every field section 4 asks for, with the resolved config stored by hash to a shared snapshot rather than duplicated per record.
- `runtime/fault_tour_demo.py` — a scripted walkthrough that produces one real, printed example of every refusal/failure class against actual `Cell` objects (not mocks): the phase-2 "an example of each refusal" deliverable.

**What's not done for Phase 2, and why:**
- Only one real gripper model exists in the provided set (Robotiq 2F-85); Panda brings its own bundled gripper instead, and UR5e has none attached. Section 1b already reasoned through the alternative (a scripted stand-in second gripper) but it wasn't built (TODO.md, low priority) — deprioritized because the exercise is about the orchestration software around the arms, not the gripper model, and the actuator-count contrast Phase 2 actually asks to prove doesn't depend on which gripper is attached.
- `Cell.recalibrate()` doesn't exist yet — a cell that trips `calibration_stale` refuses every cycle forever (TODO.md; section 6 already scoped it as a manual injected event, just not implemented).
- Multi-waypoint cycles (reach → grasp → move → release) aren't built; `run_cycle` takes one `target_qpos` and one convergence check. The brief's "the manipulation does not have to work well" gave headroom to skip this — it costs the *appearance* of a real pick-and-place, not the correctness of anything the phases actually grade.

## Phase 3: What Was Built, and Why

Several stations on a shared Line, with faults, plus new hardware added by config — the phase-3 brief.

**Built:**
- `Line` (`runtime/line.py`) owns the clock, topology, buffers, variant registry, and part release — exactly the boundary section 1 draws. The preflight topology validator (section 1a) runs once at construction, before any part is released. `tick()` processes cells downstream-first and reads `blocked` live rather than off a pre-tick snapshot — the fix documented in section 1a's build-time addendum, found empirically (a permanent one-tick "ping-pong" bubble at `buffer_size`=1 without it).
- Two tick triggers sharing one interface (`TickTrigger`): `ManualTickTrigger` (keypress-advanced, built first, used in `fault_tour_demo.py`'s Line section) and `AutomaticTickTrigger` (real-timer-driven, used by `line_runner.py`).
- Fault-injection and task-override channels, both file-polled (section 1b): `task_input/task_{cell_id}.json` (per-cycle instruction override, consumed on read, stays active until cleared or replaced) and `commands/{command}_{cell_id}.json` (`kill` — checked every physics step since it has to interrupt an in-progress cycle; `obstruct`/`clear_failure` — checked once per tick). `runtime/commands.py`, `runtime/task_input.py`.
- `runtime/line_runner.py` — the actual shift driver: two cells (Panda at `site_a` with a per-unit tolerance correction, UR5e on pure fleet defaults) running against a real automatic takt clock, printing per-tick state (running / starved / blocked / refused / over-takt) and buffer occupancy, drivable live from a second terminal through the two file channels. This is the phase-3 "parts on the beat... injected faults... the line keeps moving" deliverable — currently demoed by hand through this live terminal output rather than through a dashboard (see below).
- `records.link_into_run` — every cycle from a shift gets symlinked into `records/runs/{run_id}/` in tick order, so a whole shift's history reads as one directory listing instead of hunting per-cell directories keyed by random `cycle_id`.

**What's not done for Phase 3, and why — flagged here rather than glossed over:**
- **No dashboard.** The brief frames it as its own visible deliverable ("a view onto your records... being able to inject a fault and watch it propagate is the version worth demoing"), and `line_runner.py`'s live terminal output already demonstrates the exact behavior a dashboard would visualize: inject a fault → the stuck cell refuses every tick → its downstream neighbor finishes what it already had, then reads starved → once the stuck cell's own upstream buffer fills, the Line stops releasing new parts entirely. Building an actual dashboard was deliberately scoped out of this session and deferred — see section 7.
- **No third arm model / new gripper / new variant added by config, no diff.** The brief's explicit test — "add a third arm model... nothing in shared code should change" — hasn't been exercised yet. `Cell`, `Line`, and `ProportionalController` were built with this in mind (transmission-aware actuator addressing, config-driven per-cell hardware, no hardcoded actuator count anywhere in shared code), so the abstraction is believed to hold, but that belief hasn't been proven the way the brief asks — an unverified claim, not a verified one, until the diff actually exists.
- `Cell.recalibrate()`, systematic-failure threshold tracking, mid-cycle sensor dropout, and the "clear" vs. "obstructing" task-outcome split are all still open (TODO.md) — none of these block a shift from running, they narrow which fault types can currently be demonstrated on it.
- `commands/` has no ordering guarantee if multiple commands land on the same cell within one tick window — currently last-file-wins per command type, adequate for the single-operator manual testing this build actually exercises, not designed for concurrent writers.

## 7. Review and Reflection

### 7.1 What counts as a successful cycle, and what that definition misses

A cycle succeeds (`run_cycle`, `runtime/cell.py`) when every controlled actuator's joint position comes within `position_tolerance_rad` of `target_qpos` before the takt-derived step budget (`takt_s / physics_dt`) runs out — `np.max(np.abs(error)) < tolerance`. Nothing else is checked.

What that misses:
- **Joint-space, not task-space.** `target_qpos` is a vector of joint angles, not an end-effector pose or a part placement. A cycle can converge on the *commanded* joint configuration and still have the gripper somewhere useless — wrong orientation, missed the part, arm folded through where the part actually sits — if the target itself was wrong. Success measures "did the controller do what it was told," not "did the part get picked up and placed."
- **No grasp verification.** There's no simulated box, no contact sensor read, no check that anything is actually held by the gripper at the moment of "success." The gripper actuator is excluded from control entirely for Panda (tendon-transmitted — `controller.py`) and doesn't exist as a controllable actuator for UR5e — so today, "success" is defined with the gripper entirely outside the loop.
- **No collision/contact awareness.** MuJoCo is running real physics underneath, so a self-collision or an illegal contact would most likely show up as the arm failing to reach the target in time (an over-takt failure) rather than being flagged as its own failure class. A collision that happens to still let the arm reach the target within budget passes silently, indistinguishable from a clean cycle.
- **No part-placement check.** The buffer model (section 1a) treats a completed cycle as an unconditional push into the downstream buffer — a "succeeded" cycle can't currently have actually misplaced the part. This is the same gap section 5 already flags under "task outcome (clear/obstructing)": that failure class exists in the design but isn't derivable from anything the sim currently measures, so it's only reachable by manual injection (`cell.obstruct()`), never by an actual missed placement the runtime notices on its own.

This was a deliberate scope choice, not an oversight: the brief is explicit that "the manipulation does not have to work well... none of the difficulty here is in making an arm grasp something. It is in the software around the arms." A joint-space convergence check is the cheapest criterion that can genuinely fail (over-takt, a logic-fault target) without requiring physical grasp simulation, which the brief explicitly says isn't the point of the exercise.

### 7.2 What was prioritized, and why

Within the time available, the build leaned toward **depth on the readiness/refusal/failure boundary and the Line's fault-propagation mechanics**, at the cost of **breadth of hardware and visualization**. Concretely:

- Getting the four-step readiness check (config → calibration → sensors → capability) and the eleven-row failure taxonomy (section 5) actually right — each with a real, reproducible example via `fault_tour_demo.py` — came before adding a third arm or building a dashboard. A fleet dashboard over a fuzzy or partially-wrong failure model would be actively misleading; a correct failure model with a plain-text view is honest and still demonstrates everything the exercise is actually checking for (section 6: "a small, provably correct fleet beats a large one with fuzzy edges").
- The Line's downstream-first tick-processing fix (section 1a) got real build time even though it's an easy thing to get subtly wrong and ship anyway — the alternative (one shared pre-tick snapshot for both starved and blocked) looks correct on paper and only fails empirically, so it was verified against an actual multi-cell run rather than trusted because it matched the original design note.
- File-based channels (`task_input/`, `commands/`) were chosen over a socket/IPC layer for fault injection and task overrides — the mechanism itself isn't what's being evaluated, and reusing one pattern (poll a directory, consume on read) across config, task input, and commands keeps the surface area small and consistent.
- The dashboard and the third-hardware extension diff are the two things cut this session — not because they're unimportant, but because they're the most demonstrable-yet-separable pieces of what's left: the fault-propagation *behavior* they would visualize/prove already exists and is exercised today by `line_runner.py` and `fault_tour_demo.py`. What's missing is presentation and one additional proof-by-repetition, not new mechanism — deferred to a following session rather than rushed into this one.

### 7.3 Outstanding submission items

Flagged here rather than silently left out:
- **Dashboard screenshots** — not included. No dashboard has been built yet; it's the next piece of work, not an abandoned one.
- **Extension diff (third arm/gripper/variant)** — not included, for the same reason. When it's added, `git diff` against the commit that adds the third cell should show it touching only `line_runner.py`'s cell list, a new site/per-unit config file if one is needed, and `variants.py` — no changes to `cell.py`, `controller.py`, or `line.py` — since that's the actual claim this deliverable exists to prove.
- **Phase 4 (optional)** — not attempted. Given a choice between polishing Phase 3's remaining gaps (dashboard, extension diff) or starting an optional Phase 4 extension, Phase 3 took priority, since the brief marks Phase 4 as strictly optional and Phase 3 as not.

This matches the take-home's own framing — "You will not finish everything; scoping down deliberately is part of the exercise" — the cut here is the dashboard and the third-hardware proof, made as a deliberate choice rather than discovered as a surprise at the deadline.
