# Config test fixtures

Drop-in replacements for `config/fleet_defaults.yaml`, one broken in a
different way each, plus `valid_baseline.yaml` (a working copy, for
diffing). Every file's actual observed behavior — verified against real
`Cell`/`config.resolve()`, not guessed — is documented in its own header
comment. Two behaviors repeat across files, worth knowing up front:

- **Constructing a `Cell` directly against a broken config** can crash
  the constructor itself (`Cell.__init__` calls `_resolve_config()`
  unguarded — no try/except). No cycle, refusal, or record exists yet
  when that happens.
- **A config file that goes bad *after* a `Cell` already exists**
  (an operator edits it live mid-shift) behaves differently: `run_cycle`
  re-resolves config every cycle, wrapped in a broad try/except — some
  failures (malformed YAML) turn into a clean `config_unresolved`
  refusal there; others (a missing field read outside that try/except,
  e.g. `takt_s`) still crash, just one line later.

This is a real, known gap (config values aren't validated up front) —
tracked in `TODO.md`, not fixed here; not in scope right now (we're only
passing known-good configs for the time being).

## How to use one

Point the runtime at a fixture instead of the real fleet defaults. Two
ways:

**A. Swap the file in place** (simplest, but touches a tracked file —
back it up first):

```bash
cp config/fleet_defaults.yaml /tmp/fleet_defaults.yaml.bak
cp config/test_fixtures/malformed_yaml.yaml config/fleet_defaults.yaml
python -m runtime.demo
# ... observe the crash / refusal ...
cp /tmp/fleet_defaults.yaml.bak config/fleet_defaults.yaml
```

**B. Point `config.py` at it directly** (no file touched, preferred for
one-off checks):

```bash
python3 -c "
from pathlib import Path
from runtime import config as config_module
config_module.FLEET_DEFAULTS_PATH = Path('config/test_fixtures/malformed_yaml.yaml')
from runtime.cell import Cell
Cell('cell_check', 'PATH/TO/some.xml')
"
```

## The files

### Broken/invalid config (crashes, or a clean `config_unresolved` refusal)

Not extended further — a known gap (config isn't validated up front), tracked but out of scope for now.

| File | What's wrong | What actually happens |
|---|---|---|
| `valid_baseline.yaml` | nothing — working copy | normal success/failure cycles |
| `malformed_yaml.yaml` | broken YAML syntax | crashes `Cell()` if constructed against it directly; a clean `config_unresolved` **refusal** if the file only goes bad after the cell already exists |
| `empty_file.yaml` | no fields at all | crashes (`KeyError`) either at construction or on the first cycle, depending on when it's introduced |
| `missing_control_gain_kp.yaml` | missing the one field `__init__` needs | crashes `Cell()` construction, always |
| `missing_takt_s.yaml` | missing the field `run_cycle` needs | crashes `run_cycle` with an uncaught `KeyError`, always — this one is **not** caught as a refusal even on a live edit |
| `non_numeric_takt_s.yaml` | `takt_s: "fast"` | crashes `run_cycle` with an uncaught `TypeError` on the first arithmetic that uses it |

### Valid config that reliably reproduces a specific DESIGN.md section 5 failure

Every file below is a **valid** config (no crash) that deterministically reproduces one named row of DESIGN.md's failure table — verified against a real `Cell`, not guessed.

| File | DESIGN.md row | What actually happens |
|---|---|---|
| `tiny_takt_s.yaml` | Failure — over takt | `takt_s: 0.01` (valid but unreachable) — reliably reports `over_takt`, the "realistic bad site override" case |
| `negative_takt_s.yaml` | Failure — over takt | `takt_s: -5.0` — silently floors to a 1-physics-step budget, reliably reports `over_takt` |
| `zero_tolerance.yaml` | Failure — over takt | `position_tolerance_rad: 0.0` — strict `<` comparison can never be satisfied, reliably reports `over_takt` |
| `calibration_stale.yaml` | Refusal — stale state | `calibration_max_age_s: 0.001` — every cycle's `calibration_age_s() + takt_s` immediately crosses the threshold, refuses with `calibration_stale`. No `recalibrate()` exists yet to clear it (TODO.md) — it refuses forever, not once |
| `aggressive_kp_safety_violation.yaml` | Failure — safety violation | `control_gain_kp: 500.0` — first physics step spikes velocity past the datasheet limit. **Config alone isn't enough** — the velocity check only runs when the `Cell` was constructed with `robot_name` set (a constructor arg, not a config field); `line_runner.py` omits it by default. See the file's own header comment for the one-liner to force it. |

### Not config-fixture-able at all — DESIGN.md section 5 rows driven by something else

These aren't in `fleet_defaults.yaml`'s shape, so no YAML swap can reproduce them. Listed here so this directory is a complete map of the table, not just the subset that happens to fit the config mechanism.

| DESIGN.md row | Why not config | Trigger it instead via |
|---|---|---|
| Refusal — config unresolved | *(excluded from this pass — see "Broken/invalid config" above, already covered)* | — |
| Refusal — capability mismatch | `capable_variants` is a `Cell(...)` constructor arg, not a config field | request an unsupported `variant` via `task_input/task_{cell_id}.json` |
| Refusal — sensor gone (pre-cycle) | `declared_sensors` is a `Cell(...)` constructor arg | construct the `Cell` with a sensor missing, or request a variant needing one it lacks |
| Failure — logic fault (transient) | driven by `target_qpos` vs. the model's real joint range, unrelated to any fleet-config field | drop an out-of-range `target_qpos` via `task_input/task_{cell_id}.json` — see the earlier fault-tour walkthrough |
| Failure — logic fault (systematic, N in a row) | built, but as a per-cell streak counter, not a config field | drop the same out-of-range `target_qpos` via the inbox 3 times in a row — the 4th attempt refuses with `logic_fault_systematic` on its own, no extra step needed (threshold configurable via `systematic_fault_threshold`, default 3) |
| Failure — sensor dropout (mid-cycle) | **not built** — no simulated sensor exists that can "drop out" during a running cycle | n/a — real gap, see `TODO.md` |
| Failure — in-cycle fault | commands-channel driven, not config | `touch commands/kill_{cell_id}.json` |
| Failure — task outcome (clear) | commands-channel driven, not config | `touch commands/drop_clear_{cell_id}.json` — one-shot failure, no halt (contrast with `obstruct`) |
| Failure — task outcome (obstructing) | commands-channel driven, not config | `touch commands/obstruct_{cell_id}.json` |
