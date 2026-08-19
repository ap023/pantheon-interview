import time

import numpy as np
import pytest

from runtime import config as config_module
from runtime import hardware_limits
from runtime import records as records_module
from runtime.buffer import Buffer
from runtime.cell import Cell


def fake_resolve(takt_s, tolerance, kp):
    def _resolve(cell_id):
        return {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": tolerance, "source": "fleet_default"},
            "control_gain_kp": {"value": kp, "source": "fleet_default"},
        }

    return _resolve


@pytest.fixture(autouse=True)
def isolate_records_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(records_module, "RECORDS_DIR", tmp_path / "records")
    monkeypatch.setattr(records_module, "CONFIG_SNAPSHOTS_DIR", tmp_path / "records" / "config_snapshots")


def test_run_cycle_succeeds_when_target_reachable_within_takt(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_test", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([0.3, 0.2]), part_id="p1", variant="default")

    assert record.outcome == "success"
    assert record.reason is None
    assert record.sim_steps > 0


def test_run_cycle_fails_over_takt_when_budget_too_small(two_joint_mjcf_path, monkeypatch):
    # One physics step (0.002s / 0.002s takt) is nowhere near enough to
    # close a large error, so this should time out deterministically.
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=0.002, tolerance=0.001, kp=5.0))
    cell = Cell("cell_test", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([1.0, 1.0]), part_id="p1", variant="default")

    assert record.outcome == "failure"
    assert record.reason == "over_takt"


def test_run_cycle_writes_a_record_to_disk(two_joint_mjcf_path, monkeypatch, tmp_path):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_test", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([0.1, 0.1]))

    record_path = tmp_path / "records" / "cell_test" / f"{record.cycle_id}.json"
    assert record_path.exists()


def test_current_qpos_reflects_sim_state(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_test", str(two_joint_mjcf_path))

    before = cell.current_qpos()
    cell.run_cycle(np.array([0.3, 0.2]))
    after = cell.current_qpos()

    assert not np.allclose(before, after)


def test_run_cycle_succeeds_exactly_at_budget_and_fails_one_step_short(two_joint_mjcf_path, monkeypatch):
    """DESIGN.md section 6 flags 'over-takt triggering exactly at the tick
    boundary rather than after it' as an edge case worth testing directly,
    not just via a budget that's obviously too small."""
    target = np.array([0.3, 0.2])
    tolerance = 0.05
    kp = 5.0
    dt = 0.002

    # Learn exactly how many sim steps this scenario needs under a
    # generous budget, using the real run_cycle loop rather than a
    # hand-rolled probe — that way the boundary values below are
    # guaranteed consistent with what the production code itself counts.
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=tolerance, kp=kp))
    probe_record = Cell("cell_probe", str(two_joint_mjcf_path)).run_cycle(target)
    assert probe_record.outcome == "success"
    steps_needed = probe_record.sim_steps

    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=steps_needed * dt, tolerance=tolerance, kp=kp))
    exact_record = Cell("cell_exact_budget", str(two_joint_mjcf_path)).run_cycle(target)
    assert exact_record.outcome == "success"
    assert exact_record.sim_steps == steps_needed

    monkeypatch.setattr(
        config_module, "resolve", fake_resolve(takt_s=(steps_needed - 1) * dt, tolerance=tolerance, kp=kp)
    )
    short_record = Cell("cell_short_budget", str(two_joint_mjcf_path)).run_cycle(target)
    assert short_record.outcome == "failure"
    assert short_record.reason == "over_takt"


def test_failed_cycle_does_not_prevent_a_later_cycle_from_succeeding(two_joint_mjcf_path, monkeypatch):
    """A cell that misses takt on one part should still be able to
    complete the next one — over-takt on cycle N shouldn't leave the
    cell stuck for cycle N+1. No Line exists yet to drive this across
    ticks, so this calls run_cycle twice on the same Cell directly to
    stand in for that."""
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=0.002, tolerance=0.001, kp=5.0))
    cell = Cell("cell_recover", str(two_joint_mjcf_path))

    failed = cell.run_cycle(np.array([1.0, 1.0]))
    assert failed.outcome == "failure"
    assert failed.reason == "over_takt"

    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    recovered = cell.run_cycle(cell.current_qpos() + 0.1)
    assert recovered.outcome == "success"
    assert recovered.cycle_id != failed.cycle_id


def test_two_cells_over_consecutive_ticks_produce_independent_outcomes(two_joint_mjcf_path, monkeypatch):
    """No Line exists yet, but this simulates what its tick loop will
    eventually do: call run_cycle on each cell once per tick,
    independently, and check each cell's outcome only depends on its
    own config/state (DESIGN.md section 1a — starved/blocked, and by
    the same logic success/failure, is a per-cell decision, not a
    line-wide one). Each cell's config resolver is pinned on the
    instance itself, bypassing the shared config stub, purely so one
    cell can be given a workable takt and the other a too-tight one at
    the same time."""

    def pinned_config(takt_s, tolerance, kp):
        return lambda: {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": tolerance, "source": "fleet_default"},
            "control_gain_kp": {"value": kp, "source": "fleet_default"},
        }

    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    fast_cell = Cell("cell_fast", str(two_joint_mjcf_path))
    fast_cell._resolve_config = pinned_config(takt_s=2.0, tolerance=0.05, kp=5.0)

    slow_cell = Cell("cell_slow", str(two_joint_mjcf_path))
    slow_cell._resolve_config = pinned_config(takt_s=0.002, tolerance=0.001, kp=5.0)

    targets = [np.array([0.1, 0.1]), np.array([0.2, 0.15]), np.array([0.05, 0.05])]

    fast_outcomes = [fast_cell.run_cycle(t, part_id=f"part_{i}").outcome for i, t in enumerate(targets)]
    slow_outcomes = [slow_cell.run_cycle(t, part_id=f"part_{i}").outcome for i, t in enumerate(targets)]

    assert fast_outcomes == ["success", "success", "success"]
    assert slow_outcomes == ["failure", "failure", "failure"]


def fake_resolve_with_autoclear(autoclear, takt_s=2.0, tolerance=0.05, kp=5.0):
    def _resolve(cell_id):
        return {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": tolerance, "source": "fleet_default"},
            "control_gain_kp": {"value": kp, "source": "fleet_default"},
            "autoclear": {"value": autoclear, "source": "fleet_default"},
        }

    return _resolve


def test_obstruction_halts_cell_and_refuses_the_next_cycle(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve_with_autoclear(autoclear=False))
    cell = Cell("cell_obstructed", str(two_joint_mjcf_path))

    cell.obstruct(reason="part_dropped_in_workspace")
    assert cell.halted is True

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "refusal"
    assert record.reason == "part_dropped_in_workspace"
    assert record.sim_steps == 0


def test_autoclear_on_means_obstruction_never_halts_the_cell(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve_with_autoclear(autoclear=True))
    cell = Cell("cell_autoclear", str(two_joint_mjcf_path))

    cell.obstruct(reason="part_dropped_in_workspace")
    assert cell.halted is False

    record = cell.run_cycle(np.array([0.1, 0.1]))
    assert record.outcome == "success"


def test_manual_clear_failure_lets_a_halted_cell_run_again(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve_with_autoclear(autoclear=False))
    cell = Cell("cell_manual_clear", str(two_joint_mjcf_path))

    cell.obstruct(reason="part_dropped_in_workspace")
    refused = cell.run_cycle(np.array([0.1, 0.1]))
    assert refused.outcome == "refusal"

    cell.clear_failure()
    assert cell.halted is False
    assert cell.halt_reason is None

    recovered = cell.run_cycle(np.array([0.1, 0.1]))
    assert recovered.outcome == "success"
    assert recovered.cycle_id != refused.cycle_id


# --- One example of each pre-cycle readiness refusal (DESIGN.md section 3 / 5) ---


def test_config_unresolved_produces_a_refusal_not_a_crash(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_config", str(two_joint_mjcf_path))

    def broken_resolve(cell_id):
        raise ValueError("malformed fleet_defaults.yaml: control_gain_kp is not a number")

    monkeypatch.setattr(config_module, "resolve", broken_resolve)

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "refusal"
    assert record.reason.startswith("config_unresolved")
    assert record.config == {}


def fake_resolve_with_calibration_max_age(max_age_s, takt_s=2.0, tolerance=0.05, kp=5.0):
    def _resolve(cell_id):
        return {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": tolerance, "source": "fleet_default"},
            "control_gain_kp": {"value": kp, "source": "fleet_default"},
            "calibration_max_age_s": {"value": max_age_s, "source": "fleet_default"},
        }

    return _resolve


def test_calibration_stale_refuses_before_expiry_would_be_crossed(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(
        config_module, "resolve", fake_resolve_with_calibration_max_age(max_age_s=100.0, takt_s=2.0)
    )
    cell = Cell("cell_calibration", str(two_joint_mjcf_path))

    # calibration_age_s() is ~0 right after construction. Dropping the
    # threshold below takt_s means age(~0) + takt already crosses it —
    # the lookahead should refuse before the cycle even starts, not wait
    # for calibration to actually be expired right now.
    monkeypatch.setattr(config_module, "resolve", fake_resolve_with_calibration_max_age(max_age_s=1.0, takt_s=2.0))

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "refusal"
    assert record.reason == "calibration_stale"


def test_calibration_not_refused_when_age_plus_takt_is_comfortably_under_threshold(
    two_joint_mjcf_path, monkeypatch
):
    """Control case for the lookahead test above — makes sure the check
    isn't just always tripping."""
    monkeypatch.setattr(
        config_module, "resolve", fake_resolve_with_calibration_max_age(max_age_s=100.0, takt_s=2.0)
    )
    cell = Cell("cell_calibration_ok", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "success"


def test_calibration_lookahead_at_the_exact_boundary_and_just_under_it(two_joint_mjcf_path, monkeypatch):
    """DESIGN.md section 6 flags 'calibration expiring exactly at a cycle
    boundary' as an edge case worth testing directly, not just via a
    threshold that's obviously already crossed."""
    takt_s = 2.0
    max_age_s = 5.0
    monkeypatch.setattr(
        config_module, "resolve", fake_resolve_with_calibration_max_age(max_age_s=max_age_s, takt_s=takt_s)
    )

    at_boundary = Cell("cell_calibration_at_boundary", str(two_joint_mjcf_path))
    at_boundary._calibrated_at = time.monotonic() - (max_age_s - takt_s)  # age + takt == max_age_s exactly
    boundary_record = at_boundary.run_cycle(np.array([0.1, 0.1]))
    assert boundary_record.outcome == "refusal"
    assert boundary_record.reason == "calibration_stale"

    just_under = Cell("cell_calibration_just_under", str(two_joint_mjcf_path))
    just_under._calibrated_at = time.monotonic() - (max_age_s - takt_s - 0.5)  # age + takt just short
    under_record = just_under.run_cycle(np.array([0.1, 0.1]))
    assert under_record.outcome == "success"


def test_sensor_missing_refuses_when_variant_needs_an_undeclared_sensor(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_no_camera", str(two_joint_mjcf_path))  # declared_sensors defaults empty

    record = cell.run_cycle(np.array([0.1, 0.1]), variant="vision_pick")

    assert record.outcome == "refusal"
    assert "sensor_missing" in record.reason
    assert "wrist_camera" in record.reason


def test_sensor_missing_refusal_clears_once_the_sensor_is_declared(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell(
        "cell_with_camera",
        str(two_joint_mjcf_path),
        declared_sensors=frozenset({"wrist_camera"}),
        capable_variants=frozenset({"default", "vision_pick"}),
    )

    record = cell.run_cycle(np.array([0.1, 0.1]), variant="vision_pick")

    assert record.outcome == "success"


def test_sensor_disappearing_between_cycles_flips_the_same_cell_to_refusal(two_joint_mjcf_path, monkeypatch):
    """Same cell, same variant, across two consecutive cycles: the first
    succeeds with the sensor declared, then the sensor 'drops out'
    (removed from declared_sensors, standing in for a runtime sensor
    failure — there's no real sensor hardware to fail here), and the
    very next cycle on that same cell refuses because of it."""
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell(
        "cell_sensor_dropout",
        str(two_joint_mjcf_path),
        declared_sensors=frozenset({"wrist_camera"}),
        capable_variants=frozenset({"default", "vision_pick"}),
    )

    working = cell.run_cycle(np.array([0.1, 0.1]), variant="vision_pick")
    assert working.outcome == "success"

    cell.declared_sensors = frozenset()  # the sensor goes away

    broken = cell.run_cycle(np.array([0.1, 0.1]), variant="vision_pick")
    assert broken.outcome == "refusal"
    assert "sensor_missing" in broken.reason
    assert "wrist_camera" in broken.reason
    assert broken.cycle_id != working.cycle_id


def test_capability_mismatch_refuses_a_variant_this_cell_cannot_support(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell(
        "cell_no_vision_capability",
        str(two_joint_mjcf_path),
        declared_sensors=frozenset({"wrist_camera"}),  # has the sensor...
        capable_variants=frozenset({"default"}),  # ...but isn't rated for the variant
    )

    record = cell.run_cycle(np.array([0.1, 0.1]), variant="vision_pick")

    assert record.outcome == "refusal"
    assert "capability_mismatch" in record.reason


def test_capability_mismatch_refuses_an_unknown_variant(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_unknown_variant", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([0.1, 0.1]), variant="not_a_real_variant")

    assert record.outcome == "refusal"
    assert "capability_mismatch" in record.reason


# --- done flag (DESIGN.md section 3) ---


def test_done_flag_true_after_successful_cycle(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_done_success", str(two_joint_mjcf_path))
    assert cell.done is False

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "success"
    assert cell.done is True


def test_done_flag_false_after_over_takt_failure(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=0.002, tolerance=0.001, kp=5.0))
    cell = Cell("cell_done_failure", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([1.0, 1.0]))

    assert record.outcome == "failure"
    assert cell.done is False


def test_done_flag_resets_false_at_the_start_of_each_new_cycle(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_done_reset", str(two_joint_mjcf_path))
    cell.run_cycle(np.array([0.1, 0.1]))
    assert cell.done is True

    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=0.002, tolerance=0.001, kp=5.0))
    cell.run_cycle(cell.current_qpos() + 1.0)

    assert cell.done is False  # a later over-takt cycle clears it again


def test_done_flag_stays_false_on_refusal(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_done_refusal", str(two_joint_mjcf_path))
    cell.obstruct(reason="test_obstruction")

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "refusal"
    assert cell.done is False


# --- Buffer, emulated in testing (no Line to own it yet) ---


def test_buffer_gates_whether_a_cell_attempts_a_cycle_across_ticks(two_joint_mjcf_path, monkeypatch):
    """No Line exists yet, but this is what its tick loop will do with a
    real upstream Buffer: only call run_cycle when the buffer actually
    holds a part (DESIGN.md section 1a's readiness rule). An empty
    buffer means starved — no cycle attempted at all, not even a
    refusal, since a refusal is a cell's own decision and starvation
    here is upstream's fault."""
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_line_emulation", str(two_joint_mjcf_path))

    upstream = Buffer(size=1)
    targets = [np.array([0.05, 0.05]), np.array([0.06, 0.06]), np.array([0.04, 0.04])]
    # Upstream produces a part before tick 1 and tick 3, but misses tick
    # 2 — with a size-1 buffer that miss starves the cell immediately,
    # matching DESIGN.md section 1a ("at size 1, there's no cushion").
    pushes_before_tick = {0: "part_1", 2: "part_3"}

    outcomes = []
    for tick, target in enumerate(targets):
        if tick in pushes_before_tick:
            upstream.push(pushes_before_tick[tick])
        part = upstream.pop()
        if part is None:
            outcomes.append("starved")
            continue
        record = cell.run_cycle(target, part_id=part)
        outcomes.append(record.outcome)

    assert outcomes == ["success", "starved", "success"]


# --- Logic fault (out-of-range target) and safety violation (velocity) ---


def test_logic_fault_when_target_exceeds_joint_range(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_logic_fault", str(two_joint_mjcf_path))

    # joint1 range is [-1, 1], joint2 range is [-2, 2] (see conftest) —
    # this asks for something clearly outside both, checked before any
    # physics runs.
    record = cell.run_cycle(np.array([50.0, 50.0]))

    assert record.outcome == "failure"
    assert record.reason.startswith("logic_fault")
    assert record.sim_steps == 0


def test_safety_violation_halts_the_cell_when_velocity_exceeds_limit(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.001, kp=1000.0))
    # An unreasonably tight limit guarantees the very first physics step
    # trips it, regardless of the fixture's actual dynamics.
    monkeypatch.setattr(hardware_limits, "max_joint_velocity", lambda robot_name: np.array([0.001, 0.001]))
    cell = Cell("cell_safety", str(two_joint_mjcf_path), robot_name="test_robot")

    record = cell.run_cycle(np.array([0.5, 0.5]))

    assert record.outcome == "failure"
    assert record.reason.startswith("safety_violation")
    assert cell.halted is True
    assert cell.halt_reason == record.reason

    # A safety violation is a hard stop (DESIGN.md section 5) — the cell
    # should refuse the next cycle exactly like an obstruction does,
    # until cleared.
    next_record = cell.run_cycle(np.array([0.1, 0.1]))
    assert next_record.outcome == "refusal"


def test_no_velocity_check_when_robot_name_is_unknown(two_joint_mjcf_path, monkeypatch):
    """robot_name defaults to None — hardware_limits has no entry to
    check against, so the velocity check should be skipped rather than
    guessed at, and a normal cycle should proceed as before."""
    monkeypatch.setattr(config_module, "resolve", fake_resolve(takt_s=2.0, tolerance=0.05, kp=5.0))
    cell = Cell("cell_no_robot_name", str(two_joint_mjcf_path))

    record = cell.run_cycle(np.array([0.1, 0.1]))

    assert record.outcome == "success"
