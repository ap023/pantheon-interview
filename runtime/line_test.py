import numpy as np
import pytest

from runtime import config as config_module
from runtime import line_config as line_config_module
from runtime import records as records_module
from runtime.cell import Cell
from runtime.line import (
    AutomaticTickTrigger,
    Line,
    LineTopologyError,
    ManualTickTrigger,
    Part,
)


def fake_cell_config(takt_s=2.0, tolerance=0.05, kp=5.0):
    def _resolve(cell_id, site_id=None):
        return {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": tolerance, "source": "fleet_default"},
            "control_gain_kp": {"value": kp, "source": "fleet_default"},
        }

    return _resolve


def fake_line_config(buffer_size=1):
    def _resolve(edge_id=None):
        return {"buffer_size": {"value": buffer_size, "source": "line_default"}}

    return _resolve


def noop_trigger():
    return ManualTickTrigger(input_fn=lambda prompt: "")


@pytest.fixture(autouse=True)
def isolate_records_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(records_module, "RECORDS_DIR", tmp_path / "records")
    monkeypatch.setattr(records_module, "CONFIG_SNAPSHOTS_DIR", tmp_path / "records" / "config_snapshots")
    monkeypatch.setattr(records_module, "RUNS_DIR", tmp_path / "records" / "runs")


@pytest.fixture(autouse=True)
def default_configs(monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_cell_config())
    monkeypatch.setattr(line_config_module, "resolve", fake_line_config())


def make_cell(cell_id, mjcf_path, **kwargs):
    return Cell(cell_id, str(mjcf_path), **kwargs)


# --- Preflight topology validator (DESIGN.md section 1a) ---


def test_line_needs_at_least_one_cell():
    with pytest.raises(LineTopologyError):
        Line(cells=[], trigger=noop_trigger())


def test_zero_size_buffer_fails_at_construction_not_partway_through(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(line_config_module, "resolve", fake_line_config(buffer_size=0))
    cell = make_cell("cell_a", two_joint_mjcf_path)

    with pytest.raises(LineTopologyError):
        Line(cells=[cell], trigger=noop_trigger())


def test_valid_topology_builds_one_more_buffer_than_cells(two_joint_mjcf_path):
    cells = [make_cell("cell_a", two_joint_mjcf_path), make_cell("cell_b", two_joint_mjcf_path)]
    line = Line(cells=cells, trigger=noop_trigger())

    assert len(line.buffers) == len(cells) + 1
    assert line.sink is line.buffers[-1]


# --- Tick triggers (DESIGN.md section 1c) ---


def test_manual_trigger_blocks_on_the_configured_input_fn():
    calls = []
    trigger = ManualTickTrigger(prompt="advance?", input_fn=lambda prompt: calls.append(prompt))

    trigger.wait()

    assert calls == ["advance?"]


def test_automatic_trigger_sleeps_for_the_configured_interval():
    calls = []
    trigger = AutomaticTickTrigger(interval_s=4.0, sleep_fn=lambda s: calls.append(s))

    trigger.wait()

    assert calls == [4.0]


def test_automatic_trigger_rejects_nonpositive_interval():
    with pytest.raises(ValueError):
        AutomaticTickTrigger(interval_s=0)


# --- Starved / blocked readiness gating (DESIGN.md section 1a) ---


def test_tick_indexes_every_ran_cell_into_the_run_directory(two_joint_mjcf_path):
    """Each cycle a cell actually attempts gets symlinked under
    records/runs/{line.run_id}/ (records.py's link_into_run) — proving
    Line.tick() wires this up for real, not just that the helper works
    in isolation (records_test.py already covers that)."""
    cell_a = make_cell("cell_a", two_joint_mjcf_path)
    cell_b = make_cell("cell_b", two_joint_mjcf_path)
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    line.tick()  # cell_a runs, cell_b starves (not indexed — never attempted)
    line.tick()  # cell_b runs too now

    run_dir = records_module.RUNS_DIR / line.run_id
    names = sorted(p.name for p in run_dir.iterdir())
    assert names == [
        "tick_0001_cell_a_success.json",
        "tick_0002_cell_a_success.json",
        "tick_0002_cell_b_success.json",
    ]


def test_single_cell_line_runs_every_tick_once_parts_are_flowing(two_joint_mjcf_path):
    # A cell is only starved relative to *its own* upstream buffer at the
    # instant of the tick snapshot — the Line releases a part into the
    # source buffer as part of the same tick, so cells[0] should actually
    # run, not starve, on tick 1. This pins down that release happens
    # before the snapshot is taken.
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())

    results = line.tick()

    assert len(results) == 1
    assert results[0].ran is True
    assert results[0].starved is False
    assert results[0].outcome == "success"


def test_downstream_cell_starves_until_upstream_completes_a_cycle(two_joint_mjcf_path):
    cell_a = make_cell("cell_a", two_joint_mjcf_path)
    cell_b = make_cell("cell_b", two_joint_mjcf_path)
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    # Tick 1: a part is released into buffers[0] and consumed by cell_a
    # this same tick; buffers[1] (cell_b's upstream) is still empty at
    # the snapshot taken before cell_a runs, so cell_b starves.
    results = line.tick()
    a_result, b_result = results
    assert a_result.ran is True
    assert a_result.outcome == "success"
    assert b_result.ran is False
    assert b_result.starved is True

    # Tick 2: buffers[1] now holds the part cell_a placed at the end of
    # tick 1, so cell_b runs this time.
    results = line.tick()
    a_result, b_result = results
    assert b_result.ran is True
    assert b_result.outcome == "success"


def test_upstream_cell_does_not_block_when_downstream_drains_the_same_tick(two_joint_mjcf_path):
    """cell_b runs downstream-first within the same tick (see line.py's
    tick() docstring), so if cell_b successfully pops buffers[1] this
    tick, cell_a is free to push its own new part into that now-empty
    slot in that same tick — no artificial one-tick "ping-pong" bubble
    just because the buffer's capacity is 1."""
    cell_a = make_cell("cell_a", two_joint_mjcf_path)
    cell_b = make_cell("cell_b", two_joint_mjcf_path)
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    line.tick()  # cell_a produces part_000 into buffers[1]; cell_b starves (buffer size 1)
    results = line.tick()  # cell_b drains part_000 -> cell_a is unblocked, produces part_001
    a_result, b_result = results
    assert b_result.ran is True and b_result.outcome == "success"
    assert a_result.ran is True and a_result.outcome == "success"
    assert a_result.blocked is False


def test_upstream_cell_blocks_when_downstream_fails_to_drain_this_tick(two_joint_mjcf_path, monkeypatch):
    """The flip side: if cell_b's own cycle does NOT complete this tick
    (a config that guarantees over_takt), it never pops buffers[1] — so
    cell_a genuinely is blocked, exactly as before, just now provably
    tied to cell_b's *actual* outcome this tick rather than a timing
    artifact of read order."""
    cell_a = make_cell("cell_a", two_joint_mjcf_path)
    cell_b = make_cell("cell_b", two_joint_mjcf_path)

    def resolve_per_cell(cell_id, site_id=None):
        # cell_b gets an impossible takt (1 physics step) so it can
        # never converge; cell_a gets a normal one.
        takt_s = 0.002 if cell_id == "cell_b" else 2.0
        return {
            "takt_s": {"value": takt_s, "source": "fleet_default"},
            "position_tolerance_rad": {"value": 0.05, "source": "fleet_default"},
            "control_gain_kp": {"value": 5.0, "source": "fleet_default"},
        }

    monkeypatch.setattr(config_module, "resolve", resolve_per_cell)
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    line.tick()  # cell_a produces part_000 into buffers[1]; cell_b starves
    results = line.tick()  # cell_b attempts part_000, over-takts, never pops
    a_result, b_result = results
    assert b_result.ran is True and b_result.outcome == "failure" and b_result.reason == "over_takt"
    assert a_result.ran is False
    assert a_result.blocked is True


# --- Part release and variant changeover (DESIGN.md section 1) ---


def test_released_parts_get_sequential_ids(two_joint_mjcf_path):
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())

    line.tick()
    line.tick()

    assert line._next_part_seq == 2


def test_release_returns_none_and_holds_the_counter_when_source_buffer_is_full(two_joint_mjcf_path):
    # A part only leaves the upstream buffer once its cycle actually
    # completes (peek, not pop, until done) — so the only way the source
    # buffer is full at release time in a fresh line is if something else
    # already occupies it. Simulate that directly rather than contriving
    # it through cell dynamics.
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())
    line.buffers[0].push(Part(id="stray_part", variant="default"))

    released = line._release_part()

    assert released is None
    assert line._next_part_seq == 0


def test_set_variant_rejects_unknown_variant(two_joint_mjcf_path):
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())

    with pytest.raises(KeyError):
        line.set_variant("not_a_real_variant")

    assert line.current_variant == "default"


def test_changeover_only_affects_parts_released_after_it(two_joint_mjcf_path):
    cell_a = make_cell("cell_a", two_joint_mjcf_path, capable_variants=frozenset({"default", "vision_pick"}))
    # No declared_sensors: cell_b can succeed under "default" but would
    # refuse under "vision_pick" (missing wrist_camera) — this is what
    # lets the test tell which variant the part it receives is actually
    # tagged with, rather than both variants trivially succeeding.
    cell_b = make_cell("cell_b", two_joint_mjcf_path, capable_variants=frozenset({"default", "vision_pick"}))
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    line.tick()  # part_000 released under "default"; cell_a consumes it, places it in buffers[1]
    line.set_variant("vision_pick")
    # part_001 releases under "vision_pick" but buffers[1] is still full
    # of part_000, so cell_a is blocked and doesn't run this tick; that
    # leaves cell_b free to pick up part_000 — still tagged "default".
    results = line.tick()

    b_result = results[1]
    assert b_result.ran is True
    assert b_result.outcome == "success"  # would have refused (sensor_missing) had it been tagged "vision_pick"


# --- over-takt read at the tick boundary (DESIGN.md section 3 / TODO.md) ---


def test_over_takt_is_read_from_done_at_the_tick_boundary(two_joint_mjcf_path, monkeypatch):
    monkeypatch.setattr(config_module, "resolve", fake_cell_config(takt_s=0.002, tolerance=1e-9, kp=5.0))
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())

    results = line.tick()

    assert results[0].outcome == "failure"
    assert results[0].over_takt is True
    assert cell.done is False


def test_refusal_is_not_reported_as_over_takt(two_joint_mjcf_path):
    cell = make_cell("cell_a", two_joint_mjcf_path)
    cell.obstruct(reason="test_obstruction")
    line = Line(cells=[cell], trigger=noop_trigger())

    results = line.tick()

    assert results[0].outcome == "refusal"
    assert results[0].over_takt is False


# --- a stuck cell holds its part for retry instead of scrapping it ---


def test_refused_part_is_held_for_retry_not_scrapped(two_joint_mjcf_path):
    cell = make_cell("cell_a", two_joint_mjcf_path)
    cell.obstruct(reason="stuck")
    line = Line(cells=[cell], trigger=noop_trigger())

    first = line.tick()  # part_000 released, then immediately refused
    assert first[0].outcome == "refusal"
    assert line.buffers[0].occupancy == 1  # still there, not scrapped

    # Source stays blocked the whole time the stuck part occupies
    # buffers[0] (size 1) — no new part is ever released underneath it.
    second = line.tick()
    assert second[0].outcome == "refusal"
    assert line.buffers[0].occupancy == 1
    assert line._next_part_seq == 1  # tick1's release succeeded; every one since has been blocked

    cell.clear_failure()
    third = line.tick()

    assert third[0].outcome == "success"
    assert line.buffers[0].occupancy == 0  # the same part_000, finally consumed
    assert line._next_part_seq == 1  # release stayed blocked on every tick after the first


def test_stuck_upstream_cell_starves_downstream_until_cleared(two_joint_mjcf_path):
    cell_a = make_cell("cell_a", two_joint_mjcf_path)
    cell_b = make_cell("cell_b", two_joint_mjcf_path)
    cell_a.obstruct(reason="stuck")
    line = Line(cells=[cell_a, cell_b], trigger=noop_trigger())

    for _ in range(3):
        a_result, b_result = line.tick()
        assert a_result.outcome == "refusal"
        assert b_result.ran is False
        assert b_result.starved is True

    cell_a.clear_failure()
    a_result, b_result = line.tick()

    assert a_result.outcome == "success"  # cell_a recovers with the same held part
    assert b_result.starved is True  # still starved this same tick — one-tick lag to flow downstream


# --- tracing a specific part's identity through the buffers, not just counts ---


def test_a_single_part_advances_exactly_one_buffer_per_tick(two_joint_mjcf_path):
    """Not just occupancy counts: the same part_id should show up one
    buffer further along each tick it's actually consumed — proof the
    Line is moving the *same* item forward, not creating/dropping parts
    that happen to keep the counts looking right."""
    cells = [make_cell(f"cell_{i}", two_joint_mjcf_path) for i in range(3)]
    line = Line(cells=cells, trigger=noop_trigger())

    def location_of(part_id):
        for index, buf in enumerate(line.buffers):
            if not buf.starved and buf.peek().id == part_id:
                return index
        return None

    line.tick()
    assert location_of("part_000000") == 1  # released this tick, consumed by cell_0 same tick

    line.tick()
    assert location_of("part_000000") == 2  # advanced by cell_1

    line.tick()
    assert location_of("part_000000") == 3  # advanced by cell_2 — reached the sink


def test_full_buffer_snapshot_across_three_ticks_of_a_three_cell_line(two_joint_mjcf_path):
    """Steady-state pipeline flow at buffer_size=1: every tick, every
    part in flight advances exactly one buffer, and cell_0 runs EVERY
    tick (never blocked) because downstream cells are processed first
    within the same tick and free up room before cell_0's own blocked
    check runs (see line.py's tick() docstring) — no one-tick "ping-
    pong" bubble."""
    cells = [make_cell(f"cell_{i}", two_joint_mjcf_path) for i in range(3)]
    line = Line(cells=cells, trigger=noop_trigger())

    def snapshot():
        return [None if buf.starved else buf.peek().id for buf in line.buffers]

    line.tick()
    assert snapshot() == [None, "part_000000", None, None]

    line.tick()
    # cell_1 drains part_000 into buffers[2] first (processed
    # downstream-first); that frees buffers[1] in time for cell_0 to
    # advance part_001 into it in this SAME tick — full throughput, not
    # a bubble.
    assert snapshot() == [None, "part_000001", "part_000000", None]

    line.tick()
    # Same steady shift again: every part in flight moves forward
    # exactly one buffer, cell_0 releases and advances part_002.
    assert snapshot() == [None, "part_000002", "part_000001", "part_000000"]


def test_sink_drains_into_completed_at_the_next_tick_boundary(two_joint_mjcf_path):
    """The sink is the completion stage, not a holding buffer: a part
    that reached it is collected into line.completed at the next tick's
    start, so the last cell never blocks on finished goods. Without the
    drain, size-1 buffers deadlock the whole line permanently after the
    first completed part."""
    cells = [make_cell("cell_a", two_joint_mjcf_path), make_cell("cell_b", two_joint_mjcf_path)]
    line = Line(cells=cells, trigger=noop_trigger())

    line.tick()  # part_000 through cell_a
    line.tick()  # part_000 through cell_b -> sink
    assert [p.id for p in line.sink.contents] == ["part_000000"]
    assert line.completed == []

    line.tick()
    assert [p.id for p in line.completed] == ["part_000000"]
    assert line.sink.starved or line.sink.peek().id != "part_000000"

    # Steady state: the line keeps flowing instead of wedging — every
    # released part eventually lands in completed, ids unique and in
    # order.
    for _ in range(8):
        line.tick()
    completed_ids = [p.id for p in line.completed]
    assert len(completed_ids) == len(set(completed_ids))  # no duplicates
    assert completed_ids == sorted(completed_ids)  # completion follows release order
    assert len(completed_ids) >= 4  # still flowing, not deadlocked


# --- run_shift ---


def test_run_shift_with_explicit_tick_count_returns_one_result_list_per_tick(two_joint_mjcf_path):
    cell = make_cell("cell_a", two_joint_mjcf_path)
    line = Line(cells=[cell], trigger=noop_trigger())

    history = line.run_shift(n_ticks=3)

    assert len(history) == 3
    assert line.tick_count == 3
