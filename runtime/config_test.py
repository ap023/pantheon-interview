from runtime import config


def test_resolve_returns_value_source_pairs():
    result = config.resolve("any_cell")
    assert result
    for entry in result.values():
        assert set(entry.keys()) == {"value", "source"}
        assert entry["source"] == "fleet_default"


def test_resolve_includes_expected_fleet_fields():
    result = config.resolve("cell_001")
    for field in ("takt_s", "position_tolerance_rad", "control_gain_kp"):
        assert field in result


def test_resolve_is_currently_cell_id_independent():
    # Phase 2 stub: site/per-unit merging isn't wired in yet, so every
    # cell_id resolves to the same fleet defaults. This should start
    # failing (correctly) once layering is implemented — see the TODO
    # in runtime/config.py and TODO.md.
    assert config.resolve("cell_a") == config.resolve("cell_b")
