from pathlib import Path

import pytest

from runtime import config


@pytest.fixture(autouse=True)
def isolate_config_dirs(tmp_path, monkeypatch):
    fleet_path = tmp_path / "fleet_defaults.yaml"
    fleet_path.write_text("takt_s: 4.0\nposition_tolerance_rad: 0.05\ncontrol_gain_kp: 4.0\n")
    site_dir = tmp_path / "site_overrides"
    site_dir.mkdir()
    per_unit_dir = tmp_path / "per_unit"
    per_unit_dir.mkdir()

    monkeypatch.setattr(config, "FLEET_DEFAULTS_PATH", fleet_path)
    monkeypatch.setattr(config, "SITE_OVERRIDES_DIR", site_dir)
    monkeypatch.setattr(config, "PER_UNIT_DIR", per_unit_dir)
    return {"fleet": fleet_path, "site": site_dir, "per_unit": per_unit_dir}


def write_yaml(path: Path, **fields) -> None:
    path.write_text("\n".join(f"{k}: {v}" for k, v in fields.items()) + "\n")


def test_resolve_returns_value_source_pairs(isolate_config_dirs):
    result = config.resolve("any_cell")
    assert result
    for entry in result.values():
        assert set(entry.keys()) == {"value", "source"}


def test_resolve_includes_expected_fleet_fields(isolate_config_dirs):
    result = config.resolve("cell_001")
    for field in ("takt_s", "position_tolerance_rad", "control_gain_kp"):
        assert field in result
        assert result[field]["source"] == "fleet_default"


def test_no_site_or_per_unit_file_falls_back_to_fleet_defaults(isolate_config_dirs):
    result = config.resolve("cell_no_overrides", site_id="site_with_no_file")
    assert result["takt_s"] == {"value": 4.0, "source": "fleet_default"}


def test_site_override_applies_only_the_fields_it_sets(isolate_config_dirs):
    write_yaml(isolate_config_dirs["site"] / "site_a.yaml", takt_s=5.0)

    result = config.resolve("cell_001", site_id="site_a")

    assert result["takt_s"] == {"value": 5.0, "source": "site_override"}
    # untouched fields still fall through to fleet defaults
    assert result["position_tolerance_rad"] == {"value": 0.05, "source": "fleet_default"}


def test_site_override_does_not_apply_without_matching_site_id(isolate_config_dirs):
    write_yaml(isolate_config_dirs["site"] / "site_a.yaml", takt_s=5.0)

    result_no_site = config.resolve("cell_001")
    result_other_site = config.resolve("cell_001", site_id="site_b")

    assert result_no_site["takt_s"] == {"value": 4.0, "source": "fleet_default"}
    assert result_other_site["takt_s"] == {"value": 4.0, "source": "fleet_default"}


def test_per_unit_correction_applies_only_to_its_own_cell_id(isolate_config_dirs):
    write_yaml(isolate_config_dirs["per_unit"] / "cell_001.yaml", control_gain_kp=9.0)

    result_cell_001 = config.resolve("cell_001")
    result_cell_002 = config.resolve("cell_002")

    assert result_cell_001["control_gain_kp"] == {"value": 9.0, "source": "per_unit_correction"}
    assert result_cell_002["control_gain_kp"] == {"value": 4.0, "source": "fleet_default"}


def test_per_unit_correction_wins_over_site_override_on_the_same_field(isolate_config_dirs):
    write_yaml(isolate_config_dirs["site"] / "site_a.yaml", takt_s=5.0)
    write_yaml(isolate_config_dirs["per_unit"] / "cell_001.yaml", takt_s=6.0)

    result = config.resolve("cell_001", site_id="site_a")

    assert result["takt_s"] == {"value": 6.0, "source": "per_unit_correction"}


def test_per_unit_correction_does_not_blow_away_site_override_on_a_different_field(isolate_config_dirs):
    write_yaml(isolate_config_dirs["site"] / "site_a.yaml", takt_s=5.0)
    write_yaml(isolate_config_dirs["per_unit"] / "cell_001.yaml", control_gain_kp=9.0)

    result = config.resolve("cell_001", site_id="site_a")

    assert result["takt_s"] == {"value": 5.0, "source": "site_override"}
    assert result["control_gain_kp"] == {"value": 9.0, "source": "per_unit_correction"}


def test_tie_value_still_reports_the_higher_precedence_source(isolate_config_dirs):
    """DESIGN.md section 2: if a site override and a per-unit correction
    set the same field to the same value, provenance still reports the
    higher layer — it reports which layer's decision governs, not
    whether the numbers happen to coincide."""
    write_yaml(isolate_config_dirs["site"] / "site_a.yaml", takt_s=5.0)
    write_yaml(isolate_config_dirs["per_unit"] / "cell_001.yaml", takt_s=5.0)  # same value

    result = config.resolve("cell_001", site_id="site_a")

    assert result["takt_s"] == {"value": 5.0, "source": "per_unit_correction"}


def test_resolve_is_no_longer_cell_id_independent(isolate_config_dirs):
    write_yaml(isolate_config_dirs["per_unit"] / "cell_a.yaml", takt_s=99.0)

    assert config.resolve("cell_a") != config.resolve("cell_b")
    assert config.resolve("cell_b") == {
        field: {"value": value, "source": "fleet_default"}
        for field, value in {"takt_s": 4.0, "position_tolerance_rad": 0.05, "control_gain_kp": 4.0}.items()
    }
