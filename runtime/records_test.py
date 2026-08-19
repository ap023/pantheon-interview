import json

import pytest

from runtime import records as records_module
from runtime.records import CycleRecord, hash_config, write_cycle_record


@pytest.fixture(autouse=True)
def isolate_records_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(records_module, "RECORDS_DIR", tmp_path / "records")
    monkeypatch.setattr(records_module, "CONFIG_SNAPSHOTS_DIR", tmp_path / "records" / "config_snapshots")


def make_record(cell_id="cell_x", cycle_id="cycle-1", outcome="success", config=None):
    return CycleRecord(
        cell_id=cell_id,
        cycle_id=cycle_id,
        outcome=outcome,
        duration_s=1.0,
        takt_s=4.0,
        config=config or {"takt_s": {"value": 4.0, "source": "fleet_default"}},
    )


def test_hash_config_is_deterministic_and_key_order_independent():
    assert hash_config({"x": 1, "y": 2}) == hash_config({"y": 2, "x": 1})


def test_hash_config_differs_for_different_config():
    assert hash_config({"x": 1}) != hash_config({"x": 2})


def test_write_cycle_record_creates_file_referencing_config_by_hash():
    record = make_record()
    path = write_cycle_record(record)

    assert path.exists()
    data = json.loads(path.read_text())
    assert "config" not in data
    assert data["config_hash"] == hash_config(record.config)
    assert data["outcome"] == "success"
    assert data["cell_id"] == "cell_x"


def test_write_cycle_record_writes_config_snapshot_matching_the_hash():
    record = make_record()
    write_cycle_record(record)

    snapshot_path = records_module.CONFIG_SNAPSHOTS_DIR / f"{hash_config(record.config)}.json"
    assert snapshot_path.exists()
    assert json.loads(snapshot_path.read_text()) == record.config


def test_identical_configs_across_cells_dedupe_to_one_snapshot():
    config = {"takt_s": {"value": 4.0, "source": "fleet_default"}}
    write_cycle_record(make_record(cell_id="cell_a", cycle_id="c1", config=config))
    write_cycle_record(make_record(cell_id="cell_b", cycle_id="c2", config=config))

    snapshots = list(records_module.CONFIG_SNAPSHOTS_DIR.glob("*.json"))
    assert len(snapshots) == 1


def test_different_configs_produce_different_snapshots():
    write_cycle_record(make_record(cycle_id="c1", config={"takt_s": {"value": 4.0, "source": "fleet_default"}}))
    write_cycle_record(make_record(cycle_id="c2", config={"takt_s": {"value": 5.0, "source": "fleet_default"}}))

    snapshots = list(records_module.CONFIG_SNAPSHOTS_DIR.glob("*.json"))
    assert len(snapshots) == 2
