import pytest

from runtime import task_input


@pytest.fixture(autouse=True)
def isolate_task_input_dir(tmp_path, monkeypatch):
    inbox = tmp_path / "task_input"
    inbox.mkdir()
    monkeypatch.setattr(task_input, "TASK_INPUT_DIR", inbox)
    return inbox


def test_empty_inbox_returns_none(isolate_task_input_dir):
    assert task_input.check_and_consume_task("cell_001") is None


def test_instruction_is_returned_and_consumed(isolate_task_input_dir):
    path = isolate_task_input_dir / "task_cell_001.json"
    path.write_text('{"target_qpos": [0.1, 0.2]}')

    instruction = task_input.check_and_consume_task("cell_001")

    assert instruction == {"target_qpos": [0.1, 0.2]}
    assert not path.exists()  # consumed, not left behind
    assert task_input.check_and_consume_task("cell_001") is None


def test_yaml_suffix_also_accepted(isolate_task_input_dir):
    (isolate_task_input_dir / "task_cell_001.yaml").write_text("target_qpos: [0.3, 0.4]")
    instruction = task_input.check_and_consume_task("cell_001")
    assert instruction == {"target_qpos": [0.3, 0.4]}


def test_only_matches_its_own_cell_id(isolate_task_input_dir):
    (isolate_task_input_dir / "task_cell_002.json").write_text('{"target_qpos": [1.0]}')
    assert task_input.check_and_consume_task("cell_001") is None
    assert task_input.check_and_consume_task("cell_002") == {"target_qpos": [1.0]}


def test_malformed_instruction_is_consumed_and_reported_not_raised(isolate_task_input_dir):
    path = isolate_task_input_dir / "task_cell_001.json"
    path.write_text("{not valid: yaml: [")

    instruction = task_input.check_and_consume_task("cell_001")

    assert "error" in instruction
    assert not path.exists()  # consumed anyway — a bad file must not wedge the inbox


def test_non_mapping_instruction_is_reported_as_error(isolate_task_input_dir):
    (isolate_task_input_dir / "task_cell_001.json").write_text("[1, 2, 3]")
    instruction = task_input.check_and_consume_task("cell_001")
    assert "error" in instruction
