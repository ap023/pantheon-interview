import pytest

from runtime import commands


@pytest.fixture(autouse=True)
def isolate_commands_dir(tmp_path, monkeypatch):
    commands_dir = tmp_path / "commands"
    commands_dir.mkdir()
    monkeypatch.setattr(commands, "COMMANDS_DIR", commands_dir)
    return commands_dir


def test_no_kill_file_returns_false(isolate_commands_dir):
    assert commands.check_and_consume_kill("cell_001") is False


def test_kill_file_present_returns_true_and_is_consumed(isolate_commands_dir):
    kill_path = isolate_commands_dir / "kill_cell_001.json"
    kill_path.write_text("{}")

    assert commands.check_and_consume_kill("cell_001") is True
    assert not kill_path.exists()  # consumed, not left behind

    # Second check finds nothing — it was a one-shot command, not a
    # standing state.
    assert commands.check_and_consume_kill("cell_001") is False


def test_kill_file_only_matches_its_own_cell_id(isolate_commands_dir):
    (isolate_commands_dir / "kill_cell_002.json").write_text("{}")

    assert commands.check_and_consume_kill("cell_001") is False
    assert commands.check_and_consume_kill("cell_002") is True


def test_clear_failure_and_obstruct_commands_are_consumed_independently(isolate_commands_dir):
    (isolate_commands_dir / "clear_failure_cell_001.json").write_text("{}")
    (isolate_commands_dir / "obstruct_cell_001.json").write_text("{}")

    # Each command name only matches its own file — no cross-talk.
    assert commands.check_and_consume_kill("cell_001") is False
    assert commands.check_and_consume_clear_failure("cell_001") is True
    assert commands.check_and_consume_clear_failure("cell_001") is False  # consumed
    assert commands.check_and_consume_obstruct("cell_001") is True
    assert commands.check_and_consume_obstruct("cell_001") is False  # consumed
