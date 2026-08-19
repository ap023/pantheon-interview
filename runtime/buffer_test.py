import pytest

from runtime.buffer import Buffer


def test_new_buffer_is_starved_and_not_blocked():
    buf = Buffer(size=2)
    assert buf.starved is True
    assert buf.blocked is False
    assert buf.occupancy == 0


def test_push_until_full_then_blocked():
    buf = Buffer(size=2)
    assert buf.push("part_1") is True
    assert buf.blocked is False
    assert buf.push("part_2") is True
    assert buf.blocked is True
    assert buf.push("part_3") is False  # rejected, buffer stays full
    assert buf.occupancy == 2


def test_pop_returns_items_fifo_and_empties_to_starved():
    buf = Buffer(size=2)
    buf.push("part_1")
    buf.push("part_2")

    assert buf.pop() == "part_1"
    assert buf.starved is False
    assert buf.pop() == "part_2"
    assert buf.starved is True
    assert buf.pop() is None


def test_size_one_buffer_has_zero_tolerance_for_a_missed_tick():
    # DESIGN.md section 1a: at size 1 there's no cushion — starved again
    # immediately on the very next tick, nothing banked to cover a gap.
    buf = Buffer(size=1)
    buf.push("part_1")
    assert buf.blocked is True

    assert buf.pop() == "part_1"
    assert buf.starved is True


def test_buffer_size_must_be_positive():
    with pytest.raises(ValueError):
        Buffer(size=0)
    with pytest.raises(ValueError):
        Buffer(size=-1)
