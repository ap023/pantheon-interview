"""Buffer: the queue between two stations (DESIGN.md section 1a).

Owned by the Line, not by either neighboring Cell — a Cell only ever
reads its own upstream/downstream buffer's occupancy (starved / blocked),
never owns one. No Line exists yet to actually wire these in, so this is
built standalone: directly instantiable now to emulate what the Line
will eventually hand each Cell (useful in tests), and pluggable into the
Line without changes once that exists — its size would come from
line_config.py's buffer_size field, but Buffer itself doesn't know
about config at all.
"""
from collections import deque
from typing import Any, Deque, Optional


class Buffer:
    def __init__(self, size: int):
        if size <= 0:
            raise ValueError(f"buffer size must be > 0, got {size}")
        self.size = size
        self._queue: Deque[Any] = deque()

    @property
    def occupancy(self) -> int:
        return len(self._queue)

    @property
    def contents(self) -> tuple:
        """Snapshot of the queued items, upstream-first. Read-only view
        for status displays (line_runner's board) — mutation still goes
        through push/pop only."""
        return tuple(self._queue)

    @property
    def starved(self) -> bool:
        """No part available — a cell reading this as its upstream
        buffer can't run a cycle (DESIGN.md section 1a readiness rule)."""
        return self.occupancy == 0

    @property
    def blocked(self) -> bool:
        """No room for another part — a cell reading this as its
        downstream buffer can't place its output."""
        return self.occupancy >= self.size

    def push(self, item: Any) -> bool:
        """Place an item into the buffer. Returns False and adds nothing
        if the buffer is already blocked (full)."""
        if self.blocked:
            return False
        self._queue.append(item)
        return True

    def pop(self) -> Optional[Any]:
        """Take the next item out of the buffer, or None if starved
        (empty)."""
        if self.starved:
            return None
        return self._queue.popleft()

    def peek(self) -> Optional[Any]:
        """Look at the next item without removing it, or None if starved
        (empty). Lets a caller attempt something against the item first
        and only actually pop() it once that attempt succeeds — an
        upstream buffer shouldn't lose an item to a cycle that failed or
        was refused."""
        if self.starved:
            return None
        return self._queue[0]
