from __future__ import annotations

from functools import lru_cache


class IDGenerator:
    def __init__(self):
        self._counter = 0

    @lru_cache(maxsize=None)
    def __call__(self, obj):
        self._counter += 1
        return self._counter

