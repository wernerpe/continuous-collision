"""Certified continuous collision checking for Drake, on stock pydrake.

A pure-Python port of drake::planning::continuous_collision::ContinuousCollisionChecker
(Drake PR #24924) with the same API, bounds, verdicts and resolution contract.
"""

from .checker import ContinuousCollisionChecker, Finding, Options, Result, Verdict

__all__ = ["ContinuousCollisionChecker", "Finding", "Options", "Result", "Verdict"]
__version__ = "0.1.0"
