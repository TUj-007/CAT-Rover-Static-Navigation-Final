from dataclasses import dataclass, field
import numpy as np


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def array(self):
        return np.array([self.x, self.y, self.yaw], dtype=float)


@dataclass(frozen=True)
class Telemetry:
    ms: int
    epoch: int
    mode: int
    fault: int
    counts: tuple
    received: float


@dataclass(frozen=True)
class Scan:
    points: np.ndarray
    angles: np.ndarray
    acquired: float
    completed: float
    generation: int = 0


@dataclass(frozen=True)
class MatchResult:
    pose: Pose
    accepted: bool
    score: float
    overlap: float
    reason: str


@dataclass
class RunResult:
    status: str
    reason: str
    stations_reached: int = 0
    coverage: dict = field(default_factory=dict)
