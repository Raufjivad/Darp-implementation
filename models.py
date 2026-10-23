from dataclasses import dataclass

FREE = -1
OBSTACLE = -2
Cell = tuple[int, int]
Grid = list[list[int]]


@dataclass(frozen=True)
class WorldSetup:
    rows: int
    cols: int
    robot_positions: tuple[Cell, ...]
    obstacle_positions: tuple[Cell, ...]
    grid_env: Grid


@dataclass(frozen=True)
class AssignmentMatrixSetup:
    MetricMatrix: list[list[list[float]]]
    TilesImportance: list[list[list[float]]]
    DesireableAssign: tuple[float, ...]
    termThr: int
    Notiles: int
    MinimumImportance: tuple[float, ...]
    MaximumImportance: tuple[float, ...]