from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from math import inf
from typing import Iterable, Sequence


Cell = tuple[int, int]
Grid = list[list[int]]

OBSTACLE = -2
UNASSIGNED = -1


@dataclass(frozen=True)
class DivisionResult:
    """Result of an initial-position based DARP-style area division."""

    assignment: Grid
    quotas: tuple[int, ...]
    counts: tuple[int, ...]


def divide_areas_by_initial_positions(
    rows: int,
    cols: int,
    robot_positions: Sequence[Cell],
    obstacles: Iterable[Cell] = (),
) -> DivisionResult:
    """Divide a grid into equal connected areas seeded by robot positions.

    This is a compact reimplementation of the DARP initialization ideas:
    each robot owns its initial cell, passable space is split into equal quotas,
    and areas are grown from robot seeds through four-neighbor connectivity.

    The returned assignment uses robot indices 0..n-1 and OBSTACLE for blocked
    cells. If the free space is disconnected or a robot is invalid, ValueError is
    raised instead of silently returning disconnected robot areas.
    """

    _validate_dimensions(rows, cols)
    robots = tuple(robot_positions)
    obstacle_set = set(obstacles)
    _validate_inputs(rows, cols, robots, obstacle_set)

    passable = rows * cols - len(obstacle_set)
    quotas = _equal_quotas(passable, len(robots))
    assignment = [[UNASSIGNED for _ in range(cols)] for _ in range(rows)]

    for r, c in obstacle_set:
        assignment[r][c] = OBSTACLE

    for robot_id, (r, c) in enumerate(robots):
        assignment[r][c] = robot_id

    _validate_free_space_connected(rows, cols, obstacle_set, robots[0])
    distances = [_bfs_distances(rows, cols, obstacle_set, start) for start in robots]

    counts = [1 for _ in robots]
    frontier: list[tuple[int, int, int, int, int]] = []
    for robot_id, start in enumerate(robots):
        _push_unassigned_neighbors(
            frontier,
            rows,
            cols,
            assignment,
            distances[robot_id],
            robot_id,
            start,
        )

    remaining = passable - len(robots)
    while remaining > 0 and frontier:
        distance, row, col, robot_id, _ = heappop(frontier)
        if assignment[row][col] != UNASSIGNED:
            continue
        if counts[robot_id] >= quotas[robot_id]:
            continue
        if distance == inf:
            continue

        assignment[row][col] = robot_id
        counts[robot_id] += 1
        remaining -= 1
        _push_unassigned_neighbors(
            frontier,
            rows,
            cols,
            assignment,
            distances[robot_id],
            robot_id,
            (row, col),
        )

    if remaining:
        raise ValueError("could not build connected areas that satisfy all quotas")

    if tuple(counts) != quotas:
        raise ValueError(f"quota mismatch: expected {quotas}, got {tuple(counts)}")
    if not all(_is_robot_area_connected(assignment, robot_id, robots[robot_id]) for robot_id in range(len(robots))):
        raise ValueError("division produced a disconnected robot area")

    return DivisionResult(assignment=assignment, quotas=quotas, counts=tuple(counts))


def _equal_quotas(total_passable_cells: int, robot_count: int) -> tuple[int, ...]:
    if robot_count <= 0:
        raise ValueError("at least one robot is required")
    if total_passable_cells < robot_count:
        raise ValueError("there must be at least one passable cell per robot")

    base, extra = divmod(total_passable_cells, robot_count)
    return tuple(base + (1 if robot_id < extra else 0) for robot_id in range(robot_count))


def _bfs_distances(rows: int, cols: int, obstacles: set[Cell], start: Cell) -> list[list[float]]:
    distances = [[inf for _ in range(cols)] for _ in range(rows)]
    distances[start[0]][start[1]] = 0
    queue: deque[Cell] = deque([start])

    while queue:
        row, col = queue.popleft()
        for next_row, next_col in _neighbors(rows, cols, row, col):
            if (next_row, next_col) in obstacles:
                continue
            if distances[next_row][next_col] != inf:
                continue
            distances[next_row][next_col] = distances[row][col] + 1
            queue.append((next_row, next_col))

    return distances


def _push_unassigned_neighbors(
    frontier: list[tuple[int, int, int, int, int]],
    rows: int,
    cols: int,
    assignment: Grid,
    distances: list[list[float]],
    robot_id: int,
    cell: Cell,
) -> None:
    for row, col in _neighbors(rows, cols, cell[0], cell[1]):
        if assignment[row][col] == UNASSIGNED:
            heappush(frontier, (distances[row][col], row, col, robot_id, len(frontier)))


def _validate_free_space_connected(rows: int, cols: int, obstacles: set[Cell], start: Cell) -> None:
    seen = _reachable_cells(rows, cols, obstacles, start)
    passable = rows * cols - len(obstacles)
    if len(seen) != passable:
        raise ValueError("passable grid cells must form one connected component")


def _is_robot_area_connected(assignment: Grid, robot_id: int, start: Cell) -> bool:
    rows = len(assignment)
    cols = len(assignment[0]) if rows else 0
    expected = {
        (row, col)
        for row in range(rows)
        for col in range(cols)
        if assignment[row][col] == robot_id
    }
    if not expected:
        return False
    return _reachable_cells(rows, cols, set(), start, allowed=expected) == expected


def _reachable_cells(
    rows: int,
    cols: int,
    obstacles: set[Cell],
    start: Cell,
    allowed: set[Cell] | None = None,
) -> set[Cell]:
    seen = {start}
    queue: deque[Cell] = deque([start])

    while queue:
        row, col = queue.popleft()
        for next_cell in _neighbors(rows, cols, row, col):
            if next_cell in seen or next_cell in obstacles:
                continue
            if allowed is not None and next_cell not in allowed:
                continue
            seen.add(next_cell)
            queue.append(next_cell)

    return seen


def _neighbors(rows: int, cols: int, row: int, col: int) -> Iterable[Cell]:
    for next_row, next_col in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
        if 0 <= next_row < rows and 0 <= next_col < cols:
            yield next_row, next_col


def _validate_dimensions(rows: int, cols: int) -> None:
    if rows <= 0 or cols <= 0:
        raise ValueError("grid dimensions must be positive")


def _validate_inputs(rows: int, cols: int, robots: tuple[Cell, ...], obstacles: set[Cell]) -> None:
    if not robots:
        raise ValueError("at least one robot is required")

    seen_robots: set[Cell] = set()
    for robot in robots:
        _validate_cell(rows, cols, robot, "robot")
        if robot in seen_robots:
            raise ValueError(f"duplicate robot position: {robot}")
        if robot in obstacles:
            raise ValueError(f"robot starts on an obstacle: {robot}")
        seen_robots.add(robot)

    for obstacle in obstacles:
        _validate_cell(rows, cols, obstacle, "obstacle")


def _validate_cell(rows: int, cols: int, cell: Cell, name: str) -> None:
    row, col = cell
    if not (0 <= row < rows and 0 <= col < cols):
        raise ValueError(f"{name} cell outside grid: {cell}")

