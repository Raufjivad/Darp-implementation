from dataclasses import dataclass
from time import perf_counter
from typing import Sequence

import numpy as np

from CalculateTrajectories import CalculateTrajectories
from kruskal import Kruskal


Cell = tuple[int, int]
PathEdge = tuple[int, int, int, int]


@dataclass(frozen=True)
class RobotSTCPlan:
    robot_id: int
    mode: int
    turns: int
    planning_seconds: float
    spanning_tree: dict[Cell, set[Cell]]
    path_edges: tuple[PathEdge, ...]
    fine_path: tuple[Cell, ...]


@dataclass(frozen=True)
class STCPlanningResult:
    plans: tuple[RobotSTCPlan, ...]
    total_planning_seconds: float


def plan_original_darp_stc(
    assignments: Sequence[Sequence[Cell]],
    robot_positions: Sequence[Cell],
    rows: int,
    cols: int,
) -> STCPlanningResult:
    """Apply the original DARP MST modes and trajectory conversion per region."""
    map_started = perf_counter()
    if rows <= 0 or cols <= 0:
        raise ValueError("Grid dimensions must be positive")
    if len(assignments) != len(robot_positions):
        raise ValueError("A starting position must be provided for each robot region")

    occupied_cells: set[Cell] = set()
    for robot_id, (cells, start) in enumerate(zip(assignments, robot_positions)):
        region = set(cells)
        if not region:
            raise ValueError(f"Robot {robot_id} has an empty region")
        if start not in region:
            raise ValueError(f"Robot {robot_id}'s start is not in its assigned region")
        for row, col in region:
            if not (0 <= row < rows and 0 <= col < cols):
                raise ValueError(f"Robot {robot_id} has an out-of-bounds cell: {(row, col)}")
        if occupied_cells & region:
            raise ValueError("Robot regions must not overlap")
        occupied_cells.update(region)

    plans: list[RobotSTCPlan] = []
    for robot_id, (cells, start) in enumerate(zip(assignments, robot_positions)):
        robot_started = perf_counter()
        region = set(cells)
        region_mask = np.zeros((rows, cols), dtype=bool)
        for row, col in region:
            region_mask[row, col] = True
        fine_mask = np.repeat(np.repeat(region_mask, 2, axis=0), 2, axis=1)
        expected_fine_cells = {
            (row * 2 + row_offset, col * 2 + col_offset)
            for row, col in region
            for row_offset in (0, 1)
            for col_offset in (0, 1)
        }

        candidates: list[RobotSTCPlan] = []
        for mode in range(4):
            kruskal = Kruskal(rows, cols)
            kruskal.initializeGraph(region_mask, True, mode)
            kruskal.performKruskal()
            if len(kruskal.mst) != len(region) - 1:
                raise ValueError(f"Robot {robot_id}'s region is not connected")

            tree = {cell: set() for cell in region}
            for edge in kruskal.mst:
                first = divmod(edge.src, cols)
                second = divmod(edge.dst, cols)
                tree[first].add(second)
                tree[second].add(first)

            trajectory = CalculateTrajectories(rows, cols, kruskal.mst)
            trajectory.initializeGraph(fine_mask, True)
            trajectory.RemoveTheAppropriateEdges()
            start_node = 4 * start[0] * cols + 2 * start[1]
            trajectory.CalculatePathsSequence(start_node)
            path_edges = tuple(tuple(map(int, edge)) for edge in trajectory.PathSequence)
            _validate_path(path_edges, expected_fine_cells, robot_id)

            fine_path = (path_edges[0][:2],) + tuple(edge[2:] for edge in path_edges)
            candidates.append(RobotSTCPlan(
                robot_id=robot_id,
                mode=mode,
                turns=_count_closed_route_turns(path_edges),
                planning_seconds=0.0,
                spanning_tree=tree,
                path_edges=path_edges,
                fine_path=fine_path,
            ))

        selected = min(candidates, key=lambda plan: (plan.turns, plan.mode))
        plans.append(RobotSTCPlan(
            robot_id=selected.robot_id,
            mode=selected.mode,
            turns=selected.turns,
            planning_seconds=perf_counter() - robot_started,
            spanning_tree=selected.spanning_tree,
            path_edges=selected.path_edges,
            fine_path=selected.fine_path,
        ))

    return STCPlanningResult(
        plans=tuple(plans),
        total_planning_seconds=perf_counter() - map_started,
    )


def _validate_path(
    path_edges: Sequence[PathEdge],
    expected_cells: set[Cell],
    robot_id: int,
) -> None:
    if not path_edges:
        raise ValueError(f"Robot {robot_id}'s trajectory is empty")

    visited = {
        point
        for edge in path_edges
        for point in (edge[:2], edge[2:])
    }
    if visited != expected_cells:
        raise ValueError(f"Robot {robot_id}'s trajectory does not cover its assigned fine cells")
    if any(
        abs(edge[0] - edge[2]) + abs(edge[1] - edge[3]) != 1
        for edge in path_edges
    ):
        raise ValueError(f"Robot {robot_id}'s trajectory contains a non-adjacent move")
    if any(first[2:] != second[:2] for first, second in zip(path_edges, path_edges[1:])):
        raise ValueError(f"Robot {robot_id}'s trajectory is discontinuous")
    if path_edges[-1][2:] != path_edges[0][:2]:
        raise ValueError(f"Robot {robot_id}'s trajectory does not return to its start")


def _count_closed_route_turns(path_edges: Sequence[PathEdge]) -> int:
    directions = [
        (edge[2] - edge[0], edge[3] - edge[1])
        for edge in path_edges
    ]
    return sum(
        directions[index] != directions[index - 1]
        for index in range(len(directions))
    )