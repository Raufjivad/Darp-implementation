from collections import deque
from dataclasses import dataclass
from datetime import datetime
import argparse
import json
from math import exp
from pathlib import Path
import pygame
from original_stc import RobotSTCPlan, STCPlanningResult, plan_original_darp_stc
import tracemalloc
from time import perf_counter

from math import sqrt
from sys import float_info, getsizeof
from typing import Iterable, Sequence

# Value Assignment for different properties of grid
FREE = -1
OBSTACLE = -2

# Format Assignment for each type of data in the algorithm
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
    MetricMatrix: list[list[list[float]]] #3 dimenstional matrix list with decimal number
    ˀTilesImportance: list[list[list[float]]]
    DesireableAssign: tuple[float, ...]  # isnt it better to use list as tuple cannot be changed?
    termThr: int
    Notiles: int
    MinimumImportance: tuple[float, ...]
    MaximumImportance: tuple[float, ...]

#PyGame Setup to present the grid
pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("Seedarp")

WHITE = (255, 255, 255)
BLUE = (0, 102, 204)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
PURPLE = (128, 0 ,128)

RECT_WIDTH = 50   # Width of each rectangle
RECT_HEIGHT = 50  # Height of each rectangle
GAP = 0          # Space between rectangles

START_X = 50
START_Y = 50
ARCHIVE_DIR = Path(__file__).resolve().parent / "run_archive"


def initialize_display() -> pygame.Surface:
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Seedarp")
    return screen


def setup_world(
    rows: int,
    cols: int,
    robot_initial_positions: Sequence[int | Cell],
    obstacle_positions: Iterable[int | Cell] = (),
) -> WorldSetup:
    """Build and validate the DARP world setup.

    This mirrors the first stage of the original Python DARP implementation:
    sanity_check() + defineGridEnv().

    GridEnv values:
        -1: free passable cell
        -2: obstacle cell
         0..n-1: robot initial cell, where the value is the robot index

    Positions can be provided either as flattened indexes or as (row, col)
    tuples. The original DARP Python code accepts flattened indexes; accepting
    tuples here makes manual testing easier.
    """

    validate_grid_size(rows, cols)

    robots = tuple(to_cell(position, cols) for position in robot_initial_positions)
    obstacles = tuple(to_cell(position, cols) for position in obstacle_positions)

    validate_robots_inside_map(rows, cols, robots)
    validate_obstacles_inside_map(rows, cols, obstacles)
    validate_unique_robot_positions(robots)
    validate_robots_not_on_obstacles(robots, obstacles)

    grid_env = create_grid_env(rows, cols, robots, obstacles)
    validate_free_space_connected(grid_env, robots[0])

    return WorldSetup(
        rows=rows,
        cols=cols,
        robot_positions=robots,
        obstacle_positions=obstacles,
        grid_env=grid_env,
    )



def construct_assignment_matrix(
    world: WorldSetup,
    portions: Sequence[float] | None = None,
) -> AssignmentMatrixSetup:
    """Construct the initial DARP metric matrix.

    This mirrors the original construct_Assignment_Matrix() logic:
        - MetricMatrix starts as Euclidean distances from each robot to each cell.
        - DesireableAssign stores each robot's desired number of non-seed cells.
        - termThr starts at 1 when equal division is not exact.
        - TilesImportance stores how important each tile is for each robot,
          computed as 1 / sum(distance from all other robots to that tile).
    """

    rows = world.rows
    cols = world.cols
    robot_positions = world.robot_positions
    droneNo = len(robot_positions)
    Notiles = rows * cols
    effectiveSize = Notiles - droneNo - len(world.obstacle_positions)

    if portions is None:
        portions = tuple(1 / droneNo for _ in range(droneNo)) #in this function the portion for each robot is determined
    validate_portions(portions, droneNo)

    termThr = 1 if effectiveSize % droneNo != 0 else 0
    DesireableAssign = [0.0 for _ in range(droneNo)]
    MaximumImportance = [0.0 for _ in range(droneNo)]
    MinimumImportance = [float_info.max for _ in range(droneNo)]
#This loop iterates desireable Asigning for each robot
    for robot_id in range(droneNo):
        DesireableAssign[robot_id] = effectiveSize * portions[robot_id]
        if DesireableAssign[robot_id] != int(DesireableAssign[robot_id]) and termThr != 1:
            termThr = 1

    MetricMatrix = create_3d_matrix(droneNo, rows, cols, 0.0)
    TilesImportance = create_3d_matrix(droneNo, rows, cols, 0.0)
#loop underneath will get the distance for each tile
    for row in range(rows):
        for col in range(cols):
            tempSum = 0.0

            for robot_id, robot_position in enumerate(robot_positions):
                distance = euclidean_distance(robot_position, (row, col))
                MetricMatrix[robot_id][row][col] = distance
                tempSum += distance

            for robot_id in range(droneNo):
                other_robot_distance_sum = tempSum - MetricMatrix[robot_id][row][col]
                if other_robot_distance_sum != 0:
                    tile_importance = 1 / other_robot_distance_sum
                else:
                    tile_importance = 1

                TilesImportance[robot_id][row][col] = tile_importance
                if tile_importance > MaximumImportance[robot_id]:
                    MaximumImportance[robot_id] = tile_importance
                if tile_importance < MinimumImportance[robot_id]:
                    MinimumImportance[robot_id] = tile_importance

    return AssignmentMatrixSetup(
        MetricMatrix=MetricMatrix,
        TilesImportance=TilesImportance,
        DesireableAssign=tuple(DesireableAssign),
        termThr=termThr,
        Notiles=Notiles,
        MinimumImportance=tuple(MinimumImportance),
        MaximumImportance=tuple(MaximumImportance),
    )

def calculate_balance_costs(
    assigned_counts: Sequence[int],
    unoccupied_cell_count: int,
) -> tuple[tuple[float, ...], float]:
    """Return each robot's signed balance error and the total squared error."""
    if not assigned_counts:
        raise ValueError("At least one robot is required to calculate balance costs")
    if unoccupied_cell_count < 0:
        raise ValueError("Unoccupied cell count cannot be negative")

    target_count = unoccupied_cell_count / len(assigned_counts)
    robot_costs = tuple(
        0.5 * (assigned_count - target_count)
        for assigned_count in assigned_counts
    )
    total_cost = sum(cost * cost for cost in robot_costs)
    return robot_costs, total_cost


def divide_tiles_by_importance(
    tiles_importance: Sequence[Sequence[Sequence[float]]],
    desirable_assign: Sequence[float],
    robot_positions: Sequence[Cell] = (),
    obstacle_positions: Sequence[Cell] = (),
    metric_matrix: Sequence[Sequence[Sequence[float]]] | None = None,
    metric_weights: Sequence[float] | None = None,
    distance_weight: float = 1.0,
) -> list[list[Cell]]:
    robot_count = len(tiles_importance)
    rows = len(tiles_importance[0])
    cols = len(tiles_importance[0][0])
    if len(desirable_assign) != robot_count:
        raise ValueError("A desirable assignment value must be provided for each robot")
    if metric_weights is None:
        metric_weights = (1.0,) * robot_count
    if len(metric_weights) != robot_count:
        raise ValueError("A metric weight must be provided for each robot")
    if any(weight <= 0 for weight in metric_weights):
        raise ValueError("Metric weights must be positive")
    if distance_weight < 0:
        raise ValueError("Distance weight cannot be negative")

    obstacle_set = set(obstacle_positions)
    passable_tiles = {
        (r, c)
        for r in range(rows)
        for c in range(cols)
        if (r, c) not in obstacle_set
    }
    max_distance = max(
        (value for robot_matrix in metric_matrix or () for row in robot_matrix for value in row),
        default=1.0,
    ) or 1.0

    assignments: list[list[Cell]] = [[] for _ in range(robot_count)]
    assigned_tiles: set[Cell] = set()

    def neighbors(cell: Cell) -> Iterable[Cell]:
        row, col = cell
        for next_row, next_col in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if 0 <= next_row < rows and 0 <= next_col < cols:
                yield next_row, next_col

    for robot_id, start in enumerate(robot_positions):
        assignments[robot_id].append(start)
        assigned_tiles.add(start)

    while len(assigned_tiles) < len(passable_tiles):
        frontier: list[tuple[float, float, float, int, Cell]] = []
        for robot_id, cells in enumerate(assignments):
            for owned_cell in cells:
                for candidate in neighbors(owned_cell):
                    if candidate in obstacle_set or candidate in assigned_tiles:
                        continue
                    row, col = candidate
                    distance_cost = metric_matrix[robot_id][row][col] if metric_matrix is not None else 0.0
                    importance_cost = -tiles_importance[robot_id][row][col]
                    projected_counts = [len(robot_cells) for robot_cells in assignments]
                    projected_counts[robot_id] += 1
                    _, balance_cost = calculate_balance_costs(projected_counts, len(passable_tiles))
                    normalized_distance = distance_cost / max_distance
                    combined_cost = balance_cost + distance_weight * metric_weights[robot_id] * normalized_distance
                    frontier.append((combined_cost, balance_cost, importance_cost, robot_id, candidate))

        if not frontier:
            raise ValueError("Could not reach every passable tile from the robot starting positions")

        _, _, _, robot_id, cell = min(frontier)
        assignments[robot_id].append(cell)
        assigned_tiles.add(cell)

    if sum(len(cells) for cells in assignments) != len(passable_tiles):
        raise ValueError(
            f"Assignment did not cover all passable tiles: expected {len(passable_tiles)}, "
            f"got {sum(len(cells) for cells in assignments)}"
        )

    return assignments


def tune_metric_weights(
    tiles_importance: Sequence[Sequence[Sequence[float]]],
    desirable_assign: Sequence[float],
    robot_positions: Sequence[Cell],
    obstacle_positions: Sequence[Cell],
    metric_matrix: Sequence[Sequence[Sequence[float]]],
    initial_weights: Sequence[float] | None = None,
    learning_rate: float = 2.0,
    max_iterations: int = 30,
    minimum_weight: float = 0.1,
    maximum_weight: float = 10.0,
) -> tuple[list[list[Cell]], tuple[float, ...], list[dict[str, object]]]:
    """Tune distance multipliers using bounded, damped balance feedback."""
    robot_count = len(robot_positions)
    if robot_count == 0:
        raise ValueError("At least one robot is required to tune metric weights")
    if learning_rate <= 0:
        raise ValueError("Learning rate must be positive")
    if max_iterations <= 0:
        raise ValueError("Maximum iterations must be positive")
    if minimum_weight <= 0 or maximum_weight < minimum_weight:
        raise ValueError("Metric weight bounds must satisfy 0 < minimum <= maximum")

    weights = list(initial_weights or (1.0,) * robot_count)
    if len(weights) != robot_count:
        raise ValueError("An initial metric weight must be provided for each robot")
    if any(weight <= 0 for weight in weights):
        raise ValueError("Initial metric weights must be positive")

    total_passable = len({
        (row, col)
        for row in range(len(tiles_importance[0]))
        for col in range(len(tiles_importance[0][0]))
        if (row, col) not in set(obstacle_positions)
    })
    target_count = total_passable / robot_count
    base_count, remainder = divmod(total_passable, robot_count)
    ideal_counts = [base_count + (robot_id < remainder) for robot_id in range(robot_count)]
    _, minimum_objective = calculate_balance_costs(ideal_counts, total_passable)

    best_assignment: list[list[Cell]] = []
    best_weights = tuple(weights)
    best_objective = float("inf")
    iteration_history: list[dict[str, object]] = []
    tracing_started_here = not tracemalloc.is_tracing()
    if tracing_started_here:
        tracemalloc.start()
    tuning_started = perf_counter()

    try:
        for iteration in range(max_iterations):
            iteration_started = perf_counter()
            iteration_weights = list(weights)
            memory_at_iteration_start, _ = tracemalloc.get_traced_memory()
            tracemalloc.reset_peak()
            assignment = divide_tiles_by_importance(
                tiles_importance=tiles_importance,
                desirable_assign=desirable_assign,
                robot_positions=robot_positions,
                obstacle_positions=obstacle_positions,
                metric_matrix=metric_matrix,
                metric_weights=iteration_weights,
            )
            counts = [len(cells) for cells in assignment]
            balance_j, objective = calculate_balance_costs(counts, total_passable)

            if objective < best_objective:
                best_assignment = assignment
                best_weights = tuple(weights)
                best_objective = objective

            should_stop = objective <= minimum_objective + 1e-12
            if not should_stop:
                weights = [
                    min(
                        maximum_weight,
                        max(
                            minimum_weight,
                            weight * exp(learning_rate * (count - target_count) / target_count),
                        ),
                    )
                    for weight, count in zip(weights, counts)
                ]

            _, iteration_peak_bytes = tracemalloc.get_traced_memory()
            temporary_peak_bytes = max(0, iteration_peak_bytes - memory_at_iteration_start)
            iteration_history.append({
                "iteration": iteration,
                "metric_weights": iteration_weights,
                "region_sizes": counts,
                "balance_j": list(balance_j),
                "balance_objective": objective,
                "elapsed_seconds": perf_counter() - iteration_started,
                "cumulative_elapsed_seconds": perf_counter() - tuning_started,
                "peak_temporary_traced_bytes": temporary_peak_bytes,
            })

            if should_stop:
                break
    finally:
        if tracing_started_here:
            tracemalloc.stop()

    return best_assignment, best_weights, iteration_history

def validate_portions(portions: Sequence[float], robot_count: int) -> None:
    if len(portions) != robot_count:
        raise ValueError("Portions should be defined for each robot")
    if abs(sum(portions) - 1) >= 0.0001:
        raise ValueError("Sum of portions should be equal to 1")
    for portion in portions:
        if portion < 0:
            raise ValueError("Portions must be non-negative")


def create_3d_matrix(depth: int, rows: int, cols: int, value: float) -> list[list[list[float]]]:
    return [[[value for _ in range(cols)] for _ in range(rows)] for _ in range(depth)]


def euclidean_distance(first: Cell, second: Cell) -> float:
    return sqrt((first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2)


def validate_grid_size(rows: int, cols: int) -> None:
    if rows <= 0 or cols <= 0:
        raise ValueError("Grid dimensions must be positive")

#convert a number into grid position following "Cell" format
def to_cell(position: int | Cell, cols: int) -> Cell:
    if isinstance(position, int):
        return position // cols, position % cols
    if len(position) != 2:
        raise ValueError(f"Position must be an index or (row, col): {position}")
    return int(position[0]), int(position[1])


def validate_robots_inside_map(rows: int, cols: int, robots: Sequence[Cell]) -> None:
    if not robots:
        raise ValueError("At least one robot initial position is required")
    for robot in robots:
        validate_cell_inside_map(rows, cols, robot, "Robot")


def validate_obstacles_inside_map(rows: int, cols: int, obstacles: Sequence[Cell]) -> None:
    for obstacle in obstacles:
        validate_cell_inside_map(rows, cols, obstacle, "Obstacle")


#this funcation make sure cell choosen are inside the grid
def validate_cell_inside_map(rows: int, cols: int, cell: Cell, label: str) -> None:
    row, col = cell
    if row < 0 or row >= rows or col < 0 or col >= cols:
        raise ValueError(f"{label} position is outside the grid: {cell}")


def validate_unique_robot_positions(robots: Sequence[Cell]) -> None:
    if len(set(robots)) != len(robots):
        raise ValueError("Robot initial positions must be unique")


def validate_robots_not_on_obstacles(robots: Sequence[Cell], obstacles: Sequence[Cell]) -> None:
    obstacle_set = set(obstacles)
    for robot in robots:
        if robot in obstacle_set:
            raise ValueError(f"Robot initial position cannot be on an obstacle: {robot}")


def create_grid_env(rows: int, cols: int, robots: Sequence[Cell], obstacles: Sequence[Cell]) -> Grid:
    grid_env = [[FREE for _ in range(cols)] for _ in range(rows)]

    for row, col in obstacles:
        grid_env[row][col] = OBSTACLE

    for robot_id, (row, col) in enumerate(robots):
        grid_env[row][col] = robot_id

    return grid_env

#make sures there's no area which is not reacheable
def validate_free_space_connected(grid_env: Grid, start: Cell) -> None:
    rows = len(grid_env)
    cols = len(grid_env[0])
    passable_count = sum(1 for row in range(rows) for col in range(cols) if grid_env[row][col] != OBSTACLE)
    reachable_count = count_reachable_passable_cells(grid_env, start)

    if reachable_count != passable_count:
        raise ValueError("The environment grid must not have unreachable or closed-shape free regions")


def count_reachable_passable_cells(grid_env: Grid, start: Cell) -> int:
    queue: deque[Cell] = deque([start])
    visited = {start}

    while queue:
        row, col = queue.popleft()
        for next_row, next_col in four_neighbors(len(grid_env), len(grid_env[0]), row, col):
            if (next_row, next_col) in visited:
                continue
            if grid_env[next_row][next_col] == OBSTACLE:
                continue
            visited.add((next_row, next_col))
            queue.append((next_row, next_col))

    return len(visited)


def four_neighbors(rows: int, cols: int, row: int, col: int) -> Iterable[Cell]:
    for next_row, next_col in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
        if 0 <= next_row < rows and 0 <= next_col < cols:
            yield next_row, next_col


def print_grid(grid_env: Grid) -> None:
    for row in grid_env:
        print(" ".join(f"{value:2d}" for value in row))

#2 dimensional list of decmial number float 
def print_matrix(matrix: list[list[float]], precision: int = 2) -> None:
    for row in matrix:
        print(" ".join(f"{value:6.{precision}f}" for value in row))


def estimate_numeric_matrices_bytes(
    matrices: Sequence[Sequence[Sequence[Sequence[float]]]],
) -> int:
    """Estimate nested-list and float storage for numeric 3D matrices."""
    total_bytes = 0
    for matrix in matrices:
        total_bytes += getsizeof(matrix)
        for robot_matrix in matrix:
            total_bytes += getsizeof(robot_matrix)
            for row in robot_matrix:
                total_bytes += getsizeof(row)
                total_bytes += sum(getsizeof(value) for value in row)
    return total_bytes


def summarize_tuning_analysis(
    rows: int,
    cols: int,
    robot_count: int,
    iteration_history: Sequence[dict[str, object]],
    metric_matrix: Sequence[Sequence[Sequence[float]]],
    tiles_importance: Sequence[Sequence[Sequence[float]]],
) -> dict[str, object]:
    grid_cells = rows * cols
    matrix_count = 2
    matrix_slots = matrix_count * robot_count * grid_cells
    actual_matrix_bytes = estimate_numeric_matrices_bytes((metric_matrix, tiles_importance))
    return {
        "grid_cells": grid_cells,
        "robot_count": robot_count,
        "numeric_matrix_count": matrix_count,
        "numeric_matrix_slots": matrix_slots,
        "idealized_float64_matrix_bytes": matrix_slots * 8,
        "python_matrix_bytes": actual_matrix_bytes,
        "iteration_count": len(iteration_history),
        "tuning_elapsed_seconds": (
            float(iteration_history[-1]["cumulative_elapsed_seconds"])
            if iteration_history
            else 0.0
        ),
        "peak_traced_iteration_bytes": max(
            (int(item["peak_temporary_traced_bytes"]) for item in iteration_history),
            default=0,
        ),
        "matrix_space_complexity": "O(beta * robots * grid_cells), beta=2",
        "tuner_time_complexity": "O(iterations * robots * grid_cells^2) worst case",
    }


def present_stc_results(
    world: WorldSetup,
    assignments: Sequence[Sequence[Cell]],
    spanning_trees: Sequence[dict[Cell, set[Cell]]],
    coverage_paths: Sequence[Sequence[Cell]],
    assignment_colors: Sequence[tuple[int, int, int]],
    planning_result: STCPlanningResult | None = None,
    analysis_summary: dict[str, object] | None = None,
) -> pygame.Surface:
    """Display tile-level spanning trees beside fine-grid coverage routes."""
    pygame.init()
    maximum_panel_size = 440
    coarse_cell_size = max(4, min(44, maximum_panel_size // max(world.rows, world.cols)))
    fine_cell_size = max(2, coarse_cell_size // 2)
    coarse_cell_size = fine_cell_size * 2
    panel_width = world.cols * coarse_cell_size
    panel_height = world.rows * coarse_cell_size
    margin = 24
    header_height = 78
    footer_height = 160
    screen = pygame.display.set_mode((margin * 3 + panel_width * 2, header_height + panel_height + footer_height))
    pygame.display.set_caption("Spanning Trees and Coverage Paths")
    screen.fill(WHITE)

    title_font = pygame.font.SysFont("arial", 23, bold=True)
    panel_font = pygame.font.SysFont("arial", 17, bold=True)
    detail_font = pygame.font.SysFont("arial", 13)
    node_font = pygame.font.SysFont("arial", max(10, fine_cell_size // 2), bold=True)
    tree_start_color = (255, 230, 80)
    path_colors = [(255, 255, 255), (0, 255, 255), (255, 0, 220), (255, 235, 0)]
    tree_left = margin
    path_left = margin * 2 + panel_width
    grid_top = header_height

    title = title_font.render("Spanning Trees + Coverage Paths", True, BLACK)
    screen.blit(title, (margin, 16))
    screen.blit(panel_font.render("TREE ON ASSIGNED TILES", True, BLACK), (tree_left, 52))
    screen.blit(panel_font.render("2x2 COVERAGE ROUTES", True, BLACK), (path_left, 52))

    assignment_by_cell = {
        cell: robot_id
        for robot_id, cells in enumerate(assignments)
        for cell in cells
    }
    robot_starts = {cell: robot_id for robot_id, cell in enumerate(world.robot_positions)}

    for row in range(world.rows):
        for col in range(world.cols):
            cell = (row, col)
            robot_id = assignment_by_cell.get(cell)
            if world.grid_env[row][col] == OBSTACLE:
                color = BLACK
            elif robot_id is None:
                color = GRAY
            else:
                color = assignment_colors[robot_id % len(assignment_colors)]
            if cell in robot_starts:
                color = tuple(max(0, channel - 50) for channel in color)
            rect = pygame.Rect(
                tree_left + col * coarse_cell_size,
                grid_top + row * coarse_cell_size,
                coarse_cell_size,
                coarse_cell_size,
            )
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 1)

    for robot_id, tree in enumerate(spanning_trees):
        for cell, neighbors in tree.items():
            row, col = cell
            center = (
                tree_left + col * coarse_cell_size + coarse_cell_size // 2,
                grid_top + row * coarse_cell_size + coarse_cell_size // 2,
            )
            for neighbor in neighbors:
                if cell >= neighbor:
                    continue
                next_row, next_col = neighbor
                next_center = (
                    tree_left + next_col * coarse_cell_size + coarse_cell_size // 2,
                    grid_top + next_row * coarse_cell_size + coarse_cell_size // 2,
                )
                pygame.draw.line(screen, BLACK, center, next_center, max(5, coarse_cell_size // 5))
                pygame.draw.line(screen, (255, 255, 255), center, next_center, max(2, coarse_cell_size // 10))

        start = world.robot_positions[robot_id]
        start_center = (
            tree_left + start[1] * coarse_cell_size + coarse_cell_size // 2,
            grid_top + start[0] * coarse_cell_size + coarse_cell_size // 2,
        )
        pygame.draw.circle(screen, BLACK, start_center, max(9, coarse_cell_size // 3))
        pygame.draw.circle(screen, tree_start_color, start_center, max(7, coarse_cell_size // 3 - 2))
        label = node_font.render(f"R{robot_id}", True, BLACK)
        screen.blit(label, label.get_rect(center=start_center))

    for row in range(world.rows * 2):
        for col in range(world.cols * 2):
            coarse_cell = (row // 2, col // 2)
            robot_id = assignment_by_cell.get(coarse_cell)
            if world.grid_env[coarse_cell[0]][coarse_cell[1]] == OBSTACLE:
                color = BLACK
            elif robot_id is None:
                color = GRAY
            else:
                color = assignment_colors[robot_id % len(assignment_colors)]
            if coarse_cell in robot_starts:
                color = tuple(max(0, channel - 50) for channel in color)
            rect = pygame.Rect(
                path_left + col * fine_cell_size,
                grid_top + row * fine_cell_size,
                fine_cell_size,
                fine_cell_size,
            )
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, BLACK, rect, 1)

    for row in range(world.rows + 1):
        y = grid_top + row * coarse_cell_size
        pygame.draw.line(screen, BLACK, (path_left, y), (path_left + panel_width, y), 2)
    for col in range(world.cols + 1):
        x = path_left + col * coarse_cell_size
        pygame.draw.line(screen, BLACK, (x, grid_top), (x, grid_top + panel_height), 2)

    for robot_id, path in enumerate(coverage_paths):
        points = [
            (
                path_left + col * fine_cell_size + fine_cell_size // 2,
                grid_top + row * fine_cell_size + fine_cell_size // 2,
            )
            for row, col in path
        ]
        if len(points) > 1:
            pygame.draw.lines(screen, BLACK, False, points, max(5, fine_cell_size // 3))
            pygame.draw.lines(
                screen,
                path_colors[robot_id % len(path_colors)],
                False,
                points,
                max(2, fine_cell_size // 7),
            )
        if points:
            start = points[0]
            pygame.draw.circle(screen, BLACK, start, max(8, fine_cell_size // 2))
            pygame.draw.circle(screen, tree_start_color, start, max(6, fine_cell_size // 2 - 2))
            label = node_font.render(f"R{robot_id}", True, BLACK)
            screen.blit(label, label.get_rect(center=start))

    if planning_result is not None:
        total_text = detail_font.render(
            f"Total STC planning: {planning_result.total_planning_seconds:.6f}s",
            True,
            BLACK,
        )
        screen.blit(total_text, (path_left, grid_top + panel_height + 10))

    for robot_id, path in enumerate(coverage_paths):
        if planning_result is None:
            details = f"R{robot_id}: {len(set(path))} cells, {len(path)} steps"
        else:
            plan = planning_result.plans[robot_id]
            details = (
                f"R{robot_id}: {len(set(path))} cells, {len(plan.path_edges)} moves, "
                f"{plan.turns} turns, plan {plan.planning_seconds:.6f}s"
            )
        detail = detail_font.render(
            details,
            True,
            BLACK,
        )
        details_top = grid_top + panel_height + (30 if planning_result is not None else 12)
        screen.blit(detail, (path_left, details_top + robot_id * 18))

    if analysis_summary is not None:
        tuning_text = detail_font.render(
            f"DARP tuning: {float(analysis_summary['tuning_elapsed_seconds']):.6f}s, "
            f"{int(analysis_summary['iteration_count'])} iterations",
            True,
            BLACK,
        )
        memory_text = detail_font.render(
            f"Matrices: {float(analysis_summary['python_matrix_bytes']) / (1024 * 1024):.3f} MiB Python objects; "
            f"iteration peak {float(analysis_summary['peak_traced_iteration_bytes']) / (1024 * 1024):.3f} MiB traced",
            True,
            BLACK,
        )
        screen.blit(tuning_text, (path_left, grid_top + panel_height + 88))
        screen.blit(memory_text, (path_left, grid_top + panel_height + 108))

    pygame.display.flip()
    return screen



def archive_run_snapshot(
    world: WorldSetup,
    assignment_setup: AssignmentMatrixSetup,
    assignment: Sequence[Sequence[Cell]],
    metric_weights: Sequence[float] | None = None,
    tuning_history: Sequence[dict[str, object]] = (),
    stc_result: STCPlanningResult | None = None,
    analysis_summary: dict[str, object] | None = None,
) -> Path:
    ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if metric_weights is None:
        metric_weights = (1.0,) * len(world.robot_positions)
    balance_j, balance_objective = calculate_balance_costs(
        [len(cells) for cells in assignment],
        world.rows * world.cols - len(world.obstacle_positions),
    )

    stc_summary = None
    if stc_result is not None:
        stc_summary = {
            "total_planning_seconds": stc_result.total_planning_seconds,
            "robots": [
                {
                    "robot_id": plan.robot_id,
                    "mode": plan.mode,
                    "planning_seconds": plan.planning_seconds,
                    "turns": plan.turns,
                    "coverage_cells": len(set(plan.fine_path)),
                    "moves": len(plan.path_edges),
                }
                for plan in stc_result.plans
            ],
        }

    snapshot = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "rows": world.rows,
        "cols": world.cols,
        "robot_positions": list(world.robot_positions),
        "obstacle_positions": list(world.obstacle_positions),
        "desireable_assign": list(assignment_setup.DesireableAssign),
        "term_thr": assignment_setup.termThr,
        "metric_matrix": assignment_setup.MetricMatrix,
        "tiles_importance": assignment_setup.TilesImportance,
        "assignment": [list(cells) for cells in assignment],
        "metric_weights": list(metric_weights),
        "tuning_history": list(tuning_history),
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
        "stc_metrics": stc_summary,
        "computational_memory_analysis": analysis_summary,
    }

    run_file = ARCHIVE_DIR / f"run_{timestamp}.json"
    run_file.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")

    history_file = ARCHIVE_DIR / "history.json"
    if history_file.exists():
        try:
            history = json.loads(history_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            history = []
    else:
        history = []

    history.append({
        "timestamp": snapshot["timestamp"],
        "run_file": run_file.name,
        "rows": world.rows,
        "cols": world.cols,
        "robot_positions": list(world.robot_positions),
        "obstacle_positions": list(world.obstacle_positions),
        "desireable_assign": list(assignment_setup.DesireableAssign),
        "term_thr": assignment_setup.termThr,
        "metric_weights": list(metric_weights),
        "tuning_iterations": len(tuning_history),
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
        "stc_metrics": stc_summary,
        "computational_memory_analysis": analysis_summary,
    })
    history_file.write_text(json.dumps(history, indent=2), encoding="utf-8")
    return run_file


def load_archived_runs() -> list[dict[str, object]]:
    run_files = sorted(ARCHIVE_DIR.glob("run_*.json"), reverse=True)
    snapshots = []
    for run_file in run_files:
        try:
            snapshot = json.loads(run_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if all(key in snapshot for key in ("rows", "cols", "robot_positions", "obstacle_positions", "assignment")):
            snapshot["_file_name"] = run_file.name
            snapshots.append(snapshot)
    return snapshots


def stc_plans_from_snapshot(snapshot: dict[str, object]) -> STCPlanningResult:
    assignments = [
        [(int(cell[0]), int(cell[1])) for cell in robot_cells]
        for robot_cells in snapshot["assignment"]
    ]
    robot_positions = [
        (int(position[0]), int(position[1]))
        for position in snapshot["robot_positions"]
    ]
    return plan_original_darp_stc(
        assignments,
        robot_positions,
        int(snapshot["rows"]),
        int(snapshot["cols"]),
    )


def show_archive_viewer() -> None:
    snapshots = load_archived_runs()
    if not snapshots:
        raise FileNotFoundError(f"No readable run snapshots found in {ARCHIVE_DIR}")

    pygame.init()
    index = 0
    show_paths = False
    plans_by_snapshot: dict[int, STCPlanningResult | str] = {}
    cell_size = 50
    margin = 24
    footer_height = 160
    title_height = 54
    screen: pygame.Surface | None = None
    fonts = {
        "title": pygame.font.SysFont("arial", 22, bold=True),
        "footer": pygame.font.SysFont("arial", 14),
        "metric": pygame.font.SysFont("arial", 10),
    }
    assignment_colors = [
        (0, 102, 204),
        (34, 139, 34),
        (255, 140, 0),
        (128, 0, 128),
        (220, 20, 60),
    ]
    path_colors = [(255, 255, 255), (0, 255, 255), (255, 0, 220), (255, 235, 0)]

    def draw_snapshot() -> None:
        nonlocal screen
        snapshot = snapshots[index]
        rows = int(snapshot["rows"])
        cols = int(snapshot["cols"])
        grid_width = cols * cell_size
        grid_height = rows * cell_size
        window_width = max(480, grid_width + margin * 2)
        window_height = grid_height + title_height + footer_height + margin
        screen = pygame.display.set_mode((window_width, window_height))
        view_name = "STC paths" if show_paths else "metrics"
        pygame.display.set_caption(f"Run archive: {snapshot['_file_name']} ({view_name})")
        screen.fill(WHITE)

        title = fonts["title"].render(
            f"{snapshot.get('timestamp', snapshot['_file_name'])}  |  {view_name}",
            True,
            BLACK,
        )
        screen.blit(title, (margin, 12))

        obstacles = {tuple(cell) for cell in snapshot["obstacle_positions"]}
        robot_positions = [tuple(cell) for cell in snapshot["robot_positions"]]
        assignment = [
            {tuple(cell) for cell in robot_cells}
            for robot_cells in snapshot["assignment"]
        ]
        metric_matrix = snapshot.get("metric_matrix")
        robot_starts = {cell: robot_id for robot_id, cell in enumerate(robot_positions)}
        assignment_by_cell = {
            cell: robot_id
            for robot_id, cells in enumerate(assignment)
            for cell in cells
        }
        grid_top = title_height
        plans = None
        plan_error = None
        planning_total_seconds = None
        robot_planning_seconds: dict[int, float] = {}
        if show_paths:
            if index not in plans_by_snapshot:
                try:
                    plans_by_snapshot[index] = stc_plans_from_snapshot(snapshot)
                except (KeyError, TypeError, ValueError) as error:
                    plans_by_snapshot[index] = str(error)
            cached_plans = plans_by_snapshot[index]
            if isinstance(cached_plans, str):
                plan_error = cached_plans
            else:
                plans = list(cached_plans.plans)
                planning_total_seconds = cached_plans.total_planning_seconds
                saved_metrics = snapshot.get("stc_metrics")
                if isinstance(saved_metrics, dict):
                    saved_total = saved_metrics.get("total_planning_seconds")
                    if isinstance(saved_total, (int, float)):
                        planning_total_seconds = float(saved_total)
                    saved_robots = saved_metrics.get("robots")
                    if isinstance(saved_robots, list):
                        for robot_metrics in saved_robots:
                            if not isinstance(robot_metrics, dict):
                                continue
                            robot_id = robot_metrics.get("robot_id")
                            planning_seconds = robot_metrics.get("planning_seconds")
                            if isinstance(robot_id, int) and isinstance(planning_seconds, (int, float)):
                                robot_planning_seconds[robot_id] = float(planning_seconds)

        for row in range(rows):
            for col in range(cols):
                cell = (row, col)
                x = margin + col * cell_size
                y = grid_top + row * cell_size
                rect = pygame.Rect(x, y, cell_size, cell_size)
                robot_id = robot_starts.get(cell, assignment_by_cell.get(cell))
                if cell in obstacles:
                    color = BLACK
                elif robot_id is not None:
                    base_color = assignment_colors[robot_id % len(assignment_colors)]
                    color = (
                        tuple(max(0, channel - 50) for channel in base_color)
                        if cell in robot_starts
                        else base_color
                    )
                else:
                    color = GRAY
                pygame.draw.rect(screen, color, rect)
                pygame.draw.rect(screen, BLACK, rect, 1)

                if not show_paths and metric_matrix is not None and cell not in obstacles:
                    for robot_id in range(min(len(robot_positions), len(metric_matrix))):
                        metric = metric_matrix[robot_id][row][col]
                        label = fonts["metric"].render(f"{metric:.1f}", True, BLACK)
                        label_y = y + 3 + robot_id * 11
                        if label_y + label.get_height() > y + cell_size:
                            break
                        screen.blit(label, (x + 3, label_y))

        if plans is not None:
            fine_cell_size = cell_size // 2
            for plan in plans:
                points = [
                    (
                        margin + col * fine_cell_size + fine_cell_size // 2,
                        grid_top + row * fine_cell_size + fine_cell_size // 2,
                    )
                    for row, col in plan.fine_path
                ]
                color = path_colors[plan.robot_id % len(path_colors)]
                if len(points) > 1:
                    pygame.draw.lines(screen, BLACK, False, points, 11)
                    pygame.draw.lines(screen, color, False, points, 6)
                if points:
                    pygame.draw.circle(screen, BLACK, points[0], 10)
                    pygame.draw.circle(screen, (255, 230, 80), points[0], 7)
                    start_label = fonts["metric"].render(f"R{plan.robot_id}", True, BLACK)
                    screen.blit(start_label, start_label.get_rect(center=points[0]))

        balance = snapshot.get("balance_objective")
        balance_text = f"Balance objective: {float(balance):.3f}" if balance is not None else "Balance objective: unavailable"
        footer = fonts["footer"].render(
            f"{index + 1}/{len(snapshots)}  {snapshot['_file_name']}    {balance_text}",
            True,
            BLACK,
        )
        controls = fonts["footer"].render(
            "P: toggle metrics/paths    Left/Right: browse runs    Esc: close",
            True,
            BLACK,
        )
        footer_top = grid_top + grid_height + 10
        screen.blit(footer, (margin, footer_top))
        screen.blit(controls, (margin, footer_top + 22))
        if plans is not None:
            total_text = fonts["footer"].render(
                f"STC planning total: {planning_total_seconds:.6f}s",
                True,
                BLACK,
            )
            screen.blit(total_text, (margin, footer_top + 42))
            for plan in plans:
                planning_seconds = robot_planning_seconds.get(
                    plan.robot_id,
                    plan.planning_seconds,
                )
                detail = fonts["footer"].render(
                    f"R{plan.robot_id}: {planning_seconds:.6f}s planning, "
                    f"{plan.turns} turns, {len(plan.path_edges)} moves",
                    True,
                    BLACK,
                )
                screen.blit(detail, (margin, footer_top + 62 + plan.robot_id * 16))
            saved_analysis = snapshot.get("computational_memory_analysis")
            if isinstance(saved_analysis, dict):
                analysis_text = fonts["footer"].render(
                    f"DARP tuning: {float(saved_analysis['tuning_elapsed_seconds']):.6f}s; "
                    f"matrices: {float(saved_analysis['python_matrix_bytes']) / (1024 * 1024):.3f} MiB",
                    True,
                    BLACK,
                )
                screen.blit(analysis_text, (margin, footer_top + 112))
        if plan_error is not None:
            error_text = fonts["footer"].render(f"Path unavailable: {plan_error}", True, (180, 0, 0))
            screen.blit(error_text, (margin, footer_top + 42))
        pygame.display.flip()

    draw_snapshot()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_RIGHT, pygame.K_PAGEDOWN):
                    index = (index + 1) % len(snapshots)
                    draw_snapshot()
                elif event.key in (pygame.K_LEFT, pygame.K_PAGEUP):
                    index = (index - 1) % len(snapshots)
                    draw_snapshot()
                elif event.key == pygame.K_p:
                    show_paths = not show_paths
                    draw_snapshot()

    pygame.quit()


def run_simulation() -> None:
    initial_metric_weights = (1.0,)
    world = setup_world(
        rows=10,
        cols=10,
        robot_initial_positions=[0],
        obstacle_positions=	[
            *range(2, 10),
            *range(12, 20),
            *range(22, 30),
            *range(32, 40),
            *range(42, 50),
            *range(52, 60),
            *range(62, 70),
            *range(72, 80),


            
        ],
    )

    print("Grid size:", world.rows, "x", world.cols)
    print("Robots:", world.robot_positions)
    print("Initial metric weights:", initial_metric_weights)
    print("Obstacles:", world.obstacle_positions)
    print("GridEnv:")
    print_grid(world.grid_env)


    assignment_setup = construct_assignment_matrix(world)
    print("\ntermThr:", assignment_setup.termThr)
    print("DesireableAssign:", assignment_setup.DesireableAssign)
    print("MetricMatrix for robot 0:")
    print_matrix(assignment_setup.MetricMatrix[0])
    print("TilesImportance for robot 0:")
    print_matrix(assignment_setup.TilesImportance[0], precision=4)
    assignment, metric_weights, tuning_history = tune_metric_weights(
        tiles_importance=assignment_setup.TilesImportance,
        desirable_assign=assignment_setup.DesireableAssign,
        robot_positions=world.robot_positions,
        obstacle_positions=world.obstacle_positions,
        metric_matrix=assignment_setup.MetricMatrix,
        initial_weights=initial_metric_weights,
        learning_rate=2.0,
        max_iterations=30,
    )
    stc_result = plan_original_darp_stc(assignment, world.robot_positions, world.rows, world.cols)
    plans = stc_result.plans
    spanning_trees = [plan.spanning_tree for plan in plans]
    coverage_paths = [plan.fine_path for plan in plans]
    analysis_summary = summarize_tuning_analysis(
        world.rows,
        world.cols,
        len(world.robot_positions),
        tuning_history,
        assignment_setup.MetricMatrix,
        assignment_setup.TilesImportance,
    )
    for plan in plans:
        tree_edges = sum(len(neighbors) for neighbors in plan.spanning_tree.values()) // 2
        print(
            f"Robot {plan.robot_id}: mode={plan.mode}, tree={tree_edges} edges, "
            f"coverage={len(set(plan.fine_path))} fine cells, "
            f"{len(plan.path_edges)} moves, {plan.turns} turns, "
            f"planning={plan.planning_seconds:.6f}s"
        )
    print(f"Total map STC planning: {stc_result.total_planning_seconds:.6f}s")
    for iteration in tuning_history:
        print(
            f"Tuning iteration {iteration['iteration']}: "
            f"sizes={iteration['region_sizes']}, "
            f"objective={iteration['balance_objective']:.4f}, "
            f"weights={iteration['metric_weights']}, "
            f"elapsed={float(iteration['elapsed_seconds']):.6f}s, "
            f"temporary peak={int(iteration['peak_temporary_traced_bytes']) / (1024 * 1024):.3f} MiB"
        )
    print(
        f"Tuning summary: {analysis_summary['iteration_count']} iterations, "
        f"{float(analysis_summary['tuning_elapsed_seconds']):.6f}s total"
    )
    print(
        f"Matrix storage: beta={analysis_summary['numeric_matrix_count']}, "
        f"slots={analysis_summary['numeric_matrix_slots']}, "
        f"float64 payload={int(analysis_summary['idealized_float64_matrix_bytes']) / (1024 * 1024):.3f} MiB, "
        f"Python object estimate={int(analysis_summary['python_matrix_bytes']) / (1024 * 1024):.3f} MiB"
    )
    print(f"Peak temporary traced iteration allocation: {int(analysis_summary['peak_traced_iteration_bytes']) / (1024 * 1024):.3f} MiB")
    print(f"Asymptotic matrix space: {analysis_summary['matrix_space_complexity']}")
    print(f"Current tuner worst-case time: {analysis_summary['tuner_time_complexity']}")
    print("Selected metric weights:", metric_weights)
    for robot_id, cells in enumerate(assignment):
        print(f"Robot {robot_id}: {len(cells)} tiles")
        print(cells)
    balance_j, balance_objective = calculate_balance_costs(
        [len(cells) for cells in assignment],
        world.rows * world.cols - len(world.obstacle_positions),
    )
    print("Balance J per robot:", balance_j)
    print("Total squared balance objective:", balance_objective)

    archive_run_snapshot(
        world,
        assignment_setup,
        assignment,
        metric_weights,
        tuning_history,
        stc_result,
        analysis_summary,
    )

    assignment_colors = [
        (0, 102, 204),
        (34, 139, 34),
        (255, 140, 0),
        (128, 0, 128),
        (220, 20, 60),
    ]
    present_stc_results(
        world,
        assignment,
        spanning_trees,
        coverage_paths,
        assignment_colors,
        stc_result,
        analysis_summary,
    )

    running = True
    while running:
        event = pygame.event.wait()
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False

    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--view-archive",
        action="store_true",
        help="Browse archived runs in Pygame",
    )
    args = parser.parse_args()
    if args.view_archive:
        show_archive_viewer()
    else:
        run_simulation()