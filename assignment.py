from math import exp, sqrt
from sys import float_info
import tracemalloc
from time import perf_counter
from typing import Iterable, Sequence

from models import AssignmentMatrixSetup, Cell, OBSTACLE, WorldSetup



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
        (row, col)
        for row in range(rows)
        for col in range(cols)
        if (row, col) not in obstacle_set
    }
    max_distance = max(
        (value for matrix in metric_matrix or () for row in matrix for value in row),
        default=1.0,
    ) or 1.0

    assignments: list[list[Cell]] = [[] for _ in range(robot_count)]
    assigned_tiles: set[Cell] = set()

    def neighbors(cell: Cell) -> Iterable[Cell]:
        row, col = cell
        for candidate in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if 0 <= candidate[0] < rows and 0 <= candidate[1] < cols:
                yield candidate

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




def calculate_balance_costs(
    assigned_counts: Sequence[int],
    unoccupied_cell_count: int,
) -> tuple[tuple[float, ...], float]:
    """Return each robot's signed balance error and total squared error."""
    if not assigned_counts:
        raise ValueError("At least one robot is required to calculate balance costs")
    if unoccupied_cell_count < 0:
        raise ValueError("Unoccupied cell count cannot be negative")

    target_count = unoccupied_cell_count / len(assigned_counts)
    robot_costs = tuple(0.5 * (count - target_count) for count in assigned_counts)
    return robot_costs, sum(cost * cost for cost in robot_costs)


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
