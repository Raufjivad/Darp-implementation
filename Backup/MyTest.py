from collections import deque
from dataclasses import dataclass
from math import sqrt
from sys import float_info
from typing import Iterable, Sequence


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
    MetricMatrix: list[list[list[float]]] #distance values in matrix
    TilesImportance: list[list[list[float]]] #global tiles importance
    DesireableAssign: tuple[float, ...]
    termThr: int
    Notiles: int
    MinimumImportance: tuple[float, ...]
    MaximumImportance: tuple[float, ...]

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
        portions = tuple(1 / droneNo for _ in range(droneNo))
    validate_portions(portions, droneNo)

    termThr = 1 if effectiveSize % droneNo != 0 else 0
    DesireableAssign = [0.0 for _ in range(droneNo)]
    MaximumImportance = [0.0 for _ in range(droneNo)]
    MinimumImportance = [float_info.max for _ in range(droneNo)]

    for robot_id in range(droneNo):
        DesireableAssign[robot_id] = effectiveSize * portions[robot_id]
        if DesireableAssign[robot_id] != int(DesireableAssign[robot_id]) and termThr != 1:
            termThr = 1

    MetricMatrix = create_3d_matrix(droneNo, rows, cols, 0.0)
    TilesImportance = create_3d_matrix(droneNo, rows, cols, 0.0)

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

"""def divisons(
        
) -> Divisionlogic:
    """

def divide_tiles_by_importance(
    tiles_importance: Sequence[Sequence[Sequence[float]]],
    desirable_assign: Sequence[float],
) -> list[list[Cell]]:
    robot_count = len(tiles_importance)
    rows = len(tiles_importance[0])
    cols = len(tiles_importance[0][0])
    total_tiles = rows * cols

    # 1) Convert desired float counts into integer target counts
    #    Use floor first, then distribute the remaining tiles by largest fractional parts
    floor_counts = [int(value) for value in desirable_assign]
    remaining_slots = total_tiles - sum(floor_counts)

    fractional_parts = [(value - int(value), i) for i, value in enumerate(desirable_assign)]
    fractional_parts.sort(reverse=True)

    target_counts = floor_counts[:]
    for _, robot_id in fractional_parts:
        if remaining_slots <= 0:
            break
        target_counts[robot_id] += 1
        remaining_slots -= 1

    # 2) Keep track of tiles still free
    remaining_tiles = {(r, c) for r in range(rows) for c in range(cols)}
    assignments: list[list[Cell]] = [[] for _ in range(robot_count)]

    # 3) For each robot, take the best tiles by importance
    for robot_id in range(robot_count):
        needed = target_counts[robot_id]
        if needed <= 0:
            continue

        ranked_tiles = []
        for r in range(rows):
            for c in range(cols):
                if (r, c) in remaining_tiles:
                    score = tiles_importance[robot_id][r][c]
                    ranked_tiles.append(((r, c), score))

        ranked_tiles.sort(key=lambda item: item[1], reverse=True)

        for (r, c), _ in ranked_tiles[:needed]:
            assignments[robot_id].append((r, c))
            remaining_tiles.remove((r, c))

    return assignments
    
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


def print_matrix(matrix: list[list[float]], precision: int = 2) -> None:
    for row in matrix:
        print(" ".join(f"{value:6.{precision}f}" for value in row))


if __name__ == "__main__":
    world = setup_world(
        rows=5,
        cols=7,
        robot_initial_positions=[0, 6, 34],
        obstacle_positions=[10, 11, 17, 24],
    )

    print("Grid size:", world.rows, "x", world.cols)
    print("Robots:", world.robot_positions)
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
    assignment = divide_tiles_by_importance(
    tiles_importance=assignment_setup.TilesImportance,
    desirable_assign=assignment_setup.DesireableAssign,
)
for robot_id, cells in enumerate(assignment):
    print(f"Robot {robot_id}: {len(cells)} tiles")
    print(cells)

