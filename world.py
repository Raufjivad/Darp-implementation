from collections import deque
from typing import Iterable, Sequence

from models import FREE, OBSTACLE, Cell, Grid, WorldSetup





def to_cell(position: int | Cell, cols: int) -> Cell:
    if isinstance(position, int):
        return position // cols, position % cols
    if len(position) != 2:
        raise ValueError(f"Position must be an index or (row, col): {position}")
    return int(position[0]), int(position[1])


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



def validate_grid_size(rows: int, cols: int) -> None:
    if rows <= 0 or cols <= 0:
        raise ValueError("Grid dimensions must be positive")



def validate_robots_inside_map(rows: int, cols: int, robots: Sequence[Cell]) -> None:
    if not robots:
        raise ValueError("At least one robot initial position is required")
    for robot in robots:
        validate_cell_inside_map(rows, cols, robot, "Robot")


#this funcation make sure cell choosen are inside the grid
def validate_cell_inside_map(rows: int, cols: int, cell: Cell, label: str) -> None:
    row, col = cell
    if row < 0 or row >= rows or col < 0 or col >= cols:
        raise ValueError(f"{label} position is outside the grid: {cell}")


def validate_obstacles_inside_map(rows: int, cols: int, obstacles: Sequence[Cell]) -> None:
    for obstacle in obstacles:
        validate_cell_inside_map(rows, cols, obstacle, "Obstacle")

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
    passable_count = sum(
        grid_env[row][col] != OBSTACLE
        for row in range(rows)
        for col in range(cols)
    )
    if count_reachable_passable_cells(grid_env, start) != passable_count:
        raise ValueError("The environment grid must not have unreachable or closed-shape free regions")


def count_reachable_passable_cells(grid_env: Grid, start: Cell) -> int:
    rows = len(grid_env)
    cols = len(grid_env[0])
    queue: deque[Cell] = deque([start])
    visited = {start}

    while queue:
        row, col = queue.popleft()
        for next_row, next_col in four_neighbors(rows, cols, row, col):
            neighbor = (next_row, next_col)
            if neighbor in visited or grid_env[next_row][next_col] == OBSTACLE:
                continue
            visited.add(neighbor)
            queue.append(neighbor)

    return len(visited)


def four_neighbors(rows: int, cols: int, row: int, col: int) -> Iterable[Cell]:
    for next_row, next_col in (
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1),
    ):
        if 0 <= next_row < rows and 0 <= next_col < cols:
            yield next_row, next_col