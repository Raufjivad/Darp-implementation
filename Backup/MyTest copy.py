from collections import deque
from dataclasses import dataclass
from datetime import datetime
import argparse
import json
from pathlib import Path
import pygame

from math import sqrt
from sys import float_info
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
    TilesImportance: list[list[list[float]]]
    DesireableAssign: tuple[float, ...]  # isnt it better to use list as tuple cannot be changed?
    termThr: int
    Notiles: int
    MinimumImportance: tuple[float, ...]
    MaximumImportance: tuple[float, ...]

#PyGame Setup to present the grid
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
    robot_set = set(robot_positions)
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


def archive_run_snapshot(
    world: WorldSetup,
    assignment_setup: AssignmentMatrixSetup,
    assignment: Sequence[Sequence[Cell]],
    metric_weights: Sequence[float] | None = None,
) -> Path:
    ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if metric_weights is None:
        metric_weights = (1.0,) * len(world.robot_positions)
    balance_j, balance_objective = calculate_balance_costs(
        [len(cells) for cells in assignment],
        world.rows * world.cols - len(world.obstacle_positions),
    )

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
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
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
        "balance_j": list(balance_j),
        "balance_objective": balance_objective,
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


def show_archive_viewer() -> None:
    snapshots = load_archived_runs()
    if not snapshots:
        raise FileNotFoundError(f"No readable run snapshots found in {ARCHIVE_DIR}")

    pygame.init()
    index = 0
    cell_size = 50
    margin = 24
    footer_height = 76
    title_height = 54
    screen: pygame.Surface | None = None
    fonts = {
        "title": pygame.font.SysFont("arial", 22, bold=True),
        "footer": pygame.font.SysFont("arial", 17),
        "metric": pygame.font.SysFont("arial", 10),
    }
    assignment_colors = [
        (0, 102, 204),
        (34, 139, 34),
        (255, 140, 0),
        (128, 0, 128),
        (220, 20, 60),
    ]

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
        pygame.display.set_caption(f"Run archive: {snapshot['_file_name']}")
        screen.fill(WHITE)

        title = fonts["title"].render(str(snapshot.get("timestamp", snapshot["_file_name"])), True, BLACK)
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

                if metric_matrix is not None and cell not in obstacles:
                    for robot_id in range(min(len(robot_positions), len(metric_matrix))):
                        metric = metric_matrix[robot_id][row][col]
                        label = fonts["metric"].render(f"{metric:.1f}", True, BLACK)
                        label_y = y + 3 + robot_id * 11
                        if label_y + label.get_height() > y + cell_size:
                            break
                        screen.blit(label, (x + 3, label_y))

        balance = snapshot.get("balance_objective")
        balance_text = f"Balance objective: {float(balance):.3f}" if balance is not None else "Balance objective: unavailable"
        footer = fonts["footer"].render(
            f"{index + 1}/{len(snapshots)}  {snapshot['_file_name']}    {balance_text}    Left/Right: browse    Esc: close",
            True,
            BLACK,
        )
        screen.blit(footer, (margin, grid_top + grid_height + 16))
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

    pygame.quit()


def run_simulation() -> None:
    metric_weights = (1.0, 1.0, 1.0)
    world = setup_world(
        rows=10,
        cols=10,
        robot_initial_positions=[0, 8, 65],
        obstacle_positions=[43, 33, 23, 46, 53, 54, 55, 56, 36, 26],
    )

    print("Grid size:", world.rows, "x", world.cols)
    print("Robots:", world.robot_positions)
    print("Metric weights:", metric_weights)
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
    print("TilesImportance for robot 1:")
    print_matrix(assignment_setup.TilesImportance[1], precision=4)
    assignment = divide_tiles_by_importance(
        tiles_importance=assignment_setup.TilesImportance,
        desirable_assign=assignment_setup.DesireableAssign,
        robot_positions=world.robot_positions,
        obstacle_positions=world.obstacle_positions,
        metric_matrix=assignment_setup.MetricMatrix,
        metric_weights=metric_weights,
    )
    for robot_id, cells in enumerate(assignment):
        print(f"Robot {robot_id}: {len(cells)} tiles")
        print(cells)
    balance_j, balance_objective = calculate_balance_costs(
        [len(cells) for cells in assignment],
        world.rows * world.cols - len(world.obstacle_positions),
    )
    print("Balance J per robot:", balance_j)
    print("Total squared balance objective:", balance_objective)

    archive_run_snapshot(world, assignment_setup, assignment, metric_weights)

    screen = initialize_display()
    assignment_colors = [
        (0, 102, 204),
        (34, 139, 34),
        (255, 140, 0),
        (128, 0, 128),
        (220, 20, 60),
    ]
    metric_font = pygame.font.SysFont("arial", 10)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(WHITE)

        robot_start_colors = {}
        for robot_id, robot_pos in enumerate(world.robot_positions):
            base_color = assignment_colors[robot_id % len(assignment_colors)]
            robot_start_colors[robot_pos] = tuple(max(0, channel - 50) for channel in base_color)

        for row in range(world.rows):
            for col in range(world.cols):
                x = START_X + col * (RECT_WIDTH + GAP)
                y = START_Y + row * (RECT_HEIGHT + GAP)
                rect = pygame.Rect(x, y, RECT_WIDTH, RECT_HEIGHT)
                cell_pos = (row, col)

                assigned_robot = None
                for robot_id, robot_cells in enumerate(assignment):
                    if cell_pos in robot_cells:
                        assigned_robot = robot_id
                        break

                cell_value = world.grid_env[row][col]
                if cell_pos in robot_start_colors:
                    current_color = robot_start_colors[cell_pos]
                elif assigned_robot is not None:
                    current_color = assignment_colors[assigned_robot % len(assignment_colors)]
                elif cell_value == OBSTACLE:
                    current_color = BLACK
                elif cell_value == FREE:
                    current_color = GRAY
                else:
                    current_color = BLUE

                pygame.draw.rect(screen, current_color, rect)
                pygame.draw.rect(screen, BLACK, rect, 1)

                for robot_id in range(len(world.robot_positions)):
                    if world.grid_env[row][col] == OBSTACLE:
                        continue

                    metric_value = assignment_setup.MetricMatrix[robot_id][row][col]
                    metric_label = f"{metric_value:.1f}"
                    metric_text = metric_font.render(metric_label, True, BLACK)

                    text_x = x + 4
                    text_y = y + 4 + robot_id * 11
                    if text_y + metric_text.get_height() > y + RECT_HEIGHT:
                        text_y = y + RECT_HEIGHT - metric_text.get_height() - 2

                    screen.blit(metric_text, (text_x, text_y))

        pygame.display.flip()

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