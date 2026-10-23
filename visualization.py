from typing import Sequence

import pygame

from archive import load_archived_runs, stc_plans_from_snapshot
from models import OBSTACLE, Cell, WorldSetup
from original_stc import STCPlanningResult, RobotSTCPlan


WHITE = (255, 255, 255)
BLUE = (0, 102, 204)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
ASSIGNMENT_COLORS = [
    (0, 102, 204),
    (34, 139, 34),
    (255, 140, 0),
    (128, 0, 128),
    (220, 20, 60),
]
PATH_COLORS = [(255, 255, 255), (0, 255, 255), (255, 0, 220), (255, 235, 0)]


def initialize_display() -> pygame.Surface:
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Seedarp")
    return screen


def print_grid(grid_env: list[list[int]]) -> None:
    for row in grid_env:
        print(" ".join(f"{value:2d}" for value in row))


def print_matrix(matrix: list[list[float]], precision: int = 2) -> None:
    for row in matrix:
        print(" ".join(f"{value:6.{precision}f}" for value in row))


def present_stc_results(
    world: WorldSetup,
    assignments: Sequence[Sequence[Cell]],
    spanning_trees: Sequence[dict[Cell, set[Cell]]],
    coverage_paths: Sequence[Sequence[Cell]],
    assignment_colors: Sequence[tuple[int, int, int]] = ASSIGNMENT_COLORS,
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
    start_color = (255, 230, 80)
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
                pygame.draw.line(screen, WHITE, center, next_center, max(2, coarse_cell_size // 10))

        start = world.robot_positions[robot_id]
        start_center = (
            tree_left + start[1] * coarse_cell_size + coarse_cell_size // 2,
            grid_top + start[0] * coarse_cell_size + coarse_cell_size // 2,
        )
        pygame.draw.circle(screen, BLACK, start_center, max(9, coarse_cell_size // 3))
        pygame.draw.circle(screen, start_color, start_center, max(7, coarse_cell_size // 3 - 2))
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
                PATH_COLORS[robot_id % len(PATH_COLORS)],
                False,
                points,
                max(2, fine_cell_size // 7),
            )
        if points:
            start = points[0]
            pygame.draw.circle(screen, BLACK, start, max(8, fine_cell_size // 2))
            pygame.draw.circle(screen, start_color, start, max(6, fine_cell_size // 2 - 2))
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
        detail = detail_font.render(details, True, BLACK)
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


def show_archive_viewer() -> None:
    snapshots = load_archived_runs()
    if not snapshots:
        raise FileNotFoundError("No readable run snapshots found in the run archive")

    pygame.init()
    index = 0
    show_paths = False
    plans_by_snapshot: dict[int, STCPlanningResult | str] = {}
    cell_size = 50
    margin = 24
    footer_height = 160
    title_height = 54
    fonts = {
        "title": pygame.font.SysFont("arial", 22, bold=True),
        "footer": pygame.font.SysFont("arial", 14),
        "metric": pygame.font.SysFont("arial", 10),
    }
    screen: pygame.Surface | None = None

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

        robot_positions = [tuple(cell) for cell in snapshot["robot_positions"]]
        assignment = [{tuple(cell) for cell in cells} for cells in snapshot["assignment"]]
        metric_matrix = snapshot.get("metric_matrix")
        robot_starts = {cell: robot_id for robot_id, cell in enumerate(robot_positions)}
        assignment_by_cell = {
            cell: robot_id
            for robot_id, cells in enumerate(assignment)
            for cell in cells
        }
        grid_top = title_height
        plans: list[RobotSTCPlan] | None = None
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

        obstacles = {tuple(cell) for cell in snapshot["obstacle_positions"]}
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
                    base_color = ASSIGNMENT_COLORS[robot_id % len(ASSIGNMENT_COLORS)]
                    color = tuple(max(0, channel - 50) for channel in base_color) if cell in robot_starts else base_color
                else:
                    color = GRAY
                pygame.draw.rect(screen, color, rect)
                pygame.draw.rect(screen, BLACK, rect, 1)

                if not show_paths and metric_matrix is not None and cell not in obstacles:
                    for metric_robot_id in range(min(len(robot_positions), len(metric_matrix))):
                        metric = metric_matrix[metric_robot_id][row][col]
                        label = fonts["metric"].render(f"{metric:.1f}", True, BLACK)
                        label_y = y + 3 + metric_robot_id * 11
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
                color = PATH_COLORS[plan.robot_id % len(PATH_COLORS)]
                if len(points) > 1:
                    pygame.draw.lines(screen, BLACK, False, points, 11)
                    pygame.draw.lines(screen, color, False, points, 6)
                if points:
                    pygame.draw.circle(screen, BLACK, points[0], 10)
                    pygame.draw.circle(screen, (255, 230, 80), points[0], 7)
                    label = fonts["metric"].render(f"R{plan.robot_id}", True, BLACK)
                    screen.blit(label, label.get_rect(center=points[0]))

        balance = snapshot.get("balance_objective")
        balance_text = f"Balance objective: {float(balance):.3f}" if balance is not None else "Balance objective: unavailable"
        footer_top = grid_top + grid_height + 10
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
                seconds = robot_planning_seconds.get(plan.robot_id, plan.planning_seconds)
                detail = fonts["footer"].render(
                    f"R{plan.robot_id}: {seconds:.6f}s planning, {plan.turns} turns, {len(plan.path_edges)} moves",
                    True,
                    BLACK,
                )
                screen.blit(detail, (margin, footer_top + 62 + plan.robot_id * 16))
            saved_analysis = snapshot.get("computational_memory_analysis")
            if isinstance(saved_analysis, dict):
                details = fonts["footer"].render(
                    f"DARP tuning: {float(saved_analysis['tuning_elapsed_seconds']):.6f}s; "
                    f"matrices: {float(saved_analysis['python_matrix_bytes']) / (1024 * 1024):.3f} MiB",
                    True,
                    BLACK,
                )
                screen.blit(details, (margin, footer_top + 112))
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
