import argparse
from typing import Sequence

import pygame

from archive import archive_run_snapshot
from analysis import summarize_tuning_analysis
from assignment import calculate_balance_costs, construct_assignment_matrix, tune_metric_weights
from models import Cell
from original_stc import plan_original_darp_stc
from visualization import ASSIGNMENT_COLORS, present_stc_results, print_grid, print_matrix, show_archive_viewer
from world import setup_world


def run_simulation() -> None:
	initial_metric_weights = (1.0,)
	world = setup_world(
		rows=10,
		cols=10,
		robot_initial_positions=[0],
		obstacle_positions=[
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
	stc_result = plan_original_darp_stc(
		assignment,
		world.robot_positions,
		world.rows,
		world.cols,
	)
	analysis_summary = summarize_tuning_analysis(
		world.rows,
		world.cols,
		len(world.robot_positions),
		tuning_history,
		assignment_setup.MetricMatrix,
		assignment_setup.TilesImportance,
	)

	for plan in stc_result.plans:
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
	print(
		"Peak temporary traced iteration allocation: "
		f"{int(analysis_summary['peak_traced_iteration_bytes']) / (1024 * 1024):.3f} MiB"
	)
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
	present_stc_results(
		world,
		assignment,
		[plan.spanning_tree for plan in stc_result.plans],
		[plan.fine_path for plan in stc_result.plans],
		ASSIGNMENT_COLORS,
		stc_result,
		analysis_summary,
	)

	running = True
	while running:
		event = pygame.event.wait()
		if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
			running = False
	pygame.quit()


def main(argv: Sequence[str] | None = None) -> None:
	parser = argparse.ArgumentParser()
	parser.add_argument(
		"--view-archive",
		action="store_true",
		help="Browse archived runs in Pygame",
	)
	args = parser.parse_args(argv)
	if args.view_archive:
		show_archive_viewer()
	else:
		run_simulation()
