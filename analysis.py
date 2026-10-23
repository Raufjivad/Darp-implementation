from sys import getsizeof
from typing import Sequence


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
	python_matrix_bytes = estimate_numeric_matrices_bytes((metric_matrix, tiles_importance))
	return {
		"grid_cells": grid_cells,
		"robot_count": robot_count,
		"numeric_matrix_count": matrix_count,
		"numeric_matrix_slots": matrix_slots,
		"idealized_float64_matrix_bytes": matrix_slots * 8,
		"python_matrix_bytes": python_matrix_bytes,
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
