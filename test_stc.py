import os
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from archive import archive_run_snapshot
from analysis import summarize_tuning_analysis
from assignment import construct_assignment_matrix, tune_metric_weights
from visualization import present_stc_results
from world import setup_world

from archive import stc_plans_from_snapshot
from original_stc import plan_original_darp_stc


class SpanningTreeCoverageTest(unittest.TestCase):
    def test_builds_separate_trees_and_complete_fine_grid_paths(self):
        assignments = [
            [(0, 0), (0, 1), (1, 0)],
            [(2, 2)],
        ]
        starts = [(0, 0), (2, 2)]

        result = plan_original_darp_stc(assignments, starts, 3, 3)
        plans = result.plans
        self.assertGreaterEqual(result.total_planning_seconds, 0)
        self.assertGreaterEqual(
            result.total_planning_seconds,
            sum(plan.planning_seconds for plan in plans),
        )

        for robot_id, (cells, plan) in enumerate(zip(assignments, plans)):
            self.assertGreaterEqual(plan.planning_seconds, 0)
            expected_fine_cells = {
                (row * 2 + row_offset, col * 2 + col_offset)
                for row, col in cells
                for row_offset in (0, 1)
                for col_offset in (0, 1)
            }
            self.assertEqual(
                sum(len(neighbors) for neighbors in plan.spanning_tree.values()) // 2,
                len(cells) - 1,
            )
            self.assertIn(plan.mode, range(4))
            self.assertEqual(set(plan.fine_path), expected_fine_cells)
            self.assertEqual(plan.fine_path[0], plan.fine_path[-1])
            self.assertTrue(all(
                abs(first[0] - second[0]) + abs(first[1] - second[1]) == 1
                for first, second in zip(plan.fine_path, plan.fine_path[1:])
            ))
            self.assertEqual(len(plan.path_edges), len(expected_fine_cells))

    def test_rejects_disconnected_robot_region(self):
        with self.assertRaisesRegex(ValueError, "region is not connected"):
            plan_original_darp_stc([[(0, 0), (2, 2)]], [(0, 0)], 3, 3)

    def test_renders_tree_and_coverage_panels(self):
        world = setup_world(2, 2, [0])
        assignments = [[(0, 0), (0, 1), (1, 0), (1, 1)]]
        result = plan_original_darp_stc(assignments, world.robot_positions, 2, 2)
        plans = result.plans

        screen = present_stc_results(
            world,
            assignments,
            [plan.spanning_tree for plan in plans],
            [plan.fine_path for plan in plans],
            [(0, 102, 204)],
            result,
        )

        self.assertEqual(screen.get_size(), (248, 326))
        self.assertIs(pygame.display.get_surface(), screen)
        self.assertEqual(screen.get_at((29, 83))[:3], (0, 52, 154))

    def test_rebuilds_paths_from_archived_snapshot(self):
        snapshot = {
            "rows": 2,
            "cols": 2,
            "robot_positions": [[0, 0]],
            "assignment": [[[0, 0], [0, 1], [1, 0], [1, 1]]],
        }

        result = stc_plans_from_snapshot(snapshot)

        self.assertEqual(len(result.plans), 1)
        self.assertEqual(len(set(result.plans[0].fine_path)), 16)
        self.assertEqual(result.plans[0].fine_path[0], result.plans[0].fine_path[-1])
        self.assertGreaterEqual(result.total_planning_seconds, 0)

    def test_archives_per_robot_and_total_stc_planning_times(self):
        world = setup_world(2, 2, [0])
        assignment_setup = construct_assignment_matrix(world)
        assignments = [[(0, 0), (0, 1), (1, 0), (1, 1)]]
        stc_result = plan_original_darp_stc(assignments, world.robot_positions, 2, 2)
        _, _, iteration_history = tune_metric_weights(
            assignment_setup.TilesImportance,
            assignment_setup.DesireableAssign,
            world.robot_positions,
            world.obstacle_positions,
            assignment_setup.MetricMatrix,
        )
        analysis_summary = summarize_tuning_analysis(
            world.rows,
            world.cols,
            len(world.robot_positions),
            iteration_history,
            assignment_setup.MetricMatrix,
            assignment_setup.TilesImportance,
        )

        with tempfile.TemporaryDirectory() as temporary_directory:
            archive_directory = Path(temporary_directory)
            with patch("archive.ARCHIVE_DIR", archive_directory):
                run_file = archive_run_snapshot(
                    world,
                    assignment_setup,
                    assignments,
                    stc_result=stc_result,
                    analysis_summary=analysis_summary,
                )

            snapshot = json.loads(run_file.read_text(encoding="utf-8"))
            history = json.loads((archive_directory / "history.json").read_text(encoding="utf-8"))

        self.assertEqual(
            snapshot["stc_metrics"]["total_planning_seconds"],
            stc_result.total_planning_seconds,
        )
        self.assertEqual(len(snapshot["stc_metrics"]["robots"]), 1)
        self.assertEqual(history[0]["stc_metrics"], snapshot["stc_metrics"])
        self.assertEqual(
            snapshot["computational_memory_analysis"],
            analysis_summary,
        )

    def test_summarizes_iteration_time_and_matrix_memory(self):
        world = setup_world(2, 2, [0])
        assignment_setup = construct_assignment_matrix(world)
        _, _, iteration_history = tune_metric_weights(
            assignment_setup.TilesImportance,
            assignment_setup.DesireableAssign,
            world.robot_positions,
            world.obstacle_positions,
            assignment_setup.MetricMatrix,
        )

        summary = summarize_tuning_analysis(
            world.rows,
            world.cols,
            len(world.robot_positions),
            iteration_history,
            assignment_setup.MetricMatrix,
            assignment_setup.TilesImportance,
        )

        self.assertTrue(all("elapsed_seconds" in item for item in iteration_history))
        self.assertTrue(all("peak_temporary_traced_bytes" in item for item in iteration_history))
        self.assertEqual(summary["numeric_matrix_count"], 2)
        self.assertEqual(summary["numeric_matrix_slots"], 8)
        self.assertEqual(summary["idealized_float64_matrix_bytes"], 64)
        self.assertGreater(summary["python_matrix_bytes"], 0)


if __name__ == "__main__":
    unittest.main()