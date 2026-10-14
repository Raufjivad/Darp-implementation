import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from MyTest import (
    build_coverage_paths,
    build_spanning_trees,
    nodes,
    present_stc_results,
    setup_world,
)


class SpanningTreeCoverageTest(unittest.TestCase):
    def test_builds_separate_trees_and_complete_fine_grid_paths(self):
        assignments = [
            [(0, 0), (0, 1), (1, 0)],
            [(2, 2)],
        ]
        starts = [(0, 0), (2, 2)]

        region_graphs = nodes(assignments)
        spanning_trees = build_spanning_trees(region_graphs, starts)
        paths = build_coverage_paths(spanning_trees, starts)

        self.assertEqual(
            [sum(len(neighbors) for neighbors in tree.values()) // 2 for tree in spanning_trees],
            [2, 0],
        )
        for robot_id, cells in enumerate(assignments):
            expected_fine_cells = {
                (row * 2 + row_offset, col * 2 + col_offset)
                for row, col in cells
                for row_offset in (0, 1)
                for col_offset in (0, 1)
            }
            self.assertEqual(set(paths[robot_id]), expected_fine_cells)
            self.assertTrue(all(
                abs(first[0] - second[0]) + abs(first[1] - second[1]) == 1
                for first, second in zip(paths[robot_id], paths[robot_id][1:])
            ))
            self.assertEqual(len(paths[robot_id]), 2 * len(expected_fine_cells) - 1)

    def test_rejects_disconnected_robot_region(self):
        region_graphs = nodes([[(0, 0), (2, 2)]])

        with self.assertRaisesRegex(ValueError, "region.*disconnected"):
            build_spanning_trees(region_graphs, [(0, 0)])

    def test_renders_tree_and_coverage_panels(self):
        world = setup_world(2, 2, [0])
        assignments = [[(0, 0), (0, 1), (1, 0), (1, 1)]]
        trees = build_spanning_trees(nodes(assignments), world.robot_positions)
        paths = build_coverage_paths(trees, world.robot_positions)

        screen = present_stc_results(
            world,
            assignments,
            trees,
            paths,
            [(0, 102, 204)],
        )

        self.assertEqual(screen.get_size(), (248, 230))
        self.assertIs(pygame.display.get_surface(), screen)
        self.assertEqual(screen.get_at((29, 83))[:3], (0, 52, 154))


if __name__ == "__main__":
    unittest.main()