import unittest

from darp_area_division import OBSTACLE, divide_areas_by_initial_positions


def robot_cells(assignment, robot_id):
    return {
        (row, col)
        for row, values in enumerate(assignment)
        for col, value in enumerate(values)
        if value == robot_id
    }


def connected(cells):
    start = next(iter(cells))
    seen = {start}
    stack = [start]
    while stack:
        row, col = stack.pop()
        for next_cell in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if next_cell in cells and next_cell not in seen:
                seen.add(next_cell)
                stack.append(next_cell)
    return seen == cells


class DarpAreaDivisionTest(unittest.TestCase):
    def test_equally_divides_open_space_from_robot_initial_positions(self):
        result = divide_areas_by_initial_positions(
            rows=4,
            cols=4,
            robot_positions=[(0, 0), (3, 3)],
        )

        self.assertEqual(result.quotas, (8, 8))
        self.assertEqual(result.counts, (8, 8))
        self.assertEqual(result.assignment[0][0], 0)
        self.assertEqual(result.assignment[3][3], 1)
        self.assertTrue(connected(robot_cells(result.assignment, 0)))
        self.assertTrue(connected(robot_cells(result.assignment, 1)))

    def test_distributes_remainder_when_space_is_not_evenly_divisible(self):
        result = divide_areas_by_initial_positions(
            rows=4,
            cols=5,
            robot_positions=[(0, 0), (0, 4), (3, 2)],
        )

        self.assertEqual(result.quotas, (7, 7, 6))
        self.assertEqual(result.counts, (7, 7, 6))
        for robot_id in range(3):
            self.assertTrue(connected(robot_cells(result.assignment, robot_id)))

    def test_preserves_obstacles_while_building_connected_areas(self):
        obstacles = {(1, 1), (2, 1)}
        result = divide_areas_by_initial_positions(
            rows=4,
            cols=4,
            robot_positions=[(0, 0), (3, 3)],
            obstacles=obstacles,
        )

        self.assertEqual(result.quotas, (7, 7))
        self.assertEqual(result.counts, (7, 7))
        for row, col in obstacles:
            self.assertEqual(result.assignment[row][col], OBSTACLE)
        self.assertTrue(connected(robot_cells(result.assignment, 0)))
        self.assertTrue(connected(robot_cells(result.assignment, 1)))

    def test_rejects_disconnected_free_space(self):
        with self.assertRaisesRegex(ValueError, "one connected component"):
            divide_areas_by_initial_positions(
                rows=3,
                cols=3,
                robot_positions=[(0, 0), (2, 2)],
                obstacles={(0, 1), (1, 0), (1, 2), (2, 1)},
            )

    def test_rejects_robot_on_obstacle(self):
        with self.assertRaisesRegex(ValueError, "robot starts on an obstacle"):
            divide_areas_by_initial_positions(
                rows=2,
                cols=2,
                robot_positions=[(0, 0)],
                obstacles={(0, 0)},
            )


if __name__ == "__main__":
    unittest.main()
