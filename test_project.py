"""Run tests with: python -m unittest -v"""

import heapq
import unittest

from field import Field
from main import online_journey
from search import heuristic, search


class SearchTests(unittest.TestCase):
    def setUp(self):
        self.field = Field()

    def assert_valid_path(self, path, obstacles):
        field = self.field

        self.assertEqual(path[0], field.start)
        self.assertEqual(path[-1], field.goal)

        for previous, cell in zip(path, path[1:]):
            self.assertIn(
                cell,
                list(field.neighbors(previous, obstacles)),
            )

    def test_all_searches_find_legal_routes(self):
        for algorithm in ("BFS", "DFS", "Greedy", "A*"):
            with self.subTest(algorithm=algorithm):
                path, expanded = search(
                    self.field,
                    algorithm,
                    self.field.start,
                    self.field.goal,
                    self.field.obstacles,
                )

                self.assert_valid_path(path, self.field.obstacles)
                self.assertGreater(expanded, 0)

    def test_a_star_matches_dijkstra_minimum_cost(self):
        field = self.field
        costs = {field.start: 0}
        queue = [(0, field.start)]

        while queue:
            cost, cell = heapq.heappop(queue)

            if cost != costs[cell]:
                continue

            if cell == field.goal:
                break

            for neighbor in field.neighbors(cell, field.obstacles):
                new_cost = cost + field.step_cost(neighbor)

                if new_cost < costs.get(neighbor, float("inf")):
                    costs[neighbor] = new_cost
                    heapq.heappush(queue, (new_cost, neighbor))

        a_star_path, _ = search(
            field, "A*", field.start, field.goal, field.obstacles
        )
        bfs_path, _ = search(
            field, "BFS", field.start, field.goal, field.obstacles
        )

        self.assertEqual(field.path_cost(a_star_path), costs[field.goal])
        self.assertLess(
            field.path_cost(a_star_path),
            field.path_cost(bfs_path),
        )
        self.assertGreater(len(a_star_path), len(bfs_path))

    def test_heuristic_is_consistent(self):
        field = self.field

        for row in range(field.rows):
            for col in range(field.cols):
                cell = (row, col)

                for neighbor in field.neighbors(cell, field.obstacles):
                    self.assertLessEqual(
                        heuristic(cell, field.goal),
                        field.step_cost(neighbor)
                        + heuristic(neighbor, field.goal),
                    )

    def test_online_detects_and_replans(self):
        field = self.field

        original, _ = search(
            field, "A*", field.start, field.goal, field.obstacles
        )

        hidden = original[len(original) // 2]
        field.hidden_obstacles = {hidden}

        journey = online_journey(field)

        self.assertTrue(journey["reached"])
        self.assertEqual(journey["replans"], 1)
        self.assertEqual(journey["events"][0][1], hidden)
        self.assertNotIn(hidden, journey["path"])

        self.assert_valid_path(
            journey["path"],
            field.obstacles | field.hidden_obstacles,
        )
        self.assertNotIn(hidden, journey["planned_routes"][1])

    def test_unreachable_and_start_equals_goal(self):
        field = self.field

        blocked = set(field.obstacles) | set(
            field.neighbors(field.start, field.obstacles)
        )

        for algorithm in ("BFS", "DFS", "Greedy", "A*"):
            with self.subTest(algorithm=algorithm):
                path, _ = search(
                    field, algorithm, field.start, field.goal, blocked
                )
                self.assertEqual(path, [])

                same, _ = search(
                    field,
                    algorithm,
                    field.start,
                    field.start,
                    field.obstacles,
                )
                self.assertEqual(same, [field.start])

    def test_replan_failure_is_reported(self):
        field = self.field

        # Only one exit is known to be open, but it is actually blocked.
        exits = list(field.neighbors(field.start, field.obstacles))
        field.obstacles.update(exits[1:])
        field.hidden_obstacles = {exits[0]}

        journey = online_journey(field)

        self.assertFalse(journey["reached"])
        self.assertEqual(journey["replans"], 1)
        self.assertEqual(journey["path"], [field.start])


if __name__ == "__main__":
    unittest.main()
