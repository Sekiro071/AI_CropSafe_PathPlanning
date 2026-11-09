"""Field layout and movement rules.

Positions are (row, column), starting from zero.
"""


class Field:
    def __init__(self):
        self.rows = 12
        self.cols = 16

        self.start = (5, 1)
        self.goal = (5, 14)

        # Potato plants.
        self.crops = {
            (row, col)
            for row in (3, 8)
            for col in range(3, 13)
        }

        # Protected regions around the plants.
        self.protected = {
            (row, col)
            for row in (2, 4, 7, 9)
            for col in range(3, 13)
        }

        self.obstacles = {(0, 8), (6, 2), (10, 13)}

        # Added by main.py for the replanning experiment.
        self.hidden_obstacles = set()

        # Extra cost for traveling next to a protected cell.
        self.near_crop_penalty = 4

    def neighbors(self, position, known_obstacles):
        """Yield valid neighboring cells."""
        row, col = position

        # Up, right, down, left.
        for dr, dc in ((-1, 0), (0, 1), (1, 0), (0, -1)):
            cell = (row + dr, col + dc)

            inside = (
                0 <= cell[0] < self.rows
                and 0 <= cell[1] < self.cols
            )

            if (
                inside
                and cell not in self.crops
                and cell not in self.protected
                and cell not in known_obstacles
            ):
                yield cell

    def step_cost(self, cell):
        """Calculate the cost of entering a cell."""
        row, col = cell

        adjacent = (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        )

        if any(cell in self.protected for cell in adjacent):
            return 1 + self.near_crop_penalty

        return 1

    def path_cost(self, path):
        """The starting cell has no entry cost."""
        return sum(self.step_cost(cell) for cell in path[1:])
