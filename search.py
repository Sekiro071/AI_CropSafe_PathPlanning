"""BFS, DFS, Greedy Best-First Search, and A*."""

from collections import deque
from heapq import heappop, heappush
from itertools import count


def heuristic(cell, goal):
    """Manhattan distance for movement in four directions."""
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])


def build_path(parent, goal):
    """Reconstruct the route from the goal back to the start."""
    path = []

    while goal is not None:
        path.append(goal)
        goal = parent[goal]

    return path[::-1]


def search(field, algorithm, start, goal, known_obstacles):
    """Return (path, expanded nodes). An empty path means no route."""

    if algorithm not in ("BFS", "DFS", "Greedy", "A*"):
        raise ValueError(f"Unknown algorithm: {algorithm}")

    blocked = field.crops | field.protected | set(known_obstacles)

    def valid(cell):
        return (
            0 <= cell[0] < field.rows
            and 0 <= cell[1] < field.cols
            and cell not in blocked
        )

    if not valid(start) or not valid(goal):
        return [], 0

    parent = {start: None}
    expanded = 0

    if algorithm in ("BFS", "DFS"):
        frontier = deque([start])

        while frontier:
            if algorithm == "BFS":
                current = frontier.popleft()
            else:
                current = frontier.pop()

            # Count nodes removed for processing, including the goal.
            expanded += 1

            if current == goal:
                return build_path(parent, goal), expanded

            for neighbor in field.neighbors(current, known_obstacles):
                if neighbor not in parent:
                    parent[neighbor] = current
                    frontier.append(neighbor)

        return [], expanded

    # A sequence number makes priority ties deterministic.
    sequence = count()

    # Entries are (priority, sequence, cell, path cost).
    frontier = [
        (heuristic(start, goal), next(sequence), start, 0)
    ]

    best_cost = {start: 0}
    processed = set()

    while frontier:
        _, _, current, cost = heappop(frontier)

        if current in processed or cost != best_cost[current]:
            continue

        processed.add(current)
        expanded += 1

        if current == goal:
            return build_path(parent, goal), expanded

        for neighbor in field.neighbors(current, known_obstacles):
            new_cost = cost + field.step_cost(neighbor)

            if (
                neighbor not in processed
                and new_cost < best_cost.get(neighbor, float("inf"))
            ):
                best_cost[neighbor] = new_cost
                parent[neighbor] = current

                # Greedy uses h. A* uses g + h.
                priority = heuristic(neighbor, goal)

                if algorithm == "A*":
                    priority += new_cost

                heappush(
                    frontier,
                    (priority, next(sequence), neighbor, new_cost),
                )

    return [], expanded
