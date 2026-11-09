# AI-Based Crop-Safe Path Planning

This project simulates a potato-field robot. It compares BFS, DFS,
Greedy Best-First Search, and A*, then demonstrates detection of an
unknown obstacle and replanning.

## Requirements

- Python 3.9 or newer
- Matplotlib 3.7 or newer

## Run

Open a terminal in the project folder and run:

    python -m pip install -r requirements.txt
    python main.py

To run the tests:

    python -m unittest -v

The program prints each path and a performance table. It also creates
an `output` folder containing:

- `field_and_routes.png`: field and algorithm routes.
- `replanning.png`: obstacle detection and replanning.
- `comparison.png`: performance table image.
- `comparison.csv` and `comparison.txt`: performance results.

Include the generated images and table when uploading the project.

## Problem formulation

- **State:** the robot's `(row, column)` position in a 12 by 16 grid.
- **Initial state:** `(5, 1)`.
- **Goal:** `(5, 14)`.
- **Actions:** move one cell up, right, down, or left.
- **Transition:** the destination must be inside the field and cannot
  be a potato cell, protected cell, or known obstacle.
- **Goal test:** the robot's position equals the goal.
- **Path cost:** each move costs 1. Entering a cell next to at least
  one protected cell costs an additional 4.

The extra cost encourages routes that stay farther from protected crops.

## Algorithms

- **BFS:** finds the fewest-move route but ignores the extra cost while
  selecting its path.
- **DFS:** explores deeply; it does not guarantee the shortest or
  lowest-cost path.
- **Greedy Best-First Search:** chooses the cell that appears closest
  to the goal.
- **A*:** chooses using `g + h`, where `g` is the cost traveled so far
  and `h` estimates the remaining cost.

The heuristic is Manhattan distance:

    abs(current_row - goal_row) + abs(current_column - goal_column)

The robot moves in four directions and each move costs at least 1, so
this heuristic does not overestimate the remaining cost. A* finds a
minimum-cost route for the map known at the time of planning.

## Unknown obstacle and replanning

The program places a hidden obstacle halfway along the initial A*
route. The robot does not know about it initially. A simulated sensor
detects the obstacle before the robot enters its cell. The robot
updates its map and runs A* again from its current position.

The red X in the images shows the obstacle to the viewer, even though
the robot does not know about it initially. The orange ring in the
replanning image marks the detection position.

## Performance measurements

- **Moves:** number of edges traveled, not the number of cells.
- **Path cost:** movement cost including proximity penalties.
- **Expanded:** nodes removed from the frontier for processing,
  including the goal.
- **Time:** execution time in milliseconds; results vary by machine.
- **Replans:** new searches after obstacle detection, excluding the
  initial search.

The four standard algorithms plan on the same initially known map.
The online A* experiment changes its map after detecting the obstacle,
so its result is a separate experiment rather than a direct same-map
comparison.

On this example, BFS and Greedy take 13 moves at cost 53, while A*
takes 25 moves at cost 25 on the initially known map. A* deliberately
takes a longer, safer route because it minimizes weighted cost. Online
A* detects the hidden obstacle, replans once, and reaches the goal.
See `output/comparison.csv` for the full table, including DFS and
timings.

## Group members

Group number: [fill in]

| Name | Roll number | Actual contribution |
| --- | --- | --- |
| [Member 1] | [Roll number] | [Contribution] |
| [Member 2] | [Roll number] | [Contribution] |
| [Member 3] | [Roll number] | [Contribution] |
| [Member 4] | [Roll number] | [Contribution] |
| [Member 5] | [Roll number] | [Contribution] |

Add a sixth member if applicable.

## References

The project uses Matplotlib for visualization. Add any references or
external code your group uses. Make sure everyone understands the code.
