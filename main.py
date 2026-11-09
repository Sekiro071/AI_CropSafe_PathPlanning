"""Run the searches, simulate replanning, and save the results."""

import csv
from pathlib import Path
from time import perf_counter

import matplotlib

# Save images without needing a graphical window.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

from field import Field
from search import search


OUTPUT = Path(__file__).resolve().parent / "output"


def online_journey(field):
    """Detect a hidden obstacle before entering it, then replan."""
    known = set(field.obstacles)
    current = field.start
    traveled = [current]

    replans = 0
    expanded_total = 0
    events = []
    planned_routes = []

    start_time = perf_counter()

    path, expanded = search(
        field, "A*", current, field.goal, known
    )
    expanded_total += expanded
    planned_routes.append(path)

    while path and current != field.goal:
        next_cell = path[1]

        if next_cell in field.hidden_obstacles:
            # Detect the obstacle before moving into its cell.
            known.add(next_cell)
            replans += 1
            events.append((current, next_cell))

            path, expanded = search(
                field, "A*", current, field.goal, known
            )
            expanded_total += expanded
            planned_routes.append(path)

        else:
            current = next_cell
            traveled.append(current)
            path = path[1:]

    elapsed = perf_counter() - start_time

    return {
        "name": "A* online",
        "path": traveled,
        "expanded": expanded_total,
        "seconds": elapsed,
        "reached": current == field.goal,
        "replans": replans,
        "events": events,
        "planned_routes": planned_routes,
    }


def draw_field(ax, field, path=None, show_hidden=False):
    """Draw the field and optionally overlay a path."""
    grid = [
        [0 for _ in range(field.cols)]
        for _ in range(field.rows)
    ]

    for row, col in field.crops:
        grid[row][col] = 1

    for row, col in field.protected:
        grid[row][col] = 2

    for row, col in field.obstacles:
        grid[row][col] = 3

    colors = ListedColormap([
        "#f7f2df",  # Safe cell
        "#4e9e54",  # Potato row
        "#f2c96b",  # Protected region
        "#59636c",  # Known obstacle
    ])

    ax.imshow(grid, cmap=colors, vmin=0, vmax=3, origin="upper")

    ax.set_xticks(range(field.cols))
    ax.set_yticks(range(field.rows))
    ax.set_xticks(
        [x - 0.5 for x in range(1, field.cols)],
        minor=True,
    )
    ax.set_yticks(
        [y - 0.5 for y in range(1, field.rows)],
        minor=True,
    )
    ax.grid(which="minor", color="white", linewidth=0.6)
    ax.tick_params(labelsize=7, length=0)
    ax.set_xlabel("Column", fontsize=8)
    ax.set_ylabel("Row", fontsize=8)

    if path:
        ax.plot(
            [col for row, col in path],
            [row for row, col in path],
            color="#2764d7",
            linewidth=2.5,
            marker=".",
            markersize=4,
            zorder=3,
        )

    if show_hidden:
        for row, col in field.hidden_obstacles:
            ax.scatter(
                col,
                row,
                marker="X",
                s=85,
                color="#d02c2c",
                edgecolor="white",
                zorder=5,
            )

    markers = (
        (field.start, "S", "#143d8c"),
        (field.goal, "G", "#792099"),
    )

    for (row, col), label, color in markers:
        ax.scatter(
            col,
            row,
            s=160,
            color=color,
            edgecolor="white",
            zorder=6,
        )
        ax.text(
            col,
            row,
            label,
            color="white",
            ha="center",
            va="center",
            weight="bold",
            fontsize=8,
            zorder=7,
        )


def save_visuals(field, results, online):
    """Save the field, routes, and replanning images."""
    legend = [
        Patch(color="#f7f2df", label="Safe"),
        Patch(color="#4e9e54", label="Potato row (blocked)"),
        Patch(color="#f2c96b", label="Protected region (blocked)"),
        Patch(color="#59636c", label="Known obstacle"),
        plt.Line2D(
            [0], [0], color="#2764d7", lw=2, label="Route"
        ),
        plt.Line2D(
            [0],
            [0],
            marker="X",
            color="w",
            markerfacecolor="#d02c2c",
            markersize=9,
            label="Initially hidden obstacle",
        ),
        plt.Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="#143d8c",
            markersize=9,
            label="Start S / goal G",
        ),
    ]

    fig, axes = plt.subplots(
        2, 3, figsize=(16, 9), constrained_layout=True
    )

    draw_field(axes[0, 0], field, show_hidden=True)
    axes[0, 0].set_title(
        "Field (hidden obstacle shown for illustration)"
    )

    for ax, result in zip(axes.flat[1:5], results):
        draw_field(ax, field, result["path"], show_hidden=True)
        ax.set_title(
            f'{result["name"]}: planned on initially known map'
        )

    draw_field(
        axes[1, 2],
        field,
        online["path"],
        show_hidden=True,
    )
    axes[1, 2].set_title(
        "A* online: actual route after detection"
    )

    fig.legend(
        handles=legend,
        loc="outside lower center",
        ncol=4,
        fontsize=9,
    )
    fig.suptitle(
        "Crop-safe field and routes",
        fontsize=16,
        weight="bold",
    )
    fig.savefig(OUTPUT / "field_and_routes.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(
        1, 2, figsize=(13, 5), constrained_layout=True
    )

    draw_field(
        axes[0],
        field,
        online["planned_routes"][0],
        show_hidden=True,
    )
    axes[0].set_title(
        "Before detection: route crosses hidden obstacle"
    )

    draw_field(
        axes[1],
        field,
        online["path"],
        show_hidden=True,
    )

    if online["events"]:
        row, col = online["events"][0][0]
        axes[1].scatter(
            col,
            row,
            s=170,
            facecolor="none",
            edgecolor="#e65f16",
            linewidth=3,
            zorder=8,
        )

    axes[1].set_title(
        "After detection: actual route\n"
        "Orange ring = detection point"
    )

    fig.savefig(OUTPUT / "replanning.png", dpi=160)
    plt.close(fig)


def main():
    OUTPUT.mkdir(exist_ok=True)
    field = Field()

    # Place a hidden obstacle on the initial A* route.
    # This guarantees the example demonstrates replanning.
    initial_path, _ = search(
        field,
        "A*",
        field.start,
        field.goal,
        field.obstacles,
    )

    if len(initial_path) < 3:
        raise RuntimeError(
            "The field needs a route with an interior cell."
        )

    hidden = initial_path[len(initial_path) // 2]
    field.hidden_obstacles = {hidden}

    results = []

    for name in ("BFS", "DFS", "Greedy", "A*"):
        start_time = perf_counter()

        path, expanded = search(
            field,
            name,
            field.start,
            field.goal,
            field.obstacles,
        )

        elapsed = perf_counter() - start_time

        results.append({
            "name": name,
            "path": path,
            "expanded": expanded,
            "seconds": elapsed,
            "reached": bool(path) and path[-1] == field.goal,
            "replans": 0,
        })

    online = online_journey(field)
    all_results = results + [online]

    headings = [
        "Algorithm",
        "Reached",
        "Moves",
        "Path cost",
        "Expanded",
        "Time (ms)",
        "Replans",
    ]

    table = []

    for result in all_results:
        path = result["path"]

        table.append([
            result["name"],
            "Yes" if result["reached"] else "No",
            str(len(path) - 1) if path else "-",
            str(field.path_cost(path)) if path else "-",
            str(result["expanded"]),
            f'{result["seconds"] * 1000:.3f}',
            (
                str(result["replans"])
                if result["name"] == "A* online"
                else "-"
            ),
        ])

    with (OUTPUT / "comparison.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.writer(file)
        writer.writerow(headings)
        writer.writerows(table)

    widths = [
        max(len(str(value)) for value in column)
        for column in zip(headings, *table)
    ]

    def format_row(row):
        return " | ".join(
            str(value).ljust(width)
            for value, width in zip(row, widths)
        )

    separator = "-+-".join("-" * width for width in widths)

    text = (
        format_row(headings)
        + "\n"
        + separator
        + "\n"
        + "\n".join(format_row(row) for row in table)
    )

    print(f"Start: {field.start}")
    print(f"Goal: {field.goal}")
    print(f"Initially hidden obstacle: {hidden}")

    for position, obstacle in online["events"]:
        print(
            f"Detected {obstacle} from {position}; "
            "updated the map and replanned with A*."
        )

    print("\n" + text)

    for result in all_results:
        print(f'\n{result["name"]} path: {result["path"]}')

    (OUTPUT / "comparison.txt").write_text(
        text + "\n",
        encoding="utf-8",
    )

    save_visuals(field, results, online)

    fig, ax = plt.subplots(figsize=(11, 3))
    ax.axis("off")
    ax.set_title(
        "Performance comparison (timings vary by machine)",
        fontsize=14,
        pad=14,
    )

    image_table = ax.table(
        cellText=table,
        colLabels=headings,
        loc="center",
        cellLoc="center",
    )
    image_table.auto_set_font_size(False)
    image_table.set_fontsize(10)
    image_table.scale(1, 2)

    fig.savefig(
        OUTPUT / "comparison.png",
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(fig)

    print(f"\nImages and comparison files saved in: {OUTPUT}")


if __name__ == "__main__":
    main()
