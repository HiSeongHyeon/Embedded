import matplotlib.pyplot as plt
from mapping.get_grid_size import get_grid_size


def visualize_grid_image(grid_pos):
    grid_size = get_grid_size()
    cols, rows = grid_size

    _, ax = plt.subplots()
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols + 1))
    ax.set_yticks(range(rows + 1))
    ax.grid(True)

    # 그리드 안쪽 채우기
    for y in range(rows):
        for x in range(cols):
            color = "orange" if (x, y) == grid_pos else "white"
            rect = plt.Rectangle(
                (x, y),
                1,
                1,
                facecolor=color,
                edgecolor="black",
            )
            ax.add_patch(rect)

    ax.set_aspect("equal")
    ax.invert_yaxis()  # (0, 0)이 좌상단처럼 보이게
    plt.title(f"Sun Position Grid: {grid_pos}")
    plt.show()
