from typing import Tuple

from .config import DEFAULT_GLASS_SIZE, DEFAULT_SUN_PROJECTED_DIAMETER


def get_grid_size() -> Tuple[int, int]:
    """
    전방 유리의 물리적 크기(DEFAULT_GLASS_SIZE)와
    태양 투영 직경(DEFAULT_SUN_PROJECTED_DIAMETER)을 기반으로
    grid의 열과 행 수를 계산

    여기서는 각 cell의 크기를 태양 투영 직경으로 설정
    """
    cell_resolution = DEFAULT_SUN_PROJECTED_DIAMETER  # 각 셀의 크기 (m)
    glass_width, glass_height = DEFAULT_GLASS_SIZE
    grid_cols = max(1, int(glass_width / cell_resolution))
    grid_rows = max(1, int(glass_height / cell_resolution))
    return (grid_cols, grid_rows)
