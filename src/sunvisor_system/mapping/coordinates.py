import math
from typing import Tuple

import cv2
import numpy as np

from .config import (
    DEFAULT_CAMERA_MATRIX,
    DEFAULT_CAMERA_POS,
    DEFAULT_DISTORTION_COEFFICIENTS,
    DEFAULT_DRIVER_POS,
    DEFAULT_FOV,
    DEFAULT_GLASS_ORIGIN,
    DEFAULT_GLASS_SIZE,
    DEFAULT_IMAGE_SIZE,
    DEFAULT_WINDSHIELD_Y,
)
from .get_grid_size import get_grid_size


def ray_plane_intersection(
    origin: np.ndarray, direction: np.ndarray, plane_y: float
) -> np.ndarray:
    """
    운전자 좌표계에서, origin에서 출발한 정규화된 direction 방향의 ray와
    y = plane_y 평면과의 교차점을 계산

    Ray 방정식: P = origin + t * direction,
            where t = (plane_y - origin[1]) / direction[1]
    """
    t = (plane_y - origin[1]) / direction[1]

    return origin + t * direction


def to_bit_list(grid_coord: Tuple[int, int], bits: int = 4) -> list[int]:
    """
    grid_coord: (col, row) 각각 0~(2^bits-1) 범위인 정수를 받아,
    [b3, b2, b1, b0] 형태의 리스트로 리턴한다.
    """
    num_cols, _ = get_grid_size()
    value = grid_coord[0] * num_cols + grid_coord[1]

    return [(value >> i) & 1 for i in reversed(range(bits))]


def camera_to_driver_coords(
    sun_center: Tuple[float, float],
    image_size: Tuple[int, int] = DEFAULT_IMAGE_SIZE,
    fov: Tuple[float, float] = DEFAULT_FOV,
    camera_pos: Tuple[float, float, float] = DEFAULT_CAMERA_POS,
    driver_eye_pos: Tuple[float, float, float] = DEFAULT_DRIVER_POS,
    windshield_y: float = DEFAULT_WINDSHIELD_Y,
    glass_size: Tuple[float, float] = DEFAULT_GLASS_SIZE,
    glass_origin: Tuple[float, float] = DEFAULT_GLASS_ORIGIN,
) -> Tuple[int, int]:
    """
    카메라 이미지 상의 태양 중심(픽셀 단위)을 받아 옴.
    1. OpenCV의 undistort 과정을 통해 왜곡을 보정
    2. 카메라의 FOV를 고려하여 방향 벡터를 계산
    3. 운전자 좌표계(driver_eye_pos)를 원점으로 하는 ray와 y = windshield_y 평면의
       교차점을 계산
    4. 이 물리적 교차점을 glass_origin(좌상단 기준) 및 glass_size를 기반으로
       윈드실드 grid 좌표(좌상단 기준)로 매핑
    """
    # 왜곡 보정: 입력된 sun_center 픽셀 좌표를 보정
    sun_center_np = np.array([[sun_center]], dtype=np.float32)
    undistorted = cv2.undistortPoints(
        sun_center_np,
        DEFAULT_CAMERA_MATRIX,
        DEFAULT_DISTORTION_COEFFICIENTS,
        P=DEFAULT_CAMERA_MATRIX,
    )
    corrected_sun_center = undistorted[0, 0]  # (x, y) 보정된 좌표

    # 1. 보정된 태양 중심(픽셀)을 [-1, 1] 범위로 정규화
    img_w, img_h = image_size
    fov_x, fov_y = fov
    x_center, y_center = corrected_sun_center
    norm_x = (x_center / img_w - 0.5) * 2
    norm_y = (y_center / img_h - 0.5) * 2

    # 2. 카메라 좌표계에서 fov를 고려하여 방향 벡터를 계산
    angle_x = math.radians(norm_x * (fov_x / 2))
    angle_y = math.radians(norm_y * (fov_y / 2))
    dx = math.tan(angle_x)
    dy = 1.0  # 카메라 기본 전방은 +y 방향
    dz = -math.tan(angle_y)
    D = np.array([dx, dy, dz])
    D /= np.linalg.norm(D)

    # 3. 광선의 원점은 driver_eye_pos로 설정
    origin_arr = np.array(driver_eye_pos)
    intersection = ray_plane_intersection(origin_arr, D, windshield_y)
    px, _, pz = intersection

    # 4. 물리적 교차점 (px, pz)을 glass_origin과 glass_size를 기반으로 grid 좌표로 매핑
    grid_cols, grid_rows = get_grid_size()
    glass_width, glass_height = glass_size
    x_left, z_top = glass_origin  # glass_origin은 좌상단 기준 (x_left, z_top)

    grid_x = int(((px - x_left) / glass_width) * grid_cols)
    grid_y = int(((z_top - pz) / glass_height) * grid_rows)

    grid_x = max(0, min(grid_x, grid_cols - 1))
    grid_y = max(0, min(grid_y, grid_rows - 1))

    return (grid_x, grid_y)
