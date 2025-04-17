from typing import Tuple

import numpy as np

# 이미지 해상도 (픽셀 단위): (width, height)
DEFAULT_IMAGE_SIZE: Tuple[int, int] = (640, 480)

# 카메라 시야각 (수평, 수직) in degrees
DEFAULT_FOV: Tuple[float, float] = (70, 60)

# 차량 내 카메라 위치 (미터 단위): (x, y, z)
# (0.5, 0.5, 1.0)으로 설정
DEFAULT_CAMERA_POS: Tuple[float, float, float] = (0.5, 0.5, 1.0)

# 운전자 눈 위치 (미터 단위): (x, y, z)
# (0.25, 0.0, 1.2)로 설정
DEFAULT_DRIVER_POS: Tuple[float, float, float] = (0.25, 0.0, 1.2)

# 운전자 좌표계에서 앞유리(윈드실드) 평면의 y 좌표 (미터 단위)
DEFAULT_WINDSHIELD_Y: float = 1.0

# 윈드실드(유리창)의 물리적 크기 (미터 단위): (width, height)
DEFAULT_GLASS_SIZE: Tuple[float, float] = (1.0, 0.5)

# 윈드실드(유리창) 좌상단 물리 좌표 (미터 단위): (x_left, z_top)
# 이미지 처리 방식(좌상단 기준)에 맞게 설정 (예: 윈드실드 영역의 좌측 상단이 (0, 1.5)라고 가정)
DEFAULT_GLASS_ORIGIN: Tuple[float, float] = (0.0, 1.5)

# 태양의 평균 시각 직경은 약 0.53°
# 운전자 좌표계에서 운전자 눈과 윈드실드(앞유리) 사이 거리를 1.0 m로 가정
# 태양 투영 직경 = 2 * tan(0.53°/2) ≈ 0.0093 m (즉, 약 9.3 mm)
DEFAULT_SUN_PROJECTED_DIAMETER: float = 0.0093

# 라즈베리 파이 카메라 모듈 v3에 대한 내부 매개변수(예시값)
DEFAULT_CAMERA_MATRIX: np.ndarray = np.array(
    [[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32
)

# 왜곡 계수 (k1, k2, p1, p2, k3) – 예시값
DEFAULT_DISTORTION_COEFFICIENTS: np.ndarray = np.array(
    [-0.2, 0.1, 0.0, 0.0, 0.0], dtype=np.float32
)
