import unittest

from src.sunvisor_system.mapping import coordinates
from src.sunvisor_system.mapping.config import DEFAULT_IMAGE_SIZE
from src.sunvisor_system.visualize.grid_display import visualize_grid_image


# 테스트를 위해 get_grid_size 함수를 덮어씁니다.
# get_grid_size를 테스트에서 고정된 값으로 설정 (예: 100 열, 50 행)
def dummy_get_grid_size():
    return (100, 50)


# monkey-patch: 원래 get_grid_size 대신 dummy_get_grid_size를 사용합니다.
coordinates.get_grid_size = dummy_get_grid_size


class TestCameraToDriverCoords(unittest.TestCase):
    def test_center(self):
        """
        카메라 이미지 중앙에 위치한 태양 (픽셀 좌표 (320,240))이
        운전자 시점으로 변환될 때, 예상되는 그리드 좌표는 (50, 0)
        """
        x, y = DEFAULT_IMAGE_SIZE
        sun_center = (x / 2, y / 2)  # 이미지 중앙
        expected = (50, 0)
        result = coordinates.camera_to_driver_coords(sun_center)
        self.assertEqual(result, expected)

    def test_top_left(self):
        """
        이미지 좌측 상단 (픽셀 좌표 (0,0))에 태양이
        운전자 시점으로 변환될 때, 예상되는 그리드 좌표는 (0, 49))
        """
        sun_center = (0, 0)  # 왼쪽 상단
        result = coordinates.camera_to_driver_coords(sun_center)
        expected = (0, 49)
        self.assertEqual(result, expected)

    def test_bottom_right(self):
        """
        이미지 우측 하단 (픽셀 좌표 (640,480))에 태양이이
        운전자 시점으로 변환될 때, 예상되는 그리드 좌표는 (99, 0)
        """
        x, y = DEFAULT_IMAGE_SIZE
        sun_center = (x, y)  # 오른쪽 하단
        result = coordinates.camera_to_driver_coords(sun_center)
        expected = (99, 0)
        self.assertEqual(result, expected)

    def test_visualize(self):
        x, y = DEFAULT_IMAGE_SIZE
        sun_center = (x / 2, y / 2)  # 이미지 중앙
        result = coordinates.camera_to_driver_coords(sun_center)
        print(f"Grid Position: {result}")
        visualize_grid_image(result)


if __name__ == "__main__":
    unittest.main()
