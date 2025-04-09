from mapping import camera_to_driver_coords
from visualize.grid_display import visualize_grid_image


def main():
    # bbox = (300, 100, 340, 140)  # 임의의 YOLO 바운딩 박스
    sun_center = (320, 120)  # 임의의 YOLO 태양 중심점
    grid = camera_to_driver_coords(sun_center)
    print(f"Grid Position: {grid}")
    visualize_grid_image(grid)


if __name__ == "__main__":
    main()
