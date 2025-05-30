# Raspberry Pi 카메라 체커보드 캘리브레이션 스크립트

이 Python 스크립트는 Raspberry Pi에서 **picamera2** 또는 일반 **OpenCV VideoCapture** 카메라를 사용하여 **체커보드 기반 카메라 캘리브레이션**을 수행하는 도구입니다. OpenCV의 `cv2.calibrateCamera()`를 활용하며, 결과를 `config.hpp` 형식으로 출력해 외부 C++ 프로젝트에서 사용할 수 있도록 지원합니다.

---

## 📌 주요 기능

- 체커보드 이미지 캡처
- 체커보드 코너 검출 및 시각화
- 3D-2D 포인트 수집 및 캘리브레이션
- 카메라 내부 행렬 및 왜곡 계수 추정
- 평균 재투영 오차 계산
- `config.hpp` 형태로 결과 출력

---

## 📁 파일 구조

```
calibration_images/      # 캘리브레이션 이미지가 저장되는 폴더 (자동 생성)
calibrate_camera.py      # 본 Python 스크립트
```

---

## ⚙️ 요구사항

### Python 패키지
- `numpy`
- `opencv-python`
- `picamera2`

설치 명령:
```bash
pip install numpy opencv-python
sudo apt install python3-picamera2
```

### 체커보드 준비
- 내부 코너 수: 예) 9x6 체커보드라면 `가로 8`, `세로 5`로 설정
- 각 칸 크기(mm): 실제 사용한 체커보드 칸의 물리적 길이 (예: 25.0 mm)

---

## 🚀 실행 방법

```bash
python3 calibrate_camera.py
```

### 실행 중 조작
- **스페이스바**: 이미지 캡처  
- **ESC**: 수동 종료  

총 `NUM_IMAGES_TO_CAPTURE` 개수만큼 캡처되면 자동 종료 후 캘리브레이션 수행

---

## ⚙️ 설정 변경 (코드 상단)

```python
CHECKERBOARD_WIDTH = 8         # 내부 가로 코너 수
CHECKERBOARD_HEIGHT = 5        # 내부 세로 코너 수
SQUARE_SIZE_MM = 25.0          # 칸 하나의 실제 길이 (mm)
NUM_IMAGES_TO_CAPTURE = 20     # 수집할 이미지 수
IMAGE_SAVE_PATH = "./calibration_images/"  # 저장 경로
```

---

## 📤 출력 예시

캘리브레이션 성공 시, 다음 형식으로 결과 출력됨:

```cpp
const cv::Mat DEFAULT_CAMERA_MATRIX = (cv::Mat_<double>(3, 3) <<
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);

const cv::Mat DEFAULT_DISTORTION_COEFFICIENTS = (cv::Mat_<double>(1, 5) <<
    k1, k2, p1, p2, k3);
```

→ C++ 프로젝트에 복붙하여 바로 활용 가능

---

## ✅ 참고 사항

- 최소 10장 이상의 다양한 각도/거리에서 촬영된 이미지가 필요합니다 (권장: 15~25장).
- 체커보드는 프레임 안에서 **기울어지고**, **작아지고**, **가장자리로 치우치는** 조건으로 다양하게 찍을수록 정확도가 올라갑니다.

---

## 🔚 종료 시

- `ESC`를 누르거나 오류 발생 시 종료됨
- 종료 후 `calibration_images/` 폴더에 캡처 이미지가 저장됩니다

---
