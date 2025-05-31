import glob
import time

import cv2
import numpy as np

# -----------------------------------------------------
# 체커보드 설정 (사용하는 패턴에 맞게 수정)
# -----------------------------------------------------
# 체커보드 내부 코너의 가로 개수 (예: 9x6 보드면 가로 코너 8개)
CHECKERBOARD_WIDTH = 8  # 사용한 체커보드의 가로 내부 코너 수 - 1
# 체커보드 내부 코너의 세로 개수 (예: 9x6 보드면 세로 코너 5개)
CHECKERBOARD_HEIGHT = 5  # 사용한 체커보드의 세로 내부 코너 수 - 1
# 체커보드 한 칸의 실제 크기 (예: mm 단위)
SQUARE_SIZE_MM = 25.0  # 예시: 한 변이 25mm

# 캡처할 이미지 개수
NUM_IMAGES_TO_CAPTURE = 20  # 최소 10-15개 이상 권장, 다양할수록 좋음

# 이미지 저장 경로 (RPi에 디렉토리 미리 생성)
IMAGE_SAVE_PATH = "./calibration_images/"
# -----------------------------------------------------

print("카메라 캘리브레이션을 시작합니다.")
print(f"체커보드 내부 코너: 가로 {CHECKERBOARD_WIDTH}, 세로 {CHECKERBOARD_HEIGHT}")
print(f"체커보드 한 칸 크기: {SQUARE_SIZE_MM}mm")
print(f"캡처할 이미지 개수: {NUM_IMAGES_TO_CAPTURE}")
print(f"이미지 저장 경로: {IMAGE_SAVE_PATH}")
print("-----------------------------------------------------")
print("카메라를 실행합니다. 체커보드를 다양한 각도와 위치에서 보여주세요.")
print("스페이스바를 누르면 이미지가 캡처됩니다. ESC를 누르면 종료합니다.")
print("-----------------------------------------------------")

# 3D 월드 좌표계에서 체커보드 코너 포인트 준비 (z=0으로 가정)
objp = np.zeros((CHECKERBOARD_HEIGHT * CHECKERBOARD_WIDTH, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_WIDTH, 0:CHECKERBOARD_HEIGHT].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE_MM  # 실제 크기 적용

# 모든 이미지에서 검출된 3D 포인트(objpoints)와 2D 이미지 포인트(imgpoints)를 저장할 배열
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane.

# 이 스크립트 실행 전에 RPi에 picamera2 설치 필요: sudo apt install -y python3-picamera2
try:
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})  # 이미지 크기 설정
    picam2.configure(config)
    picam2.start()
    print("picamera2 초기화 성공.")
    use_picamera2 = True
except ImportError:
    print("picamera2 라이브러리를 찾을 수 없습니다. OpenCV VideoCapture를 시도합니다.")
    print("RPi 카메라 사용 시 GStreamer 파이프라인이 필요할 수 있습니다.")
    cap = cv2.VideoCapture(0)  # 기본 카메라 시도
    if not cap.isOpened():
        print("오류: 카메라를 열 수 없습니다. VideoCapture(0) 실패.")
        exit()
    print("OpenCV VideoCapture 초기화 성공 (카메라 0번).")
    use_picamera2 = False
except Exception as e:
    print(f"카메라 초기화 중 오류 발생: {e}")
    exit()


# 이미지 저장 디렉토리 생성
import os

if not os.path.exists(IMAGE_SAVE_PATH):
    os.makedirs(IMAGE_SAVE_PATH)

images_captured_count = 0
frame_width, frame_height = (640, 480)  # 기본값, 실제 프레임에서 다시 얻을 수 있음

while images_captured_count < NUM_IMAGES_TO_CAPTURE:
    if use_picamera2:
        frame = picam2.capture_array()
        # picamera2는 기본적으로 RGB 배열을 반환, OpenCV는 BGR을 사용
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        ret, frame = cap.read()
        if not ret:
            print("오류: 프레임을 읽을 수 없습니다.")
            break

    if frame_width is None or frame_height is None:
        frame_height, frame_width = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾기
    ret_corners, corners = cv2.findChessboardCorners(
        gray, (CHECKERBOARD_WIDTH, CHECKERBOARD_HEIGHT), None
    )

    # 코너가 성공적으로 찾아지면
    if ret_corners:
        # 코너 위치를 더 정확하게 찾기 (서브픽셀 단위)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 화면에 코너 그리기
        cv2.drawChessboardCorners(
            frame, (CHECKERBOARD_WIDTH, CHECKERBOARD_HEIGHT), corners2, ret_corners
        )

        cv2.imshow("Calibration Image - Press SPACE to capture, ESC to quit", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("캘리브레이션 중단됨.")
            break
        elif key == 32:  # SPACE
            imgpoints.append(corners2)
            objpoints.append(objp)
            images_captured_count += 1
            img_name = os.path.join(
                IMAGE_SAVE_PATH, f"calib_img_{images_captured_count}.png"
            )
            cv2.imwrite(img_name, frame)  # 캡처된 이미지 저장 (코너 그려진 것)
            print(
                f"이미지 {images_captured_count}/{NUM_IMAGES_TO_CAPTURE} 캡처됨: {img_name}"
            )
            # 다음 캡처를 위해 잠시 대기 (체커보드 위치 변경 시간)
            if images_captured_count < NUM_IMAGES_TO_CAPTURE:
                print("체커보드 위치와 각도를 변경하고 스페이스바를 누르세요...")
            time.sleep(0.5)  # 약간의 딜레이
    else:
        cv2.imshow("Calibration Image - Press SPACE to capture, ESC to quit", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("캘리브레이션 중단됨.")
            break

if images_captured_count < 5:  # 유효한 이미지가 너무 적으면 캘리브레이션 불가
    print(f"캘리브레이션을 위한 이미지가 충분하지 않습니다 ({images_captured_count}개). 최소 5개 이상 필요합니다.")
else:
    print(f"\n총 {images_captured_count}개의 이미지로 캘리브레이션을 수행합니다...")
    # 카메라 캘리브레이션 수행
    ret_calib, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    # gray.shape[::-1]은 (width, height) 튜플을 의미

    if ret_calib:
        print("\n캘리브레이션 성공!")
        print("\n카메라 내부 파라미터 행렬 (Camera Matrix):")
        print(camera_matrix)
        print("\n왜곡 계수 (Distortion Coefficients):")
        print(dist_coeffs)

        # (선택 사항) 재투영 오차 계산
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print(f"\n총 평균 재투영 오차 (Mean Reprojection Error): {mean_error / len(objpoints)}")

        # 결과를 config.hpp 형식으로 출력 (복사해서 사용)
        print("\n--- config.hpp 에 추가할 내용 ---")
        print("const cv::Mat DEFAULT_CAMERA_MATRIX = (cv::Mat_<double>(3, 3) <<")
        print(f"    {camera_matrix[0,0]}, {camera_matrix[0,1]}, {camera_matrix[0,2]},")
        print(f"    {camera_matrix[1,0]}, {camera_matrix[1,1]}, {camera_matrix[1,2]},")
        print(f"    {camera_matrix[2,0]}, {camera_matrix[2,1]}, {camera_matrix[2,2]});")
        print(
            "\nconst cv::Mat DEFAULT_DISTORTION_COEFFICIENTS = (cv::Mat_<double>(1, 5) <<"
        )  # k1, k2, p1, p2, k3 가정
        print(
            f"    {dist_coeffs[0,0]}, {dist_coeffs[0,1]}, {dist_coeffs[0,2]}, {dist_coeffs[0,3]}, {dist_coeffs[0,4]});"
        )
        print("--- config.hpp 내용 끝 ---")

    else:
        print("\n캘리브레이션 실패.")

# 자원 해제
if use_picamera2:
    picam2.stop()
    print("picamera2 중지됨.")
else:
    if "cap" in locals() and cap.isOpened():
        cap.release()
        print("VideoCapture 중지됨.")
cv2.destroyAllWindows()
