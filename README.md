# 🚗 SunVisorRobot: 차량 운전자 눈부심 방지 선바이저 로봇

**차량 운전 중 발생하는 태양 및 강한 조명으로 인한 눈부심(Glare)을 자동으로 감지하고, 지능적으로 선바이저를 제어하여 운전자의 시야를 안전하게 확보하는 임베디드 시스템 프로젝트입니다.**

## 🌟 프로젝트 목표

*   실시간으로 변화하는 외부 조명 환경에서 Glare를 정확하게 인식합니다.
*   카메라로 인식된 Glare의 위치를 운전자 시야 기준으로 정밀하게 좌표 변환합니다.
*   변환된 좌표에 따라 선바이저를 3x3 그리드 영역으로 신속하고 정확하게 이동시켜 효과적으로 눈부심을 차단합니다.
*   Raspberry Pi에서 고수준 인식 및 판단을, Arduino에서 저수준 모터 제어를 담당하는 분산 시스템으로 구현하여 실시간성과 안정성을 확보합니다.

## 🛠️ 시스템 아키텍처

본 시스템은 크게 다음과 같은 모듈로 구성됩니다:

1.  **Glare 감지 모듈 (C++ on Raspberry Pi):**
    *   Raspberry Pi 카메라 모듈 v3로부터 실시간 영상 스트림을 입력받습니다.
    *   OpenCV 라이브러리를 사용하여 HSV 색 공간 변환, 임계값 필터링, Contour 분석 등을 통해 이미지 내 Glare 영역을 탐지하고 중심 픽셀 좌표를 추출합니다.
2.  **좌표 변환 모듈 (C++ on Raspberry Pi):**
    *   Glare 감지 모듈에서 전달받은 픽셀 좌표를 입력으로 사용합니다.
    *   카메라 내부 파라미터, 렌즈 왜곡 계수, 차량 내 카메라/운전자/윈드실드의 3D 기하학적 관계를 이용하여 픽셀 좌표를 3x3 그리드 좌표 `(col, row)`로 변환합니다.
3.  **시리얼 통신 모듈 (C++ on Raspberry Pi & Arduino):**
    *   Raspberry Pi에서 Arduino로 제어 명령을 전송합니다.
    *   1바이트 통신 프로토콜 사용:
        *   비트 7: Glare 감지 유무 (1: 감지, 0: 미감지/접기)
        *   비트 6-4: 예약
        *   비트 3-2: 컬럼(Col, x) 좌표 (0-2, 2비트)
        *   비트 1-0: 로우(Row, y) 좌표 (0-2, 2비트)
4.  **모터 제어 모듈 (Arduino):**
    *   RPi로부터 수신된 1바이트 명령을 해석합니다.
    *   해석된 그리드 좌표에 따라 스텝 모터와 서보 모터를 제어하여 선바이저를 해당 위치로 이동시킵니다.

## 🌳 프로젝트 디렉토리 구조

```
Embedded/
├── .gitignore
├── README.md
│
├── src/ # 소스 코드 루트
│ ├── glare_detection/ # Glare 감지 및 메인 제어 루프
│ │ ├── CMakeLists.txt
│ │ ├── include/ (SunDetector.h, return_position.h)
│ │ └── src/ (SunDetector.cpp, return_position.cpp, main.cpp)
│ │
│ ├── coordinate_mapping/ # 좌표 변환 모듈
│ │ ├── CMakeLists.txt
│ │ ├── include/ (config.hpp, coordinates.hpp, get_grid_size.hpp)
│ │ └── src/ (coordinates.cpp, test_visualization.cpp 등)
│ │
│ └── serial_communication/ # 시리얼 통신 모듈
│   ├── CMakeLists.txt
│   ├── include/ (serial_communication.hpp)
│   └── src/ (serial_communication.cpp)
│
├── CAD/ # HW 제작 CAD 파일
│  
└── arduino/ # Arduino 모터 제어 코드
```

## ⚙️ 개발 환경 및 필수 설치 요소

*   **하드웨어:**
    *   Raspberry Pi 5 (또는 상위 모델)
    *   Raspberry Pi 카메라 모듈 v3
    *   Arduino Mega 2560 (또는 호환 보드)
    *   스텝 모터 및 드라이버
    *   서보 모터
    *   기타 전원 공급 장치, 회로 부품
*   **소프트웨어 (Raspberry Pi):**
    *   Raspberry Pi OS (Debian 기반 Linux)
    *   C++ 컴파일러 (g++ 7.5.0 이상, C++17 지원)
    *   CMake (3.10 이상)
    *   OpenCV 개발 라이브러리 (`libopencv-dev`)
    *   Python 3.x (주로 빌드 스크립트, 테스트 또는 유틸리티용)
    *   pip (Python 패키지 관리자)
    *   PlatformIO Core (예시/Arduino 코드 빌드 및 업로드용, `pip install platformio`로 설치)
    *   Git
*   **소프트웨어 (개발용 PC - 선택적):**
    *   C++ IDE (VS Code with CMake Tools, CLion 등)
    *   Arduino IDE 또는 PlatformIO IDE (VS Code 확장)

## 🚀 설정, 빌드 및 실행 방법

**1. 소스 코드 클론:**
```bash
git clone https://github.com/HiSeongHyeon/Embedded.git
cd Embedded
```

**2. 필수 패키지 설치 (Raspberry Pi 터미널에서):**
```bash
sudo apt update
sudo apt install build-essential g++ cmake libopencv-dev python3-opencv python3-pip git
```

**3. 개별 빌드 및 실행 (수동으로 진행할 경우):**

*   **C++ 모듈 (Glare 감지 및 메인 제어):**
    ```bash
    cd Embedded # 프로젝트 루트 (최상위 CMakeLists.txt가 있는 곳)
    mkdir build && cd build
    cmake ..
    make
    # 실행 (예시)
    ./src/glare_detection/glareTracker
    ```

*   **실행 전 아래와 같은 Arduino 업데이트 필수**
    ```bash
    cd arduino
    pio run # 빌드 (예시/다른 IDE 사용 가능)
    pio run -t upload # 업로드 (포트 자동 감지 또는 platformio.ini에 지정)
    ```

## 📽️ 시연 영상
프로젝트의 작동 방식을 아래 영상으로 확인해보세요.
[![시연 영상]](https://www.youtube.com/watch?v=IC63PYgABys)

## 👏 포스터
![포스터](assets/poster.png)

## 🤝 팀원

*   **최성현 (팀장): doq1324@uos.ac.kr**
*   **이종현: whdgus985@uos.ac.kr**
*   **오정택: wjdxor9534@uos.ac.kr**
*   **조경호: vitus00@uos.ac.kr**
*   **조민규: gyull911@uos.ac.kr**