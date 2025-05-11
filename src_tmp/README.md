# 임시 통합

## 프로젝트 구조

```
src_tmp/                    # 프로젝트 최상위 디렉토리리
├── CMakeLists.txt          # CMakeLists 파일일
├── src/                    
│   ├── coordinate_mapping/     # 좌표 변환 모듈
│   ├── glare_detection/        # glare 인식 모듈
│   ├── serial_communication/   # 시리얼 통신 모듈
└── build/              
    └── src         
        └── ...     # 각 모듈별 실행파일 여기에
```

---

## 빌드 방법

```bash
cd src_tmp/build
cmake ..
make
```

## 실행 방법 (예시: main 함수)

```bash
cd src_tmp/build/src/glare_detection
./solarTracker
```

---

## TODO

- [ ] 카메라 있는 환경에서 잘 돌아가는지 체크
- [ ] 시리얼 통신 추가