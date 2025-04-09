# 운전자 기준 전방 유리에 투영되는 태양의 위치

## 프로젝트 구조

```
sunvisor_system/
├── main.py                 # 메인 실행 파일
├── mapping/                # 좌표 변환 로직
│   ├── coordinates.py      # 카메라 → 운전자 기준 좌표 변환
│   ├── config.py           # 매핑 관련 기본 상수 설정
│   ├── get_grid_size.py    # 그리드 사이즈 계산
│   └── __init__.py
└── visualize/              # 그리드 시각화 (coordinates.py 테스트 용 프로젝트엔 필요 x)
    ├── grid_display.py     # Matplotlib 시각화 도구
    └── __init__.py
```

---

## 실행 방법

```bash
python sunvisor_system/main.py
```

---

## 주요 함수 설명

### `camera_to_driver_coords(sun_center, ...)`

- **설명**: YOLO가 반환한 태양 중심점을 운전자 기준 전방 유리상의 그리드로 변환
- **매개변수**:
  - `sun_center`: (x, y) 형태의 태양 중심점 좌표
- **리턴값**: `(grid_x, grid_y)`
- **사용 위치**: `/sunvisor_system/main.py` 또는 외부 제어 모듈에서 호출

---

## 예시 출력

```
Grid Position: (2, 0)
[ ] [ ] [X] [ ] [ ] ~
[ ] [ ] [ ] [ ] [ ] ~
[ ] [ ] [ ] [ ] [ ] ~
 ~   ~   ~   ~   ~  ~
```

---

## TODO

- [ ] 그리드 사이즈 계산 로직 수정
- [ ] config.py에 들어가있는 디폴트값 수정
- [ ] test code 수정
- [ ] 카메라 캘리브레이션