# Embedded
For 2025 Embedded

# 초기 파일 설명

## `.gitignore`

> GitHub에 특정 형식의 파일들이 커밋되지 않도록 방지해주는 파일

- 각자의 작업 공간에서 따로 설정할 필요 없이 자동으로 활성화됨
- 예: 캐시, 로그, 가상환경 등 불필요한 파일 무시

---

## `.gitattributes`

> 줄바꿈(EOL) 문제 방지 + `git diff` 시 코드/문서 파일을 더 직관적으로 보여주는 설정

- 운영체제별 인코딩 차이(`CRLF` ↔ `LF`) 자동 처리
- 예: `*.md`, `*.py`, `*.cpp` 등 파일 형식에 따라 diff 방식 설정

---

## `.gitmessage`

> Git commit 시 기본으로 출력되는 커밋 메시지 템플릿

### 사용법:

```bash
git config --global commit.template /(YOUR_WORKSPACE)/Embedded/.gitmessage
git commit    # 또는: git commit <file_name>
```

---

## `.pre-commit-config.yaml`

> 커밋 전에 자동으로 코드 형식(format) 검사 및 정리해주는 설정 파일

- 따로 설정할 필요 없이 커밋 시 자동 실행됨
- 미리 실행해보고 싶다면:

```bash
pre-commit clean
pre-commit run --all-files
```

---

## `requirements.txt`

> 프로젝트에서 사용하는 파이썬 라이브러리 목록 정리

- 예시: `numpy`, `matplotlib`, `pytest` 등
- 설치 방법:

```bash
pip install -r requirements.txt --encoding utf-8
```

---

## `/tests/` 디렉토리

> 테스트 코드를 정리해두는 공간

### 파이썬 테스트 파일

- 파일명 형식: `test_*.py`, `*_test.py`

### 실행 방법:

```bash
pip install pytest
pytest
```