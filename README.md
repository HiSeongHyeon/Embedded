# Embedded
For 2025 Embedded

## 초기 파일 설명
- .gitignore: 커밋할 때 github에 특정 형식의 파일들이 커밋 되지 않도록 방지해주는 파일
    각자 작업공간에서 다른 거 설정할 필요없이 자동으로 활성화 됨

- .gitattributes: 윈도우와 리눅스(or 맥)의 인코딩 차이로 인한 오류를 방지해주기 위한 자동 줄바꿈 설정(리눅스에서 사용되는 LF로) + modified 상태의 파일을 git diff로 확인할 때 좀 더 직관적으로 볼 수 있게 해주는 설정

- .gitmessage: $ git commit 사용 시 메시지 템플릿 출력되고 거기서 메시지 작성할 수 있게 해주는 파일
    - 사용법
        - $ git config --global commit.template /(YOUR_WORKSPACE)/Embedded/.gitmessage
        - $ git commit (or $ git commit (file_name))

- .pre-commit-config.yaml: commit 전에 코드 형식을 자동으로 포맷팅해주고 검사해주는 파일
    - 사전 설정 필요없이 커밋 전에 자동으로 한번 해주고, 만약 커밋하기 전에 사용해보고 싶다면
        - $ pre-commit clean
        $ pre-commit run --all-files

- requirements.txt: 이건 그냥 각자 사용한 라이브러리(install 해줘야 되는 것들) 정리해두는 파일

- /tests/: 테스트 코드 정리해두는 디렉토리
    - 파이썬 코드의 경우(test_*.py, *_test.py)
        - $ pip install pytest
        - $ pytest