#include <Servo.h>

// ----------- 핀번호 설정 --------------
// 부저
#define buzzer 4

// 서보모터
#define Servo_main 2
#define Servo_mini 3

// 스텝모터
#define dir_1 28
#define steps_1 5
#define ms1_1 6
#define ms2_1 7
#define ms3_1 8
#define dir_2 9
#define steps_2 10
#define ms1_2 11
#define ms2_2 12
#define ms3_2 13

// ------------ 서보모터 제어 설정 ------------------
Servo mainServo;
Servo miniServo;

// ------------- 부저 멜로디 설정 ---------------------
int arraySize = 5;
int melody[] = {245, 260, 292, 328, 348, 390, 439, 492, 523, 586, 1317};
int song1[]   = {1, 2, 3, 4, 5};
int song2[]   = {5, 4, 3, 2, 1};
int noteDuration[] = {2, 2, 2, 2, 2};

// ------------- 위치 테이블 (1~9) ---------------------
int positionTable[9][2] = {
  {-1200,    0},   // 1
  {-600,   600},   // 2
  {   0,  1200},   // 3
  {-850,    0},    // 4
  {-425,  425},    // 5
  {   0,  850},    // 6
  {-400,    0},    // 7
  {-200,  200},    // 8
  {   0,  400}     // 9
};

// 위치 추적 변수
int position_left  = -600;   // 왼쪽 모터의 현재 위치 (음수)
int position_right = 600;    // 오른쪽 모터의 현재 위치 (양수)
int currentIndex = 0; // 최신 위치 (1번째)

// 글래어 디텍트 이전상태 저장 변수
bool prevGlareDetected = false;   // 이전 루프에서의 값 (초기값은 false)

// =========================================================================================

// ▼▼▼▼▼ 테스트용 더미 코드 활성화/비활성화 ▼▼▼▼▼
// #define ENABLE_ASCII_DEBUG_INPUT // 이 줄을 주석 처리하면 RPi와의 1바이트 통신 모드로 작동

void setup() {
  // 서보모터
  mainServo.attach(Servo_main);
  miniServo.attach(Servo_mini);

  // 스텝모터
  pinMode(dir_1, OUTPUT); pinMode(steps_1, OUTPUT);
  pinMode(ms1_1, OUTPUT); pinMode(ms2_1, OUTPUT); pinMode(ms3_1, OUTPUT);
  pinMode(dir_2, OUTPUT); pinMode(steps_2, OUTPUT);
  pinMode(ms1_2, OUTPUT); pinMode(ms2_2, OUTPUT); pinMode(ms3_2, OUTPUT);
  digitalWrite(ms1_1, LOW); digitalWrite(ms2_1, LOW); digitalWrite(ms3_1, HIGH);
  digitalWrite(ms1_2, LOW); digitalWrite(ms2_2, LOW); digitalWrite(ms3_2, HIGH);

  // 부저
  pinMode(buzzer, OUTPUT);

  Serial.begin(115200);
  delay(100);


  // Serial 통신 시작
  #ifdef ENABLE_ASCII_DEBUG_INPUT
    Serial.println("ASCII Debug Input Mode: Send '0' to retract, '1'-'9' for grid.");
  #else
    Serial.println("RPi Communication Mode: Waiting for 1-byte command.");
  #endif
}

// ------------------------- 스텝모터 위치 제어 ----------------------------

void moveTo(int target_left, int target_right) {
  int steps_needed = abs(target_left - position_left);  // 동일 스텝 수 가정
  bool dir_left = (target_left > position_left) ? HIGH : LOW;
  bool dir_right = (target_right > position_right) ? HIGH : LOW;

  digitalWrite(dir_1, dir_left);
  digitalWrite(dir_2, dir_right);

  for (int i = 0; i < steps_needed; i++) {
    digitalWrite(steps_1, HIGH);
    digitalWrite(steps_2, HIGH);
    delayMicroseconds(1600);
    digitalWrite(steps_1, LOW);
    digitalWrite(steps_2, LOW);
    delayMicroseconds(1600);

    position_left  += (dir_left == HIGH) ? 1 : -1;
    position_right += (dir_right == HIGH) ? 1 : -1;
  }
}

// ----------------------- 계단식 이동 구현 -----------------------------

void moveToGridPosition(int targetIndex) {
  int curRow = currentIndex / 3;
  int curCol = currentIndex % 3;
  int tgtRow = targetIndex / 3;
  int tgtCol = targetIndex % 3;

  // 1단계: 현재 층의 중간으로 이동
  if (curCol != 1) {
    int midIndex = curRow * 3 + 1;
    moveTo(positionTable[midIndex][0], positionTable[midIndex][1]);
    currentIndex = midIndex;
  }
  delay(500);

  // 2단계: 다른 층의 중간으로 이동
  if ((currentIndex / 3) != tgtRow) {
    int tgtMidIndex = tgtRow * 3 + 1;
    moveTo(positionTable[tgtMidIndex][0], positionTable[tgtMidIndex][1]);
    currentIndex = tgtMidIndex;
  }
  delay(500);

  // 3단계: 최종 복구
  if (currentIndex != targetIndex) {
    moveTo(positionTable[targetIndex][0], positionTable[targetIndex][1]);
    currentIndex = targetIndex;
  }
}

// ----------------------- 썬가드 작동 함수 -----------------------------
void sungard_down() {
  for (int note = 0; note < arraySize; note++){
    int duration = 700/noteDuration[note];
    tone(buzzer, melody[song1[note]], duration);
    delay(duration+30);
  }
  delay(500);
  for (int i = 110; i >=0; i--) {
        mainServo.write(i);
        delay(10);
  }
}

void sungard_up() {
  for (int note2 = 0; note2 < arraySize; note2++){
    int duration2 = 700/noteDuration[note2];
    tone(buzzer, melody[song2[note2]], duration2);
    delay(duration2+30);
  }
  delay(500);
  for (int i = 0; i <= 110; i++) {
        mainServo.write(i);
        delay(10);
  }
}

// ----------------------- 미니서보 작동 함수 -----------------------------
void minisurvo_out() {
  for (int i = 0; i <= 90; i++) {
    miniServo.write(i);
    delay(10);
  }
}

void minisurvo_in() {
  for (int i = 90; i >= 0; i--) {
    miniServo.write(i);
    delay(10);
  }
}

// =========================================================================================

int get_grid_coords(int index) {
  switch(index) {
    case 0b0000: return 3;
    case 0b0100: return 2;
    case 0b1000: return 1;
    case 0b0001: return 6;
    case 0b0101: return 5;
    case 0b1001: return 4;
    case 0b0010: return 9;
    case 0b0110: return 8;
    case 0b1010: return 7;
    default: return 0;
  }
}
// grid_coords (1~9)
// 1 2 3
// 4 5 6
// 7 8 9

// =========================================================================================
void loop() {
  byte receivedByte = 0; // 처리할 최종 명령 바이트, 기본값은 Glare 없음 (접기)
  bool newDataToProcess = false; // 새로운 유효한 명령이 있는지 여부

  // --------------------------------------------------------------------
  // 데이터 수신 및 receivedByte 설정 (모드에 따라 분기)
  // --------------------------------------------------------------------
  #ifdef ENABLE_ASCII_DEBUG_INPUT
    // --- ASCII 문자 디버그 입력 처리 ---
    if (Serial.available() > 0) {
      byte asciiInput = Serial.read(); // 일단 읽음
      // 버퍼에 남은 다른 문자들 비우기 (가장 마지막 입력만 고려)
      while (Serial.available() > 0) {
        asciiInput = Serial.read();
      }

      Serial.print("ASCII Debug Input: '");
      Serial.write(asciiInput);
      Serial.print("' (DEC: ");
      Serial.print(asciiInput, DEC);
      Serial.println(")");

      if (asciiInput >= '1' && asciiInput <= '9') {
        int grid_target_1_to_9 = asciiInput - '0'; // 정수 1~9로 변환

        // 명령 바이트 생성: Glare 감지(1) + 그리드 인덱스
        receivedByte = (1 << 7) | (static_cast<byte>(grid_target_1_to_9) & 0x0F);
        newDataToProcess = true;
        Serial.print("  ASCII Debug: Glare ON, Grid Index: ");
        Serial.println(grid_target_1_to_9);

      } else if (asciiInput == '0') {
        // 명령 바이트 생성: Glare 없음 (0)
        receivedByte = 0; // 00000000
        newDataToProcess = true;
        Serial.println("  ASCII Debug: Glare OFF (Retract)");
      } else {
        // 유효하지 않은 ASCII 입력은 무시 (newDataToProcess = false 유지)
        Serial.println("  ASCII Debug: Unknown input, no action.");
      }
    }

  #else
    // --- RPi 1바이트 통신 처리 ---
    byte latestRPiByte = 0; // RPi에서 온 가장 마지막 바이트
    bool rpiDataAvailable = false;

    while (Serial.available() > 0) {
      latestRPiByte = Serial.read();
      rpiDataAvailable = true;
    }

    if (rpiDataAvailable) {
      receivedByte = latestRPiByte; // RPi에서 온 값을 최종 처리할 바이트로 설정
      newDataToProcess = true;

      Serial.print("RPi Byte Received: 0b");
      for (int i = 7; i >= 0; i--) {
        Serial.print((receivedByte >> i) & 0x01);
        if (i == 7) Serial.print(" ");
        if (i == 4) Serial.print("_");
        if (i == 2 && i > 0) Serial.print(" ");
      }
      Serial.print(" (Decimal: ");
      Serial.print(receivedByte);
      Serial.println(")");
    }
  #endif
  // --------------------------------------------------------------------
  // 데이터 수신 및 receivedByte 설정 끝
  // --------------------------------------------------------------------


  // ▼▼▼▼▼ 공통 처리 로직 시작 (newDataToProcess가 true일 때만 실행) ▼▼▼▼▼
  if (newDataToProcess) {
    // 최상위 비트(MSB, 비트 7)를 추출하여 Glare 감지 유무 판단
    bool glareDetectedBySerial = (receivedByte >> 7) & 0x01;

    if (!prevGlareDetected && glareDetectedBySerial) {
      // 0 → 1 로 바뀐 경우
      Serial.println("  Glare DETECTED (rising edge)");

      minisurvo_out();
      delay(500);
      sungard_down();
    }

    else if (prevGlareDetected && !glareDetectedBySerial) {
      // 1 → 0 로 바뀐 경우
      Serial.println("  Glare CLEARED (falling edge)");

      moveToGridPosition(1);

      sungard_up();
      delay(500);
      minisurvo_in();
    }

    // 글래어가 식별될 경우
    if (glareDetectedBySerial) {
      // Glare가 감지된 경우 (최상위 비트가 1)
      // 하위 4비트(비트 0-3)를 추출하여 그리드 인덱스 얻기
      int gridIndexFromSerial = receivedByte & 0x0F; // 00001111 마스크
      Serial.print("  Serial Command: Glare DETECTED. Grid Index: ");
      Serial.println(gridIndexFromSerial);

      int grid_coords = get_grid_coords(gridIndexFromSerial);

      #ifdef ENABLE_ASCII_DEBUG_INPUT
        if (gridIndexFromSerial >= 1 && gridIndexFromSerial <= 9) {
            moveToGridPosition(gridIndexFromSerial - 1);  // 인덱스 0~8
        }
        else {
            Serial.println("Invalid command.");
        }

      #else
        if (grid_coords >= 1 && grid_coords <= 9) {
            moveToGridPosition(grid_coords - 1);  // 인덱스 0~8
        }
        else {
            Serial.println("Invalid command.");
        }
     
      #endif
    }
    // 상태 저장
    prevGlareDetected = glareDetectedBySerial;
  }
}