#include <Servo.h>

// ----------- 핀번호 설정 --------------
// 부저
#define buzzer 4
// 서보모터
#define Servo_main 2
#define Servo_mini 3
// 스텝모터
#define dir_1 28    // 0,1번 핀 쓰면 시리얼 통신이랑 꼬일 가능성 있어서 바꿈.
#define steps_1 5
#define ms1_1 6
#define ms2_1 7
#define ms3_1 8
#define dir_2 9
#define steps_2 10
#define ms1_2 11
#define ms2_2 12
#define ms3_2 13

// 버튼
#define BTN_30 30
#define BTN_31 31
#define BTN_32 32
#define BTN_33 33
#define BTN_34 34

 // ------------ 서보모터 제어 설정 ------------------
Servo mainServo;
Servo miniServo;

// ------------- 부저 멜로디 설정 ---------------------
int arraySize = 11;
int melody[] = {245, 260, 292, 328, 348, 390, 439, 492, 523, 586, 1317};
int song[]   = {3,   5,   6,   10,  6,   5,   6,   10,  6,   5,   6   };
int noteDuration[] = {4, 4, 2, 2, 2, 4, 2, 4, 4, 4, 2};

// ------------- 변수 설정 ---------------------
// 서보모터 각도 제어 변수
int angle_main = 0;
int angle_mini = 0;

// 서보모터 구동 범위 바꾸려면 이 변수를 바꾸면 됨.
int mainServoAngle = 100;   // 0도 ~ 100도
int miniServoAngle = 90;    // 0도 ~ 90도

// 스텝모터 위치 제어 변수
int current_position = 0;

// 위치 설정값. 일단 400 스텝 기준으로 맞춤.
const int POSITION_START = 0;
const int POSITION_MID = 300;
const int POSITION_END = 800;

// 버튼 상태 변수
bool buttonPressed_33 = false;
bool buttonPressed_34 = false;

bool lastState_33 = HIGH;
bool lastState_34 = HIGH;

// ============================================================================================

void setup() {
  // 서보모터 제어 핀
  mainServo.attach(Servo_main);
  miniServo.attach(Servo_mini);

  // 스텝모터 제어 핀
  pinMode(dir_1, OUTPUT); pinMode(steps_1, OUTPUT);
  pinMode(ms1_1, OUTPUT); pinMode(ms2_1, OUTPUT); pinMode(ms3_1, OUTPUT);
  pinMode(dir_2, OUTPUT); pinMode(steps_2, OUTPUT);
  pinMode(ms1_2, OUTPUT); pinMode(ms2_2, OUTPUT); pinMode(ms3_2, OUTPUT);
  digitalWrite(ms1_1, LOW); digitalWrite(ms2_1, LOW); digitalWrite(ms3_1, HIGH);
  digitalWrite(ms1_2, LOW); digitalWrite(ms2_2, LOW); digitalWrite(ms3_2, HIGH);

  // 부저 제어 핀
  pinMode(buzzer, OUTPUT);

  // 버튼 입력핀
  pinMode(BTN_30, INPUT_PULLUP);
  pinMode(BTN_31, INPUT_PULLUP);
  pinMode(BTN_32, INPUT_PULLUP);
  pinMode(BTN_33, INPUT_PULLUP);
  pinMode(BTN_34, INPUT_PULLUP);

  // 시리얼 통신 시작
  Serial.begin(115200); // RPi와 Baud rate 일치하도록 
}

// ------------------------- 스텝모터 위치 제어 함수 ----------------------------------
// START, MID, END 위치로 이동.
void moveTo(int target_position) {
  int steps_needed = target_position - current_position;
  bool direction = (steps_needed >= 0) ? HIGH : LOW;
  int steps = abs(steps_needed);

  // 양쪽 스텝모터의 회전방향은 반대 방향.
  digitalWrite(dir_1, direction);
  digitalWrite(dir_2, !direction);

  // 스텝 모터 구동 코드
  for (int i = 0; i < steps; i++) {
    digitalWrite(steps_1, HIGH);
    digitalWrite(steps_2, HIGH);
    delayMicroseconds(1600);  // 스텝모터 구동 속도 바꾸고 싶으면 이 값 바꾸면 됨.
    digitalWrite(steps_1, LOW);
    digitalWrite(steps_2, LOW);
    delayMicroseconds(1600);  // 위랑 동일하게 바꿔줘야 함.
    current_position += (direction == HIGH) ? 1 : -1;
  }
}

// ----------------------- 썬가드 작동 함수 -----------------------------
// 부저 멜로디 나오면서 선가드가 내려오도록 하는 코드.
void playServoAndMelody() {
  int totalDuration = 0;
  for (int i = 0; i < arraySize; i++)
    totalDuration += (700 / noteDuration[i]) + 30;

  int currentAngle = 0;
  int musicIndex = 0;
  unsigned long startTime = millis();

  unsigned long prevNoteTime = 0;
  int noteDur = 0;

  // 부저 음악과 서보모터 구동을 동시에 구현하기 위한 코드.
  while (millis() - startTime < totalDuration) {
    unsigned long elapsed = millis() - startTime;
    int expectedAngle = map(elapsed, 0, totalDuration, 0, mainServoAngle);
    // 메인 서보모터 구동 코드
    if (expectedAngle != currentAngle) {
      currentAngle = expectedAngle;
      mainServo.write(currentAngle);
    }
    // 부저 음악 출력 코드
    if (musicIndex < arraySize) {
      if (millis() - prevNoteTime >= noteDur) {
        noteDur = 700 / noteDuration[musicIndex];
        tone(buzzer, melody[song[musicIndex]], noteDur);
        prevNoteTime = millis();
        musicIndex++;
      }
    }
  }

  noTone(buzzer);
  delay(500);
}

// =========================================================================================
// *****************************************************************************************
// =========================================================================================

// 여기서는 버튼으로 제어되도록 했는데, 이제 이쪽 부분을 시리얼 통신으로 제어되도록 바꾸면 됨.
// 일단 지금은 상중하 위아래로 3단계 움직이는 제어코드만 들어가있음.

int get_grid_coords(int gridIndex) {
  int gridIndexBits[4]; // 4개의 정수를 저장할 배열 (MSB부터 LSB 순서로 저장)
      for (int i = 0; i < 4; i++) {
        // gridIndexFromSerial의 (3-i)번째 비트를 추출하여 배열에 저장
        // 예: i=0 -> 3번째 비트 (MSB)
        //     i=1 -> 2번째 비트
        //     i=2 -> 1번째 비트
        //     i=3 -> 0번째 비트 (LSB)
        gridIndexBits[i] = (gridIndex >> (3 - i)) & 0x01;
      }

      int grid_coords = 0;
      int grid_x = 0;
      int grid_y = 0;

      if (gridIndexBits[0] == 0) {
        if (gridIndexBits[1] == 0) {
          grid_x = 1;
        }
        else if (gridIndexBits[1] == 1) {
          grid_x = 2;
        }
      }
      else if (gridIndexBits[0] == 1) {
        if gridIndexBits[1] == 0 {
          grid_x = 3;
        }
      }
      else grid_x = 0;

      if (gridIndexBits[2] == 0) {
        if (gridIndexBits[3] == 0) {
          grid_y = 1;
        }
        else if (gridIndexBits[3] == 1) {
          grid_y = 2;
        }
      }
      else if (gridIndexBits[2] == 1) {
        if gridIndexBits[3] == 0 {
          grid_y = 3;
        }
      }
      else grid_y = 0;

      if (grid_x=1 && grid_y=1) grid_coords=1;
      else if (grid_x=1 && grid_y=2) grid_coords=2;
      else if (grid_x=1 && grid_y=3) grid_coords=3;
      else if (grid_x=2 && grid_y=1) grid_coords=4;
      else if (grid_x=2 && grid_y=2) grid_coords=5;
      else if (grid_x=2 && grid_y=3) grid_coords=6;
      else if (grid_x=3 && grid_y=1) grid_coords=7;
      else if (grid_x=3 && grid_y=2) grid_coords=8;
      else if (grid_x=3 && grid_y=3) grid_coords=9;
      else grid_coords=0;

      return grid_coords;
}
// grid_coords (1~9)
// 1 2 3
// 4 5 6
// 7 8 9

void loop() {
  // (추가) 시리얼 통신 수신 및 기본 해석 로직
  byte receivedByte;
  bool newDataAvailable = false;

  // 시리얼 버퍼 초기화 후 데이터 읽어오기
  while (Serial.available() > 0) {
    receivedByte = Serial.read();
    newDataAvailable = true;
  }

  if (newDataAvailable) {

    // 수신된 바이트 값 확인 (디버깅용)
    Serial.print("Received Byte: 0b");
    for (int i = 7; i >= 0; i--) {
      Serial.print((receivedByte >> i) & 0x01);
      if (i == 4) Serial.print("_"); // 가독성을 위해 중간에 공백 추가
    }
    Serial.print(" (Decimal: ");
    Serial.print(receivedByte);
    Serial.println(")");

    // 최상위 비트(MSB, 비트 7)를 추출하여 Glare 감지 유무 판단
    bool glareDetectedBySerial = (receivedByte >> 7) & 0x01;

    if (glareDetectedBySerial) {
      // Glare가 감지된 경우 (최상위 비트가 1)
      // 하위 4비트(비트 0-3)를 추출하여 그리드 인덱스 얻기
      int gridIndexFromSerial = receivedByte & 0x0F; // 00001111 마스크

      Serial.print("  Serial Command: Glare DETECTED. Grid Index: ");
      Serial.println(gridIndexFromSerial);

      int grid_coords = get_grid_coords(gridIndexFromSerial);

    } else {
      // Glare가 감지되지 않은 경우 (최상위 비트가 0)
      Serial.println("  Serial Command: No Glare DETECTED. Retract SunVisor.");
    }
  }
  // 여기까지
  // glareDetectedBySerial이 0이면 접고, 1이면 펼치기
  // Use parameter grid_coords(integer from 1 to 9)

  // 버튼 1번이 눌리면 START 위치로 스텝모터 이동. (썬가드 위치는 제일 아래로 이동)
  if (digitalRead(BTN_30) == LOW) {
    moveTo(POSITION_START);
    delay(300);
  }

  // 버튼 2번이 눌리면 MID 위치로 스텝모터 이동. (썬가드 위치는 중간 부분으로 이동)
  if (digitalRead(BTN_31) == LOW) {
    moveTo(POSITION_MID);
    delay(300);
  }

  // 버튼 3번이 눌리면 END 위치로 스텝모터 이동. (썬가드 위치는 제일 위쪽으로 이동)
  if (digitalRead(BTN_32) == LOW) {
    moveTo(POSITION_END);
    delay(300);
  }

  // 버튼 4번을 한번 누르면 썬가드 내려오면서 부저 멜로디 출력, 한번 더 누르면 썬가드가 올라감.
  bool currentState_33 = digitalRead(BTN_33);
  if (lastState_33 == HIGH && currentState_33 == LOW) {
    if (!buttonPressed_33) {
      playServoAndMelody();
      buttonPressed_33 = true;
    } else {
      for (int i = mainServoAngle; i >= 0; i--) {
        mainServo.write(i);
        delay(10);
      }
      buttonPressed_33 = false;
    }
    delay(300);
  }
  lastState_33 = currentState_33;

  // 버튼 5번을 한번 누르면 서보모터 90도 회전, 한번 더 누르면 원상태로 복귀.
  bool currentState_34 = digitalRead(BTN_34);
  if (lastState_34 == HIGH && currentState_34 == LOW) {
    if (!buttonPressed_34) {
      for (angle_mini = 0; angle_mini <= miniServoAngle; angle_mini++) {
        miniServo.write(angle_mini);
        delay(10);
      }
      buttonPressed_34 = true;
    } else {
      for (angle_mini = miniServoAngle; angle_mini >= 0; angle_mini--) {
        miniServo.write(angle_mini);
        delay(10);
      }
      buttonPressed_34 = false;
    }
    delay(300);
  }
  lastState_34 = currentState_34;
}