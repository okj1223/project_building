// LAFVIN TB6612 쉴드(UNO) — 한쪽 방향 연속 회전 + 스타트 펄싱킥 + 시리얼 START/STOP
// A: DIR=D2, PWM=D5 / B: DIR=D4, PWM=D6

// ===== 핀 정의 =====
const int PIN_DIR_A = 2;
const int PIN_PWM_A = 5;
const int PIN_DIR_B = 4;
const int PIN_PWM_B = 6;

// ===== 방향 반전 스위치 =====
const bool A_REVERSED = false;   // 필요 시 true
const bool B_REVERSED = true;

// ===== 목표 속도(듀티) =====
const int SPEED_A = 25;          // 0~255
const int SPEED_B = 25;          // 0~255

// ===== 스타트 펄싱 파라미터(단일 펄스) =====
const int  KICK_PWM = 120;       // 펄스 순간 듀티
const int  KICK_MS  = 100;       // 펄스 ON 시간 [ms]

// ===== 상태 =====
bool running = false;
unsigned long kick_deadline_ms = 0;

// ===== 유틸 =====
inline void setDir(int dirPin, bool forward, bool reversed){
  digitalWrite(dirPin, (forward ^ reversed) ? HIGH : LOW);
}
inline void setPWM(int pwmPin, int duty){
  analogWrite(pwmPin, constrain(duty, 0, 255));
}

// 방향 고정 + 목표 듀티 유지
inline void holdMotors(){
  setDir(PIN_DIR_A, true, A_REVERSED); setPWM(PIN_PWM_A, SPEED_A);
  setDir(PIN_DIR_B, true, B_REVERSED); setPWM(PIN_PWM_B, SPEED_B);
}

void setup() {
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);

  // 초기 정지
  setPWM(PIN_PWM_A, 0);
  setPWM(PIN_PWM_B, 0);
  setDir(PIN_DIR_A, true, A_REVERSED);  // 정방향 고정(필요 시 반전 스위치로 처리)
  setDir(PIN_DIR_B, true, B_REVERSED);

  Serial.begin(115200);
  delay(50);
  Serial.println("READY");
}

void loop() {
  // 1) 명령 처리
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim(); line.toUpperCase();

    if (line == "START") {
      running = true;

      // 킥 스타트: 방향 고정 후 순간 듀티
      setDir(PIN_DIR_A, true, A_REVERSED);
      setDir(PIN_DIR_B, true, B_REVERSED);
      setPWM(PIN_PWM_A, KICK_PWM);
      setPWM(PIN_PWM_B, KICK_PWM);

      kick_deadline_ms = millis() + KICK_MS;  // KICK_MS 뒤 평속 복귀
      Serial.println("ACK:START");

    } else if (line == "STOP") {
      running = false;
      setPWM(PIN_PWM_A, 0);
      setPWM(PIN_PWM_B, 0);
      kick_deadline_ms = 0;
      Serial.println("ACK:STOP");

    } else {
      Serial.print("NAK:"); Serial.println(line);
    }
  }

  // 2) 킥 → 평속 자동 복귀
  if (running && kick_deadline_ms != 0 && (long)(millis() - kick_deadline_ms) >= 0) {
    holdMotors();                  // 평상시 속도로 복귀
    kick_deadline_ms = 0;          // 한 번만 실행
  }

  // 3) 유지 구동(START 후 킥이 끝났다면 계속 듀티 유지)
  if (running && kick_deadline_ms == 0) {
    holdMotors();
  }

  delay(10);
}
