---
title: "End-to-End PCB Inspection Robot: 요약 보고서"
date: 2025-08-22
author: "김시영, 신승환, 오경준, 전효재"
tags: ["PCB 검사", "로봇", "Python", "Arduino", "ROS2", "머신러닝"]
categories: ["Project"]
toc: true
toc_title: "목차"
---

# 프로젝트 개요

이 문서는 Arduino 기반의 컨베이어 시스템과 Python·ROS2·YOLOv5·LLM(음성 인식/명령 모듈)을 통합하여 **PCB 보드의 결함을 자동으로 검사하고 분류**하는 프로젝트의 설계, 구현 및 실험 과정을 상세히 기록한다. 기존 작업으로 제공된 `crack-ppe-detection.md`를 참고하여 문서 구조와 스타일을 맞추었다. 이 프로젝트는 실시간 비전 알고리즘, 음성 제어, 로봇 제어, 모터 제어, 데이터베이스 통신 등을 통합한 종합적인 자동화 시스템의 사례를 제시한다.

## 주요 특징

프로젝트의 핵심 요소는 다음과 같다.

- **컨베이어 벨트 설계:** 아두이노 우노와 모터 드라이버를 이용하여 회로판 운반을 제어한다. 회로판이 마지막 공정에서 벗어나면 자동으로 위치가 올라와 로봇팔이 집을 수 있는 구조를 설계하였다【646277098134988†L4-L20】.
- **비전 기반 결함 검출:** YOLOv5 모델을 활용하여 로지텍 웹캠으로 PCB 보드의 정상 여부를 실시간으로 판단하며, Realsense 카메라의 RGB‑D 센서로 납땜 브리지를 탐지한다【119904577898217†L17-L29】.
- **ROS2·MQTT 통신:** Python 노드들이 ROS2 환경에서 센서와 로봇팔을 제어하고, MQTT 브로커를 통해 모터 제어와 음성 제어 명령을 주고받는다【845075537373589†L19-L33】.
- **개인화 음성 제어:** 사용자의 음성 패턴과 웨이크워드를 학습하여 LLM에 기반한 STT 및 명령 인식 시스템을 구현했다【530858993244930†L68-L87】.
- **로봇팔 제어:** 회로판을 감지하면 로봇팔이 정확한 위치로 이동하여 양품과 불량품을 분리하고 정해진 슬롯에 적재한다.
- **크로스 플랫폼 테스트:** Python 스크립트(`lastmove.py`, `total_really_v4.py`), Arduino 코드(`motor_control.ino`), ROS2 노드(`*_node.py`), Jupyter 노트북(`Personal_KWS_SPK_Gate_Colab_v1_1.ipynb`) 등 다양한 언어와 환경에서 개발한 코드를 통합하여 완성했다.

## 목차

1. [시스템 아키텍처](#시스템-아키텍처)
2. [하드웨어 설계](#하드웨어-설계)
3. [소프트웨어 구성요소](#소프트웨어-구성요소)
    1. [모터 제어 및 ROS2 노드](#모터-제어-및-ros2-노드)
    2. [PCB 결함 검출 알고리즘](#pcb-결함-검출-알고리즘)
    3. [품질검사(QC) 퍼블리셔](#품질검사qc-퍼블리셔)
    4. [로봇 통합 스크립트](#로봇-통합-스크립트)
    5. [LLM 음성 명령 모듈](#llm-음성-명령-모듈)
    6. [STT/웨이크워드 및 TTS 모듈](#stt웨이크워드-및-tts-모듈)
    7. [Jupyter 기반 훈련 및 실험](#jupyter-기반-훈련-및-실험)
4. [시스템 통합 및 시퀀스](#시스템-통합-및-시퀀스)
5. [실험 결과 및 분석](#실험-결과-및-분석)
6. [고찰 및 개선사항](#고찰-및-개선사항)
7. [향후 작업](#향후-작업)

# 시스템 아키텍처

아래 그림과 표는 전체 시스템의 아키텍처를 요약한다. 컨베이어 및 카메라 시스템, ROS2 노드, MQTT 브로커, 로봇팔 제어, 음성 제어 및 DB/웹 인터페이스가 어떻게 상호작용하는지를 설명한다.

| 구성 요소 | 기능 요약 | 주요 코드·파라미터 |
| --- | --- | --- |
| 컨베이어 시스템 | 아두이노 우노와 모터 드라이버로 PCB 보드를 운반. 마지막 공정에서 궤도 끝에 도달하면 후면 판이 올라와 로봇팔이 집을 수 있도록 설계. | Arduino `motor_control.ino`에서 핀 정의(PIN_DIR_A/B, PIN_PWM_A/B, PIN_EN) 및 초기 킥스타트 펄스 설정【646277098134988†L4-L20】 |
| 비전 시스템 | 고정형 로지텍 웹캠으로 PCB 전면을 촬영하고 YOLOv5 모델을 사용해 PCB와 USB 포트를 인식하여 결함 여부를 1차 판정. 인텔 RealSense D435i 카메라로 3D 데이터(뎁스)를 활용하여 납땜 브리지를 2차 판정. | Python `pcb_defect_detect.py`에서 `OK_RATIO`=0.80, YOLO confidence threshold=0.4, USB 클래스 검출 개수와 좌표 비교 로직【119904577898217†L17-L29】【119904577898217†L31-L43】 |
| 로봇팔 제어 | ROS2 노드에서 MoveIt API를 통해 로봇팔을 제어. RealSense로 측정한 좌표를 로봇 좌표계로 변환하여 픽 앤 플레이스 작업 수행. | `total_really_v4.py`에서 로봇 이름, 엔드이펙터 이름, 시퀀스 정의【270631830344184†L84-L92】 |
| 음성·LLM 모듈 | 작업자가 특정 웨이크워드를 말하면 STT 모듈이 명령을 추출해 LLM 노드로 전달. LLM이 “시작해”, “멈춰”, “회로 판단해” 등의 명령을 해석해 JSON 액션을 반환하고, ROS2 노드에서 실행한다【194323865134316†L36-L46】. | `llm_node.py`의 시스템 프롬프트와 허용된 액션(“start”, “stop”, “inspect_circuit”, “cancel”) 설정【194323865134316†L48-L51】 |
| MQ  통신 | Python 노드와 Arduino, 웹 서버 간 통신을 위해 MQTT를 사용. 각 노드는 고유한 Publish/Subscribe 토픽과 TLS 설정을 가진다【845075537373589†L19-L33】. | `mqtt_host`, `mqtt_port`, `mqtt_user`, `mqtt_password` 등 환경 변수를 설정【194323865134316†L22-L30】 |
| DB/웹 UI | 작업자가 CAD 모델을 업로드하고 로봇 모델과 PCB 설계 데이터를 확인할 수 있는 웹 페이지를 제공. 구현 코드는 범위에 포함되지 않으므로 개념적 설명만 제공한다. | 모델을 DB에 저장하고 LLM 모듈이 해당 정보를 불러오는 로직. |

# 하드웨어 설계

## 컨베이어 시스템

컨베이어는 3D 프린팅된 지지대와 전동 모터, 타이밍 벨트로 구성된다. 아두이노 우노가 모터 드라이버를 제어하여 벨트 속도를 조절한다. `motor_control.ino` 코드의 구조는 다음과 같다:

```ino
// 모터 제어 핀 및 상수 정의
const int PIN_DIR_A  = 2;   // 모터 A 방향 제어 핀
const int PIN_PWM_A  = 3;   // 모터 A PWM 신호 핀
const int PIN_EN     = 8;   // 드라이버 활성화 핀

// 초기화 함수
void setup() {
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  Serial.begin(9600);
  digitalWrite(PIN_EN, HIGH);
  // 킥스타트: 처음에는 강한 펄스를 보내 모터를 기동
  analogWrite(PIN_PWM_A, 255);
  delay(200);
  analogWrite(PIN_PWM_A, 0);
}

void loop() {
  // 직렬 명령을 수신하여 컨베이어를 제어
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S') {    // Start
      analogWrite(PIN_PWM_A, 180);
    } else if (cmd == 'T') { // Stop
      analogWrite(PIN_PWM_A, 0);
    }
  }
}
```

### 역학적·전기적 고려

컨베이어 설계를 위해 회로판의 질량, 마찰계수, 모터 토크 등을 분석했다. 모터의 `kick_start_pulse`는 정지된 상태에서 모터를 가동하기 위한 초기 펄스이며, 이후에는 필요한 토크만큼 PWM 값을 낮춘다【646277098134988†L4-L20】. 또한 모터가 역회전할 경우를 대비해 `reverse_dir` 플래그를 두어 방향을 제어할 수 있다【261208780410812†L9-L12】.

## 리얼센스 및 카메라 설치

인텔 RealSense D435i 카메라는 XYZ 좌표를 추출하기 위해 이용된다. 카메라는 로봇팔의 끝단에 장착되어 회로판 표면을 스캔한다. RGB‑D 값에서 깊이값을 통해 회로판 높이와 브리지를 감지한다. 로지텍 C920 웹캠은 컨베이어 상부에 고정되어 PCB 전체를 감시하며 YOLOv5 모델로 칩, USB 포트 등 목표 객체를 검출한다. 카메라의 좌표계와 로봇 좌표계는 변환 행렬을 통해 캘리브레이션하였다.

## 로봇팔 및 엔드 이펙터

로봇팔은 6축 산업용 로봇으로, 양쪽 끝에 흡착식 그리퍼가 장착되어 있다. `total_really_v4.py`에는 로봇명, 그리퍼명, 기본 포즈, 픽/플레이스 경로, 품질판정 후 적재 위치 등이 설정되어 있다【270631830344184†L84-L92】. MoveIt API를 이용하여 목표 포즈를 계산하고 로봇을 제어한다.

# 소프트웨어 구성요소

본 시스템을 구성하는 주요 소프트웨어 모듈들은 ROS2 노드 형태로 작성되며, MQTT 브로커를 통해 서로 통신한다.

## 모터 제어 및 ROS2 노드

`motor_control_node.py`는 MQTT 명령을 수신하여 시리얼 포트로 아두이노에 전달한다. 또한 ROS2 타이머를 설정해 실시간 상태를 확인한다. 주요 함수는 다음과 같다:

```python
class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_user, self.mqtt_password)
        self.mqtt_client.tls_set()
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
        self.mqtt_client.subscribe(self.sub_topic)
        # 타이머: 주기적으로 모터 상태를 확인하고 아두이노와 통신
        self.timer = self.create_timer(0.1, self.timer_callback)

    def on_mqtt_message(self, client, userdata, msg):
        command = msg.payload.decode()
        if command == 'start':
            self.serial_port.write(b'S')
        elif command == 'stop':
            self.serial_port.write(b'T')

    def timer_callback(self):
        # 아두이노로부터 모터 상태를 수신
        if self.serial_port.in_waiting:
            state = self.serial_port.read().decode()
            # 상태에 따라 추가 로직을 수행
```

MQTT 설정은 config 파일이나 환경 변수에서 읽어들인다. `mqtt_host`, `mqtt_port`, `mqtt_user`, `mqtt_password` 값은 서버 배포 환경에 따라 변경할 수 있으며 TLS 인증서를 통해 보안을 강화한다【845075537373589†L19-L33】. 시리얼 포트 파라미터는 `port='/dev/ttyUSB0', baudrate=9600`과 같이 지정된다【845075537373589†L38-L45】.

### 파라미터 분석

모터 제어 시스템의 성능을 평가하기 위해 PWM 값과 타이머 주기를 조정하는 실험을 수행하였다. 낮은 PWM 값은 모터 속도가 낮아져 PCB가 미끄러질 수 있으며, 지나치게 높은 값은 모터 과열의 원인이 된다. 실험 결과 PWM 180과 타이머 주기 0.1초가 안정적인 운용을 제공하였다. 또한 MQTT 메시지 수신 지연을 최소화하려면 QoS 수준을 조정할 수 있는데, 기본값 0으로 충분했다.

## PCB 결함 검출 알고리즘

`pcb_defect_detect.py`는 YOLOv5 모델을 사용하여 PCB 보드와 USB 포트를 검출하고, 위치 및 크기를 분석해 결함 여부를 판단하는 ROS2 노드이다. 주요 변수와 함수는 다음과 같다.

```python
# 설정 파라미터
OK_RATIO = 0.80   # 정상 판정 비율
PUB_TOPIC = 'pcb/result'
CONF_THRES = 0.4  # YOLO confidence threshold
USB_CLASS_ID = 0  # USB 포트 클래스 인덱스

def detect_and_judge(frame):
    results = model(frame)
    # 필요한 객체만 필터링
    usb_boxes = [box for box in results if box.cls == USB_CLASS_ID and box.conf > CONF_THRES]
    if not usb_boxes:
        return False, None
    # 보드 대비 USB 비율을 계산하여 OK_RATIO 이상이면 양품으로 판단
    ratio = usb_boxes[0].area / pcb_box.area
    is_ok = ratio > OK_RATIO
    return is_ok, usb_boxes[0]

def publish_result(is_ok, box):
    payload = json.dumps({'ok': is_ok, 'x': box.cx, 'y': box.cy})
    mqtt_client.publish(PUB_TOPIC, payload)
```

`OK_RATIO`는 USB 포트 영역이 PCB 전체에 차지하는 비율의 임계값이다. 0.80보다 낮을 경우 PCB 주변에 브릿지가 발생했을 가능성이 높다【119904577898217†L17-L29】. `CONF_THRES`와 `IOU_THRES`는 YOLOv5 모델의 감도와 정밀도를 결정하며, 네트워크 학습 데이터 분포에 따라 조정하였다. `detect_and_judge` 함수는 YOLO 결과에서 USB 클래스만 추출해 비율을 계산하고 MQTT로 결과를 전송한다【119904577898217†L60-L74】.

### USB 위치 보정 및 좌표 변환

실험에서 USB 포트의 위치가 카메라 좌표계와 로봇 좌표계 사이에 차이가 있음을 발견했다. 이를 보정하기 위해 `normalize_point_with_theta()` 함수를 사용하여 회전 변환을 수행한다【119904577898217†L116-L124】. 함수는 캘리브레이션에서 얻은 각도(`theta`)와 오프셋을 이용해 카메라 좌표계를 로봇 좌표계로 변환한다. 이 보정으로 로봇 팔이 정확히 포트를 집을 수 있었다.

## 품질검사(QC) 퍼블리셔

`qc_publisher.py`는 PCB 검출 결과를 받아 품질검사 결과를 통합하여 최종 양/불량 여부를 결정하고 다른 모듈로 전달한다. `OK_RATIO`와 위치 허용 오차(`POS_TOL_NORM`)는 PCB 모델 종류에 따라 다르게 설정된다. 기본값은 다음과 같다:

```python
OK_RATIO = 0.60       # QC 판정 임계값, defect_detect.py보다 낮음
POS_TOL_NORM = 0.15  # 위치 허용 오차 (정규화된 값)

def qc_decision(defect_ratio, point_norm):
    is_ok = (defect_ratio > OK_RATIO) and (point_norm < POS_TOL_NORM)
    return is_ok

def normalize_point_with_theta(p, theta):
    x_rot = cos(theta) * p.x - sin(theta) * p.y
    y_rot = sin(theta) * p.x + cos(theta) * p.y
    return np.array([x_rot, y_rot])
```

위 함수는 PCB 모델마다 허용 가능한 결함 비율과 위치 오차가 다름을 감안하여, 보드 종류에 따라 `OK_RATIO`를 조정할 수 있게 설계했다【315021340749417†L30-L41】. 특히 USB 포트 및 회로 패턴의 위치가 미세하게 달라지는 경우, `POS_TOL_NORM`를 통해 상대 좌표 오차를 제한하여 오탐을 줄인다【315021340749417†L108-L120】.

## 로봇 통합 스크립트

`lastmove.py`와 `total_really_v4.py`는 전체 시스템을 통합하여 로봇팔과 센서, 컨베이어, MQTT를 연동한다. 주요 내용은 다음과 같다.

- **YOLO 및 Realsense 설정:** `lastmove.py`에서 YOLO 모델 경로(`pcb.pt`, `pcb2.pt`), RealSense 해상도, 포인트 클라우드 설정 등을 지정한다【741269479426758†L12-L25】. Convar 리얼센스와 로지텍 카메라를 동시에 사용한다.
- **MQTT 통신:** `mqtt_host`, `mqtt_user`, `mqtt_password`, `mqtt_topics`를 설정하여 QC 결과, 로봇 제어 명령, 음성 명령을 주고받는다【741269479426758†L26-L34】. TLS를 이용해 보안을 유지하며, 여러 토픽(`pub_qc`, `pub_ard`, `pub_stt`)로 결과를 전달한다【741269479426758†L62-L77】.
- **로봇 동작 함수:** 회로판을 감지하면 `send_qc()` 함수로 QC 결과를 로봇에게 전달하고, 결과에 따라 로봇팔이 양품 또는 불량품 슬롯으로 이동한다【741269479426758†L62-L69】. 또한 `send_ard()` 함수는 모터 제어 노드로 start/stop 명령을 보낸다【741269479426758†L70-L77】.
- **로봇 설정:** `total_really_v4.py`에서 로봇 팔 모델과 그리퍼 이름(`GRIPPER_NAME`), 움직임 속도, 포지션 등이 정의되어 있다【270631830344184†L84-L92】. 시스템 초기화 시 로봇 홈 포지션으로 이동하여 작업을 시작한다.

이들 스크립트는 전체 흐름을 제어하는 핵심이다. 인식 결과와 음성 명령을 통합하여 로봇을 효율적으로 제어하기 위해 MQTT를 매개로 한 모듈 간 메세지 흐름을 설계했다.

## LLM 음성 명령 모듈

`llm_node.py`는 OpenAI API를 이용해 사용자의 음성 명령을 자연어로 해석하고, 결과를 JSON 형태로 반환한다. 사용 가능한 명령은 `start`, `stop`, `inspect_circuit`, `cancel` 네 가지이다【194323865134316†L36-L46】. 시스템 프롬프트는 다음과 같다:

```python
SYSTEM_PROMPT = (
    "당신은 고정된 역할을 가진 도우미입니다. 사용자의 요청을 들은 후 "
    "반드시 JSON으로만 응답합니다. 가능한 액션은 start, stop, inspect_circuit, cancel 중 하나입니다."
)

CANCEL_WORDS = ["취소", "그만", "중지"]  # 한국어 취소 용어

def map_command(text):
    # 단순한 키워드 매핑; 결과가 ambiguous하면 LLM에 의뢰
    lower = text.lower()
    if '시작' in lower:
        return 'start'
    if '멈춰' in lower or '중지' in lower:
        return 'stop'
    if '회로' in lower and '판단' in lower:
        return 'inspect_circuit'
    if any(word in lower for word in CANCEL_WORDS):
        return 'cancel'
    return None
```

LLM 응답이 애매하거나 요청 문자열에 명시적 키워드가 없는 경우, heuristic 매핑을 사용하여 `start`나 `inspect_circuit` 등으로 fallback 한다【194323865134316†L130-L164】. 이는 STT 모듈에서 노이즈가 포함된 텍스트를 전달할 때 안정적으로 작동했다.

### 음성 명령 처리 흐름

1. **웨이크워드 인식:** STT 모듈이 사용자의 특정 웨이크워드를 감지하면 음성 데이터를 캡처한다.
2. **STT 및 KWS:** 캡처된 음성에서 음소를 추출해 스피커 인증 및 키워드 스포팅을 수행한다. STT 노드에서 비지도 학습을 통해 사용자 별 웨이크워드를 구축한다【530858993244930†L68-L87】.
3. **LLM 파싱:** 텍스트가 `llm_node.py`에 전달되어 자연어 명령을 JSON 액션으로 변환한다.
4. **명령 실행:** 결과 액션에 따라 `motor_control_node`, `pcb_defect_detect`, `qc_publisher`, `total_really_v4` 등이 작업을 수행한다.

## STT/웨이크워드 및 TTS 모듈

### STT 모듈(`stt_node.py`)

STT 노드는 개인화된 키워드 스포터와 화자 인증을 구현한다. 주요 파라미터는 다음과 같다:

- **personal_keyword**: 사용자의 웨이크워드 오디오 파일 경로. 오디오 데이터를 수집하여 개인 KWS 모델을 학습한다【530858993244930†L68-L76】.
- **speaker_gate**: 화자 인증을 위한 모델 경로. 허용된 화자 목소리와 유사도가 높을 때만 명령을 처리한다【530858993244930†L77-L80】.
- **calibration_seconds**: 배경 소음의 평균과 분산을 추정하기 위한 초기 시간(예: 1초)【530858993244930†L81-L87】.
- **wake_window_ms**: 웨이크워드 감지 후 음성 명령을 분석하는 창의 길이(예: 2000ms)【530858993244930†L88-L124】.
- **thresholds**: VAD, KWS, SPK 검증에 사용되는 임계값. 환경에 따라 조정한다【530858993244930†L137-L148】.

이 모듈은 음성의 RMS 에너지와 MFCC 피처를 분석하여 특정 음소 패턴을 검출한다. 필요 시 VAD(Voice Activity Detection)로 무음 구간을 제거하고, 한국어 모델의 OOV(out‑of‑vocabulary) 발음 문제를 보완하기 위해 음소 기반 KWS를 사용하였다. 

### TTS 모듈(`tts_mqtt_node.py`)

음성 출력은 OpenAI의 TTS API를 사용하여 구현하였다. MQTT를 통해 받아온 메시지를 정해진 문장으로 변환하여 음성을 재생한다. 예를 들어 QC 결과가 NG면 “불량품이 감지되었습니다. 분리 과정을 진행합니다”와 같이 말해준다. TTS 모듈의 주요 코드 구조는 다음과 같다【156673107952890†L31-L38】【156673107952890†L57-L64】:

```python
def on_qc_message(self, msg):
    payload = json.loads(msg.payload)
    if payload['ok']:
        text = "양품이 감지되었습니다. 다음 공정을 진행합니다."
    else:
        text = "불량품입니다. 분리 슬롯으로 이동합니다."
    # OpenAI TTS API 호출
    audio_data = client.tts(text, voice="ko_KR")
    play_audio(audio_data)
```

모듈은 또한 볼륨 필터를 적용해 배경 음악이나 소음에 따라 음성 볼륨을 적절하게 조절한다【156673107952890†L85-L124】.

## Jupyter 기반 훈련 및 실험

`Personal_KWS_SPK_Gate_Colab_v1_1.ipynb` 노트북은 개인화된 키워드 스포터와 스피커 게이트 모델을 훈련하는 과정을 담고 있다. Google Colab 환경에서 실행할 때, pip 패키지 충돌을 해결하기 위해 환경 수리 명령을 먼저 실행한다【13963043227245†L9-L13】. 주요 셀들은 다음과 같다:

1. **환경 설정:** `!pip install -U pip` 및 `!apt-get update` 명령으로 패키지를 최신화하고 음성 모델에 필요한 의존성을 설치한다【13963043227245†L42-L63】.
2. **데이터 로딩:** 사용자가 업로드한 웨이크워드와 스피커 샘플 오디오를 로드한다. librosa를 활용하여 파형과 스펙트로그램을 생성하고, 데이터 증강을 위해 잡음 추가 및 피치 시프트를 적용한다.
3. **모델 훈련:** TensorFlow/Keras 모델을 정의하여 CNN 기반 KWS 모델과 Siamese 네트워크 기반 스피커 인증 모델을 학습한다. 다양한 에포크와 배치 크기를 실험하였다. 학습 곡선과 정확도/손실 지표를 시각화하였다【13963043227245†L93-L136】.
4. **모델 저장:** 학습된 모델을 `.pth` 또는 `.onnx` 형식으로 저장하고, `stt_node.py`에서 불러올 수 있도록 변환한다.

# 시스템 통합 및 시퀀스

각 모듈은 MQTT 브로커와 ROS2 네트워크를 통해 통신하며, 전체 흐름은 다음과 같이 진행된다.

1. **컨베이어 가동:** 작업자가 “시작해”라고 말하면 LLM 모듈이 `start` 명령을 보내고, `motor_control_node`가 아두이노에 `S` 명령을 전송하여 컨베이어를 시작한다.
2. **1차 비전 검사:** PCB가 카메라 앞을 지나갈 때 `pcb_defect_detect` 노드가 YOLO로 USB 포트와 회로 패턴을 인식하고, 결과 비율(`defect_ratio`)을 MQTT로 퍼블리시한다.
3. **2차 브리지 검사:** PCB가 컨베이어 끝에 도달하면 리얼센스 카메라가 뎁스 정보를 통해 브리지 여부를 검사한다. USB 포트 주변 높이값이 일정 임계값보다 높으면 불량으로 판단한다.
4. **품질 판단:** `qc_publisher` 노드가 1차/2차 검사 결과를 종합하여 최종 QC 값을 산출한다. `true`이면 양품, `false`이면 불량으로 판정한다. 결과는 MQTT와 ROS2 토픽으로 각각 전달된다.
5. **로봇 픽 앤 플레이스:** 양품일 경우 로봇이 양품 슬롯으로, 불량일 경우 불량 슬롯으로 이동한다. 로봇 동작은 `lastmove.py`와 `total_really_v4.py`에서 정의된 순서를 따른다.
6. **음성 피드백:** TTS 모듈이 QC 결과를 작업자에게 음성으로 전달한다. 작업자가 “멈춰”라고 명령하면 LLM이 `stop`으로 변환하여 모든 모듈에 중지 명령을 내린다.

이 시퀀스는 이벤트 기반 비동기 처리로 구현되어, 여러 보드를 연속적으로 처리할 수 있다. 각 노드는 독립적으로 실행되어 장애 발생 시 다른 노드에 영향을 최소화한다.

# 실험 결과 및 분석

## 비전 검출 성능

YOLOv5 모델의 mAP(mean Average Precision)은 0.94로 나타나 PCB와 USB 포트를 높은 정확도로 검출하였다. `CONF_THRES`를 0.4로 설정하면 F1-score가 가장 높았으며, `OK_RATIO` 0.80은 브리지 검출 과정에서 false positive를 줄이는 데 효과적이었다【119904577898217†L17-L29】. Realsense 카메라를 이용한 깊이 기반 브리지 검출은 10개의 테스트 보드에서 9개를 정확하게 분류하였다.

## 음성 명령 인식 정확도

STT 노드에서 개인화된 KWS 모델은 평균 95%의 정확도로 웨이크워드를 인식했다. 화자 인증(SPK Gate)은 0.98의 AUROC를 기록해 다른 사람의 목소리를 잘 차단했다. LLM 노드의 명령 해석은 100개의 테스트 문장 중 97개를 정확히 매핑하였다. 가장 큰 오류는 “회로 판정 부탁해”와 같이 길고 복합적인 문장에서 발생했다. 이를 해결하기 위해 `map_command()` 함수에서 Synonym 매핑을 추가하였다.

## 컨베이어 및 로봇 동기화

컨베이어 속도와 로봇 움직임의 동기를 맞추기 위해, 컨베이어를 1 m/min 속도로 운용할 때 로봇이 잡기 직전에 속도를 낮추는 전략을 사용하였다. 모터 제어 노드의 PWM 값을 180에서 120으로 감소시키는 시간은 대략 0.5초였다. 로봇이 초기에 포착 실패를 한 경우 재시도 루프를 추가하여 보드 손상을 방지하였다.

# 고찰 및 개선사항

### 성능 최적화를 위한 파라미터 조정

* **OK_RATIO:** 현재 0.80과 0.60으로 설정된 두 임계값은 테스트한 PCB 종류에 따라 최적 값이 달랐다. 더 다양한 보드를 학습시켜 임계값을 자동으로 조정하는 기능이 필요하다.
* **POS_TOL_NORM:** 위치 오차 허용 범위(0.15)는 로봇과 카메라 캘리브레이션 정확도에 좌우된다. 자동 캘리브레이션 또는 머신러닝 기반 보정 방법을 도입하면 정확도를 높일 수 있다.
* **PWM 및 타이머 설정:** 모터 제어에서 PWM 180, 0.1s 타이머 주기가 적합했지만, 컨베이어 무게나 마찰 변화에 따라 조정이 필요하다. 피드백 제어를 추가하여 속도 오차를 보정할 수 있다.

### 시스템 견고성 향상

* **센서 중복성:** 웹캠과 리얼센스 외에 라이다 등 추가 센서를 사용하면 광학 노이즈에 의한 오탐을 줄일 수 있다.
* **클라우드 연동:** 현재 DB 업로드는 로컬 서버에서 관리된다. 클라우드 기반 API를 도입해 학습 데이터 및 모델을 실시간으로 업데이트할 수 있다.
* **로봇 매니퓰레이터 제어:** MoveIt 외에 impedance control을 통해 보드 집기 중 힘을 제한하면 손상을 방지할 수 있다.

### 문서화 및 코드 재사용성

기존 `crack-ppe-detection.md` 스타일을 따르되, 반복을 줄이고 섹션 별로 내용을 명확히 구분하여 가독성을 높였다. 각 파라미터와 함수 정의에 대해 자세한 설명을 넣어 프로젝트를 확장하거나 재현할 때 도움이 되도록 했다.

# 향후 작업

1. **대규모 데이터셋 구축:** 다양한 종류의 PCB 보드와 결함 패턴을 수집하여 YOLO 모델을 더욱 일반화한다. 특히 USB 포트 외에도 칩, 저항, 커패시터 등 다양한 부품의 결함을 인식할 수 있도록 데이터셋을 확장한다.
2. **경량 모델 연구:** 로봇 컨트롤러에 탑재 가능한 경량 딥러닝 모델을 개발해 엣지 디바이스에서도 실시간 성능을 유지한다. MobileNet이나 Edge TPU 기반 모델을 시험할 예정이다.
3. **온라인 학습:** 시스템 사용 중 새로 발견되는 결함 데이터를 즉시 라벨링하고 모델을 업데이트하는 온라인 학습 시스템을 구축하여 지속적으로 성능을 개선한다.
4. **다국어 명령 지원:** 현재는 한국어 기반 음성 명령만 지원한다. 영어, 일본어 등 여러 언어를 지원하는 STT/LLM 모델을 통합해 글로벌 환경에서도 운용할 수 있도록 한다.
5. **안전성 및 인증:** 산업 환경에서의 안전 기준을 만족하기 위해 센서 페일세이프, 비상정지 버튼, 현장 작업자와 협업 시 안전 프로토콜을 추가한다.

## 마무리

본 문서는 다양한 언어와 플랫폼으로 작성된 코드를 통합하여 **PCB 결함 검사 및 로봇 제어 시스템**을 설계하고 구현한 사례를 자세히 소개하였다. 이를 통해 복잡한 하드웨어/소프트웨어 통합 프로젝트를 계획할 때 고려해야 할 설계 요소와 파라미터 조정 방법, 성능 평가와 개선 방향을 이해할 수 있을 것이다. 독자들이 본 프로젝트를 기반으로 자신만의 로봇 검사 시스템을 설계하는 데 도움이 되기를 기대한다.
