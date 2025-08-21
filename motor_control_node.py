#!/usr/bin/env python3
# motor_control_mqtt_node.py
# MQTT (llm/action) -> START/STOP over Serial to Arduino
# 변경사항:
#  - 시리얼 수신 폴링 추가(_poll_serial): READY/ACK/VBATT 로그 처리
#  - START 송신 후 "VBATT:" 라인을 한 번만 캡처하여 기록 (이후 무시)
#  - /object_entered, /qc_result 관련 로직은 없음(무시)

import time
import json
import ssl
import serial
import queue

import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt

# =====================
# 하드코딩된 MQTT 설정
# =====================
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883  # TLS
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"

# 토픽
TOPIC_LLM_ACTION = "llm/action"      # {"action":"start"|"stop"|...}
TOPIC_QC_ENTERED = "/object_entered" # (무시)
TOPIC_QC_RESULT  = "/qc_result"      # (무시)
MQTT_QOS = 1
MQTT_CLIENT_ID = "motor-control-" + str(int(time.time()))

class MotorControlMQTTNode(Node):
    def __init__(self):
        super().__init__('motor_control_mqtt_node')

        # === 시리얼 파라미터 ===
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.ser = None
        self.last_try = 0.0

        # === MQTT 수신 큐 ===
        self._rx_q: "queue.Queue[str]" = queue.Queue(maxsize=100)

        # === START 후 배터리 1회 캡처 상태 ===
        self.vbatt_captured = False
        self.vbatt_value = None

        # === MQTT 클라이언트 ===
        self.mqtt = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
        self.mqtt.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt.tls_set(certfile=None, keyfile=None,
                          cert_reqs=ssl.CERT_REQUIRED,
                          tls_version=ssl.PROTOCOL_TLS_CLIENT)
        self.mqtt.tls_insecure_set(False)

        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_disconnect = self._on_disconnect
        self.mqtt.on_message = self._on_message
        self.mqtt.enable_logger()

        # 비동기 연결 및 루프 시작
        self.mqtt.connect_async(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.mqtt.loop_start()

        # === ROS 타이머 ===
        self.timer_ensure  = self.create_timer(0.5,  self._ensure_serial)    # 시리얼 연결 보장
        self.timer_process = self.create_timer(0.02, self._process_commands) # 큐 처리
        self.timer_poll    = self.create_timer(0.05, self._poll_serial)      # 시리얼 수신 폴링

        self.get_logger().info(
            f"MotorControlMQTTNode: MQTT {MQTT_HOST}:{MQTT_PORT} sub='{TOPIC_LLM_ACTION}', qos={MQTT_QOS}"
        )
        self.get_logger().info(f"Serial target: {self.port}@{self.baud}")
        self.get_logger().info("QC topics are ignored: /object_entered, /qc_result")

    # ===== MQTT 콜백 =====
    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected rc={rc}; subscribing topics")
        # 오직 llm/action 만 구독
        client.subscribe(TOPIC_LLM_ACTION, qos=MQTT_QOS)

    def _on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8').strip()
        except Exception:
            return

        # LLM 액션(JSON or 텍스트)
        if topic == TOPIC_LLM_ACTION:
            action = None
            try:
                obj = json.loads(payload)
                if isinstance(obj, dict):
                    action = (obj.get("action") or "").lower()
                elif isinstance(obj, str):
                    action = obj.lower()
            except Exception:
                action = payload.lower()

            # 회로 점검 액션 → 즉시 STOP
            # if action == "inspect_circuit":
            #     self.get_logger().info("LLM: inspect_circuit -> STOP")
            #     self._enqueue_action("stop")
            #     return

            if action in ("start", "stop"):
                self._enqueue_action(action)
            else:
                self.get_logger().info(f"Ignored action on {topic}: {payload}")
            return

        # 그 외 토픽은 무시
        self.get_logger().info(f"Ignored payload on {topic}: {payload}")

    # ===== 시리얼 유틸 =====
    def _enqueue_action(self, action: str):
        try:
            self._rx_q.put_nowait(action)
        except queue.Full:
            self.get_logger().warn("RX queue full; dropping action")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info("Serial opened; waiting reset...")
            time.sleep(2.0)  # UNO auto-reset 대기
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info("Serial ready.")
        except Exception as e:
            self.get_logger().warn(f"Open failed: {e}")
            self.ser = None

    def _ensure_serial(self):
        if self.ser and self.ser.is_open:
            return
        now = time.time()
        if now - self.last_try < 1.5:
            return
        self.last_try = now
        self._open_serial()

    def _send_serial(self, line: str):
        if not (self.ser and self.ser.is_open):
            self.get_logger().warn("Serial not ready; drop")
            return
        try:
            self.ser.write((line.strip() + "\n").encode('ascii'))
            self.get_logger().info(f"TX -> Arduino: {line.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ===== 시리얼 수신 폴링 =====
    def _poll_serial(self):
        if not (self.ser and self.ser.is_open):
            return
        try:
            # in_waiting 만큼 라인 단위로 빠르게 소진
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # READY / ACK 로그
                if line.startswith("READY"):
                    self.get_logger().info("Arduino: READY")
                    continue
                if line.startswith("ACK:"):
                    self.get_logger().info(f"Arduino: {line}")
                    continue

                # 배터리 전압: START 직후 1회만 기록
                if line.startswith("VBATT:"):
                    if not self.vbatt_captured:
                        try:
                            val_txt = line.split("VBATT:")[1].strip()
                            self.vbatt_value = float(val_txt)
                            self.vbatt_captured = True
                            self.get_logger().info(f"VBATT (captured once): {self.vbatt_value:.2f} V")
                        except Exception:
                            self.get_logger().warn(f"VBATT parse failed: '{line}'")
                    # 이후 VBATT 라인은 무시
                    continue

                # 그 외 로그
                self.get_logger().info(f"Arduino: {line}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    # ===== 큐 처리 =====
    def _process_commands(self):
        # 시리얼 보장
        self._ensure_serial()

        # 대기 중인 액션 처리
        handled = 0
        while not self._rx_q.empty() and handled < 20:
            try:
                action = self._rx_q.get_nowait()
            except queue.Empty:
                break
            handled += 1
            if action == "start":
                # START 전압 1회 캡처 모드로 전환
                self.vbatt_captured = False
                self.vbatt_value = None
                self._send_serial("START")
            elif action == "stop":
                self._send_serial("STOP")

    # ===== 정리 =====
    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlMQTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
