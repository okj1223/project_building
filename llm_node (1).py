#!/usr/bin/env python3
# llm_node_mqtt.py — cancel intent 패스스루 + 명령 매핑
# MQTT 'stt/voice_command' -> (의도 판별) -> MQTT 'llm/action'
#
# 입력:
#  - JSON: {"text": "<문장>", "ts": <epoch>, "intent": "cancel"|... }  # intent는 선택
#  - 또는 순수 텍스트: "<문장>"
#
# 출력:
#  - JSON: {"action":"start"|"stop"|"inspect_circuit"|"cancel"}
#
import os
import json
import ssl
import time
import uuid
import queue
import threading
from openai import OpenAI
import paho.mqtt.client as mqtt

# ===================== MQTT =====================
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883  # TLS
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"
SUB_TOPIC = "stt/voice_command"
PUB_TOPIC = "llm/action"
MQTT_QOS = 1
MQTT_CLIENT_ID = f"llm-node-{uuid.uuid4().hex[:8]}"

# ===================== LLM =====================
MODEL_NAME = os.getenv("LLM_MODEL", "gpt-4o")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

SYSTEM_PROMPT = (
  "Convert the phrase to EXACT JSON.\n"
  "Korean allowed. Output one of: start/stop/inspect_circuit/cancel.\n"
  "- If it means cancel/negation like '아니야','아냐','취소','그만','no','cancel' -> {\"action\":\"cancel\"}\n"
  "- If inspect/recognize circuit board -> {\"action\":\"inspect_circuit\"}\n"
  "- If start motor -> {\"action\":\"start\"}\n"
  "- If stop motor -> {\"action\":\"stop\"}\n"
  "Output JSON ONLY."
)

CANCEL_WORDS = ["아니야","아냐"]

ALLOWED = {"start","stop","inspect_circuit","cancel"}

class LLMNodeMQTT:
    def __init__(self):
        if not OPENAI_API_KEY:
            raise RuntimeError("환경변수 OPENAI_API_KEY가 필요합니다.")
        self.client = OpenAI(api_key=OPENAI_API_KEY)

        self.rx_q: "queue.Queue[dict]" = queue.Queue(maxsize=200)

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
        self.mqtt.connect_async(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.mqtt.loop_start()

        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        print(f"[LLM] sub='{SUB_TOPIC}' pub='{PUB_TOPIC}', model={MODEL_NAME}")

    # MQTT
    def _on_connect(self, client, userdata, flags, rc):
        print(f"[LLM] MQTT connected rc={rc}; subscribing '{SUB_TOPIC}'")
        client.subscribe(SUB_TOPIC, qos=MQTT_QOS)

    def _on_disconnect(self, client, userdata, rc):
        print(f"[LLM] MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8').strip()
        except Exception:
            return

        text, intent = None, None
        try:
            obj = json.loads(payload)
            if isinstance(obj, dict):
                text = (obj.get("text") or "").strip()
                intent = (obj.get("intent") or "").strip().lower()
            elif isinstance(obj, str):
                text = obj.strip()
        except Exception:
            text = payload

        if not text:
            print(f"[LLM] Ignored: {payload}")
            return

        # intent == cancel 이면 즉시 publish (LLM 호출 생략)
        if intent == "cancel":
            self._publish_action({"action":"cancel"})
            return

        try:
            self.rx_q.put_nowait({"text":text})
        except queue.Full:
            print("[LLM] RX queue full; dropping")

    def _worker_loop(self):
        while True:
            item = self.rx_q.get()
            try:
                text = item["text"]
                out = self._map_to_action(text)
                self._publish_action(out)
            except Exception as e:
                print(f"[LLM] worker error: {e}")

    def _map_to_action(self, text: str) -> dict:
        # 휴리스틱: 취소어는 즉시 cancel
        t = (text or "").lower()
        if any(w in t for w in CANCEL_WORDS):
            return {"action":"cancel"}

        # LLM 호출
        result = {"action":"stop"}
        try:
            resp = self.client.chat.completions.create(
                model=MODEL_NAME,
                messages=[
                    {"role":"system","content":SYSTEM_PROMPT},
                    {"role":"user","content":f"Phrase: {text}"},
                ],
                temperature=0
            )
            raw = (resp.choices[0].message.content or "").strip()
            parsed = json.loads(raw)
            act = (parsed.get("action") or "").lower()
            result = {"action": act if act in ALLOWED else "stop"}
        except Exception:
            # 백업 규칙
            if ("회로" in t and ("판별" in t or "인식" in t or "보드" in t)) or ("inspect" in t and "circuit" in t):
                result = {"action":"inspect_circuit"}
            elif any(k in t for k in ["멈춰","정지","중지","스톱","그만","stop","halt"]):
                result = {"action":"stop"}
            elif any(k in t for k in ["시작","출발","가동","돌아","전진","스타트","start","go","run"]):
                result = {"action":"start"}
            else:
                result = {"action":"stop"}
        return result

    def _publish_action(self, data: dict):
        try:
            payload = json.dumps(data, ensure_ascii=False)
            r = self.mqtt.publish(PUB_TOPIC, payload=payload.encode('utf-8'),
                                  qos=MQTT_QOS, retain=False)
            if r.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"[LLM] publish rc={r.rc}")
            else:
                print(f"[LLM] → {PUB_TOPIC}: {payload}")
        except Exception as e:
            print(f"[LLM] Publish error: {e}")

def main():
    node = LLMNodeMQTT()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.mqtt.loop_stop(); node.mqtt.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
