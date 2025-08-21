#!/usr/bin/env python3
# tts_mqtt_node_openai_tts.py — Mimic3 제거, OpenAI TTS로 교체 + 볼륨 조절(ko-only)
#
# 필요:
#   pip install openai paho-mqtt
#   export OPENAI_API_KEY="sk-..."  # 환경변수 설정
#   aplay(alsa-utils) 또는 ffplay(ffmpeg) 또는 paplay(PulseAudio) 중 1개
#
# ✅ 유지 기능
#   - MQTT 'llm/action' 구독 → "시작하겠습니다."/"멈추겠습니다." 등 발화
#   - QC 토픽('/qc_result', 'qc/verdict') 합·불 발화
#   - 'inspect_circuit' 액션 안내 멘트
# ✅ 변경/추가
#   - TTS: OpenAI gpt-4o-mini-tts 사용
#   - 볼륨 조절: VOLUME(기본 1.0, 0.0~2.0 권장)
#   - 영어 TTS 관련 분기/문구 제거(ko only)

import ssl
import json
import time
import uuid
import queue
import threading
import subprocess
import shutil
import tempfile
import os

import paho.mqtt.client as mqtt
from openai import OpenAI

# ===== MQTT 설정 =====
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"
SUB_TOPIC = "llm/action"
MQTT_QOS = 1
CLIENT_ID = f"tts-openai-{uuid.uuid4().hex[:8]}"

# ★ QC 결과 토픽(기존 유지)
SUB_QC_RESULT = "/qc_result"   # 아두이노: {"result":"OK"|"NG"}
SUB_QC_VERDICT = "qc/verdict"  # 써킷: {"verdict":"PASS"|"FAIL"|"OK"|"NG"}

# ===== TTS 모델 & 설정 =====
OPENAI_TTS_MODEL = "gpt-4o-mini-tts"   # OpenAI TTS 모델
OPENAI_TTS_VOICE = "alloy"             # 음색
AUDIO_FORMAT = "wav"                    # "wav" 권장(지연 낮음)
VOLUME = 2.0                            # 0.0(무음) ~ 2.0 권장, 기본 1.0

# 재생기 우선순위
PLAYERS = [
    ["aplay", "-q"],
    ["ffplay", "-loglevel", "quiet", "-autoexit", "-nodisp"],
    ["paplay"],
]

# ===== 발화 문구(한국어 전용) =====
TEXT_START = "시작하겠습니다."
TEXT_STOP = "멈추겠습니다."
TEXT_CANCEL = "알겠습니다."
TEXT_INSPECT = "회로 판별을 시작하겠습니다."
TEXT_OK = "양품입니다."
TEXT_NG = "불량입니다."

ALLOWED_ACTIONS = {"start", "stop", "inspect_circuit", "cancel"}


def pick_text(action: str) -> str:
    a = (action or "").lower()
    if a == "inspect_circuit":
        return TEXT_INSPECT
    if a == "cancel":
        return TEXT_CANCEL
    return TEXT_START if a == "start" else TEXT_STOP


def pick_text_qc(result: str) -> str:
    is_ok = (str(result).upper() == "OK")
    return TEXT_OK if is_ok else TEXT_NG


def pick_text_verdict(verdict: str) -> str:
    v = (verdict or "").upper()
    is_ok = (v in ("PASS", "OK"))
    return TEXT_OK if is_ok else TEXT_NG


# ===== OpenAI TTS 호출 + 재생 =====
_openai_client = OpenAI()  # OPENAI_API_KEY 환경변수 사용


def _find_player():
    for cmd in PLAYERS:
        if shutil.which(cmd[0]):
            return cmd
    return None


def _play_audio(path: str) -> bool:
    player = _find_player()
    if not player:
        print("[TTS] 오디오 플레이어(aplay/ffplay/paplay)를 찾을 수 없습니다.")
        return False

    base = player.copy()
    exe = base[0]

    try:
        if exe == "ffplay":
            # ffplay는 소프트 볼륨 필터 지원
            args = base[:]
            if abs(VOLUME - 1.0) > 1e-3:
                args += ["-af", f"volume={VOLUME}"]
            args += [path]
            subprocess.run(args, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif exe == "paplay":
            args = base[:]
            if abs(VOLUME - 1.0) > 1e-3:
                pa_vol = int(max(0.0, min(VOLUME, 2.0)) * 65536)  # 0~65536
                args += [f"--volume={pa_vol}"]
            args += [path]
            subprocess.run(args, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:  # aplay 등 볼륨 파라미터 없음
            subprocess.run(base + [path], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if abs(VOLUME - 1.0) > 1e-3:
                print("[TTS] 참고: aplay는 내부 볼륨 조절 미지원입니다. ffplay/paplay 사용 시 소프트 볼륨 적용됩니다.")
        return True
    except Exception as e:
        print("[TTS] 재생 오류:", e)
        return False


def speak(text: str) -> bool:
    """OpenAI TTS로 합성 후 로컬 재생기(aplay/ffplay/paplay)로 재생."""
    tmp = tempfile.NamedTemporaryFile(prefix="oai_tts_", suffix=f".{AUDIO_FORMAT}", delete=False)
    tmp_path = tmp.name
    tmp.close()

    try:
        # 스트리밍 저장 (지연 최소화)
        with _openai_client.audio.speech.with_streaming_response.create(
            model=OPENAI_TTS_MODEL,
            voice=OPENAI_TTS_VOICE,
            input=text,
            response_format=AUDIO_FORMAT,
        ) as resp:
            resp.stream_to_file(tmp_path)

        ok = _play_audio(tmp_path)
        return ok
    except Exception as e:
        print("[TTS] OpenAI TTS 오류:", e)
        return False
    finally:
        try:
            os.remove(tmp_path)
        except Exception:
            pass


# ===== MQTT Node =====
class TTSOpenAIMQTTNode:
    def __init__(self):
        self.q: "queue.Queue[str]" = queue.Queue(maxsize=32)
        self._running = True
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        # 중복 억제
        self.last_action = None
        self.last_time = 0.0
        self.debounce_sec = 0.8

        # MQTT
        self.mqtt = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
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
        print(f"[TTS] MQTT {MQTT_HOST}:{MQTT_PORT} sub='{SUB_TOPIC}', qos={MQTT_QOS}")
        print(f"[TTS] MODEL={OPENAI_TTS_MODEL}, VOICE={OPENAI_TTS_VOICE}, FORMAT={AUDIO_FORMAT}, VOLUME={VOLUME}")

    def _worker(self):
        while self._running:
            try:
                text = self.q.get()
                ok = speak(text)
                if not ok:
                    print("[TTS] 합성/재생 실패. 네트워크/플레이어/키 설정 확인 필요.")
                self.q.task_done()
            except Exception as e:
                print("[TTS] worker error:", e)
                time.sleep(0.1)

    def _enqueue(self, text: str):
        try:
            self.q.put_nowait(text)
        except queue.Full:
            try:
                _ = self.q.get_nowait()
                self.q.task_done()
            except Exception:
                pass
            try:
                self.q.put_nowait(text)
            except Exception:
                pass

    # MQTT
    def _on_connect(self, client, userdata, flags, rc):
        print(f"[TTS] MQTT connected rc={rc}; subscribing '{SUB_TOPIC}', '{SUB_QC_RESULT}', '{SUB_QC_VERDICT}'")
        client.subscribe(SUB_TOPIC, qos=MQTT_QOS)
        client.subscribe(SUB_QC_RESULT, qos=MQTT_QOS)
        client.subscribe(SUB_QC_VERDICT, qos=MQTT_QOS)

    def _on_disconnect(self, client, userdata, rc):
        print(f"[TTS] MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8').strip()
        except Exception:
            return

        # 1) QC 결과
        if msg.topic == SUB_QC_RESULT:
            try:
                obj = json.loads(payload)
                result = (obj.get("result") or "").upper()
            except Exception:
                print(f"[TTS] Bad JSON on {msg.topic}: {payload}")
                return
            if result in ("OK", "NG"):
                text = pick_text_qc(result)
                self._enqueue(text)
                print("[TTS] SAY:", text)
            else:
                print(f"[TTS] Ignored qc result: {payload}")
            return

        if msg.topic == SUB_QC_VERDICT:
            try:
                obj = json.loads(payload)
                verdict = str(obj.get("verdict", ""))
            except Exception:
                print(f"[TTS] Bad JSON on {msg.topic}: {payload}")
                return
            text = pick_text_verdict(verdict)
            self._enqueue(text)
            print("[TTS] SAY:", text)
            return

        # 2) 액션 처리
        action = None
        parsed = None
        if payload.startswith("{"):
            try:
                parsed = json.loads(payload)
            except Exception:
                parsed = None
        if isinstance(parsed, dict):
            action = (parsed.get("action") or "").lower()
        else:
            action = payload.lower()

        if action in ALLOWED_ACTIONS:
            now = time.time()
            if action == self.last_action and (now - self.last_time) < self.debounce_sec:
                return
            self.last_action = action
            self.last_time = now
            say = pick_text(action)
            self._enqueue(say)
            print("[TTS] SAY:", say)
        else:
            print(f"[TTS] Ignored payload on {msg.topic}: {payload}")

    def close(self):
        self._running = False
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass


def main():
    node = TTSOpenAIMQTTNode()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()


if __name__ == "__main__":
    main()
