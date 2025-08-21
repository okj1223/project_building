#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
개요
----
'내 목소리만 반응' 강제 버전.
- 개인 KWS(ONNX) + 화자게이트(ECAPA) 둘 다 통과해야 웨이크
- ONNX 출력 logit/prob 자동 판별
- 부팅 직후 소음 기반 동적 임계값 자동 보정
- 최근 3회 중 2회 통과 + 평균 대비 마진

필수 산출물:
  - /home/okj1812/models/kws_model.onnx
  - /home/okj1812/models/thresholds.json   (kws_thresh, clip_sec, n_mels, fmin, fmax, spk_thresh)
  - /home/okj1812/models/user_embed.npy     (화자게이트)
"""

import os
import sys
import tempfile
import threading
import select
import termios
import tty
import uuid
import json
import numpy as np
import sounddevice as sd
import soundfile as sf
import ssl
import time
from collections import deque
import re
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

import paho.mqtt.client as mqtt

# ===== 외부 라이브러리 =====
try:
    import webrtcvad
    _HAS_WEBRTCVAD = True
except Exception:
    _HAS_WEBRTCVAD = False

try:
    from openwakeword.model import Model as OWWModel
    _HAS_OWW = True
except Exception:
    _HAS_OWW = False

try:
    import onnxruntime as ort
    _HAS_ORT = True
except Exception:
    _HAS_ORT = False

try:
    from speechbrain.pretrained import EncoderClassifier
    _HAS_SPEECHBRAIN = True
except Exception:
    _HAS_SPEECHBRAIN = False

# ===== 경로/플래그 =====
USE_PERSONAL_KWS = True
PERSONAL_KWS_ONNX = os.getenv('PERSONAL_KWS_ONNX', '/home/okj1812/models/kws_model.onnx')
PERSONAL_THRESHOLDS = os.getenv('PERSONAL_THRESHOLDS', '/home/okj1812/models/thresholds.json')
PERSONAL_USER_EMBED = os.getenv('PERSONAL_USER_EMBED', '/home/okj1812/models/user_embed.npy')

FORCE_PERSONAL_ONLY = True        # 백업 비활성
REQUIRE_SPK_GATE = True           # 화자게이트 없으면 웨이크 절대 불가
LOG_KWS_DEBUG = True

# ===== 자동 캘리브레이션(소음 기반) =====
KWS_CALIBRATION_SEC = 4.0   # 시작 후 이만큼 소음 샘플링
KWS_P95_BOOST = 0.02        # p95 + margin
KWS_SIGMA_BOOST = 3.0       # mean + 3*std 가드
KWS_MIN_MARGIN = 0.05       # 평균 대비 최소 마진
KWS_CONSEC_PASS = 1         # 최근 3중 2회 이상 통과

# ===== MQTT =====
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"
MQTT_TOPIC = "stt/voice_command"
MQTT_QOS = 1
MQTT_CLIENT_ID = f"stt-client-{uuid.uuid4().hex[:8]}"
ENABLE_ROS_PUB = False

# ===== 웨이크/자동 =====
AUTO_WAKE_MODE = True
WAKE_WINDOW_SEC = 1.2
WAKE_COOLDOWN_SEC = 1.6
USE_WEBRTCVAD = False and _HAS_WEBRTCVAD

# ===== 취소 =====
CANCEL_TEXTS = ["아니야", "아냐"]
CANCEL_WINDOW_SEC = 1.0
CANCEL_COOLDOWN_SEC = 1.0

# ===== VAD =====
STOP_SILENCE_SEC = 0.9
MIN_UTTERANCE_SEC = 0.6
ENERGY_SPEECH_RMS = 800.0
ENERGY_SILENCE_RMS = 350.0

# ===== 민감도 =====
WAKE_RMS_THRESHOLD = 2000.0
WAKE_RMS_GATE_MULTIPLIER = 2.0
WAKE_MIN_ENERGY_DURATION = 0.35

# ===== openwakeword (미사용) =====
OWW_THRESHOLD = 0.85

# ===== 길이 제한 =====
MAX_UTTERANCE_SEC = 3.0

# ===== 효과음 =====
CHIME_VOL = 0.2
CHIME_SR = 16000
CHIME_START = {"freq": 1200, "ms": 120}
CHIME_END   = {"freq": 900,  "ms": 180}
CHIME_REFRACTORY_SEC = 0.35

# ===== TTS 리프랙토리 =====
TTS_REFRACTORY_SEC = 5.0
ENABLE_TTS_REFRACTORY = True

# ===== LLM 연동 =====
PUBLISH_LLM_ACTION = False
LLM_ACTION_TOPIC = "llm/action"

# ===== 화자게이트 파라미터 =====
SPK_WIN_SEC = 1.28
SPK_MULTI_CROP = 3
SPK_JITTER_SEC = 0.22
SPK_MARGIN = 0.01
SPK_MAX_STD = 0.08
FAILED_SPK_COOLDOWN = 2.0
# ----- 강제 화자검증 바닥선(소프트 바이패스) -----
SPK_FORCE_FLOOR = 0.60           # prob가 이 값 이상이면 화자검증 강제 진입
SPK_FORCE_DELTA = 0.05           # dyn_th - 0.05 이상이면 진입 허용


class STTManualMQTTNode(Node):
    def __init__(self):
        super().__init__('stt_manual_mqtt_node')

        self._last_spk_fail_at = 0.0
        self._has_voice = False
        self._idle_last_rms = 0.0
        self._suppress_input_until = 0.0

        # === OpenAI ===
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise RuntimeError("환경변수 OPENAI_API_KEY가 설정되어 있지 않습니다.")
        self.client = OpenAI(api_key=api_key)

        # === ROS2 퍼블리셔(옵션) ===
        self.enable_ros_pub = ENABLE_ROS_PUB
        if self.enable_ros_pub:
            self.pub = self.create_publisher(String, '/voice_command', 10)

        # === 오디오 ===
        self.fs = 16000
        self.channels = 1
        self.dtype = 'int16'

        # === 상태 ===
        self.is_recording = False
        self._buf_lock = threading.Lock()
        self._frames = []
        self._running = True
        self.state = 'IDLE'
        self._silence_accum = 0.0
        self._utter_time = 0.0

        # 웨이크 보조
        self._ring = deque(maxlen=int(self.fs * 3))
        self._last_wake_try = 0.0
        self._wake_busy = False

        # 취소 보조
        self._last_cancel_try = 0.0
        self._cancel_busy = False

        # 에너지 누적
        self._wake_energy_start = 0.0
        self._wake_energy_accum = 0.0

        # OWW (강제 미사용)
        self._oww = None
        if _HAS_OWW and not FORCE_PERSONAL_ONLY:
            try:
                self._oww = OWWModel()
                self.get_logger().info("openwakeword 활성화")
            except Exception as e:
                self.get_logger().warn(f"openwakeword 초기화 실패: {e}")
                self._oww = None

        # === 개인 KWS/화자게이트 ===
        self._kws_sess = None
        self._kws_input_name = None
        self._kws_output_name = None
        self._kws_thresh = None
        self._kws_cfg = {}
        self._mel_fb = None
        self._hann = None

        self._spk_centroid = None
        self._spk_thresh = None
        self._spk_model = None

        # 동적 임계 관련 상태
        self._kws_output_is_prob = None
        self._dyn_thresh = None
        self._ambient_stats = None
        self._prob_hist = deque(maxlen=3)

        # KWS 로드
        if USE_PERSONAL_KWS and _HAS_ORT:
            try:
                if os.path.exists(PERSONAL_KWS_ONNX) and os.path.exists(PERSONAL_THRESHOLDS):
                    self._kws_sess = ort.InferenceSession(
                        PERSONAL_KWS_ONNX,
                        providers=['CPUExecutionProvider']
                    )
                    self._kws_input_name = self._kws_sess.get_inputs()[0].name
                    self._kws_output_name = self._kws_sess.get_outputs()[0].name

                    with open(PERSONAL_THRESHOLDS, 'r', encoding='utf-8') as f:
                        self._kws_cfg = json.load(f) or {}
                    self._kws_thresh = float(self._kws_cfg.get('kws_thresh', 0.99))

                    clip_sec = float(self._kws_cfg.get('clip_sec', 1.28))
                    hop = 160
                    exp_frames = 1 + int((self.fs * clip_sec) // hop)
                    self.get_logger().info(
                        f"개인 KWS 로드 완료 (th={self._kws_thresh:.3f}) | "
                        f"expect frames={exp_frames} (n_mels={self._kws_cfg.get('n_mels',40)})"
                    )
                else:
                    self.get_logger().error("개인 KWS 산출물 미존재 — onnx/thresholds 경로 확인")
            except Exception as e:
                self.get_logger().warn(f"개인 KWS 초기화 실패: {e}")

        # 화자게이트 로드 (필수)
        if _HAS_SPEECHBRAIN and os.path.exists(PERSONAL_USER_EMBED):
            try:
                self._spk_centroid = np.load(PERSONAL_USER_EMBED).astype(np.float32)
                self._spk_centroid = self._spk_centroid / (np.linalg.norm(self._spk_centroid) + 1e-9)
                self._spk_thresh = float(self._kws_cfg.get('spk_thresh', 0.5))  # thresholds.json에 있어야 함
                self._spk_model = EncoderClassifier.from_hparams(
                    source="speechbrain/spkrec-ecapa-voxceleb",
                    savedir=os.path.join(tempfile.gettempdir(), "ecapa_runtime")
                )
                self.get_logger().info(f"화자게이트 활성화 (th={self._spk_thresh:.3f})")
            except Exception as e:
                self.get_logger().error(f"화자게이트 초기화 실패: {e}")
                self._spk_centroid = None
                self._spk_model = None
        else:
            self.get_logger().error("user_embed.npy 또는 speechbrain 미탑재")

        # 화자게이트 필수 모드: 없으면 자동 웨이크 비활성
        if REQUIRE_SPK_GATE and (self._spk_model is None or self._spk_centroid is None or self._spk_thresh is None):
            self.get_logger().error("REQUIRE_SPK_GATE=ON 이지만 화자게이트 자원 미존재 → AUTO_WAKE_MODE 비활성")
            # 자동모드 끄고 수동만 허용
            self.auto_allowed = False
        else:
            self.auto_allowed = True

        # === MQTT ===
        self.mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.mqtt_client.tls_set_context(context)
        self.mqtt_client.max_queued_messages_set(100)
        self.mqtt_client.max_inflight_messages_set(20)
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_log = self._on_mqtt_log
        try:
            self.mqtt_client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
            raise

        # === 오디오 스트림 ===
        try:
            self.stream = sd.InputStream(
                samplerate=self.fs,
                channels=self.channels,
                dtype=self.dtype,
                callback=self._audio_callback,
                blocksize=1024,
            )
            self.stream.start()
        except Exception as e:
            self.get_logger().error(f"오디오 스트림 초기화 실패: {e}")
            raise

        # === 부팅 직후 소음 캘리브레이션 ===
        threading.Thread(target=self._kws_autocalibration, daemon=True).start()

        # === 키보드 리스너 ===
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(
            f"STT Node 시작 | MQTT {MQTT_HOST}:{MQTT_PORT} topic='{MQTT_TOPIC}', qos={MQTT_QOS} | "
            f"AUTO_WAKE_MODE={'ON' if AUTO_WAKE_MODE else 'OFF'} | "
            f"FORCE_PERSONAL_ONLY={'ON' if FORCE_PERSONAL_ONLY else 'OFF'} | "
            f"REQUIRE_SPK_GATE={'ON' if REQUIRE_SPK_GATE else 'OFF'} | "
            f"AUTO_ALLOWED={'YES' if self.auto_allowed else 'NO (manual r/s only)'}"
        )

    # ===== MQTT =====
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT 연결 성공")
        else:
            self.get_logger().error(f"MQTT 연결 실패 rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f"MQTT 연결 끊김 rc={rc}")

    def _on_mqtt_log(self, client, userdata, level, buf):
        if level <= logging.WARNING:
            self.get_logger().debug(f"MQTT: {buf}")

    # ===== 유틸 =====
    @staticmethod
    def _rms_int16(x: np.ndarray) -> float:
        if x.size == 0:
            return 0.0
        xf = x.astype(np.float32)
        return float(np.sqrt(np.mean(xf * xf)))

    def _pad_or_trim(self, y: np.ndarray, target_len: int) -> np.ndarray:
        if y.size < target_len:
            return np.pad(y, (0, target_len - y.size))
        return y[:target_len]

    def _hz_to_mel(self, hz: float) -> float:
        return 2595.0 * np.log10(1.0 + hz / 700.0)

    def _mel_to_hz(self, mel: float) -> float:
        return 700.0 * (10.0**(mel / 2595.0) - 1.0)

    def _mel_filterbank(self, sr=16000, n_fft=512, n_mels=40, fmin=20.0, fmax=7600.0) -> np.ndarray:
        if self._mel_fb is not None:
            return self._mel_fb
        m_min = self._hz_to_mel(fmin)
        m_max = self._hz_to_mel(fmax)
        m_pts = np.linspace(m_min, m_max, n_mels + 2)
        f_pts = self._mel_to_hz(m_pts)
        fft_bins = np.floor((n_fft + 1) * f_pts / sr).astype(int)
        fb = np.zeros((n_mels, n_fft // 2 + 1), dtype=np.float32)
        for m in range(1, n_mels + 1):
            f_m_minus, f_m, f_m_plus = fft_bins[m - 1], fft_bins[m], fft_bins[m + 1]
            if f_m_minus == f_m:
                f_m -= 1
            if f_m == f_m_plus:
                f_m_plus += 1
            for k in range(max(f_m_minus, 0), min(f_m, n_fft // 2) + 1):
                fb[m - 1, k] = (k - f_m_minus) / float(max(f_m - f_m_minus, 1))
            for k in range(max(f_m, 0), min(f_m_plus, n_fft // 2) + 1):
                fb[m - 1, k] = (f_m_plus - k) / float(max(f_m_plus - f_m, 1))
        self._mel_fb = fb
        return fb

    def _logmel_from_int16(self, int16_pcm: np.ndarray, clip_sec=1.28) -> np.ndarray:
        n_fft = 512
        win = 400
        hop = 160
        n_mels = int(self._kws_cfg.get('n_mels', 40))
        fmin = float(self._kws_cfg.get('fmin', 20.0))
        fmax = float(self._kws_cfg.get('fmax', 7600.0))
        sr = self.fs

        target_len = int(sr * clip_sec)
        y = (int16_pcm.astype(np.float32) / 32768.0)
        y = self._pad_or_trim(y, target_len)

        pad = n_fft // 2  # center=True
        y = np.pad(y, (pad, pad), mode='constant')

        if self._hann is None or self._hann.size != win:
            self._hann = np.hanning(win).astype(np.float32)
        win_buf = np.zeros(n_fft, dtype=np.float32)
        win_buf[:win] = self._hann

        frames = 1 + (y.size - n_fft) // hop
        if frames < 1:
            frames = 1

        S = np.empty((n_fft // 2 + 1, frames), dtype=np.float32)
        for i in range(frames):
            s = i * hop
            frame = y[s:s + n_fft]
            if frame.size < n_fft:
                frame = np.pad(frame, (0, n_fft - frame.size))
            frame = frame * win_buf
            spec = np.fft.rfft(frame, n=n_fft)
            S[:, i] = (spec.real**2 + spec.imag**2)

        fb = self._mel_filterbank(sr=sr, n_fft=n_fft, n_mels=n_mels, fmin=fmin, fmax=fmax)
        M = np.dot(fb, S)
        M = np.maximum(M, 1e-10)
        M_db = 10.0 * np.log10(M)
        M_db = (M_db - M_db.mean()) / (M_db.std() + 1e-6)

        return M_db[np.newaxis, np.newaxis, :, :].astype(np.float32)

    # ===== KWS 추론 (logit/prob 자동 판별) =====
    def _kws_forward_prob(self, logmel: np.ndarray) -> float:
        if self._kws_sess is None:
            return 0.0
        outs = self._kws_sess.run(
            [self._kws_output_name],
            {self._kws_input_name: logmel.astype(np.float32)}
        )[0]
        val = float(np.asarray(outs).ravel()[0])

        if self._kws_output_is_prob is None:
            self._kws_output_is_prob = (0.0 <= val <= 1.0)
            self.get_logger().info(f"[KWS] output type auto-detect: "
                                   f"{'prob' if self._kws_output_is_prob else 'logit'}")

        prob = val if self._kws_output_is_prob else 1.0 / (1.0 + np.exp(-val))

        if LOG_KWS_DEBUG:
            x = logmel
            self.get_logger().info(
                f"[KWS] in{tuple(x.shape)} μ={float(x.mean()):.3f} σ={float(x.std()):.3f} "
                f"min={float(x.min()):.2f} max={float(x.max()):.2f} | raw={val:.3f} prob={prob:.3f}"
            )
        return prob

    # ===== 부팅 직후 자동 캘리브레이션 =====
    def _kws_autocalibration(self):
        if not (self._kws_sess and self._kws_thresh is not None):
            return
        t_end = time.time() + KWS_CALIBRATION_SEC
        probs = []
        clip_sec = float(self._kws_cfg.get('clip_sec', 1.28))
        need = int(self.fs * clip_sec)
        self.get_logger().info(f"[KWS] ambient calibration start ({KWS_CALIBRATION_SEC:.1f}s)")
        while time.time() < t_end:
            time.sleep(0.08)
            ring = np.array(self._ring, dtype=np.int16)
            if ring.size < need:
                continue
            seg = ring[-need:]
            m = self._logmel_from_int16(seg, clip_sec=clip_sec)
            p = self._kws_forward_prob(m)
            probs.append(p)

        if len(probs) < 5:
            self.get_logger().warn("[KWS] ambient calibration insufficient samples; skip")
            return

        arr = np.asarray(probs, dtype=np.float32)
        p95 = float(np.percentile(arr, 95))
        mean = float(arr.mean())
        std = float(arr.std())
        dyn = max(self._kws_thresh, p95 + KWS_P95_BOOST, mean + KWS_SIGMA_BOOST * std)
        cap = min(0.995, self._kws_thresh + 0.03)
        dyn = min(max(dyn, 0.5), cap)
        self._ambient_stats = {"mean": mean, "std": std, "p95": p95}
        self._dyn_thresh = dyn
        self.get_logger().info(f"[KWS] ambient stats: mean={mean:.3f} std={std:.3f} p95={p95:.3f} → dyn_th={dyn:.3f}")

    # ---- 텍스트 유틸 (취소어) ----
    @staticmethod
    def _preemphasis(x: np.ndarray, alpha: float = 0.97) -> np.ndarray:
        if x.size == 0:
            return x
        y = np.copy(x)
        y[1:] = y[1:] - alpha * y[:-1]
        return y

    @staticmethod
    def _normalize_ko(s: str) -> str:
        if not s:
            return ""
        s = s.lower()
        s = re.sub(r"[^0-9a-z가-힣]+", "", s)
        s = s.replace("끼", "키")
        return s

    @staticmethod
    def _edit_distance(a: str, b: str) -> int:
        la, lb = len(a), len(b)
        dp = list(range(lb + 1))
        for i, ca in enumerate(a, 1):
            prev = dp[0]
            dp[0] = i
            for j, cb in enumerate(b, 1):
                cur = dp[j]
                dp[j] = min(dp[j] + 1, dp[j-1] + 1, prev + (0 if ca == cb else 1))
                prev = cur
        return dp[-1]

    # ===== 효과음 =====
    def _play_chime(self, freq_hz: int, ms: int, vol: float = CHIME_VOL):
        try:
            dur = ms / 1000.0
            t = np.linspace(0, dur, int(CHIME_SR * dur), endpoint=False)
            tone = np.sin(2 * np.pi * freq_hz * t).astype(np.float32)
            n = tone.size
            if n == 0:
                return
            a = max(1, int(0.02 * n))
            d = max(1, int(0.1 * n))
            env = np.ones(n, dtype=np.float32)
            if a > 1:
                env[:a] = np.linspace(0, 1, a, dtype=np.float32)
            if d > 1 and d < n:
                env[-d:] = np.linspace(1, 0, d, dtype=np.float32)
            out = (vol * tone * env).reshape(-1, 1)
            sd.play(out, samplerate=CHIME_SR, blocking=True)
        except Exception as e:
            self.get_logger().warn(f"효과음 재생 실패: {e}")

    def _chime_start(self):
        self._play_chime(CHIME_START["freq"], CHIME_START["ms"])

    def _chime_end(self):
        self._play_chime(CHIME_END["freq"], CHIME_END["ms"])

    # ===== MQTT publish =====
    def _mqtt_publish(self, topic: str, data: str):
        try:
            if not self.mqtt_client.is_connected():
                self.get_logger().warn("MQTT 연결되지 않음 - 메시지 드롭")
                return
            result = self.mqtt_client.publish(topic, payload=data.encode('utf-8'), qos=MQTT_QOS, retain=False)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f"MQTT publish 실패 rc={result.rc}")
        except Exception as e:
            self.get_logger().error(f"MQTT publish 에러: {e}")

    def _suppress_for(self, sec: float):
        try:
            sec = float(sec)
        except (ValueError, TypeError):
            sec = 0.0
        suppress_until = time.time() + max(0.0, sec)
        self._suppress_input_until = max(self._suppress_input_until, suppress_until)
        if sec > 1.0:
            self.get_logger().info(f"입력 억제: {sec:.1f}초")

    def set_tts_refractory(self, seconds: float):
        global TTS_REFRACTORY_SEC
        TTS_REFRACTORY_SEC = max(0.0, float(seconds))
        self.get_logger().info(f"TTS 억제 시간 변경: {TTS_REFRACTORY_SEC}초")

    # ===== 메인 오디오 콜백 =====
    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")

        now = time.time()
        if now < self._suppress_input_until:
            self.get_logger().debug("skip: suppress window")
            return

        chunk = indata.copy().reshape(-1)
        self._ring.extend(chunk.tolist())

        if self.state == 'LISTENING':
            with self._buf_lock:
                self._frames.append(indata.copy())
            rms = self._rms_int16(chunk)
            dt = frames / float(self.fs)
            self._utter_time += dt

            if rms < ENERGY_SILENCE_RMS:
                self._silence_accum += dt
            elif rms > ENERGY_SILENCE_RMS * 1.2:
                self._silence_accum = 0.0
                self._has_voice = True

            if (now - self._last_cancel_try) >= CANCEL_COOLDOWN_SEC and not self._cancel_busy:
                if rms > ENERGY_SILENCE_RMS:
                    self._last_cancel_try = now
                    threading.Thread(target=self._try_detect_cancelword, daemon=True).start()

            if self._utter_time >= MAX_UTTERANCE_SEC:
                self.get_logger().info(f"최대 발화 길이 {MAX_UTTERANCE_SEC:.1f}s 도달 → 강제 종료")
                threading.Thread(target=self._finish_and_transcribe, daemon=True).start()
                self.state = 'BUSY'
                return

            if self._has_voice and self._utter_time >= MIN_UTTERANCE_SEC and self._silence_accum >= STOP_SILENCE_SEC:
                threading.Thread(target=self._finish_and_transcribe, daemon=True).start()
                self.state = 'BUSY'
                return

        if AUTO_WAKE_MODE and self.state == 'IDLE' and self.auto_allowed:
            if (now - self._last_wake_try) >= WAKE_COOLDOWN_SEC and not self._wake_busy:
                rms = self._rms_int16(chunk)
                self._idle_last_rms = rms
                wake_threshold = max(WAKE_RMS_THRESHOLD, ENERGY_SILENCE_RMS * WAKE_RMS_GATE_MULTIPLIER)

                if rms > wake_threshold:
                    if self._wake_energy_start == 0.0:
                        self._wake_energy_start = now
                        self._wake_energy_accum = 0.0
                    dt = frames / float(self.fs)
                    self._wake_energy_accum += dt
                    if self._wake_energy_accum >= WAKE_MIN_ENERGY_DURATION:
                        self._last_wake_try = now
                        self._wake_energy_start = 0.0
                        self._wake_energy_accum = 0.0
                        threading.Thread(target=self._try_detect_wakeword, daemon=True).start()
                else:
                    if rms < wake_threshold * 0.7:
                        self._wake_energy_start = 0.0
                        self._wake_energy_accum = 0.0


    # ===== 웨이크 시도 =====
    def _try_detect_wakeword(self):
        if self._wake_busy:
            return
        self._wake_busy = True
        try:
            # 개인 KWS(+화자)만 사용
            if self._kws_sess is not None and self._kws_thresh is not None:
                ok, prob, th = self._personal_wake_detect()
                self.get_logger().info(f"[KWS] prob={prob:.3f} th={th:.3f}"
                                    f"{' (dyn)' if self._dyn_thresh is not None else ' (static)'}")
                if ok:
                    self.get_logger().info("개인 KWS + 화자게이트 통과 → LISTENING")
                    self._trigger_listen()
                return


            # (여기까지 왔다는 건 KWS가 없음) — 강제 모드면 아무 것도 안 함
            if FORCE_PERSONAL_ONLY:
                return

        except Exception as e:
            self.get_logger().error(f"웨이크워드 감지 에러: {e}")
        finally:
            self._wake_busy = False

    # ===== 취소어 =====
    def _try_detect_cancelword(self):
        if self._cancel_busy:
            return
        self._cancel_busy = True
        try:
            ring = np.array(self._ring, dtype=np.int16)
            if ring.size < int(self.fs * CANCEL_WINDOW_SEC * 0.8):
                return
            seg = ring[-int(self.fs * CANCEL_WINDOW_SEC):]
            text = self._quick_transcribe(seg)
            if text and any(c in text for c in CANCEL_TEXTS):
                self.get_logger().info(f"취소어 감지: '{text}'")
                self._cancel_listen(text)
        except Exception as e:
            self.get_logger().warn(f"취소어 감지 에러: {e}")
        finally:
            self._cancel_busy = False

    def _cancel_listen(self, text: str = ""):
        with self._buf_lock:
            self._frames.clear()
        self.is_recording = False
        self.state = 'BUSY'
        self._silence_accum = 0.0
        self._utter_time = 0.0
        self._has_voice = False
        try:
            payload = {"text": (text or "아니야"), "ts": time.time(), "intent": "cancel"}
            self._mqtt_publish(MQTT_TOPIC, json.dumps(payload, ensure_ascii=False))
        except Exception as e:
            self.get_logger().warn(f"취소 퍼블리시 실패: {e}")
        self._suppress_for(CHIME_END["ms"] / 1000.0 + CHIME_REFRACTORY_SEC)
        self._chime_end()
        self.state = 'IDLE'
        if PUBLISH_LLM_ACTION:
            self._mqtt_publish(LLM_ACTION_TOPIC, json.dumps({"action": "stop"}, ensure_ascii=False))

    # ===== 전사 =====
    def _quick_transcribe(self, int16_pcm: np.ndarray, mode: str = 'generic') -> str:
        try:
            if int16_pcm.size == 0:
                return ""
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as tmp:
                sf.write(tmp.name, int16_pcm, self.fs, format='wav', subtype='PCM_16')
                tmp.flush()
                with open(tmp.name, 'rb') as f:
                    kwargs = {"model": "whisper-1", "file": f, "language": "ko"}
                    if mode == 'wake':
                        kwargs["prompt"] = "로키, 헤이 로키, 로키야"
                    resp = self.client.audio.transcriptions.create(**kwargs)
            text = getattr(resp, 'text', None) or (resp.get('text') if isinstance(resp, dict) else "")
            return (text or "").strip()
        except Exception as e:
            self.get_logger().warn(f"전사 실패 ({mode}): {e}")
            return ""

    def _trigger_listen(self):
        self.state = 'BUSY'
        self._suppress_for(CHIME_START["ms"] / 1000.0 + CHIME_REFRACTORY_SEC)
        self._chime_start()
        if PUBLISH_LLM_ACTION:
            self._mqtt_publish(LLM_ACTION_TOPIC, json.dumps({"action": "start"}, ensure_ascii=False))
        with self._buf_lock:
            self._frames.clear()
        self.is_recording = True
        self.state = 'LISTENING'
        self._silence_accum = 0.0
        self._utter_time = 0.0
        self._has_voice = False
        self.get_logger().info("웨이크워드 감지 → LISTENING 진입")

    def _finish_and_transcribe(self):
        self.is_recording = False
        with self._buf_lock:
            if len(self._frames) == 0:
                self.get_logger().warn("녹음된 프레임이 없습니다")
                self.state = 'IDLE'
                return
            audio_np = np.concatenate(self._frames, axis=0)
            self._frames.clear()

        duration_sec = audio_np.shape[0] / float(self.fs)
        self.get_logger().info(f"자동 종료 — 길이: {duration_sec:.2f}s, 전사 요청 중...")

        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as tmp:
                sf.write(tmp.name, audio_np, self.fs, format='wav', subtype='PCM_16')
                tmp.flush()
                with open(tmp.name, 'rb') as f:
                    resp = self.client.audio.transcriptions.create(
                        model="whisper-1", file=f, language="ko"
                    )
            text = getattr(resp, 'text', None) or (resp.get('text') if isinstance(resp, dict) else "")
            text = (text or "").strip()
            if text:
                payload = {"text": text, "ts": time.time()}
                self._mqtt_publish(MQTT_TOPIC, json.dumps(payload, ensure_ascii=False))
                if self.enable_ros_pub:
                    msg = String()
                    msg.data = text
                    self.pub.publish(msg)
                self.get_logger().info(f"Published MQTT '{MQTT_TOPIC}': {payload}")
            else:
                self.get_logger().info("전사 결과가 비어 있습니다.")
        except Exception as e:
            self.get_logger().error(f"STT 처리 오류: {e}")
        finally:
            refractory_time = CHIME_END["ms"] / 1000.0 + CHIME_REFRACTORY_SEC
            if ENABLE_TTS_REFRACTORY:
                refractory_time += TTS_REFRACTORY_SEC
                self.get_logger().info(f"TTS 응답 대기 중... {TTS_REFRACTORY_SEC}초간 웨이크워드 인식 억제")
            self._suppress_for(refractory_time)
            self._chime_end()
            self.state = 'IDLE'
            if PUBLISH_LLM_ACTION:
                self._mqtt_publish(LLM_ACTION_TOPIC, json.dumps({"action": "stop"}, ensure_ascii=False))

    # ===== 개인 KWS + 화자게이트 =====
    def _personal_wake_detect(self):
        """
        항상 (ok, prob, th) 형태로 반환:
        ok  : bool  (웨이크 통과 여부)
        prob: float (KWS 확률)
        th  : float (당시 사용한 임계값; 동적 존재 시 그 값)
        """
        # 필수 리소스 체크
        if (self._kws_sess is None or self._kws_thresh is None or
            self._spk_model is None or self._spk_centroid is None or self._spk_thresh is None):
            return (False, 0.0, float(self._kws_thresh or 0.0))

        # 최근 화자 실패 쿨다운
        now = time.time()
        if now - getattr(self, "_last_spk_fail_at", 0.0) < FAILED_SPK_COOLDOWN:
            return (False, 0.0, float(self._dyn_thresh or self._kws_thresh))

        # 링버퍼 확보
        ring = np.array(self._ring, dtype=np.int16)
        clip_sec = float(self._kws_cfg.get('clip_sec', 1.28))
        need = int(self.fs * clip_sec)
        if ring.size < need:
            return (False, 0.0, float(self._dyn_thresh or self._kws_thresh))

        # RMS 게이트 (작은 소음 컷)
        recent_rms = self._rms_int16(ring[-int(self.fs * 0.3):])
        if recent_rms < max(WAKE_RMS_THRESHOLD * 0.6, ENERGY_SILENCE_RMS * 1.8):
            return (False, 0.0, float(self._dyn_thresh or self._kws_thresh))

        # === KWS 확률 ===
        seg = ring[-need:]
        m = self._logmel_from_int16(seg, clip_sec=clip_sec)
        prob = self._kws_forward_prob(m)

        # 동적 임계
        base_th = float(self._kws_thresh)
        dyn_th = float(self._dyn_thresh) if self._dyn_thresh is not None else base_th
        th = max(base_th, dyn_th)

        # ---- 소프트 바이패스: prob가 바닥선 이상이면 화자검증 진입 허용 ----
        force_enter = (prob >= SPK_FORCE_FLOOR) or (prob >= max(0.0, th - SPK_FORCE_DELTA))

        # 주변 평균 대비 마진 가드 (완화)
        ambient_ok = True
        if self._ambient_stats is not None:
            guard = self._ambient_stats["mean"] + max(KWS_MIN_MARGIN, 1.2 * self._ambient_stats["std"])  # (1.5 → 1.2)
            if prob < guard and not force_enter:
                return (False, prob, th)

        # 최근 3회 중 KWS_CONSEC_PASS 회 통과 (완화)
        self._prob_hist.append(prob)
        passes = sum(1 for p in self._prob_hist if p >= th)
        if (passes < KWS_CONSEC_PASS) and not force_enter:
            return (False, prob, th)

        # === 화자 검증 ===
        import torch
        y = (seg.astype(np.float32) / 32768.0)
        target_len = int(self.fs * SPK_WIN_SEC)
        y = self._pad_or_trim(y, target_len)

        sims = []
        offsets = np.linspace(-SPK_JITTER_SEC, SPK_JITTER_SEC, SPK_MULTI_CROP)
        for off in offsets:
            shift = int(off * self.fs)
            if shift >= 0:
                y_off = np.pad(y, (shift, 0))[:target_len]
            else:
                # 음수 shift 처리: 앞쪽을 0으로 채우고 뒤를 자름
                pad_len = -shift
                y_off = np.pad(y, (0, pad_len))[-(target_len + pad_len):-pad_len] if pad_len > 0 else y[:target_len]
            with torch.no_grad():
                wav_t = torch.from_numpy(y_off).float().unsqueeze(0)
                e = self._spk_model.encode_batch(wav_t).squeeze(0).squeeze(0).cpu().numpy().astype(np.float32)
            e = e / (np.linalg.norm(e) + 1e-9)
            sim = float(np.dot(e, self._spk_centroid) / (np.linalg.norm(self._spk_centroid) + 1e-9))
            sims.append(sim)

        sim_std = float(np.std(sims))

        # K-of-N (3중 2) + std 완화
        K = 2
        margin = SPK_MARGIN
        per_crop_pass = [(s >= self._spk_thresh + margin) for s in sims]
        num_pass = sum(per_crop_pass)
        std_ok = (sim_std <= max(0.06, SPK_MAX_STD))

        if num_pass >= K and std_ok:
            self.get_logger().info(
                f"화자 통과(K-of-N): sims={['%.3f'%s for s in sims]} | "
                f"pass={num_pass}/{len(sims)} th={self._spk_thresh:.3f}+{margin:.3f} std={sim_std:.3f}"
            )
            return (True, prob, th)
        else:
            self.get_logger().info(
                f"화자 컷(K-of-N): sims={['%.3f'%s for s in sims]} | "
                f"pass={num_pass}/{len(sims)} th={self._spk_thresh:.3f}+{margin:.3f} std={sim_std:.3f}"
            )
            self._last_spk_fail_at = time.time()
            return (False, prob, th)


    # ===== 수동 컨트롤 =====
    def start_recording(self):
        if self.is_recording:
            self.get_logger().info("이미 녹음 중입니다.")
            return
        with self._buf_lock:
            self._frames.clear()
        self.state = 'BUSY'
        self._silence_accum = 0.0
        self._utter_time = 0.0
        self._has_voice = False
        self._suppress_for(CHIME_START["ms"] / 1000.0 + CHIME_REFRACTORY_SEC)
        self._chime_start()
        self.is_recording = True
        self.state = 'LISTENING'
        self.get_logger().info("녹음 시작 — 말하세요... (s: 정지)")

    def stop_and_transcribe(self):
        if not self.is_recording and self.state != 'LISTENING':
            self.get_logger().info("현재 녹음 중이 아닙니다.")
            return
        self.state = 'BUSY'
        threading.Thread(target=self._finish_and_transcribe, daemon=True).start()

    def _keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().warn("터미널이 아닌 환경에서는 키보드 입력을 사용할 수 없습니다.")
            return
        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            self.get_logger().info("키보드 컨트롤:")
            self.get_logger().info("  'r' = 녹음 시작, 's' = 녹음 정지")
            self.get_logger().info("  '+/-' = TTS 억제시간 조정")
            self.get_logger().info("  'w/W' = 웨이크 RMS 임계 조정 (w:민감↑, W:민감↓)")
            self.get_logger().info("  'd'   = 현재 RMS/상태 표시, 'q' = 종료")
            while self._running and rclpy.ok():
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        ch = sys.stdin.read(1).lower()
                        if ch == 'r':
                            self.start_recording()
                        elif ch == 's':
                            threading.Thread(target=self.stop_and_transcribe, daemon=True).start()
                        elif ch == 'q':
                            self.get_logger().info("종료 요청")
                            self._running = False
                            break
                        elif ch == '+':
                            global TTS_REFRACTORY_SEC
                            TTS_REFRACTORY_SEC = min(10.0, TTS_REFRACTORY_SEC + 1.0)
                            self.get_logger().info(f"TTS 억제 시간: {TTS_REFRACTORY_SEC}초")
                        elif ch == '-':
                            TTS_REFRACTORY_SEC = max(0.0, TTS_REFRACTORY_SEC - 1.0)
                            self.get_logger().info(f"TTS 억제 시간: {TTS_REFRACTORY_SEC}초")
                        elif ch == 'w':
                            global WAKE_RMS_THRESHOLD
                            WAKE_RMS_THRESHOLD = max(200.0, WAKE_RMS_THRESHOLD - 100.0)
                            self.get_logger().info(f"웨이크 RMS 임계값: {WAKE_RMS_THRESHOLD}")
                        elif ch == 'w'.upper():
                            WAKE_RMS_THRESHOLD = min(4000.0, WAKE_RMS_THRESHOLD + 100.0)
                            self.get_logger().info(f"웨이크 RMS 임계값: {WAKE_RMS_THRESHOLD}")
                        elif ch == '[':
                            # 화자 임계 ↓ 0.02
                            if self._spk_thresh is not None:
                                self._spk_thresh = max(0.30, float(self._spk_thresh) - 0.02)
                                self.get_logger().info(f"spk_thresh ↓ → {self._spk_thresh:.3f}")
                        elif ch == ']':
                            # 화자 임계 ↑ 0.02
                            if self._spk_thresh is not None:
                                self._spk_thresh = min(0.95, float(self._spk_thresh) + 0.02)
                                self.get_logger().info(f"spk_thresh ↑ → {self._spk_thresh:.3f}")
                        elif ch == '{':  # margin down
                            global SPK_MARGIN
                            SPK_MARGIN = max(0.0, SPK_MARGIN - 0.005)
                            self.get_logger().info(f"SPK_MARGIN ↓ → {SPK_MARGIN:.3f}")
                        elif ch == '}':  # margin up
                            SPK_MARGIN = min(0.05, SPK_MARGIN + 0.005)
                            self.get_logger().info(f"SPK_MARGIN ↑ → {SPK_MARGIN:.3f}")
                        elif ch == '(':
                            global SPK_MAX_STD
                            SPK_MAX_STD = max(0.01, SPK_MAX_STD - 0.005)
                            self.get_logger().info(f"SPK_MAX_STD ↓ → {SPK_MAX_STD:.3f}")
                        elif ch == ')':
                            SPK_MAX_STD = min(0.20, SPK_MAX_STD + 0.005)
                            self.get_logger().info(f"SPK_MAX_STD ↑ → {SPK_MAX_STD:.3f}")

                        elif ch == '1':
                            # 연속통과 요구 낮춤 (최저 1)
                            global KWS_CONSEC_PASS
                            KWS_CONSEC_PASS = max(1, KWS_CONSEC_PASS - 1)
                            self.get_logger().info(f"KWS_CONSEC_PASS ↓ → {KWS_CONSEC_PASS}")
                        elif ch == '2':
                            KWS_CONSEC_PASS = min(3, KWS_CONSEC_PASS + 1)
                            self.get_logger().info(f"KWS_CONSEC_PASS ↑ → {KWS_CONSEC_PASS}")
                        elif ch == ',':
                            # SPK_FORCE_FLOOR 낮추기
                            global SPK_FORCE_FLOOR
                            SPK_FORCE_FLOOR = max(0.40, SPK_FORCE_FLOOR - 0.02)
                            self.get_logger().info(f"SPK_FORCE_FLOOR ↓ → {SPK_FORCE_FLOOR:.2f}")
                        elif ch == '.':
                            SPK_FORCE_FLOOR = min(0.95, SPK_FORCE_FLOOR + 0.02)
                            self.get_logger().info(f"SPK_FORCE_FLOOR ↑ → {SPK_FORCE_FLOOR:.2f}")
                        elif ch == ';':
                            # SPK_FORCE_DELTA( dyn_th에서 허용 여유 ) 확대
                            global SPK_FORCE_DELTA
                            SPK_FORCE_DELTA = min(0.20, SPK_FORCE_DELTA + 0.01)
                            self.get_logger().info(f"SPK_FORCE_DELTA ↑ → {SPK_FORCE_DELTA:.2f}")
                        elif ch == '\'':
                            SPK_FORCE_DELTA = max(0.00, SPK_FORCE_DELTA - 0.01)
                            self.get_logger().info(f"SPK_FORCE_DELTA ↓ → {SPK_FORCE_DELTA:.2f}")

                        elif ch == 'd':
                            th_log = self._dyn_thresh if self._dyn_thresh is not None else self._kws_thresh
                            self.get_logger().info(
                                f"RMS:{self._idle_last_rms:.1f} wakeRMS:{WAKE_RMS_THRESHOLD} "
                                f"state:{self.state} KWS_th:{(self._dyn_thresh or self._kws_thresh):.3f} "
                                f"spk_th:{(self._spk_thresh or -1):.3f} margin:{SPK_MARGIN:.3f} std_max:{SPK_MAX_STD:.3f}"
                            )
                except Exception as e:
                    self.get_logger().warn(f"키보드 입력 처리 오류: {e}")
                    break
        except Exception as e:
            self.get_logger().error(f"키보드 루프 오류: {e}")
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
            except Exception:
                pass

    def destroy_node(self):
        self.get_logger().info("STT 노드 종료 중...")
        self._running = False
        try:
            if hasattr(self, 'stream') and self.stream:
                self.stream.stop()
                self.stream.close()
                self.get_logger().info("오디오 스트림 종료")
        except Exception as e:
            self.get_logger().warn(f"오디오 스트림 종료 오류: {e}")
        try:
            if hasattr(self, 'mqtt_client'):
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                self.get_logger().info("MQTT 연결 종료")
        except Exception as e:
            self.get_logger().warn(f"MQTT 종료 오류: {e}")
        super().destroy_node()


def main(args=None):
    startup_delay = float(os.getenv('STT_STARTUP_DELAY', '2.0'))
    if startup_delay > 0:
        print(f"STT 노드 시작 대기 중... ({startup_delay}초)")
        time.sleep(startup_delay)

    rclpy.init(args=args)
    node = None
    try:
        node = STTManualMQTTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n사용자 중단 요청")
    except Exception as e:
        print(f"노드 실행 오류: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
