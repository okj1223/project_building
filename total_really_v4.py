#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, math, threading, collections, json, uuid, hashlib
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ===== 판정 발행 정책 =====
VERDICT_PUBLISH_MODE = "once"   # "once" | "on_change" | "always"

# ===== YOLO / RealSense 설정 =====
MODEL_PATH_MAIN  = r"/home/rokey/ros2_ws/runs/train/yolo11_back/weights/last.pt"   # 평소 추론
MODEL_PATH_AUTO1 = r"/home/rokey/ros2_ws/runs/train/yolo11_auto1/weights/last.pt"  # 1차 센터링
MODEL_PATH_AUTO2 = r"/home/rokey/ros2_ws/runs/train/yolo11_0b/weights/last.pt"     # 2차 센터링/판정
BASELINE_IMAGE   = r"/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/rokey/rokey/basic/frame.png"
CONF_THRESHOLD   = 0.30
IMG_SIZE         = 640
ANGLE_LIMIT      = 45.0
ALIGN_DEPTH_TO_COLOR = True

# ===== (옵션) MQTT 전송 설정 =====
MQTT_ENABLE   = True
MQTT_BROKER   = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT     = 8883
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"
MQTT_TOPIC_A  = "route/A"
MQTT_TOPIC_B  = "route/B"
MQTT_TOPIC_QC = "qc_result"        # 우리가 퍼블리시하는 PASS/FAIL (문자열 "pass"/"fail")
MQTT_TLS      = True
MQTT_TOPIC_QC_ARDUINO = "qc_result_arduino"

# ---- 외부 QC 결과 구독용(요청사항): '/qc_result' ----
MQTT_QC_SUB_TOPIC = "/qc_result"   # 구독 토픽

CLIENT_ID_QC  = f"rokey-qc-{uuid.uuid4().hex[:8]}"

if MQTT_ENABLE:
    import ssl
    import paho.mqtt.client as mqtt
    def build_mqtt():
        c = mqtt.Client(client_id=CLIENT_ID_QC, protocol=mqtt.MQTTv311)
        if MQTT_TLS:
            c.tls_set(certfile=None, keyfile=None,
                      cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS)
            c.tls_insecure_set(False)
        c.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        c.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
        c.loop_start()
        return c
    _MQTT = build_mqtt()
else:
    _MQTT = None

def _mqtt_pub(topic, payload, tag="[MQTT]"):
    """payload가 문자열이면 그대로, dict면 JSON 직렬화해서 발행"""
    msg = payload if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False)
    if _MQTT:
        _MQTT.publish(topic, msg, qos=1)
    print(f"{tag} {topic}: {msg}")

def send_A(p):  _mqtt_pub(MQTT_TOPIC_A, p, tag="[SEND-A]")
def send_B(p):  _mqtt_pub(MQTT_TOPIC_B, p, tag="[SEND-B]")
def send_QC(p): _mqtt_pub(MQTT_TOPIC_QC, p, tag="[QC]")  # p: "pass" or "fail"
def send_QC_ARD(val: str):
    # val: "PASS" | "FAIL" | "OK" | "NG" 등
    _mqtt_pub(MQTT_TOPIC_QC_ARDUINO, val, tag="[ARD-QC]")

# ===== 로봇/그리퍼/ROS =====
import rclpy
import DR_init
from rclpy.node import Node
from .onrobot import RG

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
FORCE_VALUE = 4   # N, Z축 외력 트리거

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===== MQTT (명령 구독) =====
MQTT_HOST  = MQTT_BROKER
MQTT_PORT  = MQTT_PORT
MQTT_USER  = MQTT_USERNAME
MQTT_PASS  = MQTT_PASSWORD
MQTT_TOPIC = "check"  # {"checkboard":1} + (optional) spec JSON

import ssl as _ssl_for_cmd
import paho.mqtt.client as _mqtt_for_cmd

flag = 0
_flag_lock = threading.Lock()

# ---- 스펙 캐시 상태 ----
_spec_lock = threading.Lock()
_spec_cached = {
    "ready": False,
    "raw": None,
    "components": None,
    "edges": None,
    "type_to_refs": None,
    "job_id": None,
    "spec_hash": None,
    "ts": None,
}
def _hash_spec(raw_obj: dict) -> str | None:
    try:
        b = json.dumps(raw_obj, sort_keys=True, separators=(",",":")).encode("utf-8")
        return hashlib.sha256(b).hexdigest()[:16]
    except Exception:
        return None

def _make_edges_set(edges_list):
    return set(tuple(sorted(pair)) for pair in edges_list)

def _build_type_to_refs(components):
    d = {}
    for c in components:
        t = str(c["type"]).lower()
        d.setdefault(t, []).append(str(c["ref"]))
    return d

def _try_cache_spec_from_payload(payload: dict) -> bool:
    if not isinstance(payload, dict): return False
    if "components" not in payload or "edges" not in payload: return False
    comps_in = payload.get("components"); edges_in = payload.get("edges")
    if not isinstance(comps_in, list) or not isinstance(edges_in, list): return False
    comps=[]; refs=set()
    for c in comps_in:
        if not isinstance(c, dict): continue
        ref=c.get("ref"); typ=c.get("type")
        if not ref or not typ: continue
        comps.append({"ref":str(ref), "type":str(typ).lower()}); refs.add(str(ref))
    edges_clean=[]
    for e in edges_in:
        if (isinstance(e,(list,tuple)) and len(e)==2
            and str(e[0]) in refs and str(e[1]) in refs and str(e[0])!=str(e[1])):
            edges_clean.append([str(e[0]), str(e[1])])
    if not comps or not edges_clean: return False

    raw = {"schema": payload.get("schema","v1"), "components": comps_in, "edges": edges_in}
    with _spec_lock:
        _spec_cached.update({
            "ready": True,
            "raw": raw,
            "components": comps,
            "edges": _make_edges_set(edges_clean),
            "type_to_refs": _build_type_to_refs(comps),
            "job_id": payload.get("job_id"),
            "spec_hash": _hash_spec(raw),
            "ts": time.strftime("%Y%m%d_%H%M%S"),
        })
    print(f"[MQTT] spec cached: {len(comps)} comps, {len(edges_clean)} edges, hash={_spec_cached['spec_hash']}")
    return True

class MqttCommand:
    def __init__(self, host, port, user, password, sub_topic, pub_topic=None, use_tls=True):
        self.host = host; self.port = port
        self.user = user; self.password = password
        self.sub_topic = sub_topic; self.pub_topic = pub_topic
        self.use_tls = use_tls
        self._client = _mqtt_for_cmd.Client(client_id="rokey-checkboard-subscriber", protocol=_mqtt_for_cmd.MQTTv311)
        if self.use_tls:
            self._client.tls_set(certfile=None, keyfile=None,
                                 cert_reqs=_ssl_for_cmd.CERT_REQUIRED, tls_version=_ssl_for_cmd.PROTOCOL_TLS)
            self._client.tls_insecure_set(False)
        self._client.username_pw_set(self.user, self.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] connected rc={rc}")
        client.subscribe(self.sub_topic, qos=1)
        print(f"[MQTT] subscribed: {self.sub_topic}")

    def _on_message(self, client, userdata, msg):
        global flag
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            print(f"[MQTT] recv {msg.topic}: {payload}")
        except Exception as e:
            print(f"[MQTT] bad json: {e} ({msg.payload[:120]!r})")
            return

        # 1) 플래그
        if isinstance(payload, dict) and int(payload.get("checkboard", 0)) == 1:
            with _flag_lock:
                flag = 1
            print("[MQTT] checkboard=1 → flag=1")

        # 2) 스펙 (있으면 캐시)
        _try_cache_spec_from_payload(payload)

    def start(self):
        self._client.connect(self.host, self.port, keepalive=30)
        self._client.loop_start()

    def stop(self):
        try: self._client.loop_stop()
        except: pass
        try: self._client.disconnect()
        except: pass

# ===== Hand-Eye =====
from scipy.spatial.transform import Rotation
T_G2C_PATH = "/home/rokey/ros2_ws/Tutorial/Calibration_Tutorial/T_gripper2camera.npy"

# ===== XY 바이어스 (mm) =====
BIAS_X_MM = -5
BIAS_Y_MM = +5

# ===== Place pose (기준 포즈 + 스택 ΔZ) =====
PLACE_A_BASE = [237.0, 256.0, 20.0, 0.0, -180.0, 0.0]
PLACE_B_BASE = [237.0,  86.0, 20.0, 0.0, -180.0, 0.0]
SAFE_UP_DZ   = 40.0
STACK_DZ     = 17.0  # 한 번 놓을 때마다 Z 증가(mm)

def PLACE_A_POSX(count: int):
    pos = PLACE_A_BASE.copy(); pos[2] += STACK_DZ * count; return pos

def PLACE_B_POSX(count: int):
    pos = PLACE_B_BASE.copy(); pos[2] += STACK_DZ * count; return pos

# ===== 스펙 JSON 경로 & 결과 저장 경로 =====
SPEC_JSON_PATH = "/home/rokey/ros2_ws/spec.json"
JUDGE_OUT_DIR  = "/home/rokey/ros2_ws/judge_out"

# ===== 유틸 =====
def clamp90(a: float) -> float:
    if a < -90: a += 180
    if a >  90: a -= 180
    return float(a)

def clamp_delta(a: float, limit_abs: float = ANGLE_LIMIT) -> float:
    return float(max(-limit_abs, min(limit_abs, a)))

def estimate_angle_minarearect(roi_bgr) -> float | None:
    if roi_bgr is None or roi_bgr.size == 0: return None
    g = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    edges = cv2.Canny(g, 60, 180)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    if cv2.contourArea(cnts[0]) < 200: return None
    rect = cv2.minAreaRect(cnts[0]); (w, h) = rect[1]; ang = rect[2]
    if w < h: ang += 90.0
    return clamp90(float(ang))

def names_dict(names):
    return names if isinstance(names, dict) else {i: n for i, n in enumerate(names)}

def select_board_bbox(result, names_map):
    if result.boxes is None or len(result.boxes) == 0: return None
    xyxy = result.boxes.xyxy.cpu().numpy().astype(int)
    conf = result.boxes.conf.cpu().numpy()
    cls  = result.boxes.cls.cpu().numpy().astype(int)
    board_ids = [i for i, n in names_map.items() if str(n).lower() == "board"]
    if board_ids:
        bid = board_ids[0]
        cand = [(i, c) for i, (cid, c) in enumerate(zip(cls, conf)) if cid == bid]
        if cand:
            idx = max(cand, key=lambda t: t[1])[0]
            return tuple(xyxy[idx])
    areas = (xyxy[:,2]-xyxy[:,0]) * (xyxy[:,3]-xyxy[:,1])
    idx = int(np.argmax(areas))
    return tuple(xyxy[idx])

def crop(img, bbox):
    x1, y1, x2, y2 = bbox
    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = max(x1c+1, x2), max(y1c+1, y2)
    return img[y1c:y2c, x1c:x2c]

def draw_all_detections(out_img, result, names_map):
    if result.boxes is None or len(result.boxes) == 0: return
    xyxy = result.boxes.xyxy.cpu().numpy().astype(int)
    conf = result.boxes.conf.cpu().numpy()
    cls  = result.boxes.cls.cpu().numpy().astype(int)
    for (x1,y1,x2,y2), cf, cidx in zip(xyxy, conf, cls):
        name = names_map.get(cidx, str(cidx))
        label = f"{name} {cf:.2f}"
        cv2.rectangle(out_img, (x1, y1), (x2, y2), (0,255,255), 2)
        cv2.putText(out_img, label, (x1, max(20, y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)

# ===== 라벨색 시각화 =====
def _color_from_name(name: str):
    h = abs(hash(str(name)))
    r = 60 + (h      % 196); g = 60 + ((h>>8) % 196); b = 60 + ((h>>16)% 196)
    return (int(b), int(g), int(r))

def draw_detections_by_label_colored(out_img, result, names_map):
    if result.boxes is None or len(result.boxes) == 0: return
    xyxy = result.boxes.xyxy.cpu().numpy().astype(int)
    conf = result.boxes.conf.cpu().numpy()
    cls  = result.boxes.cls.cpu().numpy().astype(int)
    for (x1,y1,x2,y2), cf, cidx in zip(xyxy, conf, cls):
        name = names_map.get(cidx, str(cidx))
        color = _color_from_name(name)
        cv2.rectangle(out_img, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {cf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        ylab = max(20, y1-7)
        cv2.rectangle(out_img, (x1, ylab-th-4), (x1+tw+6, ylab+2), color, -1)
        cv2.putText(out_img, label, (x1+3, ylab-2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 2, cv2.LINE_AA)

# ===== 회로 판정 유틸 =====
CABLE_CLASS_NAME = "cable"
TWO_PIN_TYPES = {"resistor","capacitor","switch","led","transistor"}

def _yolo_detections_from_result(r, conf_thres: float, names) -> list[tuple[str,float,tuple[int,int,int,int]]]:
    dets=[]; boxes=r.boxes
    if boxes is None or len(boxes)==0: return dets
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls  = boxes.cls.cpu().numpy().astype(int)
    for i in range(len(boxes)):
        c=float(conf[i]); 
        if c<conf_thres: continue
        cid=int(cls[i])
        name = names[cid] if isinstance(names,(list,tuple)) else (names.get(cid,str(cid)))
        x1,y1,x2,y2 = map(int, xyxy[i].tolist())
        dets.append((str(name).lower(), c, (x1,y1,x2,y2)))
    return dets

def _estimate_two_pins_from_box(x1,y1,x2,y2):
    w,h=(x2-x1),(y2-y1)
    if w>=h: cy=int((y1+y2)/2); return (x1,cy),(x2,cy)
    else:    cx=int((x1+x2)/2); return (cx,y1),(cx,y2)

def _cable_endpoints_from_crop(crop_bgr: np.ndarray):
    h,w=crop_bgr.shape[:2]
    gray=cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    gray=cv2.bilateralFilter(gray,5,25,25)
    gray=cv2.equalizeHist(gray)
    edges=cv2.Canny(gray,40,120)
    edges=cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),1)
    skel=np.zeros_like(edges)
    element=cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    tmp=edges.copy()
    while True:
        opened=cv2.morphologyEx(tmp, cv2.MORPH_OPEN, element)
        sub=cv2.subtract(tmp, opened)
        eroded=cv2.erode(tmp, element)
        skel=cv2.bitwise_or(skel, sub); tmp=eroded
        if cv2.countNonZero(tmp)==0: break
    ys,xs=np.where(skel>0)
    if xs.size>=20:
        pts=np.column_stack((xs,ys)).astype(np.float32)
        vx,vy,x0,y0=cv2.fitLine(pts, cv2.DIST_L2,0,0.01,0.01).flatten()
        v=np.array([vx,vy],dtype=np.float32); p0=np.array([x0,y0],dtype=np.float32)
        t=(pts-p0)@v
        p1=p0+v*float(t.min()); p2=p0+v*float(t.max())
        p1=(int(np.clip(p1[0],0,w-1)), int(np.clip(p1[1],0,h-1)))
        p2=(int(np.clip(p2[0],0,w-1)), int(np.clip(p2[1],0,h-1)))
        return [p1,p2]
    return list(_estimate_two_pins_from_box(0,0,w,h))

def _detect_components_cables_from_r(r, frame_bgr, conf, cable_class_name=CABLE_CLASS_NAME):
    names=r.names; dets=_yolo_detections_from_result(r, conf, names)
    H,W=frame_bgr.shape[:2]
    components=[]; cables=[]; type_counts={}
    for (t,score,(x1,y1,x2,y2)) in dets:
        if t==cable_class_name:
            x1c,y1c,x2c,y2c=max(0,x1),max(0,y1),min(W,x2),min(H,y2)
            crop=frame_bgr[y1c:y2c, x1c:x2c]
            ends_local=_cable_endpoints_from_crop(crop)
            ends=[(x1c+ex, y1c+ey) for (ex,ey) in ends_local]
            type_counts[t]=type_counts.get(t,0)+1
            cables.append({"ref":f"CAB{type_counts[t]}", "ends":ends, "bbox":[x1,y1,x2,y2]})
        else:
            type_counts[t]=type_counts.get(t,0)+1
            components.append({"ref":f"{t.upper()}{type_counts[t]}", "type":t, "bbox":[x1,y1,x2,y2]})
    return components, cables

def _map_refs_by_type(spec_type_to_refs, detected_components):
    type_to_detected={}
    for c in detected_components:
        x1,y1,x2,y2=c["bbox"]; cx=(x1+x2)/2.0
        type_to_detected.setdefault(c["type"].lower(), []).append((cx, c["ref"]))
    for t in type_to_detected:
        type_to_detected[t].sort(key=lambda x:x[0])
    det2spec={}
    for t, spec_refs in (spec_type_to_refs or {}).items():
        det_list=type_to_detected.get(t, [])
        if len(det_list)!=len(spec_refs): continue
        for (_,det_ref), spec_ref in zip(det_list, spec_refs):
            det2spec[det_ref]=spec_ref
    return det2spec

def _build_pins_by_ref(components):
    pins={}
    for c in components:
        x1,y1,x2,y2=c["bbox"]; t=c["type"].lower()
        if t in TWO_PIN_TYPES:
            p1,p2=_estimate_two_pins_from_box(x1,y1,x2,y2); pins[c["ref"]]=[p1,p2]
        else:
            cx,cy=int((x1+x2)/2), int((y1+y2)/2); pins[c["ref"]]=[(cx,cy)]
    return pins

def _nearest_component(pt, components):
    x,y=pt; best=(None,float("inf"))
    for c in components:
        x1,y1,x2,y2=c["bbox"]
        dx=max(x1-x,0, x-x2); dy=max(y1-y,0, y-y2)
        d=math.hypot(dx,dy)
        if d<best[1]: best=(c["ref"], d)
    return best

def _measure_edges(components, cables, r_snap=18.0):
    pins_by_ref=_build_pins_by_ref(components)
    edges=set(); details=[]
    def nearest_pin(pt):
        best=(None,None,float("inf"))
        for ref, pin_list in pins_by_ref.items():
            for i,(px,py) in enumerate(pin_list):
                d=math.hypot(px-pt[0], py-pt[1])
                if d<best[2]: best=(ref,i,d)
        return best
    for cab in cables:
        if len(cab["ends"])<2: continue
        a,b=cab["ends"][0], cab["ends"][1]
        Aref,Api,Ad=nearest_pin(a)
        Bref,Bpi,Bd=nearest_pin(b)
        if Aref is None or Ad>r_snap: Aref,Ad=_nearest_component(a,components); Api=None
        if Bref is None or Bd>r_snap: Bref,Bd=_nearest_component(b,components); Bpi=None
        info={"cable":cab["ref"],
              "A":{"pt":a,"to_ref":Aref,"pin":Api,"dist":round(Ad,2) if Ad is not None else None},
              "B":{"pt":b,"to_ref":Bref,"pin":Bpi,"dist":round(Bd,2) if Bd is not None else None},
              "issues":[]}
        if Aref is None or Ad is None: info["issues"].append("OPEN_A")
        if Bref is None or Bd is None: info["issues"].append("OPEN_B")
        if Aref and Bref and Aref==Bref: info["issues"].append("WRONG_SAME_COMPONENT")
        if Aref and Bref and Aref!=Bref: edges.add(tuple(sorted([Aref,Bref])))
        details.append(info)
    return edges, details

def _compare(spec_edges:set, measured:set):
    missing=sorted(list(spec_edges-measured))
    extra  =sorted(list(measured-spec_edges))
    ok     =sorted(list(spec_edges & measured))
    verdict="PASS" if (not missing and not extra) else "FAIL"
    return verdict, ok, missing, extra

def _draw_cj_overlay(vis, components, cables, measured_edges_spec, det2spec):
    # 케이블 (뒤 레이어)
    for cab in cables:
        if len(cab["ends"])>=2:
            p1=tuple(map(int,cab["ends"][0])); p2=tuple(map(int,cab["ends"][1]))
            cv2.line(vis,p1,p2,(40,40,40),4,cv2.LINE_AA)
            cv2.line(vis,p1,p2,(245,245,245),2,cv2.LINE_AA)
            for p in (p1,p2):
                cv2.circle(vis,p,6,(0,0,0),-1,cv2.LINE_AA)
                cv2.circle(vis,p,4,(255,255,255),-1,cv2.LINE_AA)
    # bbox + 라벨
    def comp_color(t):
        m={"resistor":(0,255,255),"capacitor":(255,0,0),"switch":(0,255,0),
           "led":(0,0,255),"transistor":(255,0,255)}
        return m.get(t.lower(), (0,255,0))
    for c in components:
        x1,y1,x2,y2=c["bbox"]; det_ref=c["ref"]; spec_ref=det2spec.get(det_ref, det_ref)
        col=comp_color(c["type"])
        cv2.rectangle(vis,(x1,y1),(x2,y2),col,2)
        org=(x1,max(0,y1-8))
        cv2.putText(vis,f"{spec_ref}",org,cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(vis,f"{spec_ref}",org,cv2.FONT_HERSHEY_SIMPLEX,0.6,col,1,cv2.LINE_AA)
    # spec edges (맨 위)
    inv={v:k for k,v in det2spec.items()}
    def center_of(spec_ref):
        det=inv.get(spec_ref, spec_ref)
        for c in components:
            if c["ref"]==det:
                x1,y1,x2,y2=c["bbox"]
                return (int((x1+x2)/2), int((y1+y2)/2))
        return None
    for a,b in measured_edges_spec:
        pa,pb=center_of(a), center_of(b)
        if pa and pb: cv2.line(vis, pa,pb,(255,0,0),2,cv2.LINE_AA)

# ===== RealSense 준비 =====
def wait_realsense_ready(timeout_sec: float = 5.0, poll=0.1) -> bool:
    t0 = time.time(); ctx = rs.context()
    while time.time() - t0 < timeout_sec:
        try:
            if len(ctx.query_devices()) > 0:
                return True
        except: pass
        time.sleep(poll)
    return False

# ===== 좌표계 변환 =====
def posx_to_T_base2gripper(posx_list):
    x, y, z, rx, ry, rz = posx_list
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4); T[:3, :3] = R; T[:3, 3]  = [x, y, z]
    return T

def cam_point_m_to_base_mm(current_posx, T_g2c, X_m, Y_m, Z_m):
    if isinstance(current_posx, tuple): p = list(current_posx[0])
    else: p = list(current_posx)
    T_b2g = posx_to_T_base2gripper(p); T_b2c = T_b2g @ T_g2c
    P_cam = np.array([X_m*1000.0, Y_m*1000.0, Z_m*1000.0, 1.0], dtype=float)
    P_base = T_b2c @ P_cam
    return P_base[:3]

# ====== 힘/순응 제어 헬퍼 ======
def compliance_force_down(target_force=-15, trigger_threshold=FORCE_VALUE, timeout=5.0):
    try:
        from DSR_ROBOT2 import (
            task_compliance_ctrl, set_desired_force, check_force_condition,
            release_force, release_compliance_ctrl, drl_script_stop,
            DR_FC_MOD_REL, DR_AXIS_Z
        )
    except Exception as e:
        print(f"[FORCE] DSR force API import failed: {e}")
        return False

    print(f"[FORCE] start (target={target_force}N, trigger>={trigger_threshold}N, timeout={timeout}s)")
    t0 = time.time()
    ok = False
    try:
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        time.sleep(0.1)
        set_desired_force(fd=[0,0,target_force,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, max=trigger_threshold):
                print("[FORCE] threshold reached")
                from DSR_ROBOT2 import drl_script_stop
                drl_script_stop(0)
                ok = True
                break
            if (time.time() - t0) > timeout:
                print("[FORCE][WARN] timeout")
                break
            time.sleep(0.05)
    except Exception as e:
        print(f"[FORCE][ERR] {e}")
    finally:
        try:
            from DSR_ROBOT2 import release_force, release_compliance_ctrl
            release_force(); time.sleep(0.05); release_compliance_ctrl(); time.sleep(0.05)
        except Exception:
            pass
        print("[FORCE] released")
    return ok

# ===== 그리퍼 래퍼 =====
class GripNode(Node):
    def __init__(self):
        super().__init__("rg2_grip_node")
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        time.sleep(0.3)
    def grip(self):
        self.get_logger().info("Closing gripper...")
        self.gripper.close_gripper(); time.sleep(0.5)
    def re(self):
        self.get_logger().info("Opening gripper...")
        self.gripper.open_gripper(); time.sleep(0.5)

def turngrip(jpos, dgree):
    jpos = list(jpos); jpos[5] = jpos[5] + dgree; return jpos

def topos(current_posx, boardpos_mm):
    pose_list = list(current_posx[0] if isinstance(current_posx, tuple) else current_posx)
    pose_list[0] -= boardpos_mm[0]; pose_list[1] -= boardpos_mm[1]
    return pose_list

# ===== 외부 QC 결과 수신 상태 =====
_qc_lock = threading.Lock()
_qc_last_result = None   # 'OK' 또는 'NG' 또는 None
_qc_last_ts = 0.0

def _normalize_qc_value(v) -> str | None:
    if v is None: return None
    s = str(v).strip().lower()
    if s in {"ok","pass","true","1","good"}:  return "OK"
    if s in {"ng","fail","false","0","bad"}:  return "NG"
    return None

def _set_qc_result(result_str: str):
    global _qc_last_result, _qc_last_ts
    with _qc_lock:
        _qc_last_result = result_str
        _qc_last_ts = time.time()
    print(f"[QC-SUB] QC result set: {result_str}")

def get_qc_result(clear: bool=True) -> str | None:
    global _qc_last_result
    with _qc_lock:
        res = _qc_last_result
        if clear:
            _qc_last_result = None
    return res

# ===== QC 결과 구독기 =====
class MqttQCResultSub:
    def __init__(self, host, port, user, password, sub_topic=MQTT_QC_SUB_TOPIC, use_tls=True):
        self.host=host; self.port=port; self.user=user; self.password=password
        self.sub_topic=sub_topic; self.use_tls=use_tls
        self._client = _mqtt_for_cmd.Client(client_id="rokey-qcresult-subscriber", protocol=_mqtt_for_cmd.MQTTv311)
        if self.use_tls:
            self._client.tls_set(certfile=None, keyfile=None,
                                 cert_reqs=_ssl_for_cmd.CERT_REQUIRED,
                                 tls_version=_ssl_for_cmd.PROTOCOL_TLS)
            self._client.tls_insecure_set(False)
        self._client.username_pw_set(self.user, self.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[QC-SUB] connected rc={rc}")
        client.subscribe(self.sub_topic, qos=1)
        print(f"[QC-SUB] subscribed: {self.sub_topic}")

    def _on_message(self, client, userdata, msg):
        raw = msg.payload
        text = None
        try:
            text = raw.decode("utf-8", errors="ignore")
        except: pass
        result = None
        try:
            obj = json.loads(text) if text is not None else None
            if isinstance(obj, dict):
                val = obj.get("result", obj.get("verdict", obj.get("qc")))
                result = _normalize_qc_value(val)
        except Exception:
            pass
        if result is None and text is not None:
            result = _normalize_qc_value(text)

        if result in {"OK","NG"}:
            _set_qc_result(result)
        else:
            print(f"[QC-SUB][WARN] unrecognized payload on {msg.topic}: {text!r}")

    def start(self):
        self._client.connect(self.host, self.port, keepalive=30)
        self._client.loop_start()

    def stop(self):
        try: self._client.loop_stop()
        except: pass
        try: self._client.disconnect()
        except: pass

# ===== 비전 스레드 =====
class VisionThread:
    def __init__(self, model: YOLO|None, names_map, baseline_angle: float):
        self.model = model
        self.names = names_map or {}
        self.baseline = float(baseline_angle)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color) if ALIGN_DEPTH_TO_COLOR else None
        self.lock = threading.Lock()
        self.running = False

        self.window_name = "vision-live"
        self.window_open = False
        self.force_close = False

        self.last_seen_ts = 0.0
        self.visible_for  = 0.0
        self.angle_hist   = collections.deque(maxlen=15)

        # overlay_mode: normal | encircle | colorlabels | judge
        self.overlay_mode = "normal"

        # judge 텍스트 표시용
        self.last_verdict = None
        self.last_ok = []
        self.last_missing = []
        self.last_extra = []

        self.judge_spec = None

        self.latest = {
            "has_board": False, "bbox": None, "delta_angle": 0.0, "cxcy": None,
            "depth_m": 0.0, "p_cam_m": None, "timestamp": 0.0,
            "classes_present": [], "only_board": False,
            "enc_center": None, "enc_radius": 0.0, "enc_depth_m": 0.0,
            "last_frame": None, "last_result": None
        }

        # judge 1회 저장/발행 게이트
        self._judge_run_id = None
        self._judge_saved_once = False
        self._judge_published_once = False

        # === [CHANGE] judge 안정화용 수집 버퍼 + 단발 PASS 스냅샷 ===
        self._judge_collect_start = 0.0
        self._judge_collect_for   = 1.0
        self._edge_counts         = collections.Counter()
        self._judge_frames        = []
        self._any_pass_frame      = False
        self._pass_frame_snapshot = None

        # 발행 정책 상태
        self._published_once = False
        self._last_sent_verdict = None

    def set_model(self, model: YOLO|None, names_map=None, baseline_angle: float|None=None):
        with self.lock:
            self.model = model
            if names_map is not None: self.names = names_map
            if baseline_angle is not None: self.baseline = float(baseline_angle)
            self.angle_hist.clear()
            self.latest.update({
                "has_board": False, "bbox": None, "delta_angle": 0.0, "cxcy": None,
                "depth_m": 0.0, "p_cam_m": None, "timestamp": time.time(),
                "classes_present": [], "only_board": False,
                "enc_center": None, "enc_radius": 0.0, "enc_depth_m": 0.0,
                "last_frame": None, "last_result": None
            })

    def set_overlay_mode(self, mode: str):
        with self.lock:
            prev = self.overlay_mode
            self.overlay_mode = mode if mode in {"normal","encircle","colorlabels","judge"} else "normal"
            if self.overlay_mode == "judge" and prev != "judge":
                self._judge_run_id = time.time()
                self._judge_saved_once = False
                self._judge_published_once = False
                self._published_once = False
                self._last_sent_verdict = None
                # === [CHANGE] 수집/스냅샷 초기화
                self._judge_collect_start = time.time()
                self._edge_counts.clear()
                self._judge_frames.clear()
                self._any_pass_frame = False
                self._pass_frame_snapshot = None

    def set_judge_spec_from_cache_or_file(self):
        spec_obj = None
        with _spec_lock:
            if _spec_cached["ready"]:
                spec_obj = {
                    "raw": _spec_cached["raw"],
                    "edges": _spec_cached["edges"],
                    "type_to_refs": _spec_cached["type_to_refs"],
                    "job_id": _spec_cached["job_id"],
                    "spec_hash": _spec_cached["spec_hash"]
                }
        if spec_obj is None and os.path.exists(SPEC_JSON_PATH):
            try:
                with open(SPEC_JSON_PATH,"r",encoding="utf-8") as f:
                    payload=json.load(f)
                if _try_cache_spec_from_payload(payload):
                    with _spec_lock:
                        spec_obj = {
                            "raw": _spec_cached["raw"],
                            "edges": _spec_cached["edges"],
                            "type_to_refs": _spec_cached["type_to_refs"],
                            "job_id": _spec_cached["job_id"],
                            "spec_hash": _spec_cached["spec_hash"]
                        }
            except Exception as e:
                print(f"[JUDGE] spec load error: {e}")
        with self.lock:
            self.judge_spec = spec_obj

    def _should_publish(self, verdict):
        if VERDICT_PUBLISH_MODE == "always":
            return True
        if VERDICT_PUBLISH_MODE == "once":
            if not self._published_once:
                self._published_once = True
                return True
            return False
        if VERDICT_PUBLISH_MODE == "on_change":
            if verdict != self._last_sent_verdict:
                self._last_sent_verdict = verdict
                return True
            return False
        return True

    def start(self):
        if not wait_realsense_ready(5.0):
            print("[VISION] WARNING: RealSense not ready, trying start anyway...")
        self.pipeline.start(self.config)
        self.running = True; self.window_open = True; self.force_close = False
        self.th = threading.Thread(target=self._loop, daemon=True); self.th.start()
        print("[VISION] started (camera ON)")

    def request_close(self): self.force_close = True

    def stop(self):
        self.running = False; self.force_close = True
        try: self.th.join(timeout=2.0)
        except: pass
        try: self.pipeline.stop()
        except: pass
        try:
            if self.window_open:
                cv2.destroyWindow(self.window_name); cv2.waitKey(1)
        except: pass
        self.window_open = False
        print("[VISION] stopped (pipeline & window closed)")

    def _loop(self):
        prev_seen=False; start_seen_ts=0.0
        os.makedirs(JUDGE_OUT_DIR, exist_ok=True)

        while self.running and not self.force_close:
            try:
                frames = self.pipeline.wait_for_frames()
                if self.align is not None: frames = self.align.process(frames)
                color_frame = frames.get_color_frame(); depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    if self.force_close: break
                    continue
                color = np.asanyarray(color_frame.get_data())

                if self.model is None:
                    out=color.copy()
                    cv2.putText(out,"YOLO: OFF (camera running)",(10,30),
                                cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2,cv2.LINE_AA)
                    if self.window_open:
                        try:
                            cv2.imshow(self.window_name,out)
                            if (cv2.waitKey(1)&0xFF)==ord('q'):
                                self.force_close=True; break
                        except: self.window_open=False
                    with self.lock:
                        self.latest.update({
                            "has_board": False,"bbox": None,"delta_angle":0.0,
                            "cxcy": None,"depth_m":0.0,"p_cam_m": None,
                            "timestamp": time.time(),"classes_present": [],
                            "only_board": False,"enc_center": None,
                            "enc_radius": 0.0,"enc_depth_m": 0.0,
                            "last_frame": color, "last_result": None
                        })
                    time.sleep(0.01); continue

                # ===== 추론 =====
                r = self.model(color, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)[0]
                bbox = select_board_bbox(r, self.names)
                has_board = bbox is not None

                xyxy_np=None
                if r.boxes is not None and len(r.boxes)>0:
                    xyxy_np = r.boxes.xyxy.cpu().numpy()

                cls_names=[]
                if r.boxes is not None and len(r.boxes)>0:
                    cls_ids = r.boxes.cls.cpu().numpy().astype(int)
                    for cid in cls_ids: cls_names.append(str(self.names.get(cid,cid)).lower())
                only_board = (len(cls_names)>0) and all(n=="board" for n in cls_names)

                delta_angle=0.0; cxcy=None; depth_m=0.0; p_cam_m=None
                if has_board:
                    roi = crop(color, bbox)
                    current_angle = estimate_angle_minarearect(roi) or 0.0
                    delta_angle = clamp_delta(current_angle - self.baseline, ANGLE_LIMIT)
                    x1,y1,x2,y2 = bbox
                    cx=(x1+x2)//2; cy=(y1+y2)//2; cxcy=(cx,cy)
                    # 깊이
                    def _median(depth_frame, x, y, k=1):
                        w,h = depth_frame.get_width(), depth_frame.get_height()
                        xs=range(max(0,x-k), min(w,x+k+1)); ys=range(max(0,y-k), min(h,y+k+1))
                        vals=[]
                        for yy in ys:
                            for xx in xs:
                                d=depth_frame.get_distance(xx,yy)
                                if d>0: vals.append(d)
                        return float(np.median(vals)) if vals else 0.0
                    depth_m = _median(depth_frame, cx, cy, k=1)
                    if depth_m>0:
                        intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
                        X,Y,Z = rs.rs2_deproject_pixel_to_point(intr, [cx,cy], depth_m)
                        p_cam_m=(float(X),float(Y),float(Z))
                    now=time.time()
                    if prev_seen: self.visible_for=now-start_seen_ts
                    else: start_seen_ts=now; self.visible_for=0.0
                    prev_seen=True; self.last_seen_ts=now; self.angle_hist.append(delta_angle)
                else:
                    prev_seen=False; self.visible_for=0.0

                out=color.copy()

                # === [CHANGE] encircle 계산: 비-보드 bbox 중심들의 "가장 조밀한 클러스터" 중심만 사용
                enc_center=None; enc_radius=0.0; enc_depth_m=0.0
                if r.boxes is not None and len(r.boxes)>0:
                    xyxy = r.boxes.xyxy.cpu().numpy()
                    cls  = r.boxes.cls.cpu().numpy().astype(int)
                    # 후보 포인트: board 제외한 bbox 중심
                    pts=[]
                    for (x1,y1,x2,y2), ci in zip(xyxy, cls):
                        name=str(self.names.get(ci,ci)).lower()
                        if name=="board": continue
                        pts.append(((x1+x2)/2.0, (y1+y2)/2.0))
                    if len(pts) >= 3:
                        P=np.array(pts, dtype=np.float32)
                        eps=80.0  # 픽셀 반경 (필요시 조절)
                        best_cnt=-1; best_mask=None
                        for i in range(len(P)):
                            d=np.linalg.norm(P-P[i], axis=1)
                            m=(d<=eps)
                            cnt=int(m.sum())
                            if cnt>best_cnt:
                                best_cnt=cnt; best_mask=m
                        if best_cnt>=3:
                            cluster=P[best_mask]
                            c=cluster.mean(axis=0)
                            enc_center=(int(round(c[0])), int(round(c[1])))
                            # 반경은 클러스터 최외곽까지
                            enc_radius=float(np.max(np.linalg.norm(cluster-c, axis=1)))
                            enc_depth_m = depth_frame.get_distance(enc_center[0], enc_center[1])
                    # (클러스터 실패 시 enc_center는 None 유지 → 세부이동 스킵)

                mode = self.overlay_mode
                if mode=="encircle":
                    # 시각화
                    if enc_center is not None:
                        cv2.circle(out, enc_center, max(6,int(enc_radius)), (0,255,255), 2)
                        cv2.circle(out, enc_center, 4, (0,0,255), -1)
                        cv2.putText(out,f"cluster={enc_center} r={enc_radius:.1f}px Z={enc_depth_m:.3f}m",
                                    (10,28), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2,cv2.LINE_AA)
                    else:
                        cv2.putText(out,"no dense cluster (skip move)",
                                    (10,28), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2,cv2.LINE_AA)

                elif mode=="colorlabels":
                    draw_detections_by_label_colored(out, r, self.names)

                elif mode=="judge":
                    verdict_text = "UNKNOWN"; ok=[]; missing=[]; extra=[]
                    try:
                        # 신뢰도 완화(깜빡임 대응)
                        comps, cabs = _detect_components_cables_from_r(r, color, conf=max(CONF_THRESHOLD,0.35))
                        det2spec={}; measured_edges_spec=set()
                        spec_obj = self.judge_spec
                        if spec_obj is not None and spec_obj.get("edges") and spec_obj.get("type_to_refs"):
                            det2spec = _map_refs_by_type(spec_obj["type_to_refs"], comps)
                            measured_edges_det, _ = _measure_edges(comps, cabs, r_snap=18.0)

                            def to_spec_edge(e):
                                a,b=e; return tuple(sorted([det2spec.get(a,a), det2spec.get(b,b)]))

                            measured_edges_spec = set(to_spec_edge(e) for e in measured_edges_det)

                            # 프레임 단위 수집/투표
                            self._edge_counts.update(measured_edges_spec)
                            fr_pack = {
                                "img": out.copy(),
                                "comps": comps,
                                "cabs": cabs,
                                "det2spec": det2spec,
                                "edges_spec": measured_edges_spec
                            }
                            self._judge_frames.append(fr_pack)
                            if len(self._judge_frames) > 60:
                                self._judge_frames.pop(0)

                            # 현재 프레임 임시 결과(표시/스냅용)
                            verdict_text, ok, missing, extra = _compare(spec_obj["edges"], measured_edges_spec)

                            # === [CHANGE] 한 번이라도 PASS면 PASS 스냅샷 고정
                            if verdict_text == "PASS" and not self._any_pass_frame:
                                self._any_pass_frame = True
                                self._pass_frame_snapshot = fr_pack

                        col = (50,220,50) if verdict_text=="PASS" else ((0,165,255) if verdict_text=="UNKNOWN" else (0,0,255))
                        cv2.putText(out, f"VERDICT: {verdict_text}", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 3, cv2.LINE_AA)
                        _draw_cj_overlay(out, comps, cabs, measured_edges_spec, det2spec)

                        # 수집 종료 → 최종 한 번만 저장/발행
                        if (not self._judge_saved_once) and (time.time() - self._judge_collect_start >= self._judge_collect_for):
                            spec_obj = self.judge_spec
                            # === [CHANGE] 우선순위: 단발 PASS가 있으면 그걸 최종으로
                            if self._any_pass_frame and self._pass_frame_snapshot is not None and spec_obj is not None:
                                final_edges = set(self._pass_frame_snapshot["edges_spec"])
                                final_verdict, final_ok, final_missing, final_extra = _compare(spec_obj["edges"], final_edges)
                                best_frame = self._pass_frame_snapshot
                            else:
                                # 과반 투표
                                N = max(1, len(self._judge_frames))
                                vote_thresh = max(1, int(0.5 * N))
                                final_edges = {e for e, c in self._edge_counts.items() if c >= vote_thresh}
                                if spec_obj is not None and spec_obj.get("edges"):
                                    final_verdict, final_ok, final_missing, final_extra = _compare(spec_obj["edges"], final_edges)
                                else:
                                    final_verdict, final_ok, final_missing, final_extra = ("UNKNOWN", [], [], [])
                                # 최종 엣지에 가장 가까운 프레임 선택
                                best_idx, best_score = 0, -1
                                for i, fr in enumerate(self._judge_frames):
                                    score = len(fr["edges_spec"] & final_edges) * 2 - len(final_edges - fr["edges_spec"])
                                    if score > best_score:
                                        best_idx, best_score = i, score
                                best_frame = self._judge_frames[best_idx] if self._judge_frames else {"img": out.copy()}

                            ts = time.strftime("%Y%m%d_%H%M%S")
                            overlay_path = os.path.join(JUDGE_OUT_DIR, f"overlay_{ts}.png")
                            report_path  = os.path.join(JUDGE_OUT_DIR, f"report_{ts}.json")
                            try:
                                cv2.imwrite(overlay_path, best_frame["img"])
                                payload_full = {
                                    "timestamp": ts,
                                    "spec_raw": spec_obj.get("raw") if spec_obj else None,
                                    "final_edges_spec": sorted(list(final_edges)),
                                    "final_compare": {
                                        "verdict": final_verdict,
                                        "ok": final_ok,
                                        "missing": final_missing,
                                        "extra": final_extra
                                    },
                                    "frames_used": int(len(self._judge_frames))
                                }
                                with open(report_path,"w",encoding="utf-8") as f:
                                    json.dump(payload_full, f, ensure_ascii=False, indent=2)
                                print(f"[JUDGE] overlay saved: {overlay_path}")
                                print(f"[JUDGE] report saved: {report_path}")
                            except Exception as e:
                                print(f"[JUDGE] save error: {e}")

                            # 발행(정책 적용)
                            if spec_obj is None:
                                print("[QC] skip publish (no spec)")
                            else:
                                if not self._judge_published_once and self._should_publish(final_verdict):
                                    lite = "pass" if final_verdict == "PASS" else "fail"
                                    send_QC(lite)
                                    self._judge_published_once = True
                                    send_QC_ARD("PASS" if final_verdict == "PASS" else "FAIL")
                                elif not self._should_publish(final_verdict):
                                    print("[QC] skip publish by policy")

                            self._judge_saved_once = True  # 한 번만

                        with self.lock:
                            self.last_verdict = verdict_text
                            self.last_ok = ok
                            self.last_missing = missing
                            self.last_extra = extra

                    except Exception as e:
                        cv2.putText(out, f"judge error: {e}", (10,28),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA)
                else:
                    draw_all_detections(out, r, self.names)
                    if has_board:
                        cv2.rectangle(out,(bbox[0],bbox[1]),(bbox[2],bbox[3]),(80,255,80),2)
                        t_angle=self.baseline+delta_angle
                        x1,y1,x2,y2=bbox
                        cx=(x1+x2)//2; cy=(y1+y2)//2
                        L=int(0.9*min(x2-x1, y2-y1)); t=math.radians(t_angle)
                        dx=int((L/2)*math.cos(t)); dy=int((L/2)*math.sin(t))
                        cv2.line(out,(cx-dx,cy-dy),(cx+dx,cy+dy),(0,255,0),3,cv2.LINE_AA)
                        cv2.circle(out,(cx,cy),4,(0,255,0),-1,cv2.LINE_AA)
                        txt=f"Δangle={delta_angle:+.1f}°, seen={self.visible_for:.2f}s"
                        cv2.putText(out, txt, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA)
                        if depth_m>0:
                            cv2.putText(out, f"Z={depth_m:.3f}m", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(out,"BOARD NOT FOUND",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),2, cv2.LINE_AA)

                if self.window_open:
                    try:
                        cv2.imshow(self.window_name, out)
                        if (cv2.waitKey(1) & 0xFF)==ord('q'):
                            self.force_close=True; break
                    except: self.window_open=False

                with self.lock:
                    self.latest.update({
                        "has_board": has_board, "bbox": bbox, "delta_angle": float(delta_angle),
                        "cxcy": cxcy, "depth_m": float(depth_m), "p_cam_m": p_cam_m,
                        "timestamp": time.time(), "classes_present": cls_names,
                        "only_board": bool(only_board),
                        "enc_center": enc_center,
                        "enc_radius": float(enc_radius),
                        "enc_depth_m": float(enc_depth_m),
                        "last_frame": color, "last_result": r
                    })
            except Exception as e:
                print(f"[VISION] loop error: {e}")
                time.sleep(0.01)

        try:
            if self.window_open: cv2.destroyWindow(self.window_name); cv2.waitKey(1)
        except: pass
        self.window_open=False
        print("[VISION] stopped loop")

    def get_state(self):
        with self.lock:
            avg_angle = float(np.mean(self.angle_hist)) if self.angle_hist else 0.0
            return {"visible_for": self.visible_for, "last_seen_ts": self.last_seen_ts,
                    "avg_delta_angle": avg_angle, **self.latest}

# ===== 메인 =====
def main(args=None):
    global flag
    # Hand-Eye
    try:
        T_g2c = np.load(T_G2C_PATH)
        if T_g2c.shape != (4,4): raise ValueError("T_gripper2camera.npy shape must be (4,4)")
    except Exception as e:
        print(f"[FATAL] Hand-Eye matrix load failed: {e}")
        return

    rclpy.init(args=args)
    node = rclpy.create_node("rokey_yolo_pick", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    grip_node = GripNode()

    try:
        from DSR_ROBOT2 import set_tool, set_tcp, movej, movel, get_current_posx
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"[FATAL] DSR_ROBOT2 import error: {e}")
        rclpy.shutdown(); return

    # 준비
    JReady = [0, 0, 90, 0, 90, 0]
    pos0 = [17.86, 6.4, 113.43, 0.11, 60.22, 107.38]
    set_tool("Tool Weight_2FG"); set_tcp("2FG_TCP")

    # MQTT 구독 (명령 + 외부 QC결과)
    mqc = MqttCommand(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_TOPIC, use_tls=True)
    qcsub = MqttQCResultSub(MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_QC_SUB_TOPIC, use_tls=True)
    try:
        mqc.start(); qcsub.start()
        print(f"[INFO] MQTT listen on '{MQTT_TOPIC}' for {{\"checkboard\":1}} (+spec)")
        print(f"[INFO] MQTT listen on '{MQTT_QC_SUB_TOPIC}' for external QC result (OK/NG)")
    except Exception:
        print("[WARN] MQTT subscriber failed to start.")

    # 비전 스레드: 카메라 ON, YOLO는 나중 장착
    vt = VisionThread(model=None, names_map={}, baseline_angle=0.0)
    vt.start()

    # 발행 정책 상태 초기화 (1회)
    vt._published_once = False
    vt._last_sent_verdict = None

    # A/B 카운터
    count_a = 0; count_b = 0

    while 1:
        movej(posj(JReady), vel=30, acc=30)

        with _flag_lock:
            f = flag

        if f == 0:
            model = YOLO(MODEL_PATH_MAIN)
            names_map = names_dict(model.names)

            base_img = cv2.imread(BASELINE_IMAGE)
            if base_img is None:
                print(f"[WARN] baseline image not found: {BASELINE_IMAGE} → baseline=0°")
                baseline_angle = 0.0
            else:
                base_res = model(base_img, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)[0]
                base_bbox = select_board_bbox(base_res, names_map)
                baseline_angle = estimate_angle_minarearect(crop(base_img, base_bbox)) if base_bbox else 0.0
            print(f"[INFO] baseline_angle = {baseline_angle:+.1f}°")

            vt.set_model(model, names_map, baseline_angle)
            grip_node.re(); time.sleep(0.5)

            while 1:
                with _flag_lock:
                    if flag == 1:
                        print("[MAIN] flag==1 detected → YOLO OFF (camera stays ON)")
                        vt.set_model(None); break

                movej(pos0, vel=VELOCITY, acc=ACC)

                print("[INFO] Waiting for board visible 1.0s to lock angle...")
                angle_timeout = time.time() + 100.0
                measured_angle = None
                while time.time() < angle_timeout:
                    with _flag_lock:
                        if flag == 1:
                            print("[MAIN] flag==1 during angle wait → YOLO OFF")
                            vt.set_model(None); break
                    s = vt.get_state()
                    if s["has_board"] and s["visible_for"] >= 1.0:
                        measured_angle = clamp_delta(s["avg_delta_angle"], ANGLE_LIMIT)
                        break
                    time.sleep(0.02)

                with _flag_lock:
                    if flag == 1: break

                if measured_angle is None:
                    print("[ERROR] Angle lock failed (board not stable for 1s).")
                    vt.set_model(None); rclpy.shutdown(); return

                print(f"[INFO] Angle locked: {measured_angle:+.1f} deg")
                print(f"[CMD] movej turngrip(pos0, {measured_angle:+.1f})")
                movej(turngrip(pos0, measured_angle), vel=VELOCITY, acc=ACC)
                time.sleep(0.3)

                print("[INFO] Measuring XYZ after rotation (hand-eye)...")
                pos_timeout = time.time() + 8.0
                boardpos_mm = None
                route_for_place = None

                while time.time() < pos_timeout:
                    with _flag_lock:
                        if flag == 1:
                            print("[MAIN] flag==1 during position measure → YOLO OFF")
                            vt.set_model(None); break
                    s = vt.get_state()
                    if s["has_board"] and s["p_cam_m"] is not None and (time.time() - s["timestamp"]) < 0.5:
                        X_m, Y_m, Z_m = s["p_cam_m"]
                        curr_posx = get_current_posx()
                        p_base_mm = cam_point_m_to_base_mm(curr_posx, np.load(T_G2C_PATH), X_m, Y_m, Z_m)
                        tcp_base = np.array((curr_posx[0] if isinstance(curr_posx, tuple) else curr_posx)[0:3], dtype=float)
                        dx, dy, dz = (p_base_mm - tcp_base).tolist()
                        # 요청 코드의 스케일 유지(0.9 없이)
                        boardpos_mm = [-dx, -dy, -dz]

                        # 외부 QC 라우팅 우선
                        qc_res = get_qc_result(clear=True)
                        if qc_res == "NG":
                            route_for_place = 'B'
                            print("[ROUTE] external QC says NG → force route B")
                            send_QC("fail")  # TTS를 위한 QC 결과 발행
                        else:
                            route_for_place = 'A' if s.get("only_board", False) else 'B'
                            # only_board가 True면 양품(A), False면 불량(B)
                            if s.get("only_board", False):
                                send_QC("pass")  # 양품
                            else:
                                send_QC("fail")  # 불량
                        arduino_verdict = "PASS" if route_for_place == 'A' else "FAIL"
                        send_QC_ARD(arduino_verdict)
                        break
                    time.sleep(0.02)

                with _flag_lock:
                    if flag == 1: break

                if boardpos_mm is None or route_for_place is None:
                    print("[ERROR] Position measurement or routing failed.")
                    vt.set_model(None); rclpy.shutdown(); return

                boardpos_mm[0] += BIAS_X_MM; boardpos_mm[1] += BIAS_Y_MM
                print(f"[INFO] offset after hand-eye + bias (mm): {boardpos_mm}")
                print(f"[ROUTE] place route = {route_for_place}")

                target_posx = topos(get_current_posx(), boardpos_mm)
                print(f"[CMD] movel(posx({target_posx}))  # pick approach")
                movel(posx(target_posx), vel=VELOCITY, acc=ACC)
                grip_node.grip(); time.sleep(1)
                curr = get_current_posx()
                up = list(curr[0] if isinstance(curr, tuple) else curr); up[2] += SAFE_UP_DZ
                movel(posx(up), vel=VELOCITY, acc=ACC)

                # 라우팅별 스택 포즈
                if route_for_place == 'A':
                    place_pos = PLACE_A_POSX(count_a); count_a += 1
                else:
                    place_pos = PLACE_B_POSX(count_b); count_b += 1

                print(f"[PLACE] routing {route_for_place} → {place_pos}")
                movel(posx(place_pos), vel=VELOCITY, acc=ACC)

                # 힘/순응 제어로 살짝 눌러 놓기
                compliance_force_down(target_force=-15, trigger_threshold=FORCE_VALUE, timeout=3.0)

                # 놓기 후 안전 상승
                grip_node.re()
                curr = get_current_posx()
                up2 = list(curr[0] if isinstance(curr, tuple) else curr); up2[2] += SAFE_UP_DZ
                movel(posx(up2), vel=VELOCITY, acc=ACC)

                payload = {
                    "timestamp": time.time(),
                    "route": route_for_place,
                    "delta_angle_deg": float(measured_angle),
                    "target_offset_mm": {"x": float(boardpos_mm[0]), "y": float(boardpos_mm[1]), "z": float(boardpos_mm[2])},
                    "stack_index": int(count_a-1 if route_for_place=='A' else count_b-1)
                }
                if route_for_place == 'A': send_A(payload)
                else: send_B(payload)

        with _flag_lock:
            f = flag
        if f == 1:
            print("[MAIN] flag==1 → go TARGET_POSE, auto-center twice, then JUDGE on SAME WINDOW")
            from DR_common2 import posx
            TARGET_POSE = [377.0, -226.0, 150.0, 0.0, -180.0, 0.0]
            movel(posx(TARGET_POSE), vel=VELOCITY, acc=ACC)

            # (1) 첫 번째 자동 센터링(보드 기준)
            try:
                model_auto1 = YOLO(MODEL_PATH_AUTO1)
            except Exception as e:
                print(f"[ERROR] failed to load AUTO1: {e}")
                with _flag_lock: flag = 0
                continue
            vt.set_model(model_auto1, names_dict(model_auto1.names), baseline_angle=0.0)

            t_end = time.time() + 1.0
            best = {"cxcy": None, "depth_m": 0.0}
            while time.time() < t_end:
                s = vt.get_state()
                if s["cxcy"] is not None and s["depth_m"] > 0:
                    best = {"cxcy": s["cxcy"], "depth_m": s["depth_m"]}
                time.sleep(0.02)
            if best["cxcy"] is not None and best["depth_m"] > 0:
                cx, cy = best["cxcy"]; Z = float(best["depth_m"])
                W, H = 640, 480; u0, v0 = W//2, H//2
                u_err, v_err = cx - u0, cy - v0
                try:
                    depth_stream = vt.pipeline.get_active_profile().get_stream(rs.stream.depth)
                    intr = depth_stream.as_video_stream_profile().get_intrinsics()
                    fx, fy = float(intr.fx), float(intr.fy)
                except Exception as e:
                    print(f"[CENTER] intrinsics error: {e}")
                    with _flag_lock: flag = 0
                    vt.set_model(None); continue
                dX_cam = -(u_err) * (Z / fx); dY_cam = -(v_err) * (Z / fy); dZ_cam = 0.0
                curr_posx = get_current_posx()
                T_b2g = posx_to_T_base2gripper(list(curr_posx[0] if isinstance(curr_posx, tuple) else curr_posx))
                T_g2c = np.load(T_G2C_PATH); T_b2c = T_b2g @ T_g2c; R_c2b = T_b2c[:3, :3]
                d_cam_mm = np.array([dX_cam*1000.0, dY_cam*1000.0, dZ_cam*1000.0], dtype=float)
                d_base_mm = (R_c2b @ d_cam_mm).tolist()
                tgt = list(curr_posx[0] if isinstance(curr_posx, tuple) else curr_posx)
                tgt[0] -= d_base_mm[0]; tgt[1] -= d_base_mm[1]; tgt[2] = 60
                print(f"[CENTER1] move base by (mm)=({d_base_mm[0]:+.1f},{d_base_mm[1]:+.1f},{d_base_mm[2]:+.1f})")
                movel(posx(tgt), vel=VELOCITY, acc=ACC)

            # (2) 두 번째 자동 센터링(클러스터 중심만 사용)
            try:
                model_auto2 = YOLO(MODEL_PATH_AUTO2)
            except Exception as e:
                print(f"[ERROR] failed to load AUTO2: {e}")
                with _flag_lock: flag = 0
                continue
            vt.set_model(model_auto2, names_dict(model_auto2.names), baseline_angle=0.0)
            vt.set_overlay_mode("encircle")

            t_end = time.time() + 1.0
            best = {"cxcy": None, "depth_m": 0.0}
            while time.time() < t_end:
                s = vt.get_state()
                enc_c = s.get("enc_center", None)
                enc_z = float(s.get("enc_depth_m", 0.0) or 0.0)
                if enc_c is not None and enc_z > 0:
                    best = {"cxcy": (int(enc_c[0]), int(enc_c[1])), "depth_m": enc_z}
                time.sleep(0.02)
            vt.set_overlay_mode("normal")

            if best["cxcy"] is not None and best["depth_m"] > 0:
                cx, cy = best["cxcy"]; Z = float(best["depth_m"])
                W, H = 640, 480; u0, v0 = W//2, H//2
                u_err, v_err = cx - u0, cy - v0
                try:
                    depth_stream = vt.pipeline.get_active_profile().get_stream(rs.stream.depth)
                    intr = depth_stream.as_video_stream_profile().get_intrinsics()
                    fx, fy = float(intr.fx), float(intr.fy)
                except Exception as e:
                    print(f"[CENTER] intrinsics error: {e}")
                    with _flag_lock: flag = 0
                    vt.set_model(None); continue
                dX_cam = -(u_err) * (Z / fx); dY_cam = -(v_err) * (Z / fy); dZ_cam = 0.0
                curr_posx = get_current_posx()
                T_b2g = posx_to_T_base2gripper(list(curr_posx[0] if isinstance(curr_posx, tuple) else curr_posx))
                T_g2c = np.load(T_G2C_PATH); T_b2c = T_b2g @ T_g2c; R_c2b = T_b2c[:3, :3]
                d_cam_mm = np.array([dX_cam*1000.0, dY_cam*1000.0, dZ_cam*1000.0], dtype=float)
                d_base_mm = (R_c2b @ d_cam_mm).tolist()

                # 이동
                tgt = list(curr_posx[0] if isinstance(curr_posx, tuple) else curr_posx)
                tgt[0] -= d_base_mm[0]; tgt[1] -= d_base_mm[1]; tgt[2] = -10
                print(f"[CENTER2] move base by (mm)=({d_base_mm[0]:+.1f},{d_base_mm[1]:+.1f},{d_base_mm[2]:+.1f})")
                movel(posx(tgt), vel=VELOCITY, acc=ACC)

                # ====== 판정 실행 ======
                vt.set_judge_spec_from_cache_or_file()
                vt.set_overlay_mode("judge")
                time.sleep(2.0)
                vt.set_overlay_mode("normal")

            with _flag_lock:
                flag = 0
            print("[MAIN] flag reset to 0")

if __name__ == "__main__":
    main()
