#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import json
import math
import hashlib
import numpy as np
from pathlib import Path
from ultralytics import YOLO
from collections import defaultdict

# [ADD] 최소 추가: MQTT 사용에 필요한 import
import uuid
import ssl
import paho.mqtt.client as mqtt

# ===== MQTT 설정 =====
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883
MQTT_USERNAME = "okj1812"
MQTT_PASSWORD = "okj1812"
SUB_TOPIC = "llm/action"
MQTT_QOS = 1
CLIENT_ID = f"tts-openai-{uuid.uuid4().hex[:8]}"

# [ADD] 최소 추가: 퍼블리시 토픽과 최종 판정 임계값
PUB_TOPIC = "/qc_result"      # TTS 노드가 구독하는 토픽
OK_RATIO  = 0.80              # 80% 이상 OK면 최종 OK

# ===== 카메라 / 모델 / 템플릿 경로 =====
CAM_INDEX      = 2 # <- 사용하는 웹캠 인덱스
MODEL_PATH     = '/home/okj1812/Downloads/uno_final_dec.pt'
TEMPLATE_JSON  = r"/home/okj1812/Downloads/image.board_usb_template.json"

# ===== 옵션 =====
CONF_THRES        = 0.50
IMG_SIZE          = 640

TRIGGER_MODE      = "line"   # "line" 또는 "appear"
TRIGGER_Y_RATIO   = 0.35     # 보드 중심 y >= H*ratio → True
HYSTERESIS_RATIO  = 0.02     # 라인 히스테리시스(비율)
CLEAR_FRAMES      = 8        # 재암 대기(꺼짐 연속 프레임 수)
N_FRAMES_TO_JUDGE = 25      # 판별 프레임 수

DETECT_LONG_SIDE  = 512      # 대기 상태에서 다운스케일 탐지
POS_TOL_NORM      = 0.15     # 허용 오차(정규화 거리)
SAVE_ANNOTATED    = False     # 판별 프레임 저장
SHOW_WINDOW       = True     # 판별 화면 보기

# ===== 카메라 세팅(선택) =====
REQ_WIDTH        = 640      # 0 또는 None이면 기본값 유지
REQ_HEIGHT       = 480
REQ_FPS          = 30
WARMUP_FRAMES    = 5

# ===== 라벨 =====
BOARD_CLASS_NAME  = "Arduino board"
USB_CLASS_NAME    = "USB"

# [ADD] 최소 추가: MQTT 퍼블리시 함수 (TLS 8883, ID/PW)
def mqtt_publish_result(is_ok: bool):
    payload = json.dumps({"result": "OK" if is_ok else "NG"}, ensure_ascii=False)
    client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.tls_set(
        certfile=None, keyfile=None,
        cert_reqs=ssl.CERT_REQUIRED,
        tls_version=ssl.PROTOCOL_TLS_CLIENT
    )
    client.tls_insecure_set(False)
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.publish(PUB_TOPIC, payload, qos=MQTT_QOS, retain=False)
    client.disconnect()
    print(f"[MQTT] published {PUB_TOPIC} -> {payload}")

# ========== 유틸 ==========
def names_dict(names):
    return names if isinstance(names, dict) else {i: n for i, n in enumerate(names)}

def color_for_class(name: str):
    import hashlib as _hashlib
    hval = int(_hashlib.sha1(name.encode("utf-8")).hexdigest(), 16)
    h = (hval % 360); s = 200; v = 255
    hsv = np.uint8([[[h // 2, s, v]]])  # OpenCV H: 0~179
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])

def draw_box(img, box, color, t=2):
    x1, y1, x2, y2 = map(int, box)
    h, w = img.shape[:2]
    x1 = max(0, min(x1, w-1)); x2 = max(0, min(x2, w-1))
    y1 = max(0, min(y1, h-1)); y2 = max(0, min(y2, h-1))
    if x2 <= x1 or y2 <= y1: return
    cv2.rectangle(img, (x1, y1), (x2, y2), color, t, cv2.LINE_AA)

def put_text(img, text, org, color=(255,255,255), scale=0.6, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (30,30,30), thick+2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

def box_center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

def rotate_point(p, center, theta):
    x, y = p; cx, cy = center
    dx, dy = x - cx, y - cy
    c, s = math.cos(theta), math.sin(theta)
    xr = dx * c - dy * s + cx
    yr = dx * s + dy * c + cy
    return (xr, yr)

def normalize_point_board_usb(pix_pt, board_box, theta_align):
    # 보드 중심을 원점, "보드→USB"가 +X가 되도록 -theta_align 회전 → 반폭/반높이로 정규화
    cx, cy = box_center(board_box)
    px, py = rotate_point(pix_pt, (cx, cy), -theta_align)
    half_w = max(1e-6, (board_box[2] - board_box[0]) / 2.0)
    half_h = max(1e-6, (board_box[3] - board_box[1]) / 2.0)
    nx = (px - cx) / half_w
    ny = (py - cy) / half_h
    return (float(nx), float(ny))

def run_yolo(model, image_bgr):
    res = model(image_bgr, conf=CONF_THRES, imgsz=IMG_SIZE, verbose=False)[0]
    out = []
    if res.boxes is None or len(res.boxes) == 0:
        return out
    names = names_dict(res.names if hasattr(res, "names") else model.names)
    xyxy = res.boxes.xyxy.cpu().numpy()
    conf = res.boxes.conf.cpu().numpy()
    cls  = res.boxes.cls.cpu().numpy().astype(int)
    for (x1, y1, x2, y2), cf, cid in zip(xyxy, conf, cls):
        cname = str(names.get(int(cid), cid))
        out.append({"box": (float(x1), float(y1), float(x2), float(y2)),
                    "conf": float(cf), "cls_id": int(cid), "cls_name": cname})
    return out

def detect_board_and_usb(dets):
    board = None; usb = None
    for d in dets:
        if d["cls_name"] == BOARD_CLASS_NAME:
            if (board is None) or (d["conf"] > board["conf"]):
                board = d
    for d in dets:
        if d["cls_name"] == USB_CLASS_NAME:
            if (usb is None) or (d["conf"] > usb["conf"]):
                usb = d
    return board, usb

def greedy_match(ref_pts, cur_pts):
    matches = []
    if not ref_pts or not cur_pts:
        return matches, set(range(len(ref_pts))), set(range(len(cur_pts)))
    used_r, used_c = set(), set()
    dist_list = []
    for i, rp in enumerate(ref_pts):
        for j, cp in enumerate(cur_pts):
            dist_list.append( (math.hypot(rp[0]-cp[0], rp[1]-cp[1]), i, j) )
    dist_list.sort(key=lambda t: t[0])
    for d, i, j in dist_list:
        if i in used_r or j in used_c: continue
        used_r.add(i); used_c.add(j)
        matches.append((i, j, d))
        if len(used_r) == len(ref_pts) or len(used_c) == len(cur_pts):
            break
    return matches, (set(range(len(ref_pts))) - used_r), (set(range(len(cur_pts))) - used_c)

def resize_for_detect(frame):
    H, W = frame.shape[:2]
    long_side = max(H, W)
    if long_side <= DETECT_LONG_SIDE:
        return 1.0, frame
    scale = DETECT_LONG_SIDE / float(long_side)
    small = cv2.resize(frame, (int(W*scale), int(H*scale)), interpolation=cv2.INTER_AREA)
    return scale, small

def detect_board_bbox_small(model, frame):
    # 트리거 대기용: 다운스케일 탐지 후 원복
    scale, small = resize_for_detect(frame)
    res = model(small, conf=CONF_THRES, imgsz=IMG_SIZE, verbose=False)[0]
    if res.boxes is None or len(res.boxes) == 0:
        return None
    xyxy = res.boxes.xyxy.cpu().numpy()
    conf = res.boxes.conf.cpu().numpy()
    cls  = res.boxes.cls.cpu().numpy().astype(int)
    names = names_dict(res.names if hasattr(res, "names") else model.names)
    best, best_cf = None, -1.0
    for (x1,y1,x2,y2), cf, cid in zip(xyxy, conf, cls):
        if str(names.get(int(cid), cid)) != BOARD_CLASS_NAME: continue
        if cf > best_cf:
            best_cf = cf; best = (float(x1),float(y1),float(x2),float(y2))
    if best is None:
        return None
    if scale != 1.0:
        inv = 1.0/scale
        x1,y1,x2,y2 = best
        return (x1*inv, y1*inv, x2*inv, y2*inv)
    return best

# ========== 템플릿 로드 ==========
def load_template(json_path):
    p = Path(json_path)
    if not p.exists():
        raise FileNotFoundError(f"TEMPLATE_JSON 파일을 찾을 수 없습니다: {json_path}")
    with open(p, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "template" not in data:
        raise RuntimeError("템플릿 JSON 형식이 올바르지 않습니다. (template 필드 필요)")
    templ = data["template"]         # {cls: [{"nx","ny","conf"}...]}
    whitelist = set(data.get("whitelist") or []) if "whitelist" in data else None
    return templ, whitelist

# ========== 1프레임 판별 ==========
def judge_frame(model, frame, template, whitelist=None):
    """
    반환: canvas(BGR), verdict(bool), issues[list[str]]
    - 현재 프레임에서 보드/USB를 찾아 보드→USB 각도(theta_cur)로 정렬한 좌표계 사용
    - 템플릿 좌표(ref)와 현재 좌표(cur)의 거리 비교
    """
    dets = run_yolo(model, frame)
    board, usb = detect_board_and_usb(dets)

    canvas = frame.copy()
    issues = []
    ok_all = True

    if board is None:
        put_text(canvas, "NG: Board not found", (14, 36), (0,0,255), 1.0, 2)
        return canvas, False, ["Board not found"]

    draw_box(canvas, board["box"], (80,255,80), 2)
    if usb is None:
        put_text(canvas, "NG: USB not found", (14, 64), (0,0,255), 1.0, 2)
        return canvas, False, ["USB not found"]

    draw_box(canvas, usb["box"], (0,255,255), 2)

    # 현재 프레임의 정렬 각도(보드→USB)
    bc = box_center(board["box"])
    uc = box_center(usb["box"])
    theta_cur = math.atan2(uc[1]-bc[1], uc[0]-bc[0])

    # 현재 프레임에서 클래스별 정규화 좌표 수집(보드/USB 제외)
    cur_by_cls = defaultdict(list)
    for d in dets:
        cname = d["cls_name"]
        if cname in (BOARD_CLASS_NAME, USB_CLASS_NAME):
            continue
        if whitelist is not None and cname not in whitelist and cname in template:
            pass
        cxy = box_center(d["box"])
        nx, ny = normalize_point_board_usb(cxy, board["box"], theta_cur)
        cur_by_cls[cname].append({"nx": nx, "ny": ny, "box": d["box"], "conf": d["conf"]})

    diag = math.sqrt(2.0)  # 반대각선(중심→코너)

    # 비교/시각화
    for cname, ref_list in template.items():
        cur_list = cur_by_cls.get(cname, [])
        # 개수 판정
        if len(cur_list) != len(ref_list):
            issues.append(f"{cname}: count {len(cur_list)}/{len(ref_list)}")
            ok_all = False

        ref_pts = [(r["nx"], "ny" in r and r["ny"] or r["ny"]) for r in ref_list]  # 의도: 기존 형식 유지
        ref_pts = [(r["nx"], r["ny"]) for r in ref_list]  # 안전하게 재정의
        cur_pts = [(c["nx"], c["ny"]) for c in cur_list]
        matches, ref_miss, cur_extra = greedy_match(ref_pts, cur_pts)

        # 매칭된 항목 판정
        for ri, ci, dist in matches:
            ok = (dist <= POS_TOL_NORM * diag)
            if not ok: ok_all = False

            # 시각화: 매칭선/라벨
            cbox = cur_list[ci]["box"]
            cpx, cpy = box_center(cbox)

            # 기준점(ref)을 현재 프레임 픽셀로 표시(현재 각도 사용)
            half_w = (board["box"][2] - board["box"][0]) / 2.0
            half_h = (board["box"][3] - board["box"][1]) / 2.0
            rx = bc[0] + ref_pts[ri][0] * half_w
            ry = bc[1] + ref_pts[ri][1] * half_h
            rx, ry = rotate_point((rx, ry), bc, +theta_cur)

            col = (0,200,0) if ok else (0,0,255)
            cv2.line(canvas, (int(rx), int(ry)), (int(cpx), int(cpy)), col, 2, cv2.LINE_AA)
            draw_box(canvas, cbox, color_for_class(cname), 2)

            pct = (dist / diag) * 100.0  # 반대각선 기준 %
            put_text(canvas, f"{cname} {'OK' if ok else 'NG'} {pct:.1f}%", (int(cpx)+8, int(cpy)-8),
                     (255,255,255), 0.55, 1)

        # 누락/추가
        for ri in ref_miss:
            half_w = (board["box"][2] - board["box"][0]) / 2.0
            half_h = (board["box"][3] - board["box"][1]) / 2.0
            rx = bc[0] + ref_pts[ri][0] * half_w
            ry = bc[1] + ref_pts[ri][1] * half_h
            rx, ry = rotate_point((rx, ry), bc, +theta_cur)
            cv2.circle(canvas, (int(rx), int(ry)), 12, (0,0,255), 2, cv2.LINE_AA)
            issues.append(f"{cname}: missing")
            ok_all = False

        for ci in cur_extra:
            cpx, cpy = box_center(cur_list[ci]["box"])
            cv2.circle(canvas, (int(cpx), int(cpy)), 12, (255,0,255), 2, cv2.LINE_AA)
            issues.append(f"{cname}: extra")
            ok_all = False

    # 템플릿에 없는 라벨이 현재 프레임에만 있을 때 표시
    for cname in cur_by_cls.keys():
        if cname not in template:
            for it in cur_by_cls[cname]:
                cpx, cpy = box_center(it["box"])
                cv2.circle(canvas, (int(cpx), int(cpy)), 12, (255,0,255), 2, cv2.LINE_AA)
            issues.append(f"{cname}: extra (no template)")
            ok_all = False

    # 헤더/이슈
    put_text(canvas, f"VERDICT: {'OK' if ok_all else 'NG'}", (14, 28),
             (0,200,0) if ok_all else (0,0,255), 1.0, 2)
    y = 56
    for s in issues[:6]:
        put_text(canvas, f"- {s}", (14, y), (255,255,255), 0.65, 1)
        y += 24
    return canvas, ok_all, issues

# ========== 메인 (실시간 웹캠) ==========
def main():
    # YOLO
    model = YOLO(MODEL_PATH)

    # 저장된 템플릿 로드
    template, whitelist = load_template(TEMPLATE_JSON)

    # 카메라 열기
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"웹캠을 열 수 없습니다: CAM_INDEX={CAM_INDEX}")

    # 원하는 해상도/FPS/포맷 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  REQ_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          REQ_FPS)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # 실제 적용된 값 확인
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    FPS = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Camera opened: index={CAM_INDEX}, {W}x{H} @ {FPS:.0f}fps (MJPG)")

    line_y  = int(H * TRIGGER_Y_RATIO)
    hyst_px = max(1, int(H * HYSTERESIS_RATIO))

    # 저장 폴더
    out_base = Path(f"cam{CAM_INDEX}_stream")
    save_dir = out_base.parent / f"{out_base.name}_judge20"
    if SAVE_ANNOTATED:
        save_dir.mkdir(parents=True, exist_ok=True)

    # 워밍업
    for i in range(WARMUP_FRAMES):
        ret, _ = cap.read()
        if not ret:
            break

    prev_state = False
    need_rearm = False
    clear_cnt  = 0
    judging    = False
    left       = 0
    frame_idx  = 0
    ok_count_run = 0

    win_name = "QC (Webcam) - JSON template"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 프레임 수신 실패")
            break
        frame_idx += 1

        # ESC / q 종료키 처리
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        # 판별 중
        if judging:
            canvas, ok_all, issues = judge_frame(model, frame, template, whitelist)
            put_text(canvas, f"Frame {N_FRAMES_TO_JUDGE-left+1}/{N_FRAMES_TO_JUDGE}",
                     (W-300, 28), (200,255,200), 0.7, 2)

            if ok_all:   # <-- OK일 때 카운트
                ok_count_run += 1

            if SHOW_WINDOW:
                cv2.imshow(win_name, canvas)

            if SAVE_ANNOTATED:
                sp = save_dir / f"judge_{frame_idx:06d}.png"
                if not cv2.imwrite(str(sp), canvas):
                    print(f"[WARN] 저장 실패: {sp}")
                else:
                    print(f"[SAVE] {sp}")

            left -= 1
            if left <= 0:
                judging   = False
                need_rearm= True
                print(f"[RUN RESULT] OK {ok_count_run}/{N_FRAMES_TO_JUDGE}")
                # [ADD] 최소 추가: 최종 MQTT 발행 (OK_RATIO 기준)
                final_is_ok = (ok_count_run >= int(N_FRAMES_TO_JUDGE * OK_RATIO))
                mqtt_publish_result(final_is_ok)
            continue

        # 재암 단계
        if need_rearm:
            board_box = detect_board_bbox_small(model, frame)
            state = False
            if TRIGGER_MODE == "line":
                if board_box is not None:
                    _, cy = box_center(board_box)
                    if cy >= line_y + hyst_px: state = True
                    if cy <= line_y - hyst_px: state = False
            elif TRIGGER_MODE == "appear":
                state = board_box is not None

            if state is False:
                clear_cnt += 1
            else:
                clear_cnt = 0

            ui = frame.copy()
            cv2.line(ui, (0, line_y), (W, line_y), (0,255,255), 2, cv2.LINE_AA)
            put_text(ui, "Re-arming...", (14, 28), (0,180,255), 0.8, 2)
            if SHOW_WINDOW:
                cv2.imshow(win_name, ui)

            if clear_cnt >= CLEAR_FRAMES:
                need_rearm = False
                prev_state = False
            continue

        # 트리거 대기(다운스케일 탐지로 보드만 감시)
        board_box = detect_board_bbox_small(model, frame)

        # 현재 상태 계산
        if TRIGGER_MODE == "appear":
            curr_state = (board_box is not None)
        else:
            if board_box is None:
                curr_state = False
            else:
                _, cy = box_center(board_box)
                if cy >= line_y + hyst_px: curr_state = True
                elif cy <= line_y - hyst_px: curr_state = False
                else: curr_state = prev_state  # 밴드 내 유지

        # 대기 UI
        ui = frame.copy()
        cv2.line(ui, (0, line_y), (W, line_y), (0,255,255), 2, cv2.LINE_AA)
        if board_box is not None:
            draw_box(ui, board_box, (80,255,80), 2)
        put_text(ui, "Waiting for board...", (14, 28), (255,255,255), 0.8, 2)
        if SHOW_WINDOW:
            cv2.imshow(win_name, ui)

        # 상승에지에서 20프레임 판별 시작
        if (prev_state is False) and (curr_state is True):
            judging = True
            left    = N_FRAMES_TO_JUDGE
            ok_count_run = 0

        prev_state = curr_state

    cap.release()
    cv2.destroyAllWindows()
    print(f"[FIN] 종료. 저장 폴더: {save_dir if SAVE_ANNOTATED else '(저장 끔)'}")

if __name__ == "__main__":
    main()
