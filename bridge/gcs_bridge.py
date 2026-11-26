#!/usr/bin/env python3
"""
드론 수색 GCS 브리지 (PX4 전용) — 앱과 드론 사이를 잇습니다.

  패드의 GCS 앱  ⇄ WebSocket ⇄  gcs_bridge.py  ⇄ MAVLink ⇄  드론 (PX4)
    · 앱 → 드론 : 경로 업로드, 이륙·임무 시작, 복귀(RTL)
    · 드론 → 앱 : 위치·고도·배터리·모드·시동 상태 (0.5초마다), PX4 상태 메시지, 명령 결과

사용법
  1) 실기체 없이 시험
       python3 gcs_bridge.py --demo --lat 33.4996 --lng 126.5312

  2) 실기체 (지상 노트북 또는 패드의 Termux에서 실행)
       pip install pymavlink
       python3 gcs_bridge.py --mav udpin:0.0.0.0:14550          # UDP로 들어오는 텔레메트리
       python3 gcs_bridge.py --mav /dev/ttyUSB0,57600            # SiK 텔레메트리 무선 모뎀
     기체마다 MAV_SYS_ID를 1, 2, 3…으로 다르게 설정하세요 (최대 250). MAV_SYS_ID 번호 = 앱의 '드론 1, 2, 3…'.

  드론에 라즈베리파이를 달았을 때 (LTE로 텔레메트리 전달)
     PX4 TELEM2 ─(UART)─ 라즈베리파이 ─(LTE)─ 지상 브리지
       PX4 파라미터:  MAV_1_CONFIG = TELEM 2,  MAV_1_MODE = Onboard,  SER_TEL2_BAUD = 921600
       파이에서:      mavlink-routerd -e <지상 IP>:14550 /dev/serial0:921600
       지상에서:      python3 gcs_bridge.py --mav udpin:0.0.0.0:14550
     LTE는 통신사 NAT 때문에 지상으로 바로 들어오지 못하는 경우가 많습니다. 파이와 지상 기기를
     Tailscale 같은 VPN에 같이 넣고 <지상 IP> 자리에 VPN 주소를 쓰는 것이 가장 간단합니다.
     파이의 사진 촬영은 PX4가 보내는 CAMERA_TRIGGER 메시지에 맞춰 찍게 하세요.
       PX4 파라미터:  TRIG_INTERFACE = MAVLink,  TRIG_MODE = Distance (on command)

  4) 인터넷 없이 Wi-Fi로 (드론 1대, 파이에서 브리지 실행) — 사진·탐지도 Wi-Fi로 앱에 바로 보냄
       파이에서 (픽스호크 시리얼 하나를 브리지와 촬영 프로그램이 같이 쓰도록 mavlink-router로 나눔):
         mavlink-routerd -e 127.0.0.1:14550 -e 127.0.0.1:14540 /dev/serial0:921600
         python3 gcs_bridge.py --mav udpin:127.0.0.1:14550 --local /home/pi/drone-data/local --token 비밀번호
         python3 pi_drone.py --config config.json --local /home/pi/drone-data/local --mav udpin:127.0.0.1:14540
       파이 핫스팟:  sudo nmcli device wifi hotspot ssid DRONE1 password 8자이상비밀번호   (파이 주소 10.42.0.1)
       태블릿:    파이 핫스팟 Wi-Fi에 연결 → 앱 설정 → 연결 방식 '브리지' → 주소 ws://<파이 IP>:8766 (핫스팟이면 보통 ws://10.42.0.1:8766)
     사진 폴더 약속 (--local 폴더 = L)
       브리지 → 촬영 프로그램: L/mission.json  {"mission","grid","reshoot","drones":{"1":{"shot","cam","route","alt",...}}} (경로 업로드 때마다 새로 씀)
       촬영 프로그램 → 브리지: L/out/<임무>/photos.jsonl, detections.jsonl 에 한 줄씩 추가 (JSON 한 줄 = 사진/탐지 하나)
         사진  {"id","drone","lat","lng","alt","hdg","takenAt"(ms),"cell","tempAvgC","thumb","file","thermal"}
         탐지  {"id","drone","lat","lng","alt","tempC","takenAt"(ms),"cell","img"}
         thumb·file·thermal·img = L/out 아래 사진 파일 경로 (예: "M123/1/thumb/0001.jpg")
       앱은 사진 목록을 http://<파이 IP>:8766/photos?mission=&cell= 로, 사진은 /files/<경로> 로 받음 (비밀번호는 t=)

  5) 텔레메트리 무선기: 노트북에 꽂으면 --mav /dev/ttyUSB0,57600 (윈도우 COM3,57600).
     태블릿에 바로 꽂으면 브리지 없이 앱이 직접 통신 (앱 설정 → 연결 방식 '텔레메트리 무선기', web/mavlink-usb.js)
     라즈베리파이의 탐지 알림은 STATUSTEXT "DET <번호> <위도> <경도> <온도>" 로 무선기를 거쳐 옴 → 앱에 탐지로 보냄
     (PX4: 무선기 포트·파이 포트 모두 MAV_0_FORWARD = 1, MAV_1_FORWARD = 1)

  6) 노트북 브리지 (드론 ─ 텔레메트리 무선기 ─ 노트북 ─ Wi-Fi ─ 태블릿 앱)
       python gcs_bridge.py --mav COM3,57600 --pi http://dmdpi.local:8080 --qgc udpout:127.0.0.1:14550 --token 비밀번호
       --pi  : 파이 촬영 프로그램(pi_camera.py). 경로를 올릴 때 임무 정보를 보내고, 파이 Wi-Fi가 닿으면(이륙 전·착륙 뒤) 사진·탐지를 받아 옴
               비행 중 탐지 알림은 무선기(STATUSTEXT "DET ...")로 바로 옴
       --qgc : 같은 무선기를 노트북 QGroundControl에서도 보기 (QGC는 UDP 14550 자동 연결)

  3) 앱 연결: 설정 → 드론 연결 → '실시간' → 브리지 주소 → 연결
     비밀번호: 브리지를 --token 비밀번호 로 켜고(또는 환경 변수 GCS_BRIDGE_TOKEN), 앱의 '브리지 비밀번호'에 같은 값을 넣습니다.
       비밀번호가 맞아야 드론 위치를 받고 명령을 보낼 수 있습니다. 비밀번호 없이 켜면 같은 네트워크의 누구나 접속할 수 있습니다.
       같은 패드에서 실행(Termux 등):  ws://127.0.0.1:8766
       노트북 등 다른 기기에서 실행:   wss://<노트북 IP>:8766  (앱이 https이므로 TLS 필요)
     TLS 인증서 (한 번만):
       openssl req -x509 -newkey rsa:2048 -nodes -keyout key.pem -out cert.pem -days 365 -subj "/CN=gcs" \\
               -addext "subjectAltName=IP:<노트북 IP>"
       python3 gcs_bridge.py --mav udpin:0.0.0.0:14550 --cert cert.pem --key key.pem
     패드 Chrome에서 https://<노트북 IP>:8766 을 한 번 열어 '고급 → 계속'으로 인증서를 허용해 두세요.

업로드되는 임무 (기체별)
  이륙(NAV_TAKEOFF, 기체 현재 위치, takeoff 고도) → 속도(DO_CHANGE_SPEED)
  → 웨이포인트들(NAV_WAYPOINT, 점마다 출발지 대비 고도) — 첫 촬영 칸에서 거리 간격 촬영 시작(DO_SET_CAM_TRIGG_DIST),
    마지막 촬영 칸에서 촬영 중지 → 복귀(NAV_RETURN_TO_LAUNCH)
  앱이 지형 고도 자료로 점마다 고도를 정해 보냄: 칸 위에서는 그 주변 가장 높은 땅 + 설정 고도,
  출발지↔구역 사이(첫 칸 위·마지막 칸 위·출발지 위 점)는 가는 길의 가장 높은 땅 + 설정 고도.
  출발지보다 낮은 골짜기 위 웨이포인트는 상대고도가 음수가 될 수 있음 (PX4는 'below home' 경고만 냄)

이륙 순서 (PX4): 시동(ARM) → MISSION 모드. PX4는 시동이 걸린 상태에서만 미션 모드로 들어가며,
  착륙 상태에서 미션 모드가 되면 TAKEOFF 항목부터 실행합니다. 시동 후 일정 시간(COM_DISARM_PRFLT)
  안에 이륙하지 않으면 PX4가 시동을 끄므로 두 명령을 곧바로 이어 보냅니다.

안전 한도 (어기면 업로드 거부): 비행 고도 설정 5~150m(국내 비행 고도 상한, 땅 기준), 속도 1~15m/s,
  웨이포인트 600개 이하, 모든 점이 출발지 반경 5km 이내, 점마다 출발지 대비 고도 -300~600m, 이륙 고도 ≥ 비행 고도.
  실기체 첫 비행 전에 반드시 PX4 SITL(시뮬레이터)로 먼저 확인하고, 조종기로 즉시 수동 전환할 수 있게 준비하세요.
  QGroundControl을 함께 켜도 됩니다 (브리지는 MAVLink 시스템 번호 254를 써서 QGC의 255와 겹치지 않음).

메시지 형식 (JSON)
  드론→앱 {"type":"telemetry","id":1,"lat":..,"lng":..,"alt":상대고도m,"bat":%,"hdg":도,"spd":m/s,"mode":"MISSION","armed":true,"ts":기체시각(초)}
          {"type":"status","id":1,"text":"Preflight Fail: ...","sev":3}
          {"type":"detection","id":1,"lat":..,"lng":..,"alt":..,"temp":36.4,"uid":"...","mission":"M..","img":"사진 경로"}   (--local: 파이의 탐지)
          {"type":"photo","id":1,"uid":"...","mission":"M..","lat":..,"lng":..,"cell":12,"tempAvgC":18.2,"takenAt":ms}   (--local: 칸 평균 온도용)
  앱→드론 {"type":"upload","mission":"M..","drones":[{"id":1,"home":[lat,lng],"alt":100,"speed":8,"shot":39.1,
                   "takeoff":120.5,"cam":[1,40],"route":[[lat,lng,상대고도m],..]}]}
          (alt = 땅에서 비행 고도 설정값, takeoff = 이륙 고도, cam = 촬영 시작·끝 웨이포인트 번호. route 점에 고도가 없으면 alt, takeoff·cam이 없으면 예전 방식)
          {"type":"start","ids":[1,2]}   {"type":"rtl","ids":[1,2]}
  결과    {"type":"ack","op":"upload|start|rtl","id":1,"ok":true,"msg":"..."}

외부 패키지 없이 동작합니다 (실기체 연결에만 pymavlink 필요).
"""
import argparse, asyncio, base64, hashlib, hmac, json, math, os, queue, random, re, ssl, struct, sys, threading, time

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
LIMITS = {"alt": (5, 150), "speed": (1, 15), "points": 600, "radius": 5000, "rel": (-300, 600)}

lock = threading.Lock()
state = {}        # id -> 최신 텔레메트리
events = []       # 한 번만 보낼 메시지 (ack, 탐지)
clients = set()
vehicles = {}     # sysid -> {"conn", "autopilot", "armed", "comp", "seen"}
TRIG_PARAMS = {"TRIG_INTERFACE": (3, "MAVLink"), "TRIG_MODE": (4, "거리, 명령 시")}   # PX4가 미션 촬영 명령을 파이 카메라로 보내는 설정
trig_checked = set()   # (기체, 파라미터) 이미 확인함
links = {}        # --mav 연결별 상태 (포트 열림·받은 메시지 수·오류) → 앱에 'link'로 보냄
diag_at = [0.0]
STALE_S = 3.0     # 기체 신호가 이만큼 없으면 앱에 위치를 더 보내지 않음 → 앱이 '신호 없음'으로 표시
jobs = {}         # 연결 -> 작업 큐
demo = None       # 데모 모드 상태
TOKEN = ""        # 접속 비밀번호 (--token 또는 환경 변수 GCS_BRIDGE_TOKEN). 비어 있으면 누구나 접속


def ack(op, sid, ok, msg):
    print(f"[{op}] 드론 {sid}: {'성공' if ok else '실패'} · {msg}")
    with lock:
        events.append({"type": "ack", "op": op, "id": sid, "ok": ok, "msg": msg})


# ======================= WebSocket 서버 (표준 라이브러리) =======================
def ws_frame(text):
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        head = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x81, 126, n)
    else:
        head = struct.pack("!BBQ", 0x81, 127, n)
    return head + data


async def send_one(writer, obj):
    writer.write(ws_frame(json.dumps(obj, ensure_ascii=False)))
    await writer.drain()


async def ws_reader(reader, writer, peer):
    """앱 메시지를 읽음. 첫 메시지는 {"type":"auth","token":"…"}: 비밀번호가 맞아야 위치를 받고 명령을 보낼 수 있음"""
    buf, authed = b"", False
    while True:
        # 비밀번호 확인 전에는 10초 안에 메시지가 와야 함 (아무것도 안 보내고 연결만 붙잡는 것 방지)
        h = await (reader.readexactly(2) if authed else asyncio.wait_for(reader.readexactly(2), 10))
        fin, op, ln = h[0] & 0x80, h[0] & 0x0F, h[1] & 0x7F
        if ln == 126:
            ln = struct.unpack("!H", await reader.readexactly(2))[0]
        elif ln == 127:
            ln = struct.unpack("!Q", await reader.readexactly(8))[0]
        mask = await reader.readexactly(4) if h[1] & 0x80 else b"\0\0\0\0"
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(await reader.readexactly(ln)))
        if op == 0x8:
            return
        if op in (0x0, 0x1):
            buf += data
            if fin:
                try:
                    m = json.loads(buf.decode("utf-8"))
                except Exception as e:
                    print("[앱 메시지 오류]", e)
                    m = None
                buf = b""
                if isinstance(m, dict) and m.get("type") == "auth":
                    ok = not TOKEN or hmac.compare_digest(str(m.get("token", "")).encode(), TOKEN.encode())
                    if not ok:
                        await asyncio.sleep(1)   # 비밀번호 맞히기 시도를 느리게
                        print(f"[앱 연결 거부] {peer} 비밀번호가 맞지 않음")
                        await send_one(writer, {"type": "auth", "ok": False,
                                                "msg": "브리지 비밀번호가 맞지 않습니다" if m.get("token") else "브리지 비밀번호가 필요합니다"})
                        return
                    if not authed:
                        authed = True
                        clients.add(writer)
                        print(f"[앱 연결] {peer} (현재 {len(clients)}개)")
                    pi = await asyncio.get_running_loop().run_in_executor(None, pi_base) if PI else None
                    await send_one(writer, {"type": "auth", "ok": True, "pi": pi})   # pi: 파이 주소(.local을 IP로 바꾼 것) → 앱이 사진을 파이에서 직접 받을 때
                    if LOCAL or PI:   # 연결이 끊긴 동안 생긴 탐지·사진 (앱이 같은 것은 거름)
                        backlog = local_backlog()
                        if backlog:
                            await send_one(writer, backlog)
                elif not authed:
                    if TOKEN:
                        await send_one(writer, {"type": "auth", "ok": False, "msg": "브리지 비밀번호가 필요합니다"})
                        return
                    authed = True                        # 비밀번호 없이 켠 브리지 + 예전 앱
                    clients.add(writer)
                    print(f"[앱 연결] {peer} (현재 {len(clients)}개)")
                    if m is not None:
                        safe_app_message(m)
                elif m is not None:
                    safe_app_message(m)


async def handle(reader, writer):
    peer = writer.get_extra_info("peername")
    try:
        req = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 10)
        headers = {}
        for line in req.decode("latin-1").split("\r\n")[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        key = headers.get("sec-websocket-key")
        target = req.decode("latin-1").split("\r\n", 1)[0].split(" ")
        if not key and (LOCAL or PI) and len(target) > 1 and target[1].startswith(("/photos", "/detections", "/files/")):
            await local_http(writer, target[1])
            return
        if not key:  # 브라우저로 주소를 직접 연 경우 (인증서 허용용)
            body = "드론 수색 GCS 브리지가 실행 중입니다. 이제 앱에서 이 주소로 연결하세요.".encode("utf-8")
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\nContent-Length: "
                         + str(len(body)).encode() + b"\r\nConnection: close\r\n\r\n" + body)
            await writer.drain()
            return
        accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
        writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                      f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
        await writer.drain()
        await ws_reader(reader, writer, peer)
    except (asyncio.IncompleteReadError, ConnectionError, asyncio.TimeoutError, asyncio.LimitOverrunError):
        pass
    finally:
        if writer in clients:
            clients.discard(writer)
            print(f"[앱 연결 끊김] {peer} (현재 {len(clients)}개)")
        try:
            writer.close()
        except Exception:
            pass


async def broadcaster(interval):
    while True:
        await asyncio.sleep(interval)
        with lock:
            now = time.time()   # 이벤트(지나간 웨이포인트 위치 등)가 현재 위치보다 먼저. 신호가 끊긴 기체는 마지막 위치를 되풀이해 보내지 않음
            msgs = events[:] + [{k: x for k, x in v.items() if k != "_seen"} for v in state.values() if now - v.get("_seen", 0) < STALE_S]
            events.clear()
            if now - diag_at[0] >= 1 and (links or PI):   # 1초마다 무선기·파이 상태 → 앱 '드론 연결' 화면
                diag_at[0] = now
                msgs.append({"type": "link", "links": [dict(v, hb_ago=round(now - v["hb"], 1) if v["hb"] else None,
                                                             hint="" if v["hb"] and now - v["hb"] < 10 else link_hint(v)) for v in links.values()],
                             "pi": PI or "", "piOk": pi_state["ok"], "piErr": pi_state.get("err", ""), "mission": local_mission["id"],
                             "photos": len(local_photos.get(local_mission["id"], [])) if local_mission["id"] else 0})
        if not msgs or not clients:
            continue
        frame = ws_frame(json.dumps(msgs, ensure_ascii=False))
        for w in list(clients):
            try:
                w.write(frame)
                await w.drain()
            except Exception:
                clients.discard(w)


# ======================= Wi-Fi 사진 허브 (--local, 파이에서 실행) =======================
LOCAL = None          # 사진 폴더. 촬영 프로그램이 out/<임무>/photos.jsonl·detections.jsonl과 사진 파일을 씀
local_photos = {}     # 임무 -> [사진]
local_dets = {}       # 임무 -> [탐지 메시지]
local_seen = set()
local_mission = {"id": None, "at": 0.0}   # 지금 임무 번호, 경로를 올린 시각
CTYPE = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png", ".json": "application/json"}


def mission_doc(m):
    """촬영 프로그램이 볼 임무 정보: 임무 번호·칸 격자·촬영 간격·경로"""
    return {"mission": m.get("mission"), "grid": m.get("grid"), "reshoot": bool(m.get("reshoot")), "at": time.time(),
            "drones": {str(int(d.get("id", 0))): {k: d.get(k) for k in ("home", "alt", "speed", "shot", "takeoff", "cam", "route")}
                       for d in m.get("drones", [])}}


def local_write_mission(m):
    """앱이 경로를 올리면 (같은 파이 안의) 촬영 프로그램이 볼 mission.json을 씀"""
    doc = mission_doc(m)
    os.makedirs(LOCAL, exist_ok=True)
    tmp = os.path.join(LOCAL, "mission.json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False)
    os.replace(tmp, os.path.join(LOCAL, "mission.json"))   # 촬영 프로그램이 반쯤 쓴 파일을 읽지 않게
    local_mission["id"] = doc["mission"]
    print(f"[사진] 임무 {doc['mission']} 정보를 촬영 프로그램에 넘김 ({LOCAL}/mission.json)")


def local_line(mid, kind, v):
    uid = str(v.get("id") or "")
    key = (mid, kind, uid)
    if not uid or key in local_seen:
        return None
    try:
        drone = int(v.get("drone"))
        lat, lng = float(v["lat"]), float(v["lng"])
    except Exception:
        return None
    local_seen.add(key)
    cell = v.get("cell") if isinstance(v.get("cell"), int) else None
    if kind == "photos":
        p = {"uid": uid, "drone": drone, "lat": lat, "lng": lng, "alt": v.get("alt"), "hdg": v.get("hdg"), "takenAt": v.get("takenAt"),
             "cell": cell, "tempAvgC": v.get("tempAvgC"), "thumb": v.get("thumb"), "file": v.get("file"), "thermal": v.get("thermal"),
             "northUp": bool(v.get("northUp"))}
        local_photos.setdefault(mid, []).append(p)
        return {"type": "photo", "id": drone, "uid": uid, "mission": mid, "lat": lat, "lng": lng, "cell": cell,
                "tempAvgC": v.get("tempAvgC"), "takenAt": v.get("takenAt")}
    d = {"type": "detection", "id": drone, "uid": "L" + uid, "mission": mid, "lat": lat, "lng": lng, "alt": v.get("alt"),
         "temp": v.get("tempC", v.get("temp")), "deltaC": v.get("deltaC"), "kind": v.get("kind"), "takenAt": v.get("takenAt"), "img": v.get("img")}
    local_dets.setdefault(mid, []).append(d)
    return d


def local_tail():
    """촬영 프로그램이 jsonl에 새 줄을 쓰면 앱으로 보냄 (1초마다 확인, 끝까지 쓴 줄만)"""
    offs = {}
    while True:
        out = os.path.join(LOCAL, "out")
        try:
            mids = [d for d in os.listdir(out) if os.path.isdir(os.path.join(out, d))]
        except FileNotFoundError:
            mids = []
        for mid in mids:
            for kind in ("detections", "photos"):        # 탐지 먼저
                p = os.path.join(out, mid, kind + ".jsonl")
                try:
                    size = os.path.getsize(p)
                except OSError:
                    continue
                o = offs.get(p, 0)
                if size < o:
                    o = 0                                 # 파일을 새로 만든 경우
                if size == o:
                    continue
                with open(p, "rb") as f:
                    f.seek(o)
                    data = f.read(size - o)
                cut = data.rfind(b"\n") + 1
                offs[p] = o + cut
                msgs = []
                with lock:
                    for line in data[:cut].splitlines():
                        try:
                            m = local_line(mid, kind, json.loads(line))
                        except Exception:
                            m = None
                        if m:
                            msgs.append(m)
                    events.extend(msgs)
                    n = sum(1 for m in msgs if m["type"] == "detection")
                    if n:
                        print(f"[사진] 임무 {mid}: 탐지 {n}건")
        time.sleep(1)


# ----- 노트북 브리지 방식 (--pi): 드론은 텔레메트리 무선기로, 사진은 파이 Wi-Fi가 닿을 때 파이의 pi_camera.py에서 -----
QGC_URL = None
PI = None          # 파이 촬영 프로그램 주소, 예: http://dmdpi.local:8080
pi_state = {"ok": None, "base": None, "at": 0.0, "err": ""}


def pi_base():
    """파이 주소의 이름(예: dmdpi.local)을 IP로 바꾼 주소 (1분마다 다시 찾음).
    노트북(Windows)은 .local 이름을 찾지만 태블릿(안드로이드)은 못 찾는 경우가 많아, 앱에는 IP 주소로 알려 줌"""
    from urllib.parse import urlsplit
    import socket
    if pi_state["base"] and time.time() - pi_state["at"] < 60:
        return pi_state["base"]
    u = urlsplit(PI)
    try:
        ip = socket.gethostbyname(u.hostname)
        base = f"{u.scheme}://{ip}" + (f":{u.port}" if u.port else "")
        if base != pi_state["base"] and ip != u.hostname:
            print(f"[파이] {u.hostname} → {ip}")
    except OSError:
        base = PI   # 못 찾으면 이름 그대로 (나중에 다시)
    pi_state["base"], pi_state["at"] = base, time.time()
    return base


def pi_url(path, **q):
    from urllib.parse import urlencode
    if TOKEN:
        q["t"] = TOKEN
    return f"{pi_base()}{path}" + ("?" + urlencode(q) if q else "")


def pi_send_mission(doc, tries=90, every=10):
    """경로를 올릴 때 파이에 임무 정보를 보냄. 못 보내면 10초마다 다시 (15분, 더 새 경로를 올리면 멈춤) → 이륙 전에 파이가 Wi-Fi에 붙으면 받음.
    끝내 못 받아도 파이는 PX4 촬영 명령·거리로 찍음 (칸 번호 없이 위치로)"""
    import urllib.request

    def say(text, ok):
        print("[파이]", text)
        with lock:
            events.append({"type": "status", "id": 0, "text": text, "sev": 6 if ok else 4})

    for n in range(tries):
        if local_mission["id"] != doc["mission"]:   # 그사이 새 경로를 올림
            return
        try:
            req = urllib.request.Request(pi_url("/mission"), data=json.dumps(doc).encode("utf-8"), headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=6).read()
            return say(f"파이에 임무 정보 전달 ({doc['mission']}){' — 다시 시도해서 성공' if n else ''}", True)
        except Exception as e:
            if n == 0:
                code = getattr(e, "code", None)
                say(f"파이에 임무 정보를 못 보냈습니다 ({'비밀번호(TOKEN)가 파이와 다름' if code == 403 else e.__class__.__name__}) — "
                    "파이 Wi-Fi가 닿으면 자동으로 다시 보냅니다. 이륙 전에 확인하세요", False)
        time.sleep(every)
    say(f"파이에 임무 정보를 끝내 못 보냈습니다 ({doc['mission']}) — 사진은 찍히지만 칸 번호 없이 위치로 정리됩니다", False)


def pi_err(e):
    """파이에 못 닿은 이유를 쉬운 말로 (앱 '드론 연결' 화면·브리지 창에 표시)"""
    code = getattr(e, "code", None)
    t = f"{type(e).__name__}: {getattr(e, 'reason', e)}"
    if code == 403:
        return "비밀번호(TOKEN)가 파이와 다름 — laptop_bridge.bat의 TOKEN과 파이 config.env의 TOKEN을 같게"
    if code:
        return f"파이 응답 {code}"
    if any(k in t for k in ("getaddrinfo", "Name or service", "nodename", "11001", "11004")):
        return f"{PI}의 이름을 못 찾음 — laptop_bridge.bat의 PI에 파이 IP를 직접 (파이에서 hostname -I)"
    if "refused" in t.lower() or "10061" in t:
        return "파이는 닿지만 촬영 프로그램이 꺼져 있음 — 파이에서 sudo systemctl status drone-camera"
    return "파이 Wi-Fi가 닿지 않음 (비행 중이면 정상, 착륙 뒤 노트북·파이가 같은 Wi-Fi인지)"


def pi_poll():
    """5초마다 파이에 이번 임무의 사진·탐지 목록을 물어봄 (안 닿으면 조용히 넘어감) → 새 것만 앱에 보냄"""
    import urllib.request
    while True:
        time.sleep(5)
        mid = local_mission["id"]
        if not mid:   # 브리지를 다시 켠 경우: 파이가 갖고 있는 지금 임무(12시간 안)를 이어서
            try:
                doc = json.loads(urllib.request.urlopen(pi_url("/mission"), timeout=4).read())
                if not pi_state["ok"]:
                    print("[파이] Wi-Fi 연결됨 (임무를 올리면 사진·탐지를 받아 옵니다)")
                pi_state["ok"], pi_state["err"] = True, ""
                if doc.get("mission") and time.time() - float(doc.get("at") or 0) < 12 * 3600:
                    local_mission["id"] = mid = doc["mission"]
                    print(f"[파이] 진행 중이던 임무 {mid}를 이어서 받습니다")
            except Exception as e:
                err = pi_err(e)
                if pi_state.get("err") != err:
                    print(f"[파이] {PI}: {err}")
                pi_state["ok"], pi_state["err"] = False, err
            if not mid:
                continue
        try:
            got = {k: json.loads(urllib.request.urlopen(pi_url("/" + k, mission=mid, since=int(local_mission["at"] * 1000)), timeout=4).read()) for k in ("detections", "photos")}
        except Exception as e:
            err = pi_err(e)
            if pi_state.get("err") != err:
                print(f"[파이] {err} — 닿으면 이어서 받음")
            pi_state["ok"], pi_state["err"] = False, err
            continue
        if not pi_state["ok"]:
            print("[파이] Wi-Fi 연결됨 — 사진·탐지를 받아 옵니다")
        pi_state["ok"], pi_state["err"] = True, ""
        n = sum(1 for v in got["photos"] if str(v.get("id") or "") and (mid, "photos", str(v.get("id"))) not in local_seen)
        if n:
            print(f"[파이] 임무 {mid}: 새 사진 {n}장 받음 (모두 {len(got['photos'])}장)")
        with lock:
            msgs = [x for k in ("detections", "photos") for v in got[k] for x in [local_line(mid, k, v)] if x]
            events.extend(msgs)


def pi_file(rel):
    import urllib.request
    from urllib.parse import quote
    with urllib.request.urlopen(pi_url("/files/" + quote(rel)), timeout=10) as r:
        return r.read()


def local_backlog():
    mid = local_mission["id"]
    if not mid:
        return None
    with lock:
        dets = list(local_dets.get(mid, []))
        phs = [{"type": "photo", "id": p["drone"], "uid": p["uid"], "mission": mid, "lat": p["lat"], "lng": p["lng"], "cell": p["cell"],
                "tempAvgC": p["tempAvgC"], "takenAt": p["takenAt"]} for p in local_photos.get(mid, [])]
    return dets + phs or None


async def local_http(writer, target):
    """/photos?mission=&cell= → 사진 목록(JSON), /files/<경로> → 사진 파일. 비밀번호는 t="""
    from urllib.parse import urlsplit, parse_qs, unquote
    u = urlsplit(target)
    q = {k: v[0] for k, v in parse_qs(u.query).items()}

    def reply(code, body, ctype="application/json; charset=utf-8", extra=""):
        head = (f"HTTP/1.1 {code}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
                f"Access-Control-Allow-Origin: *\r\n{extra}Connection: close\r\n\r\n")
        writer.write(head.encode() + body)

    def reply_list(lst):
        reply("200 OK", json.dumps(lst, ensure_ascii=False).encode("utf-8"), extra="Cache-Control: no-store\r\n")

    if TOKEN and not hmac.compare_digest(q.get("t", "").encode(), TOKEN.encode()):
        reply("403 Forbidden", b'{"error":"token"}')
    elif u.path == "/detections":   # 무선기 방식: 무선기로 먼저 받은 탐지에 사진을 붙일 때
        with lock:
            lst = list(local_dets.get(q.get("mission", ""), []))
        reply_list(lst)
    elif u.path == "/photos":
        mid = q.get("mission", "")
        try:
            cell = int(q["cell"]) if "cell" in q else None
        except ValueError:
            cell = None
        with lock:
            lst = [p for p in local_photos.get(mid, []) if cell is None or p["cell"] in (cell, None)]
        reply_list(lst)
    elif not LOCAL:   # --pi: 사진 파일은 파이에서 가져다 줌
        rel = unquote(u.path[len("/files/"):])
        try:
            data = await asyncio.get_running_loop().run_in_executor(None, pi_file, rel)
            reply("200 OK", data, CTYPE.get(os.path.splitext(rel)[1].lower(), "application/octet-stream"), "Cache-Control: max-age=86400\r\n")
        except Exception:
            reply("504 Gateway Timeout", b'{"error":"pi unreachable"}')
    else:
        root = os.path.realpath(os.path.join(LOCAL, "out"))
        path = os.path.realpath(os.path.join(root, unquote(u.path[len("/files/"):])))
        if not path.startswith(root + os.sep) or not os.path.isfile(path):   # 폴더 밖(../) 접근 막음
            reply("404 Not Found", b'{"error":"not found"}')
        else:
            with open(path, "rb") as f:
                data = f.read()
            reply("200 OK", data, CTYPE.get(os.path.splitext(path)[1].lower(), "application/octet-stream"), "Cache-Control: max-age=86400\r\n")
    await writer.drain()


# ======================= 앱 명령 처리 =======================
def route_of(d):
    """[(위도, 경도, 출발지 대비 고도)], 이륙 고도, (촬영 시작, 끝) 번호. 고도·takeoff·cam이 없으면 예전 방식(모두 alt)"""
    alt = float(d["alt"])
    pts = [(float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else alt) for p in d["route"]]
    cam = d.get("cam") or (0, len(pts) - 1)
    return pts, float(d.get("takeoff", alt)), (int(cam[0]), int(cam[1]))


def check_route(d):
    """업로드 전 안전 검사. 문제가 있으면 이유(문자열), 없으면 None"""
    try:
        alt, spd, route = float(d["alt"]), float(d["speed"]), d["route"]
        hlat, hlng = map(float, d["home"])
        pts, takeoff, (i0, i1) = route_of(d)
    except Exception:
        return "임무 형식이 올바르지 않습니다"
    if not LIMITS["alt"][0] <= alt <= LIMITS["alt"][1]:
        return f"고도 {alt:.0f}m가 허용 범위({LIMITS['alt'][0]}~{LIMITS['alt'][1]}m)를 벗어납니다"
    if not LIMITS["speed"][0] <= spd <= LIMITS["speed"][1]:
        return f"속도 {spd}m/s가 허용 범위({LIMITS['speed'][0]}~{LIMITS['speed'][1]}m/s)를 벗어납니다"
    if not route:
        return "웨이포인트가 없습니다"
    if len(route) > LIMITS["points"]:
        return f"웨이포인트 {len(route)}개가 한도({LIMITS['points']}개)를 넘습니다"
    zlo, zhi = LIMITS["rel"]
    if not alt <= takeoff <= zhi:
        return f"이륙 고도 {takeoff:.0f}m가 허용 범위({alt:.0f}~{zhi}m)를 벗어납니다"
    if not 0 <= i0 <= i1 < len(pts):
        return "촬영 시작·끝 웨이포인트 번호가 올바르지 않습니다"
    mlng = m_per_lng(hlat)
    for la, lo, z in pts:
        if math.hypot((la - hlat) * 111320, (lo - hlng) * mlng) > LIMITS["radius"]:
            return f"출발지에서 {LIMITS['radius'] / 1000:.0f}km를 넘는 웨이포인트가 있습니다"
        if not zlo <= z <= zhi:
            return f"웨이포인트 고도 {z:.0f}m(출발지 대비)가 허용 범위({zlo}~{zhi}m)를 벗어납니다"
    return None


def safe_app_message(m):
    """앱 명령 하나가 잘못돼도(번호·형식 오류) 앱 연결은 끊지 않고 알림만"""
    try:
        on_app_message(m)
    except Exception as e:
        op = m.get("type") if isinstance(m, dict) else "?"
        print(f"[앱 명령 오류] {op}: {type(e).__name__}: {e}")
        with lock:
            events.append({"type": "status", "text": f"브리지: '{op}' 명령을 처리하지 못했습니다 ({type(e).__name__})"})


def on_app_message(m):
    if not isinstance(m, dict):
        return
    op = m.get("type")
    if op == "upload":
        print(f"[upload] 임무 {m.get('mission')} · 드론 {len(m.get('drones', []))}기")
        if LOCAL:
            local_write_mission(m)
        if PI:
            local_mission["id"], local_mission["at"] = m.get("mission"), time.time()
            threading.Thread(target=pi_send_mission, args=(mission_doc(m),), daemon=True).start()
        for d in m.get("drones", []):
            sid = int(d.get("id", 0))
            err = check_route(d)
            if err:
                ack("upload", sid, False, err)
            elif demo is not None:
                demo_upload(sid, d)
            else:
                dispatch(sid, ("upload", d))
    elif op in ("start", "rtl"):
        for sid in m.get("ids", []):
            sid = int(sid)
            if demo is not None:
                (demo_start if op == "start" else demo_rtl)(sid)
            else:
                dispatch(sid, (op, None))


def dispatch(sid, job):
    v = vehicles.get(sid)
    if not v:
        return ack(job[0], sid, False, "이 번호(SYSID)의 기체 신호가 없습니다")
    if time.time() - v.get("seen", 0) > STALE_S:
        return ack(job[0], sid, False, f"기체 신호가 끊겼습니다 ({time.time() - v.get('seen', 0):.0f}초째 하트비트 없음)")
    jobs[v["conn"]].put((sid, job))


# ======================= MAVLink (pymavlink, PX4) =======================
GCS_SYSID = 254                     # QGroundControl(255)과 겹치지 않게
MAX_SYSID = 250                     # 드론 번호로 쓸 수 있는 범위 (1~250)
MAV_TYPE_GCS, MAV_AUTOPILOT_PX4 = 6, 12
FRAME_MISSION, FRAME_REL_ALT = 2, 3  # QGC와 같은 프레임 (PX4는 INT/일반 둘 다 받음)
CMD = {"WAYPOINT": 16, "RTL": 20, "TAKEOFF": 22, "SET_MODE": 176, "SPEED": 178, "CAM_DIST": 206, "ARM": 400}
PX4_MODE = {"MISSION": (1, 4, 4), "RTL": (1, 4, 5), "LOITER": (1, 4, 3)}   # (base_mode, main_mode, sub_mode)
MAV_RESULT = {1: "일시 거부", 2: "거부", 3: "지원 안 함", 4: "실패", 5: "진행 중", 6: "취소됨"}
MISSION_RESULT = {1: "오류", 2: "지원 안 하는 프레임", 3: "지원 안 하는 명령", 4: "저장 공간 부족(SD카드 확인)",
                  5: "잘못된 값", 13: "순서 오류", 15: "거부됨"}
NAN = float("nan")                   # PX4: 방향(yaw) NaN = 현재 방향 유지
DET_RE = re.compile(r"^DET\s+(\S+)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)(?:\s+(-?\d+(?:\.\d+)?))?(?:\s+([PF]))?")
# "DET <번호> <위도> <경도> <온도> [주변보다 +°C] [P=사람 후보·F=불]" (뒤 두 칸은 새 촬영 프로그램만)
MAV_AUTOPILOT_INVALID = 8            # 비행 제어기가 아닌 장치(카메라·짐벌·라즈베리파이 등)의 하트비트
last_hb = {}                         # 연결 -> 마지막 GCS 하트비트 시각
QGC = {"out": None, "src": None, "str": None}     # --qgc: 무선기 하나를 QGroundControl과 같이 씀 (드론 메시지를 그대로 넘기고, QGC 명령을 드론에)


def qgc_pump(conn):
    q = QGC["out"]
    if not q or conn is not QGC["src"]:
        return
    for _ in range(50):
        try:
            m = q.recv_msg()
        except Exception:
            return
        if m is None:
            return
        if m.get_type() != "BAD_DATA":
            conn.write(m.get_msgbuf())


def gcs_heartbeat(conn):
    """1초마다 GCS 하트비트. 업로드처럼 오래 걸리는 작업 중에도 보내야 PX4가 '지상국 끊김'으로 판단하지 않음"""
    qgc_pump(conn)
    now = time.time()
    if now - last_hb.get(conn, 0) >= 1:
        conn.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb[conn] = now


def for_us(msg):
    """QGC 등 다른 지상국에 보낸 응답(임무 요청·명령 응답)은 무시. target_system이 없거나 0이면 우리 것으로 봄"""
    return getattr(msg, "target_system", 0) in (0, GCS_SYSID)


def tel(sid):
    """기체별 텔레메트리 항목 (없으면 만듦). lock 안에서 부름"""
    return state.setdefault(sid, {"type": "telemetry", "id": sid})


def m_per_lng(lat):
    """그 위도에서 경도 1도의 길이(m)"""
    return 111320 * math.cos(math.radians(lat))


def handle_msg(conn, msg, mavutil):
    if QGC["out"] and conn is QGC["src"] and msg.get_type() != "BAD_DATA":
        try:
            QGC["out"].write(msg.get_msgbuf())   # 드론 메시지를 QGroundControl에도 그대로
        except Exception:
            pass
    sid = msg.get_srcSystem()
    if not 1 <= sid <= MAX_SYSID:
        return
    t = msg.get_type()
    with lock:
        if t == "HEARTBEAT":
            # 지상국, 그리고 같은 기체 번호를 쓰는 카메라·짐벌·라즈베리파이 하트비트는 무시
            # (이걸 비행 제어기로 알면 'PX4 기체가 아닙니다'로 명령이 거부되고 모드·시동 표시가 흔들림)
            if msg.type == MAV_TYPE_GCS or msg.autopilot == MAV_AUTOPILOT_INVALID:
                return
            armed = bool(msg.base_mode & 128)
            if sid not in vehicles:
                print(f"[MAVLink] 기체 {sid}번 연결됨 ({'PX4' if msg.autopilot == 12 else f'autopilot {msg.autopilot} — PX4가 아니면 명령은 거부됨'})")
                if msg.autopilot == MAV_AUTOPILOT_PX4:   # 미션의 촬영 명령이 파이 카메라까지 가는 설정인지 물어봄 (답은 PARAM_VALUE)
                    for p in TRIG_PARAMS:
                        try:
                            conn.mav.param_request_read_send(sid, msg.get_srcComponent(), p.encode(), -1)
                        except Exception:
                            pass
            vehicles[sid] = {"conn": conn, "autopilot": msg.autopilot, "armed": armed, "comp": msg.get_srcComponent(), "seen": time.time()}
            d = tel(sid)
            d["armed"] = armed
            d["_seen"] = time.time()
            try:
                d["mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                pass
        elif t == "GLOBAL_POSITION_INT":
            d = tel(sid)
            d["_seen"] = time.time()
            d.update(lat=msg.lat / 1e7, lng=msg.lon / 1e7, alt=round(msg.relative_alt / 1000, 1),
                     spd=round(math.hypot(msg.vx, msg.vy) / 100, 1), ts=msg.time_boot_ms / 1000)
            if msg.hdg != 65535:
                d["hdg"] = msg.hdg / 100
        elif t in ("SYS_STATUS", "BATTERY_STATUS"):
            if msg.battery_remaining >= 0:
                tel(sid)["bat"] = msg.battery_remaining
            if t == "SYS_STATUS" and 0 < msg.voltage_battery < 65535:
                tel(sid)["volt"] = round(msg.voltage_battery / 1000, 2)   # 배터리 % 를 모를 때도 전압은 보임
        elif t == "GPS_RAW_INT":   # GPS 상태: 위치가 아직 없을 때(실내·켠 직후) 앱에 'GPS 대기 · 위성 n개'로 보여 줌
            tel(sid).update(fix=msg.fix_type, sats=msg.satellites_visible if msg.satellites_visible != 255 else None)
        elif t == "PARAM_VALUE":   # 촬영 설정 확인 (QGC가 파라미터를 받을 때도 같이 옴)
            pid = msg.param_id.decode("ascii", "ignore") if isinstance(msg.param_id, bytes) else str(msg.param_id)
            pid = pid.strip("\0 ")
            want = TRIG_PARAMS.get(pid)
            key = (sid, pid)
            if want is not None and key not in trig_checked:
                trig_checked.add(key)
                v = int(round(msg.param_value))
                if v != want[0]:
                    text = (f"PX4 {pid}={v} → 미션의 촬영 명령이 파이 카메라로 안 갑니다. QGC 파라미터에서 {pid}={want[0]}({want[1]})로 바꾸고 재부팅하세요 "
                            "(그동안 파이는 위치를 보고 직접 찍음)")
                    print(f"[MAVLink] 기체 {sid}번: {text}")
                    events.append({"type": "status", "id": sid, "text": text, "sev": 4})
                else:
                    print(f"[MAVLink] 기체 {sid}번: {pid}={v} 확인 (촬영 명령 → 파이 카메라)")
        elif t == "STATUSTEXT":    # 사전 점검 실패 등 PX4 안내 문구
            text = msg.text.decode("utf-8", "replace") if isinstance(msg.text, bytes) else str(msg.text)
            text = text.strip("\0 ").strip()
            det = DET_RE.match(text)   # 라즈베리파이의 탐지 알림 (픽스호크를 거쳐 무선기로 옴)
            if det:
                events.append({"type": "detection", "id": sid, "uid": "L" + det.group(1), "lat": float(det.group(2)), "lng": float(det.group(3)),
                               "temp": float(det.group(4)), "alt": state.get(sid, {}).get("alt"),
                               "deltaC": float(det.group(5)) if det.group(5) else None, "kind": {"P": "person", "F": "fire"}.get(det.group(6))})
            else:
                events.append({"type": "status", "id": sid, "text": text, "sev": msg.severity})


def wait_for(conn, sid, types, mavutil, pred=None, timeout=4.0):
    """원하는 메시지가 올 때까지 기다리면서 그 사이 텔레메트리도 계속 갱신"""
    end = time.time() + timeout
    while time.time() < end:
        gcs_heartbeat(conn)
        msg = conn.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        handle_msg(conn, msg, mavutil)
        if msg.get_type() in types and msg.get_srcSystem() == sid and for_us(msg) and (pred is None or pred(msg)):
            return msg
    return None


def build_items(d):
    hlat, hlng = map(float, d["home"])
    spd, shot = float(d["speed"]), float(d.get("shot", 0))
    pts, takeoff, (i0, i1) = route_of(d)
    items = [(FRAME_REL_ALT, CMD["TAKEOFF"], 0, 0, 0, NAN, hlat, hlng, takeoff),  # (frame, cmd, p1~p4, lat, lng, alt)
             (FRAME_MISSION, CMD["SPEED"], 1, spd, -1, 0, 0, 0, 0)]               # 1 = 대지 속도
    for i, (la, lo, z) in enumerate(pts):
        items.append((FRAME_REL_ALT, CMD["WAYPOINT"], 0, 0, 0, NAN, la, lo, z))
        if shot > 0 and i == i0:
            items.append((FRAME_MISSION, CMD["CAM_DIST"], shot, 0, 1, 0, 0, 0, 0))   # 첫 촬영 칸부터 shot m마다 촬영, 시작 즉시 1장
        if shot > 0 and i == i1:
            items.append((FRAME_MISSION, CMD["CAM_DIST"], 0, 0, 0, 0, 0, 0, 0))      # 마지막 촬영 칸에서 촬영 중지
    items.append((FRAME_MISSION, CMD["RTL"], 0, 0, 0, 0, 0, 0, 0))
    return items


def comp_of(sid):
    return vehicles.get(sid, {}).get("comp", 1)   # 비행 제어기 컴포넌트 (PX4는 보통 1)


def job_upload(conn, sid, d, mavutil):
    items = build_items(d)
    n, comp = len(items), comp_of(sid)
    for attempt in range(3):
        conn.mav.mission_count_send(sid, comp, n, 0)
        sent = set()
        while True:
            msg = wait_for(conn, sid, ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"), mavutil, timeout=3)
            if msg is None:
                break                                   # 응답 없음 → 처음부터 다시
            if msg.get_type() == "MISSION_ACK":
                if msg.type == 0 and len(sent) == n:
                    return True, f"{n}개 항목 업로드 완료 (웨이포인트 {len(d['route'])}개)"
                if msg.type != 0:
                    return False, f"PX4가 임무를 거부했습니다 ({MISSION_RESULT.get(msg.type, msg.type)})"
                continue
            seq = msg.seq
            if not 0 <= seq < n:
                continue
            fr, cmd, p1, p2, p3, p4, la, lo, al = items[seq]
            if msg.get_type() == "MISSION_REQUEST_INT":   # 요청한 형식 그대로 답함
                conn.mav.mission_item_int_send(sid, comp, seq, fr, cmd, 0, 1, p1, p2, p3, p4,
                                               int(round(la * 1e7)), int(round(lo * 1e7)), al, 0)
            else:
                conn.mav.mission_item_send(sid, comp, seq, fr, cmd, 0, 1, p1, p2, p3, p4, la, lo, al, 0)
            sent.add(seq)
        print(f"[upload] 드론 {sid}: 응답 없음, 재시도 {attempt + 1}/3")
    return False, "기체 응답 없음 (3회 시도)"


def command(conn, sid, cmd, mavutil, *p):
    p = list(p) + [0] * (7 - len(p))
    conn.mav.command_long_send(sid, comp_of(sid), cmd, 0, *p)
    a = wait_for(conn, sid, ("COMMAND_ACK",), mavutil, pred=lambda x: x.command == cmd, timeout=4)
    if a is None:
        return False, "응답 없음"
    return a.result == 0, MAV_RESULT.get(a.result, str(a.result))


def set_mode(conn, sid, name, mavutil):
    base, main, sub = PX4_MODE[name]
    return command(conn, sid, CMD["SET_MODE"], mavutil, base, main, sub)


def job_start(conn, sid, mavutil):
    ok, r = command(conn, sid, CMD["ARM"], mavutil, 1)
    if not ok:
        return False, f"시동 거부 ({r}) · PX4 사전 점검 메시지를 확인하세요 (GPS, 배터리, 안전 스위치, 원격 조종기)"
    ok, r = set_mode(conn, sid, "MISSION", mavutil)
    if not ok:
        command(conn, sid, CMD["ARM"], mavutil, 0)     # 미션 모드 실패 시 시동을 다시 끔
        return False, f"미션 모드 전환 실패 ({r}) · 시동을 껐습니다"
    return True, "시동 · 미션 모드 · 이륙"


def job_rtl(conn, sid, mavutil):
    ok, r = set_mode(conn, sid, "RTL", mavutil)
    return (True, "복귀 모드로 전환") if ok else (False, f"복귀 전환 실패 ({r})")


JOBS = {"upload": lambda c, s, d, mu: job_upload(c, s, d, mu),
        "start": lambda c, s, d, mu: job_start(c, s, mu),
        "rtl": lambda c, s, d, mu: job_rtl(c, s, mu)}


def open_error(conn_str, e):
    """포트를 못 열 때 원인을 쉬운 말로 (Windows 기준)"""
    port = conn_str.split(",")[0]
    t = f"{type(e).__name__}: {e}"
    if any(k in t for k in ("PermissionError", "Access is denied", "액세스", "거부", "busy", "Resource busy")):
        return (f"{port}을(를) 다른 프로그램이 쓰고 있습니다. QGroundControl이 무선기에 바로 붙었을 가능성이 큽니다 → "
                "QGC 설정(Application Settings) → 일반 → 자동 연결(AutoConnect)에서 'SiK Radio'·'Pixhawk' 체크를 끄고 QGC를 다시 켜세요. "
                "QGC는 이 브리지가 UDP 14550으로 넘겨 주는 데이터로 자동 연결됩니다.")
    if any(k in t for k in ("FileNotFoundError", "cannot find", "지정된 파일", "No such file", "could not open port")):
        return f"{port}이(가) 없습니다. 무선기 USB가 꽂혀 있는지, 장치 관리자 → 포트(COM 및 LPT)의 번호가 laptop_bridge.bat의 COM과 같은지 확인하세요."
    return f"{port}을(를) 열 수 없습니다 ({t})"


def link_diag(conn_str):
    """앱 '드론 연결' 화면에 보여 줄 무선기 상태 (lock 안에서 부름)"""
    return links.setdefault(conn_str, {"port": conn_str, "open": False, "err": "", "msgs": 0, "bad": 0, "hb": 0, "since": time.time()})


def mav_worker(conn_str):
    os.environ.setdefault("MAVLINK20", "1")   # PX4는 MAVLink 2 (임무 종류·명령 응답 대상 같은 확장 필드가 v1에서는 빠짐)
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink가 없습니다. 'pip install pymavlink' 후 다시 실행하세요.")
    kw = {"source_system": GCS_SYSID}
    last_err = None
    while True:   # 포트가 안 열리거나(QGC가 먼저 잡음·USB 빠짐) 도중에 끊겨도 3초마다 다시 시도
        try:
            if "," in conn_str and not conn_str.startswith(("udp", "tcp")):
                dev, baud = conn_str.split(",", 1)
                conn = mavutil.mavlink_connection(dev, baud=int(baud), **kw)
            else:
                conn = mavutil.mavlink_connection(conn_str, **kw)
        except Exception as e:
            err = open_error(conn_str, e)
            with lock:
                link_diag(conn_str).update(open=False, err=err)
            if err != last_err:
                print(f"[MAVLink] ❌ {err}\n[MAVLink]    3초마다 다시 시도합니다.")
                last_err = err
            time.sleep(3)
            continue
        last_err = None
        with lock:
            link_diag(conn_str).update(open=True, err="", since=time.time())
        try:
            mav_loop(conn_str, conn, mavutil)
        except Exception as e:
            print(f"[MAVLink] {conn_str} 끊김 ({e}) — 다시 연결합니다")
            with lock:
                link_diag(conn_str).update(open=False, err=f"무선기 연결이 끊겼습니다 ({e}). USB를 확인하세요.")
            for sid in [k for k, v in vehicles.items() if v.get("conn") is conn]:
                vehicles.pop(sid, None)
            jobs.pop(conn, None)
            try:
                conn.close()
            except Exception:
                pass
            time.sleep(3)


def mav_loop(conn_str, conn, mavutil):
    jobs[conn] = queue.Queue()
    if QGC_URL and QGC.get("str") in (None, conn_str):   # QGC는 첫 번째 무선기와 짝 (끊겼다 다시 열려도 새 연결로 이어 줌)
        if QGC["out"] is None:
            QGC["out"] = mavutil.mavlink_connection(QGC_URL, source_system=GCS_SYSID)
            print(f"[MAVLink] QGroundControl로도 보냄: {QGC_URL}")
        QGC["src"], QGC["str"] = conn, conn_str
    print(f"[MAVLink] {conn_str} 열림 · 기체 신호 기다리는 중")
    warned = time.time()
    while True:
        gcs_heartbeat(conn)   # PX4 데이터 링크 끊김 페일세이프 판단에 쓰임
        try:
            sid, (op, data) = jobs[conn].get_nowait()
            if vehicles.get(sid, {}).get("autopilot") != MAV_AUTOPILOT_PX4:
                ok, msg = False, "PX4 기체가 아닙니다 (이 브리지는 PX4 전용)"
            else:
                try:
                    ok, msg = JOBS[op](conn, sid, data, mavutil)
                except Exception as e:
                    ok, msg = False, f"오류: {e}"
            ack(op, sid, ok, msg)
        except queue.Empty:
            pass
        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            with lock:
                d = link_diag(conn_str)
                if msg.get_type() == "BAD_DATA":
                    d["bad"] += 1
                else:
                    d["msgs"] += 1
                    if msg.get_type() == "HEARTBEAT" and getattr(msg, "autopilot", 0) != MAV_AUTOPILOT_INVALID:
                        d["hb"] = time.time()
            handle_msg(conn, msg, mavutil)
        if time.time() - warned > 10:   # 10초마다: 기체 하트비트가 없으면 이유를 짐작해 알려 줌
            warned = time.time()
            with lock:
                d = dict(link_diag(conn_str))
            if time.time() - d["hb"] > 10:
                print(f"[MAVLink] ⚠ {conn_str}: 10초 동안 기체 신호 없음 · {link_hint(d)}")


def link_hint(d):
    if d["msgs"] == 0 and d["bad"] == 0:
        return "받은 데이터 0 → 드론 전원·무선기 짝(두 무선기 초록불 켜짐)·드론 쪽 무선기 연결(TELEM1) 확인"
    if d["msgs"] == 0:
        return f"깨진 데이터 {d['bad']}개 → 속도가 다름: laptop_bridge.bat의 BAUD(57600)와 무선기 속도 확인"
    return f"데이터 {d['msgs']}개는 오지만 비행 제어기 하트비트 없음 → PX4 MAV_0_CONFIG = TELEM 1 확인"


# ======================= 데모 드론 =======================
def demo_upload(sid, d):
    with lock:
        known = sid in demo["drones"]   # 데모에 없는 기체 번호 (실기체로 치면 연결 안 된 드론) → 실패 응답
    if not known:
        return ack("upload", sid, False, f"데모: 드론 {sid}번이 없습니다 (--drones {len(demo['drones'])})")
    with lock:
        la, lo = demo["home"]
        mlng = m_per_lng(la)
        route, takeoff, _ = route_of(d)
        pts = [(0.0, 0.0, 0.0), (0.0, 0.0, takeoff)] + [((p[1] - lo) * mlng, (p[0] - la) * 111320, p[2]) for p in route]
        if len(d["route"][-1]) < 3:
            pts.append((0.0, 0.0, pts[-1][2]))              # 예전 형식: 마지막 칸 → 출발지 위
        pts.append((0.0, 0.0, 0.0))                         # 착륙
        demo["drones"][sid] = {"route": pts, "t0": None, "speed": float(d["speed"]),
                               "rtl": False, "prev": 0.0, "flown": 0.0, "sent": 0, "hdg": 0.0}
    ack("upload", sid, True, f"데모: 웨이포인트 {len(d['route'])}개 업로드 완료")


def demo_start(sid):
    with lock:
        v = demo["drones"].get(sid)
        ok = bool(v and v.get("route") and v["t0"] is None)
        if ok:
            v["t0"] = time.time()
    ack("start", sid, ok, "데모: 시동 · 미션 모드 · 이륙" if ok else "업로드된 경로가 없거나 이미 비행 중입니다")


def demo_rtl(sid):
    with lock:
        v = demo["drones"].get(sid)
        ok = bool(v and v.get("t0") and not v.get("done") and v.get("pos"))
        if ok:
            x, y, z = v["pos"]
            v.update(route=[(x, y, z), (0.0, 0.0, z), (0.0, 0.0, 0.0)], t0=time.time(), rtl=True,
                     prev=v["prev"] + v["flown"], flown=0.0, sent=0)
    ack("rtl", sid, ok, "데모: 복귀 모드로 전환" if ok else "비행 중이 아닙니다")


def demo_worker(n, accel):
    with lock:
        for k in range(1, n + 1):
            demo["drones"][k] = {"route": None, "t0": None}
    while True:
        out, now = {}, time.time()
        with lock:
            la, lo = demo["home"]
            mlng = m_per_lng(la)
            for k, v in demo["drones"].items():
                tel = {"type": "telemetry", "id": k, "lat": la, "lng": lo, "alt": 0.0, "bat": 100.0, "hdg": 0.0,
                       "spd": 0, "mode": "STANDBY", "armed": False, "ts": round(now * accel, 2), "_seen": now}
                if v.get("route") and v.get("t0") is not None:
                    # 실제 PX4처럼: 제자리 수직 이륙 → 점마다 정해진 고도로 경로 비행(고도가 바뀌면 상승 3m/s·하강 1.5m/s)
                    # → 출발지 위에서 수직 착륙. 구간 시간 = 수평·수직 중 오래 걸리는 쪽
                    pts, spd = v["route"], v["speed"]
                    dur = [max(math.dist(pts[i][:2], pts[i + 1][:2]) / spd,
                               (pts[i + 1][2] - pts[i][2]) / 3.0 if pts[i + 1][2] > pts[i][2] else (pts[i][2] - pts[i + 1][2]) / 1.5)
                           for i in range(len(pts) - 1)]
                    total, el = sum(dur), (now - v["t0"]) * accel
                    tt, i = min(el, total), 0
                    while i < len(dur) - 1 and tt > dur[i]:
                        tt -= dur[i]
                        i += 1
                    a, b = pts[i], pts[i + 1]
                    f = min(1.0, tt / dur[i]) if dur[i] else 1.0
                    x, y, alt = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)
                    landed = el >= total
                    v["flown"] = min(el, total)
                    # 지나간 웨이포인트 위치도 보냄 (배속 시험에서 한 번에 수십 m씩 움직여도 모서리를 건너뛰지 않게)
                    passed = len(pts) - 1 if landed else i
                    cum = 0.0
                    for j in range(1, passed + 1):
                        cum += dur[j - 1]
                        if j > v.get("sent", 0):
                            events.append({"type": "telemetry", "id": k, "lat": round(la + pts[j][1] / 111320, 7), "lng": round(lo + pts[j][0] / mlng, 7),
                                           "alt": round(pts[j][2], 1), "spd": spd, "armed": j < len(pts) - 1,   # 모드는 빼서 앱이 현재 모드를 유지
                                           "ts": round(v["t0"] * accel + cum, 2)})
                    v["sent"] = max(v.get("sent", 0), passed)
                    v["pos"], v["done"] = (x, y, alt), landed
                    if math.dist(a[:2], b[:2]) > 0.01:
                        v["hdg"] = (math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) + 360) % 360
                    last = len(pts) - 2
                    tel.update(lat=round(la + y / 111320, 7), lng=round(lo + x / mlng, 7), alt=round(0.0 if landed else alt, 1),
                               bat=round(max(5.0, 100 - (v["prev"] + v["flown"]) / 12), 1), hdg=round(v["hdg"], 1),
                               spd=0 if landed else spd, armed=not landed,
                               mode="LAND" if landed or i >= last else ("TAKEOFF" if i == 0 and not v["rtl"] else "RTL" if v["rtl"] else "MISSION"))
                    if not landed and alt > 50 and random.random() < 0.15 * accel / 600:   # 데모 열화상 탐지
                        fire = random.random() < 0.25
                        temp = random.uniform(55, 220) if fire else random.uniform(33.0, 37.5)   # 34°C 미만은 앱이 거름
                        demo["uid"] = demo.get("uid", 0) + 1
                        events.append({"type": "detection", "id": k, "lat": tel["lat"], "lng": tel["lng"], "alt": tel["alt"],
                                       "temp": round(temp, 1), "uid": f"demo-{demo['uid']}"})
                out[k] = tel
            state.update(out)
        time.sleep(0.1)


async def main():
    global demo
    ap = argparse.ArgumentParser(description="GCS 앱 ⇄ PX4 드론(MAVLink) 브리지")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--mav", action="append", default=[], help="MAVLink 연결 (여러 번 지정 가능)")
    ap.add_argument("--demo", action="store_true", help="실기체 없이 데모 드론으로 시험")
    ap.add_argument("--lat", type=float, default=33.4996, help="데모 출발지 위도")
    ap.add_argument("--lng", type=float, default=126.5312, help="데모 출발지 경도")
    ap.add_argument("--drones", type=int, default=4, help="데모 드론 수")
    ap.add_argument("--accel", type=float, default=1, help="데모 시간 배속 (시험용)")
    ap.add_argument("--rate", type=float, default=2, help="앱으로 보내는 횟수(초당)")
    ap.add_argument("--token", default=os.environ.get("GCS_BRIDGE_TOKEN", ""),
                    help="앱 접속 비밀번호 (앱 설정 → 브리지 비밀번호에 같은 값). 환경 변수 GCS_BRIDGE_TOKEN으로도 지정")
    ap.add_argument("--pi", help="노트북 브리지 방식: 파이 촬영 프로그램 주소 (예: http://dmdpi.local:8080). 경로를 올릴 때 임무 정보를 보내고, Wi-Fi가 닿으면 사진·탐지를 받아 옴")
    ap.add_argument("--qgc", help="무선기를 QGroundControl과 같이 쓰기 (예: udpout:127.0.0.1:14550)")
    ap.add_argument("--local", help="사진 폴더: 인터넷 없이 Wi-Fi로 쓸 때 (파이에서 실행). 촬영 프로그램과 같은 폴더")
    ap.add_argument("--cert")
    ap.add_argument("--key")
    a = ap.parse_args()
    try:   # Windows: 콘솔 글꼴에 없는 글자가 있어도 브리지가 멈추지 않게
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass
    global TOKEN, LOCAL, PI, QGC_URL
    TOKEN = a.token
    QGC_URL = a.qgc
    if a.pi:
        PI = a.pi.rstrip("/")
        if not PI.startswith("http"):
            PI = "http://" + PI
        threading.Thread(target=pi_poll, daemon=True).start()
        print(f"[파이] 촬영 프로그램: {PI} (경로 업로드 때 임무 정보 전달, Wi-Fi가 닿으면 사진·탐지 받기)")
    if a.local:
        LOCAL = os.path.abspath(a.local)
        os.makedirs(os.path.join(LOCAL, "out"), exist_ok=True)
        try:
            with open(os.path.join(LOCAL, "mission.json"), encoding="utf-8") as f:
                local_mission["id"] = json.load(f).get("mission")   # 브리지를 다시 켜도 지금 임무를 이어서
        except Exception:
            pass
        threading.Thread(target=local_tail, daemon=True).start()
        print(f"[사진] Wi-Fi 사진 허브: {LOCAL}")
    if a.demo == bool(a.mav):
        ap.error("--demo 또는 --mav 중 하나만 지정하세요.")
    if a.demo:
        demo = {"home": (a.lat, a.lng), "drones": {}}
        threading.Thread(target=demo_worker, args=(max(1, a.drones), a.accel), daemon=True).start()
    for c in a.mav:
        threading.Thread(target=mav_worker, args=(c,), daemon=True).start()
    ctx = None
    if a.cert and a.key:
        ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ctx.load_cert_chain(a.cert, a.key)
    try:
        server = await asyncio.start_server(handle, a.host, a.port, ssl=ctx)
    except OSError as e:   # 10048 / EADDRINUSE: 브리지 창이 이미 하나 켜져 있음
        if getattr(e, "winerror", None) == 10048 or e.errno in (98, 48, 10048):
            sys.exit(f"[브리지] ❌ {a.port}번 포트를 이미 다른 브리지가 쓰고 있습니다. 브리지는 하나만 켜야 합니다.\n"
                     "[브리지]    열려 있는 다른 검은 창(laptop_bridge.bat)을 모두 닫고 다시 실행하세요. "
                     "창이 안 보이면 작업 관리자에서 python 을 끝내세요.")
        raise
    print(f"[브리지] {'wss' if ctx else 'ws'}://{a.host}:{a.port} 에서 앱 연결 대기 ({'데모' if a.demo else '실기체'})")
    if not TOKEN:
        print("[브리지] 주의: 비밀번호 없이 실행 중입니다. 같은 네트워크의 누구나 드론에 명령을 보낼 수 있습니다. --token 으로 비밀번호를 정하세요.")
    asyncio.create_task(broadcaster(1 / a.rate))
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[브리지] 종료")
