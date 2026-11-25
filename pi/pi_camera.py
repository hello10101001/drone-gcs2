#!/usr/bin/env python3
"""
드론 라즈베리파이 촬영 프로그램 — 실시간 영상 + 임무 사진 + 열화상 탐지

  하는 일
  1) 실시간 영상: http://<파이 IP>:8080/video (MJPEG). 앱 지도 화면의 '영상' 버튼으로 봄. (cam_test.py와 같은 방식)
  2) 임무 사진: 앱이 경로를 올리면 브리지(gcs_bridge.py --local)가 mission.json을 씀 → 이 프로그램이 보고,
     드론이 시동 + 미션 모드 + 수색 구역 안에 있을 때 촬영 간격(m)마다 사진을 찍어
     out/<임무>/photos.jsonl 에 기록 → 브리지가 앱에 보냄 (앱에서 칸을 누르면 사진이 보임)
  3) 열화상 (MLX90640이 연결돼 있으면): 사진마다 평균 온도(칸 열지도), 주변보다 뜨거운 점은 탐지로
     out/<임무>/detections.jsonl 에 기록 + MAVLink 알림 "DET <번호> <위도> <경도> <온도>" (텔레메트리 무선기로도 앱에 감)

  연결 (파이 안)
    픽스호크 ─ /dev/serial0 ─ mavproxy ─┬─ udp 노트북(QGroundControl)
                                       ├─ udp 127.0.0.1:14560 → gcs_bridge.py (앱과 통신)
                                       └─ udp 127.0.0.1:14561 → 이 프로그램 (위치·모드)

  노트북 브리지 방식 (드론 ─ 텔레메트리 ─ 노트북 브리지 ─ 앱)
    노트북 브리지를 --pi http://<파이 IP>:8080 으로 켜면, 경로를 올릴 때 브리지가 이 프로그램에 임무 정보를 보내고
    (POST /mission), 파이 Wi-Fi에 닿을 때 사진·탐지 목록(/photos, /detections)과 사진 파일(/files/...)을 가져갑니다.

  실행
    libcamerify python3 pi_camera.py --local ~/drone-data/local          # CSI 카메라 (cam_test.py처럼 libcamerify)
    python3 pi_camera.py --local ~/drone-data/local --camera 0            # USB 카메라
    python3 pi_camera.py --local ./local --fake                           # 파이·카메라 없이 시험 (가짜 드론이 mission.json 경로를 날며 찍음)
"""
import argparse
import collections, hmac, json, math, os, shutil, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import cv2
import numpy as np

M_LAT = 111320.0


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ======================= 카메라 =======================
class Camera:
    """마지막 화면을 계속 갖고 있음 (영상 스트림과 사진이 같이 씀)"""

    def __init__(self, dev, w, h, fake=False):
        self.dev, self.w, self.h, self.fake = dev, w, h, fake
        self.frame, self.at, self.lock = None, 0.0, threading.Lock()
        self.jpg, self.jpg_at = None, 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        cap = cv2.VideoCapture(self.dev)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        return cap

    def _run(self):
        cap, fails = None, 0
        while True:
            if self.fake:
                f = np.full((self.h, self.w, 3), (60, 110, 70), np.uint8)
                cv2.putText(f, time.strftime("%H:%M:%S"), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                self._set(f)
                time.sleep(0.1)
                continue
            if cap is None or not cap.isOpened():
                cap = self._open()
                if not cap.isOpened():
                    log("[카메라] 열 수 없음 — 5초 뒤 다시 (CSI 카메라면 libcamerify로 실행했는지 확인)")
                    time.sleep(5)
                    continue
                log(f"[카메라] 열림 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
            ok, f = cap.read()
            if not ok:
                fails += 1
                if fails > 20:   # 계속 실패하면 다시 열기 (케이블 흔들림 등)
                    cap.release()
                    cap, fails = None, 0
                time.sleep(0.05)
                continue
            fails = 0
            self._set(f)

    def _set(self, f):
        with self.lock:
            self.frame, self.at = f, time.time()

    def latest(self, max_age=1.0):
        with self.lock:
            if self.frame is None or time.time() - self.at > max_age:
                return None
            return self.frame.copy()

    def stream_jpeg(self):
        """영상용 JPEG (가로 640으로 줄여서, 같은 화면은 한 번만 압축 → 여러 명이 봐도 가벼움)"""
        with self.lock:
            f, at = self.frame, self.at
        if f is None:
            return None
        if at != self.jpg_at:
            if f.shape[1] > 640:
                f = cv2.resize(f, (640, int(f.shape[0] * 640 / f.shape[1])))
            ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.jpg, self.jpg_at = buf.tobytes(), at
        return self.jpg


# ======================= 열화상 (MLX90640, 있으면) =======================
class Thermal:
    W, H = 32, 24

    def __init__(self, fake=False, rate=4):
        self.fake, self.arr, self.at, self.lock, self.ok = fake, None, 0.0, threading.Lock(), False
        self.hot = None   # 가짜 모드: 뜨거운 점 위치(위도, 경도) — 드론이 근처를 지나면 화면에 나타남
        if fake:
            self.ok = True
            return
        try:
            import board, busio, adafruit_mlx90640   # pip install adafruit-circuitpython-mlx90640
            i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
            self.mlx = adafruit_mlx90640.MLX90640(i2c)
            self.mlx.refresh_rate = {2: adafruit_mlx90640.RefreshRate.REFRESH_2_HZ, 4: adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
                                     8: adafruit_mlx90640.RefreshRate.REFRESH_8_HZ}.get(rate, adafruit_mlx90640.RefreshRate.REFRESH_4_HZ)
            self.ok = True
            threading.Thread(target=self._run, daemon=True).start()
            log("[열화상] MLX90640 연결됨")
        except Exception as e:
            log(f"[열화상] 없음 — 일반 사진만 찍습니다 ({e.__class__.__name__}: {e})")

    def _run(self):
        buf = [0.0] * 768
        while True:
            try:
                self.mlx.getFrame(buf)
                a = np.array(buf, np.float32).reshape(self.H, self.W)
                if np.all((a > -40) & (a < 300)):   # 깨진 값이 있는 화면은 버림
                    with self.lock:
                        self.arr, self.at = a, time.time()
            except (ValueError, RuntimeError, OSError):
                time.sleep(0.05)   # I2C 읽기 오류는 가끔 생김 → 다음 화면

    def latest(self, max_age=1.5):
        with self.lock:
            if self.arr is None or time.time() - self.at > max_age:
                return None
            return self.arr.copy()

    def fake_frame(self, v, hfov, vfov):
        """가짜 열화상: 15°C 땅 + 가짜 사람(36°C)이 화각 안에 있으면 그 화소가 따뜻함"""
        a = np.random.normal(15, 0.3, (self.H, self.W)).astype(np.float32)
        if self.hot and v.alt and v.alt > 2:
            k = M_LAT * math.cos(math.radians(v.lat))
            n, e = (self.hot[0] - v.lat) * M_LAT, (self.hot[1] - v.lng) * k
            h = math.radians(v.hdg or 0)
            fwd, right = n * math.cos(h) + e * math.sin(h), -n * math.sin(h) + e * math.cos(h)
            x = (math.degrees(math.atan2(right, v.alt)) / hfov + 0.5) * self.W
            y = (0.5 - math.degrees(math.atan2(fwd, v.alt)) / vfov) * self.H
            if 0 <= x < self.W and 0 <= y < self.H:
                a[int(y), int(x)] = 26.0   # 화소 안에 사람이 일부만 → 주변보다 +11°C
        return a


def colorize(a, size=(640, 480)):
    """열화상을 보기 좋게: 부드럽게 키우고 색 입힘 (색 범위 최소 5°C)"""
    lo, hi = float(a.min()), float(a.max())
    if hi - lo < 5:
        hi = lo + 5
    g = np.clip((a - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(cv2.resize(g, size, interpolation=cv2.INTER_CUBIC), cv2.COLORMAP_INFERNO)


def north_up(img, deg):
    """사진 위쪽이 deg(북쪽에서 시계 방향, 도) 방향을 보고 있을 때 → 북쪽이 위로 가게 돌림.
    90°의 배수가 아니면 잘리지 않게 캔버스를 넓히고 빈 곳은 검게"""
    deg %= 360
    for q, code in ((90, cv2.ROTATE_90_CLOCKWISE), (180, cv2.ROTATE_180), (270, cv2.ROTATE_90_COUNTERCLOCKWISE)):
        if abs(deg - q) < 0.5:
            return cv2.rotate(img, code)
    if deg < 0.5 or deg > 359.5:
        return img
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), -deg, 1.0)   # OpenCV는 + 가 반시계 → 시계 방향으로 deg
    c, s_ = abs(m[0, 0]), abs(m[0, 1])
    nw, nh = int(round(h * s_ + w * c)), int(round(h * c + w * s_))
    m[0, 2] += nw / 2 - w / 2
    m[1, 2] += nh / 2 - h / 2
    return cv2.warpAffine(img, m, (nw, nh), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))


def mark_hot(img, a, x, y, circle=False):
    """색 입힌 열화상(img)에 화소 (x, y) 표시 + 그 온도 (탐지 사진은 동그라미, 확인 화면은 십자)"""
    sx, sy = img.shape[1] / a.shape[1], img.shape[0] / a.shape[0]
    cx, cy = int((x + 0.5) * sx), int((y + 0.5) * sy)
    if circle:
        cv2.circle(img, (cx, cy), 22, (255, 255, 255), 2)
    else:
        cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(img, f"{float(a[y, x]):.1f}C", (min(cx + 26, img.shape[1] - 110), max(cy + 8, 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


def find_hot(a, delta, fire, gap=2):
    """주변(5×5 중앙값)보다 delta°C 이상 따뜻하거나 fire°C 이상인 점 → [(x, y, 온도, 주변온도)].
    따뜻한 곳이 붙어 있어도 봉우리(주변 3×3에서 가장 뜨거운 화소)가 gap화소 이상 떨어져 있으면 따로 (가까이 있는 두 사람을 하나로 합치지 않게)"""
    a = a.astype(np.float32)
    bg = cv2.medianBlur(a, 5)
    hot = ((a - bg) >= delta) | (a >= fire)
    peak = hot & (a >= cv2.dilate(a, np.ones((3, 3), np.uint8)) - 1e-6)
    ys, xs = np.where(peak)
    out = []
    for j in np.argsort(-a[ys, xs]):   # 뜨거운 봉우리부터, 이미 고른 봉우리와 너무 가까운 것(같은 사람)은 뺌
        y, x = int(ys[j]), int(xs[j])
        if all(max(abs(x - o[0]), abs(y - o[1])) >= gap for o in out):
            out.append((x, y, float(a[y, x]), float(bg[y, x])))
    return out


# ======================= 드론 상태 (MAVLink) =======================
class VehicleState:
    """촬영에 쓰는 드론 상태 (실기체·가짜 공통)"""

    def __init__(self, sysid=None):
        self.lat = self.lng = self.alt = self.hdg = None
        self.armed, self.mission_mode, self.sysid, self.pos_at = False, False, sysid, 0.0
        self.time_off = 0.0   # PX4 GPS 시각 - 파이 시각 (파이 시계는 인터넷 없으면 틀림)
        self.triggers = collections.deque()   # PX4가 "찍어" 한 시각들 (촬영 스레드가 꺼내 찍음)
        self.trig_at, self.interval = 0.0, None   # 마지막 촬영 명령 시각, IMAGE_START_CAPTURE 간격 촬영(초)

    def trigger(self, why):
        """PX4의 촬영 명령: 0.3초 안에 겹쳐 온 것(같은 명령 재전송)은 하나로"""
        now = time.time()
        if now - self.trig_at < 0.3:
            return
        self.trig_at = now
        self.triggers.append((now, why))

    def now_ms(self):
        return int((time.time() + self.time_off) * 1000)

    def fresh(self):
        return self.lat is not None and time.time() - self.pos_at < 2


class Vehicle(VehicleState):
    """mavproxy 출구(udp)에서 위치·시동·모드를 받음. 탐지 알림(STATUSTEXT)을 보냄"""

    def __init__(self, url):
        super().__init__()
        from pymavlink import mavutil   # mavproxy를 설치하면 같이 깔림
        self.mavutil = mavutil
        # 100 = 카메라: PX4가 미션의 촬영 명령(DO_SET_CAM_TRIGG_DIST → 촬영마다 DO_DIGICAM_CONTROL)을 이 카메라로 보냄
        self.conn = mavutil.mavlink_connection(url, source_system=1, source_component=100)
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_hb = 0
        while True:
            if time.time() - last_hb > 1:
                with self.lock:   # 촬영 스레드의 탐지 알림과 동시에 보내지 않게
                    try:
                        self.conn.mav.heartbeat_send(30, 8, 0, 0, 4)   # 30 = 카메라 (비행 제어기 아님 → 앱·브리지가 무시)
                    except Exception:
                        pass
                last_hb = time.time()
            try:
                m = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as e:   # 깨진 데이터 등으로 받기 스레드가 죽지 않게
                log("[MAVLink] 받기 오류", e)
                time.sleep(0.5)
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == "HEARTBEAT":
                if m.autopilot == 8 or m.type == 6:   # 지상국·카메라 등
                    continue
                self.sysid = m.get_srcSystem()
                self.conn.mav.srcSystem = self.sysid   # 기체와 같은 번호로 알림을 보냄
                self.armed = bool(m.base_mode & 128)
                main, sub = (m.custom_mode >> 16) & 0xff, (m.custom_mode >> 24) & 0xff
                self.mission_mode = main == 4 and sub == 4   # PX4 AUTO.MISSION
            elif t == "GLOBAL_POSITION_INT":
                self.lat, self.lng = m.lat / 1e7, m.lon / 1e7
                self.alt, self.pos_at = m.relative_alt / 1000.0, time.time()
                if m.hdg != 65535:
                    self.hdg = m.hdg / 100.0
            elif t == "CAMERA_TRIGGER":   # PX4 촬영 트리거 (TRIG_INTERFACE = MAVLink)
                self.trigger(f"PX4 트리거 #{m.seq}")
            elif t == "COMMAND_LONG" and m.target_component in (0, 100) and m.target_system in (0, self.sysid or m.target_system):
                self.command(m)
            elif t == "SYSTEM_TIME" and m.time_unix_usec > 1.6e15:
                self.time_off = m.time_unix_usec / 1e6 - time.time()

    def command(self, m):
        """카메라 명령: DO_DIGICAM_CONTROL(203, param5=1 한 장) · IMAGE_START_CAPTURE(2000) · IMAGE_STOP_CAPTURE(2001)"""
        c = m.command
        if c == 203 and round(m.param5) == 1:
            self.trigger("PX4 촬영 명령")
        elif c == 2000:
            if m.param2 > 0 and round(m.param3) != 1:   # 간격 촬영: param2초마다 (멈추라 할 때까지)
                self.interval = max(0.5, float(m.param2))
            else:
                self.trigger("촬영 명령")
        elif c == 2001:
            self.interval = None
        else:
            return
        with self.lock:   # 응답을 안 하면 PX4가 같은 명령을 다시 보냄
            try:
                self.conn.mav.command_ack_send(c, 0)
            except Exception:
                pass

    def notify(self, text, sev=6):
        b = text.encode()[:50].decode("utf-8", "ignore").encode()   # 50바이트 (한글이 중간에 잘리지 않게)
        with self.lock:
            try:
                self.conn.mav.statustext_send(sev, b)   # 6 = 안내, 4 = 주의 (앱에 화면 알림)
            except Exception as e:
                log("[MAVLink] 알림 보내기 실패", e)


class FakeVehicle(VehicleState):
    """시험용: mission.json 경로를 따라 날아가는 가짜 드론"""

    def __init__(self, speed=15, low=0.0):
        super().__init__(sysid=1)
        self.speed, self.sent, self.route, self.low = speed, [], None, low   # low: 경로보다 이만큼(m) 낮게 (시험용)
        self.px4 = None   # (촬영 시작 웨이포인트, 끝 웨이포인트, 간격 m): PX4 촬영 트리거 흉내 (--fake-px4)

    def fly(self, route, home):
        self.route = [(float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 50.0) for p in route] + [(home[0], home[1], 0.0)]
        self.lat, self.lng, self.alt = home[0], home[1], 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        self.armed = self.mission_mode = True
        acc = 0.0
        for j, (la, lo, z) in enumerate(self.route):
            z = max(0.0, z - self.low) if z > 0 else 0.0
            k = M_LAT * math.cos(math.radians(self.lat))
            z0, d0 = self.alt, math.hypot((la - self.lat) * M_LAT, (lo - self.lng) * k)
            while True:
                k = M_LAT * math.cos(math.radians(self.lat))
                dn, de = (la - self.lat) * M_LAT, (lo - self.lng) * k
                d = math.hypot(dn, de)
                if d > 0.5:
                    self.hdg = math.degrees(math.atan2(de, dn)) % 360
                step = self.speed * 0.1
                if d <= step:
                    self.lat, self.lng = la, lo
                    if z > 0:
                        self.alt = z
                    if self.px4 and j == self.px4[0]:   # 촬영 시작 웨이포인트: 시작하자마자 1장
                        self.pos_at, acc = time.time(), 0.0
                        self.trigger("PX4 트리거(가짜)")
                    break
                self.lat += dn / d * step / M_LAT
                self.lng += de / d * step / k
                if self.px4 and self.px4[0] < j <= self.px4[1]:   # PX4처럼: 촬영 구간에서 간격만큼 갈 때마다 "찍어"
                    acc += step
                    if acc >= self.px4[2]:
                        acc -= self.px4[2]
                        self.trigger("PX4 트리거(가짜)")
                if z > 0:   # PX4처럼 구간을 따라 고도를 고르게 바꿈
                    self.alt = z0 + (z - z0) * (1 - (d - step) / d0) if d0 > 0 else z
                self.pos_at = time.time()
                time.sleep(0.1)
        self.alt, self.armed, self.mission_mode, self.pos_at = 0.0, False, False, time.time()
        log("[가짜 드론] 착륙")

    def notify(self, text, sev=6):
        self.sent.append(text)
        log("[가짜 MAVLink]", text)


# ======================= 임무 정보 (브리지가 쓴 mission.json) =======================
def cell_of(g, lat, lng):
    x = (lng - g["lng0"]) * M_LAT * math.cos(math.radians(g["lat0"]))
    y = (lat - g["lat0"]) * M_LAT
    c, r = math.floor((x - g["x0"]) / g["cw"]), math.floor((y - g["y0"]) / g["ch"])
    return r * g["cols"] + c if 0 <= c < g["cols"] and 0 <= r < g["rows"] else None


class Mission:
    def __init__(self, root):
        self.path, self.mtime, self.doc = os.path.join(root, "mission.json"), None, None

    def poll(self):
        """새 임무면 True"""
        try:
            t = os.path.getmtime(self.path)
        except OSError:
            return False
        if t == self.mtime:
            return False
        try:
            with open(self.path, encoding="utf-8") as f:
                doc = json.load(f)
        except (OSError, ValueError):
            return False   # 브리지가 쓰는 중 → 다음에
        self.mtime, self.doc = t, doc
        return True

    def drone(self, sysid):
        ds = (self.doc or {}).get("drones") or {}
        return ds.get(str(sysid)) or (next(iter(ds.values())) if ds else None)


# ======================= 촬영 =======================
class Shooter:
    def __init__(self, a, cam, th, veh, mis):
        self.a, self.cam, self.th, self.veh, self.mis = a, cam, th, veh, mis
        self.out = os.path.join(a.local, "out")
        self.last = None   # 마지막 촬영 위치
        self.slot = None   # (비행 줄 번호, 줄 방향 촬영 칸 번호): 줄마다 같은 위치에서 찍어 옆 줄 사진과 나란히 이어지게
        self.turn = (None, 0.0)   # (직전 방향, 시각): 도는 중에는 안 찍음
        self.zplan = None  # 지금 자리의 계획 고도 (출발지 기준, 지형 따라 나는 경로면 자리마다 다름)
        self.dense = 1     # 촬영 간격을 몇 등분해서 찍는 중인지 (낮게 날면 2·3)
        self.line_shot = None   # 마지막으로 사진을 찍은 줄 번호
        self.flying = False     # 시동 걸린 동안 True
        self.px4_mode = False   # 이번 비행에서 PX4 촬영 명령이 옴 (임무 정보가 없을 때·재촬영 비행은 그 명령대로 찍음)
        self.said, self.said_at = None, 0.0   # 사진을 못 찍는 이유 (바뀔 때만 알림)
        self.int_at = 0.0       # 간격 촬영 마지막 시각
        self.n = 0
        self.dets = []     # 최근 탐지 (중복 합치기)
        self.det_n = 0

    def loop(self):
        while True:
            if self.mis.poll():
                d = self.mis.drone(self.veh.sysid)
                log(f"[임무] {self.mis.doc.get('mission')} · 촬영 간격 {d and d.get('shot')}m · 칸 {self.mis.doc['grid']['cols']}×{self.mis.doc['grid']['rows']}")
                self.last, self.slot, self.line_shot, self.dets = None, None, None, []
                if self.a.fake and d:
                    if self.th.hot is None:   # 가짜 사람: 경로 가운데 근처
                        r = d["route"][len(d["route"]) // 2]
                        self.th.hot = (float(r[0]) + 0.00005, float(r[1]))
                    if self.a.fake_px4:
                        cam = d.get("cam") or [0, len(d["route"]) - 1]
                        self.veh.px4 = (int(cam[0]), int(cam[1]), float(d.get("shot") or self.a.shot))
                    self.veh.fly(d["route"], d["home"])
            self.px4_shots()
            self.tick()
            time.sleep(0.1)

    def say(self, why, sev=None):
        """사진을 못 찍는 이유: 바뀔 때만 기록, sev를 주면 앱에도 알림 (같은 이유는 1분에 한 번)"""
        now = time.time()
        if why == self.said and (sev is None or now - self.said_at < 60):
            return
        if why != self.said:
            log(f"[촬영] {why}")
        self.said = why
        if sev is not None:
            self.said_at = now
            self.veh.notify("카메라: " + why, sev)

    def mission_id(self):
        """임무 정보를 못 받았을 때 사진 폴더 이름"""
        return (self.mis.doc or {}).get("mission") or time.strftime("M%Y%m%d-nomission")

    def own_timing(self, v):
        """파이가 줄을 맞춰 직접 찍는 중인지: 임무 정보(칸·경로)가 있고, 미션 비행 중이고, 위치가 있음 (재촬영 비행은 아님)"""
        doc = self.mis.doc or {}
        return bool(doc.get("grid")) and not doc.get("reshoot") and v.mission_mode and v.fresh()

    def px4_shots(self):
        """PX4(또는 지상국)가 보낸 촬영 명령: 파이가 줄을 맞춰 직접 찍는 중이면 그쪽이 더 정확해서 넘김 (겹치지 않게).
        임무 정보를 못 받았거나·재촬영 비행·미션 모드를 못 알아봤을 때는 이 명령대로 찍음 (위치만 있으면)"""
        v = self.veh
        if v.interval and v.armed and time.time() - self.int_at >= v.interval:   # 간격 촬영
            self.int_at = time.time()
            v.trigger("간격 촬영")
        while v.triggers:
            _, why = v.triggers.popleft()
            self.px4_mode = True
            if self.own_timing(v):
                continue
            if not v.fresh():
                self.say("찍으라 했지만 GPS 위치 없음", 4)
                continue
            doc = self.mis.doc or {}
            g, d = doc.get("grid"), self.mis.drone(v.sysid) or {}
            self.zplan = None
            if g and d and not doc.get("reshoot"):
                self.on_line(g, d, v, float(d.get("shot") or self.a.shot))   # 지형 따라 나는 경로면 그 자리 땅 높이
            if self.n == 0 or self.said:
                log(f"[촬영] {why} → 찍음" + ("" if g else " (임무 정보 없음: 칸 번호 없이 위치로)"))
                self.said = None
            self.shoot(cell_of(g, v.lat, v.lng) if g else None, self.agl(v, float(d.get("alt") or 0)))

    def agl(self, v, plan_alt):
        """땅에서 높이: 지형 따라 나는 경로면 (계획 고도 − 설정 고도) = 그 자리 땅 높이를 뺌. self.zplan은 이번에 on_line이 구한 값"""
        return (v.alt or 0) - ((self.zplan - plan_alt) if (self.zplan is not None and plan_alt) else 0)

    def shoot(self, cell, agl):
        try:
            self.capture(self.mission_id(), cell, agl)
        except Exception as e:
            log("[촬영] 오류", e)

    def tick(self):
        doc, v = self.mis.doc, self.veh
        if not v.armed:
            if self.flying:   # 착륙(시동 꺼짐): 다음 비행은 처음부터
                self.flying, self.px4_mode, self.said, self.slot, self.last = False, False, None, None, None
                v.interval = None
            return
        self.flying = True
        if not v.mission_mode:   # 손 조종·정지 비행 중에는 안 찍음 (앱 알림은 안 함: 정상)
            return self.say("미션 모드가 아니라 안 찍음")
        if not v.fresh():
            return self.say("GPS 위치가 없어 못 찍음", 4)
        doc = doc or {}
        g = doc.get("grid")
        if self.px4_mode and (not g or doc.get("reshoot")):   # PX4 촬영 명령대로 찍는 중 (px4_shots)
            return
        if not g:   # 노트북이 임무 정보를 못 보냄: 그래도 거리마다 찍음 (칸 번호 없이 위치로)
            self.say("임무 정보 없음, 거리로 찍음", 4)
        d = self.mis.drone(v.sysid) or {}
        shot, plan_alt = float(d.get("shot") or self.a.shot), float(d.get("alt") or 30)
        cell = cell_of(g, v.lat, v.lng) if g else None
        if g and cell is None:   # 수색 구역 밖
            return
        if self.turning(v):   # 줄 끝에서 도는 중: 사진이 기울고 흔들림
            return
        self.zplan = None
        line = None if (doc.get("reshoot") or not g) else self.on_line(g, d, v, shot)   # 재촬영 비행은 고른 칸을 여기저기 돌아서 줄이 없음 → 거리로
        if line == "skip":   # 줄 사이 넘어가는 구간·줄 시작·오가는 길: 안 찍음
            return
        end_line = None
        if isinstance(line, tuple) and line[0] == "end":
            # 줄 끝: 보통은 안 찍지만, 줄이 짧아(작은 영역) 정해진 자리를 하나도 못 지났으면 여기서 한 장 → 작은 영역도 빠짐없이
            if self.line_shot == line[1]:
                return
            end_line, line = line[1], None
        # 실제 땅 위 높이 (지형 따라 나는 경로면 계획 고도와 출발지 기준 고도의 차이 = 그 자리 땅 높이)
        agl = self.agl(v, plan_alt)
        if agl < plan_alt * 0.6:   # 이륙·착륙 중에는 안 찍음
            return
        # 계획보다 낮게 날면 사진 한 장이 덮는 땅이 좁아짐 → 촬영 간격을 2·3등분해 더 자주 (나눈 자리도 모든 줄이 같아 나란히 이어짐).
        # 높게 날면 겹침이 늘 뿐이라 간격은 그대로
        dense = lambda h: 1 if h >= plan_alt * 0.9 else min(4, math.ceil(plan_alt / h - 1e-9))
        n = dense(agl)
        if n < self.dense:   # 덜 자주로 돌아갈 때는 10% 여유 (고도가 조금 흔들릴 때 간격이 왔다 갔다 하지 않게)
            n = max(n, dense(agl * 0.9))
        step = shot / n
        if line is not None:
            # 비행 줄 위: 줄 방향으로 촬영 간격마다 정해진 자리(모든 줄이 같은 자리)를 지날 때 찍음 → 옆 줄 사진과 나란히 이어짐.
            # 새 줄에 들어선 순간은 자리만 기억 (들어서자마자 찍으면 줄마다 위치가 달라짐)
            slot = (line[0], n, math.floor(line[1] / step))
            prev, self.slot = self.slot, slot
            if prev is None or prev[:2] != slot[:2] or prev[2] == slot[2]:
                return
        elif end_line is None:
            # 경로 정보가 없으면: 마지막 사진에서 촬영 간격만큼 가면 찍음
            if self.last:
                k = M_LAT * math.cos(math.radians(v.lat))
                if math.hypot((v.lat - self.last[0]) * M_LAT, (v.lng - self.last[1]) * k) < step:
                    return
        if n != self.dense:
            self.dense = n
            log(f"[촬영] 땅에서 {agl:.0f}m (계획 {plan_alt:.0f}m) → 촬영 간격 {step:.0f}m" + (" (낮게 날아 더 자주)" if n > 1 else ""))
        self.last = (v.lat, v.lng)
        if end_line is not None or line is not None:
            self.line_shot = end_line if end_line is not None else line[0]   # 이 줄에서 한 장 이상 찍음
        self.shoot(cell, agl)

    def turning(self, v, rate=12.0):
        """방향이 초당 rate° 넘게 바뀌면 도는 중"""
        now, (h0, t0) = time.time(), self.turn
        h = v.hdg
        self.turn = (h, now)
        if h is None or h0 is None or now - t0 > 2:
            return False
        dh = abs((h - h0 + 180) % 360 - 180)
        return dh / max(now - t0, 0.05) > rate

    def on_line(self, g, d, v, shot):
        """지금 날고 있는 촬영 줄 → (줄 번호, 줄 방향 거리 m). 거리는 격자 원점에서 같은 축으로 재서 왕복하는 모든 줄이 같은 자리에서 찍음.
        "skip" = 촬영 줄이 아님(줄 사이 넘어가기·줄 끝·출발지에서 오가는 길, 찍지 않음), None = 경로 정보가 없음(거리로 찍음).
        덤으로 self.zplan = 이 자리의 계획 고도"""
        self.zplan = None
        route = d.get("route") or []
        if len(route) < 2 or shot <= 0:
            return None
        k = M_LAT * math.cos(math.radians(g["lat0"]))
        xy = lambda la, lo: ((lo - g["lng0"]) * k, (la - g["lat0"]) * M_LAT)
        px, py = xy(v.lat, v.lng)
        cam = d.get("cam") or [0, len(route) - 1]
        legs = []
        for i in range(max(0, int(cam[0])), min(len(route) - 1, int(cam[1]))):
            ax, ay = xy(float(route[i][0]), float(route[i][1]))
            bx, by = xy(float(route[i + 1][0]), float(route[i + 1][1]))
            L = math.hypot(bx - ax, by - ay)
            if L > 0.5:
                za, zb = (float(route[j][2]) if len(route[j]) > 2 else float(d.get("alt") or 30) for j in (i, i + 1))
                legs.append((i, ax, ay, (bx - ax) / L, (by - ay) / L, L, za, zb))
        if not legs:
            return None
        # 촬영 줄 방향 = 같은 방향(왕복은 같은 축) 구간 길이를 모두 더해 가장 긴 방향. 줄 사이 넘어가는 구간은 이와 직각.
        # (가장 긴 구간 하나로 고르면 작은 영역에서 줄보다 넘어가는 구간이 길 때 방향을 거꾸로 앎)
        axis = {}
        for g_ in legs:
            ang = round(math.degrees(math.atan2(g_[4], g_[3])) % 180 / 5) * 5 % 180
            axis[ang] = axis.get(ang, 0.0) + g_[5]
        ang = math.radians(max(axis, key=axis.get))
        mx, my = math.cos(ang), math.sin(ang)
        best = None
        for i, ax, ay, ux, uy, L, za, zb in legs:
            t = (px - ax) * ux + (py - ay) * uy
            if t < -shot or t > L + shot:
                continue
            off = abs((px - ax) * uy - (py - ay) * ux)
            if best is None or off < best[0]:
                best = (off, i, ux, uy, t, L, za, zb)
        if best is None or best[0] > max(25.0, shot):   # 촬영 줄 위가 아님 (출발지에서 구역으로 가는 길 등)
            return "skip"
        _, i, ux, uy, t, L, za, zb = best
        self.zplan = za + (zb - za) * min(1.0, max(0.0, t / L))
        if abs(ux * mx + uy * my) < 0.9 or t < shot * 0.1:   # 줄 사이 넘어가는 구간, 줄 시작(돌고 난 곳)
            return "skip"
        if t > L - shot * 0.1:   # 줄 끝(돌기 전)
            return ("end", i)
        if v.hdg is not None:   # 기체가 줄 방향을 보고 있을 때만 (돌기 시작하면 아님)
            lh = math.degrees(math.atan2(ux, uy)) % 360
            if abs((v.hdg - lh + 180) % 360 - 180) > 30:   # 줄 근처를 비스듬히 지나감 (돌기 시작·출발지로 돌아가는 길)
                return "skip"
        if ux < 0 or (abs(ux) < 1e-9 and uy < 0):   # 갈 때·올 때 같은 축으로 재도록 방향을 하나로
            ux, uy = -ux, -uy
        return i, px * ux + py * uy

    def capture(self, mid, cell, agl=None):
        v, a = self.veh, self.a
        lat, lng, alt, hdg, at = v.lat, v.lng, agl if agl is not None else v.alt, v.hdg or 0.0, v.now_ms()   # alt = 땅에서 높이 (탐지 위치 계산·사진 기록)
        frame = self.cam.latest()
        therm = self.th.fake_frame(v, a.hfov, a.vfov) if (self.th.ok and a.fake) else (self.th.latest() if self.th.ok else None)
        if frame is None and therm is None:
            log("[촬영] 카메라 화면 없음 — 건너뜀")
            return
        sid = str(v.sysid or 1)
        base = os.path.join(self.out, mid, sid)
        for sub in ("full", "thumb", "thermal"):
            os.makedirs(os.path.join(base, sub), exist_ok=True)
        self.n += 1
        # 이름 = GPS 시각(ms) + 순번: 파이를 다시 켜서 시계가 되돌아가도 같은 이름으로 덮어쓰지 않게
        name = f"{at}_{self.n:04d}"
        low = shutil.disk_usage(self.out).free < self.a.min_free_mb * 1024 * 1024   # SD카드가 거의 차면 원본은 빼고 축소본만
        rel = lambda sub, ext="jpg": f"{mid}/{sid}/{sub}/{name}.{ext}"
        p = {"id": f"{sid}-{name}", "drone": int(sid), "lat": round(lat, 7), "lng": round(lng, 7), "alt": round(alt, 1),
             "hdg": round(hdg, 1), "takenAt": at, "cell": cell, "northUp": True}   # northUp: 사진은 북쪽이 위로 돌려 저장됨
        if frame is not None:
            frame = north_up(frame, hdg + a.cam_rot)   # 사진 방향을 지도와 같게 (북쪽이 위) → 왕복 비행해도 줄마다 뒤집히지 않음
            th = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1]))) if frame.shape[1] > 640 else frame
            cv2.imwrite(os.path.join(self.out, rel("thumb")), th, [cv2.IMWRITE_JPEG_QUALITY, 75])
            p["thumb"] = p["file"] = rel("thumb")
            if not low:
                cv2.imwrite(os.path.join(self.out, rel("full")), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                p["file"] = rel("full")
            elif self.n % 50 == 1:
                log(f"[촬영] SD카드 남은 공간이 {self.a.min_free_mb}MB보다 적어 원본은 저장하지 않습니다")
        if therm is not None:
            p["tempAvgC"] = round(float(therm.mean()), 2)
            cv2.imwrite(os.path.join(self.out, rel("thermal")), north_up(colorize(therm), hdg + a.therm_rot), [cv2.IMWRITE_JPEG_QUALITY, 80])
            p["thermal"] = rel("thermal")
            if "thumb" not in p:
                p["thumb"] = p["file"] = rel("thermal")
            self.detect(mid, sid, name, therm, lat, lng, alt, hdg, at, cell)
        self.append(mid, "photos", p)

    def detect(self, mid, sid, name, a, lat, lng, alt, hdg, at, cell):
        A = self.a
        for x, y, t, bg in find_hot(a, A.delta, A.fire):
            # 화면 위치 → 땅 위치 (짐벌이 바로 아래를 본다고 가정, 사진 위쪽 = 기체 앞쪽)
            ax = math.radians(((x + 0.5) / Thermal.W - 0.5) * A.hfov)
            ay = math.radians((0.5 - (y + 0.5) / Thermal.H) * A.vfov)
            right, fwd, h = alt * math.tan(ax), alt * math.tan(ay), math.radians(hdg + A.therm_rot)   # 열화상 위쪽 = 기체 앞쪽 + 장착 각도
            n, e = fwd * math.cos(h) - right * math.sin(h), fwd * math.sin(h) + right * math.cos(h)
            la, lo = lat + n / M_LAT, lng + e / (M_LAT * math.cos(math.radians(lat)))
            k = M_LAT * math.cos(math.radians(la))
            # 같은 열원: dedup_m 안(높이 날면 화소 하나가 넓어 위치가 흔들리므로 최소 화소 1.5개 폭) · dedup_s 안
            dm = max(A.dedup_m, 1.5 * 2 * alt * math.tan(math.radians(A.hfov / 2)) / Thermal.W)
            if any(math.hypot((la - d[0]) * M_LAT, (lo - d[1]) * k) < dm and at - d[2] < A.dedup_s * 1000 for d in self.dets):
                continue   # 같은 열원을 여러 장에서 봄 → 하나로
            self.dets.append((la, lo, at))
            # 온도는 센서가 잰 값 그대로 (높이 날면 사람이 화소보다 작아 체온보다 낮게 나옴 → 사람 판단은 '주변보다 몇 °C 따뜻한가')
            kind, temp, delta = ("fire" if t >= A.fire else "person"), t, t - bg
            self.det_n += 1
            did = f"d{int(time.time()) % 100000}{self.det_n}"   # 프로그램을 다시 켜도 같은 임무 안에서 번호가 겹치지 않게 (앱이 같은 번호는 하나로 봄)
            img = mark_hot(colorize(a), a, x, y, circle=True)
            rel = f"{mid}/{sid}/thermal/{name}_{did}.jpg"
            cv2.imwrite(os.path.join(self.out, rel), north_up(img, hdg + A.therm_rot), [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.append(mid, "detections", {"id": did, "drone": int(sid), "lat": round(la, 7), "lng": round(lo, 7), "alt": round(alt, 1),
                                            "tempC": round(temp, 1), "bgC": round(bg, 1), "deltaC": round(delta, 1), "kind": kind,
                                            "takenAt": at, "cell": cell, "img": rel})
            self.veh.notify(f"DET {did} {la:.6f} {lo:.6f} {temp:.1f} {delta:.1f} {kind[0].upper()}")   # 텔레메트리 무선기로도 앱에 알림 (50바이트 안)
            log(f"[탐지] {did} {'불' if kind == 'fire' else '사람 후보'} {temp:.1f}°C (주변 {bg:.1f}°C보다 +{delta:.1f}°C) {la:.6f},{lo:.6f}")

    def append(self, mid, kind, obj):
        with open(os.path.join(self.out, mid, kind + ".jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")   # 한 줄을 끝까지 쓴 뒤 브리지가 읽음


# ======================= 영상 서버 =======================
def read_jsonl(path):
    out = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass   # 쓰는 중인 마지막 줄
    except OSError:
        pass
    return out


class Handler(BaseHTTPRequestHandler):
    """영상·열화상 서버 공통: 로그 끔, 답장 보내기"""
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)


def start_server(handler, port):
    """서버를 뒤에서 켬 (포트를 못 열면 OSError)"""
    srv = ThreadingHTTPServer(("0.0.0.0", port), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def mjpeg(h, frame, every):
    """실시간 영상(MJPEG): frame()이 주는 JPEG를 보는 쪽이 닫을 때까지 계속 보냄 (카메라 /video, 열화상 /thermal 공통)"""
    h.send_response(200)
    h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
    h.send_header("Cache-Control", "no-store")
    h.send_header("Access-Control-Allow-Origin", "*")
    h.end_headers()
    try:
        while True:
            j = frame()
            if j:
                h.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
            time.sleep(every)
    except OSError:   # 보는 쪽이 닫음 (BrokenPipe·ConnectionReset 등)
        return


def gallery(out_root, q):
    """사진 확인 화면 HTML: ?mission=임무 (없으면 가장 최근 임무). 사진 주소에 비밀번호(t)를 붙여 바로 열림"""
    from html import escape
    from urllib.parse import quote
    t = q.get("t", "")
    tq = f"?t={quote(t)}" if t else ""
    try:
        ms = sorted((d for d in os.listdir(out_root) if os.path.isdir(os.path.join(out_root, d))),
                    key=lambda d: os.path.getmtime(os.path.join(out_root, d)), reverse=True)
    except OSError:
        ms = []
    mid = os.path.basename(q.get("mission", "")) or (ms[0] if ms else "")
    photos = read_jsonl(os.path.join(out_root, mid, "photos.jsonl")) if mid else []
    dets = read_jsonl(os.path.join(out_root, mid, "detections.jsonl")) if mid else []
    f = lambda rel: f"/files/{quote(rel)}{tq}"
    opt = "".join(f"<option value='{escape(m)}'{' selected' if m == mid else ''}>{escape(m)}</option>" for m in ms)
    tm = lambda ms_: time.strftime("%H:%M:%S", time.localtime((ms_ or 0) / 1000))
    det = "".join(f"<a class=c href='{f(x['img'])}'><img loading=lazy src='{f(x['img'])}'><b class=r>탐지 {x.get('tempC', '')}°C</b> {tm(x.get('takenAt'))} · {x.get('lat')}, {x.get('lng')}</a>"
                  for x in reversed(dets) if x.get("img"))
    ph = "".join(f"<div class=c><a href='{f(p.get('file') or p['thumb'])}'><img loading=lazy src='{f(p['thumb'])}'></a>"
                 + (f"<a href='{f(p['thermal'])}'><img loading=lazy src='{f(p['thermal'])}'></a>" if p.get("thermal") and p.get("thermal") != p.get("thumb") else "")
                 + f"드론 {p.get('drone')} · {tm(p.get('takenAt'))} · 칸 {p.get('cell') if p.get('cell') is not None else '–'} · {p.get('alt')}m</div>"
                 for p in reversed(photos) if p.get("thumb"))
    return f"""<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>드론 사진</title>
<style>body{{margin:0;padding:12px;background:#0b1220;color:#e5e7eb;font:15px sans-serif}}select{{font-size:16px;padding:6px}}
.g{{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px;margin:10px 0 24px}}
.c{{display:block;background:#111a2e;border-radius:10px;padding:6px;color:#cbd5e1;text-decoration:none;font-size:13px}}
.c img{{width:100%;aspect-ratio:4/3;object-fit:contain;background:#000;border-radius:6px;margin-bottom:4px}}.c a+a img{{margin-top:4px}}.r{{color:#fca5a5}}</style>
<h2>드론 사진</h2><form>임무 <select name=mission onchange="this.form.submit()">{opt or "<option>임무 없음</option>"}</select>{f"<input type=hidden name=t value='{escape(t)}'>" if t else ""}</form>
<p>사진 {len(photos)}장 · 탐지 {len(dets)}건 — 사진을 누르면 원본이 열립니다 (사진 · 열화상 순서)</p>
{f"<h3>탐지</h3><div class=g>{det}</div>" if det else ""}<h3>사진</h3><div class=g>{ph or "아직 사진이 없습니다."}</div>"""


def serve(cam, port, local, token):
    out_root = os.path.realpath(os.path.join(local, "out"))
    CT = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png"}

    class H(Handler):
        def _q(self):
            u = urlsplit(self.path)
            return u.path, {k: v[0] for k, v in parse_qs(u.query).items()}

        def _authed(self, q):
            return not token or hmac.compare_digest(q.get("t", "").encode(), token.encode())

        def do_POST(self):
            # 노트북 브리지가 경로를 올릴 때 임무 정보(mission.json)를 보냄
            path, q = self._q()
            if path != "/mission":
                return self._send(404, b'{"error":"not found"}')
            if not self._authed(q):
                return self._send(403, b'{"error":"token"}')
            try:
                n = int(self.headers.get("Content-Length", "0"))
                doc = json.loads(self.rfile.read(min(n, 2_000_000)))
                assert doc.get("mission") and doc.get("grid")
            except Exception:
                return self._send(400, b'{"error":"bad mission"}')
            tmp = os.path.join(local, "mission.json.tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(doc, f, ensure_ascii=False)
            os.replace(tmp, os.path.join(local, "mission.json"))
            log(f"[임무] 노트북 브리지에서 받음: {doc['mission']}")
            self._send(200, b'{"ok":true}')

        def do_GET(self):
            path, q = self._q()
            if (path in ("/photos", "/detections", "/mission", "/gallery") or path.startswith("/files/")) and not self._authed(q):
                return self._send(403, b'{"error":"token"}')
            if path == "/gallery":   # 브라우저로 사진 확인: 임무 고르기 → 사진·열화상·탐지 목록 (파일 경로를 직접 칠 필요 없음)
                return self._send(200, gallery(out_root, q).encode("utf-8"), "text/html; charset=utf-8", {"Cache-Control": "no-store"})
            if path == "/mission":   # 지금 임무 정보 (노트북 브리지를 다시 켰을 때 이어 받기용)
                try:
                    with open(os.path.join(local, "mission.json"), "rb") as f:
                        return self._send(200, f.read(), extra={"Cache-Control": "no-store"})
                except OSError:
                    return self._send(200, b"{}", extra={"Cache-Control": "no-store"})
            if path in ("/photos", "/detections"):   # 사진·탐지 목록 (out/<임무>/*.jsonl 그대로)
                mid = os.path.basename(q.get("mission", ""))
                lst = read_jsonl(os.path.join(out_root, mid, path[1:] + ".jsonl")) if mid else []
                try:   # since(ms): 임무 정보를 못 받고 임시 폴더(*-nomission)에 찍은 것도 그 뒤 것이면 같이 (10분 여유: 시계 차이)
                    since = int(q.get("since", "0")) - 600_000
                except ValueError:
                    since = -1
                if mid and since > 0:
                    for d in sorted(os.listdir(out_root)):
                        if d.endswith("-nomission") and d != mid:
                            lst += [x for x in read_jsonl(os.path.join(out_root, d, path[1:] + ".jsonl")) if (x.get("takenAt") or 0) >= since]
                return self._send(200, json.dumps(lst, ensure_ascii=False).encode("utf-8"), extra={"Cache-Control": "no-store"})
            if path.startswith("/files/"):
                p = os.path.realpath(os.path.join(out_root, unquote(path[len("/files/"):])))
                if not p.startswith(out_root + os.sep) or not os.path.isfile(p):   # 폴더 밖(../) 막음
                    return self._send(404, b'{"error":"not found"}')
                with open(p, "rb") as f:
                    return self._send(200, f.read(), CT.get(os.path.splitext(p)[1].lower(), "application/octet-stream"), {"Cache-Control": "max-age=86400"})
            if self.path.startswith("/video"):
                return mjpeg(self, cam.stream_jpeg, 0.1)   # 초당 약 10장 (Wi-Fi 부담 줄임)
            elif self.path.startswith("/snapshot"):
                j = cam.stream_jpeg()
                self._send(200 if j else 503, j or b"", "image/jpeg")
            else:
                self._send(200, "<!doctype html><meta charset=utf-8><title>드론 영상</title><body style='margin:0;background:#000'><img src='/video' style='width:100%'>".encode(),
                           "text/html; charset=utf-8")

    start_server(H, port)
    log(f"[영상] http://<파이 IP>:{port}/video")


THERM_PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>드론 열화상</title>
<body style="margin:0;height:100vh;display:flex;flex-direction:column;background:#000;color:#fff;font:15px sans-serif">
<img src="/thermal" style="flex:1;min-height:0;width:100%;object-fit:contain" alt="열화상">
<div id=t style="padding:6px 10px;background:#111;white-space:nowrap;overflow:hidden;text-overflow:ellipsis">열화상 기다리는 중…</div>
<script>
async function u(){try{const r=await fetch("/thermal.json",{cache:"no-store"}),d=await r.json();
document.getElementById("t").textContent=d.ok?`최고 ${d.max}°C · 가운데 ${d.center}°C · 평균 ${d.avg}°C · 최저 ${d.min}°C`:"열화상 센서 없음 (MLX90640 연결 확인)"}catch(e){}}
u();setInterval(u,1000);
</script>"""


def thermal_now(th):
    """지금 열화상 한 장 (가짜 모드면 가짜 화면)"""
    if th.fake:
        return th.fake_frame(VehicleState(sysid=1), 55, 35)
    return th.latest()


def thermal_jpeg(a):
    y, x = np.unravel_index(int(np.argmax(a)), a.shape)   # 가장 뜨거운 점
    img = mark_hot(colorize(a), a, int(x), int(y))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes() if ok else None


def serve_thermal(th, port):
    """열화상 확인용 화면 (브라우저로 http://<파이>:8090): / 화면, /thermal 실시간 영상(MJPEG), /thermal.json 온도 숫자.
    같은 센서를 이 프로그램만 읽으므로 다른 열화상 프로그램과 같이 켜지 마세요"""
    class T(Handler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/thermal.json":
                a = thermal_now(th) if th.ok else None
                d = {"ok": a is not None}
                if a is not None:
                    d.update(min=round(float(a.min()), 1), max=round(float(a.max()), 1), avg=round(float(a.mean()), 1),
                             center=round(float(a[a.shape[0] // 2, a.shape[1] // 2]), 1), w=int(a.shape[1]), h=int(a.shape[0]),
                             at=round(time.time(), 1))
                return self._send(200, json.dumps(d).encode(), extra={"Cache-Control": "no-store"})
            if path == "/thermal":
                def frame():
                    a = thermal_now(th) if th.ok else None
                    return thermal_jpeg(a) if a is not None else None
                return mjpeg(self, frame, 0.25)   # 센서가 초당 4장
            return self._send(200, THERM_PAGE.encode("utf-8"), "text/html; charset=utf-8", {"Cache-Control": "no-store"})

    try:
        start_server(T, port)
    except OSError as e:
        log(f"[열화상] {port} 포트를 다른 프로그램이 쓰고 있어 열화상 화면은 켜지 않습니다 ({e}). "
            "다른 열화상 프로그램이 같은 MLX90640 센서를 동시에 읽으면 값이 깨질 수 있으니 하나만 켜세요.")
        return
    log(f"[열화상] http://<파이 IP>:{port}  (브라우저로 열화상 확인)")


def main():
    ap = argparse.ArgumentParser(description="드론 라즈베리파이 촬영 프로그램")
    ap.add_argument("--local", default=os.path.expanduser("~/drone-data/local"), help="브리지 --local 과 같은 폴더")
    ap.add_argument("--mav", default="udpin:127.0.0.1:14561", help="mavproxy 출구 (--out=udp:127.0.0.1:14561)")
    ap.add_argument("--camera", default="0", help="카메라 번호 또는 장치 경로")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--port", type=int, default=8080, help="실시간 영상 포트")
    ap.add_argument("--tport", type=int, default=8090, help="열화상 화면 포트 (0이면 끔)")
    ap.add_argument("--shot", type=float, default=30, help="임무에 촬영 간격이 없을 때 (m)")
    ap.add_argument("--cam-rot", type=float, default=float(os.environ.get("CAM_ROT") or 0),
                    help="카메라 장착 각도: 사진 위쪽이 기체 앞쪽에서 시계 방향으로 몇 도 (보통 0, 뒤를 보게 달았으면 180)")
    ap.add_argument("--therm-rot", type=float, default=float(os.environ.get("THERM_ROT") or 0), help="열화상 장착 각도 (위와 같음)")
    ap.add_argument("--hfov", type=float, default=55, help="열화상 가로 화각 (MLX90640 BAA 55°, BAB 110°)")
    ap.add_argument("--vfov", type=float, default=35, help="열화상 세로 화각 (BAA 35°, BAB 75°)")
    env = lambda k, d: float(os.environ.get(k) or d)   # config.env 값 (drone-camera 서비스가 읽음)
    ap.add_argument("--delta", type=float, default=env("HOT_DELTA", 3.0), help="주변보다 이만큼(°C) 따뜻하면 사람 후보")
    ap.add_argument("--fire", type=float, default=env("FIRE_C", 45.0), help="이 온도(°C) 이상은 불")
    ap.add_argument("--dedup-m", type=float, default=env("DEDUP_M", 3.0), help="이 거리(m) 안에서 다시 보이면 같은 열원으로 합침")
    ap.add_argument("--dedup-s", type=float, default=env("DEDUP_S", 5.0), help="이 시간(초) 안에 다시 보이면 같은 열원으로 합침")
    ap.add_argument("--min-free-mb", type=float, default=500, help="SD카드 남은 공간이 이보다 적으면 원본 사진은 저장하지 않음")
    ap.add_argument("--token", default=os.environ.get("TOKEN", ""), help="사진·임무 주고받을 때 비밀번호 (브리지 --token과 같게)")
    ap.add_argument("--fake", action="store_true", help="파이·카메라 없이 시험")
    ap.add_argument("--fake-hot", help="가짜 사람 위치 '위도,경도' (--fake)")
    ap.add_argument("--fake-px4", action="store_true", help="가짜 드론이 PX4처럼 촬영 명령을 보냄 (--fake)")
    ap.add_argument("--fake-low", type=float, default=0.0, help="가짜 드론이 경로보다 이만큼(m) 낮게 남 (--fake, 고도 맞춤 시험)")
    a = ap.parse_args()
    os.makedirs(os.path.join(a.local, "out"), exist_ok=True)
    dev = int(a.camera) if a.camera.isdigit() else a.camera
    cam = Camera(dev, a.width, a.height, fake=a.fake)
    th = Thermal(fake=a.fake)
    if a.fake and a.fake_hot:
        th.hot = tuple(map(float, a.fake_hot.split(",")))
    veh = FakeVehicle(low=a.fake_low) if a.fake else Vehicle(a.mav)
    serve(cam, a.port, a.local, a.token)
    if a.tport:
        serve_thermal(th, a.tport)
    log(f"[촬영] 임무 정보 기다림: {os.path.join(a.local, 'mission.json')}")
    Shooter(a, cam, th, veh, Mission(a.local)).loop()


if __name__ == "__main__":
    main()
