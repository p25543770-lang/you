"""Модуль работы с веб-камерами (USB V4L2 /dev/video*, OpenCV, MJPEG-стриминг).

Обеспечивает:
1. Захват видеопотоков с USB-веб-камер (/dev/video0 — передняя, /dev/video1 — задняя).
2. Раздачу MJPEG-видеопотока (/api/camera/<id>/stream) и отдельных кадров (/api/camera/<id>/frame).
3. Приём кадров от внешних ROS-нод или скриптов (POST /api/camera/<id>/frame).
4. Список доступных камер (/api/cameras) для фронтенда.
5. Генерацию тестового кадра в режиме demo / при отсутствии оборудования.
"""

from __future__ import annotations

import glob
import hmac
import logging
import os
import threading
import time
from typing import Any, Generator, Optional

log = logging.getLogger("robot_control.camera")


#: Заголовок внешнего процесса, который присылает кадры камеры.
FEED_TOKEN_HEADER = "X-Camera-Token"


def feed_denied_reason(headers) -> Optional[str]:
    """Почему кадр нельзя принять; None — можно.

    Приём кадров по умолчанию закрыт: без ``RC_CAMERA_FEED_TOKEN`` любой клиент
    в сети мог бы подставить изображение, которое оператор примет за камеру.
    С токеном кадры принимаются только от процесса, знающего секрет.
    """
    expected = os.environ.get("RC_CAMERA_FEED_TOKEN", "").strip()
    if not expected:
        return "feed_disabled"
    provided = str(headers.get(FEED_TOKEN_HEADER, "") or "")
    if not hmac.compare_digest(provided.encode("utf-8"), expected.encode("utf-8")):
        return "bad_token"
    return None


class CameraDevice:
    """Управление одной веб-камерой: фоновый захват кадров и сжатие в JPEG."""

    def __init__(self, cam_id: int, device_path: str, name: str = ""):
        self.cam_id = cam_id
        self.device_path = device_path
        self.name = name or f"Камера {cam_id}"
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_time: float = 0.0
        self.width = 640
        self.height = 360
        self.fps = 30
        self.is_active = False
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        self.is_active = False

    def feed_frame(self, jpeg_data: bytes) -> None:
        """Ручная загрузка кадра (от внешнего источника / ROS)."""
        with self.lock:
            self.latest_jpeg = jpeg_data
            self.latest_time = time.time()
            self.is_active = True

    def get_frame(self, max_age_s: float = 2.0) -> Optional[bytes]:
        with self.lock:
            if self.latest_jpeg and (time.time() - self.latest_time) < max_age_s:
                return self.latest_jpeg
        return None

    def _capture_loop(self) -> None:
        try:
            import cv2
        except ImportError:
            log.info("Камера %s: cv2 не установлен, ожидается внешний поток или браузерный доступ", self.cam_id)
            return

        cap = None
        while not self._stop:
            try:
                # Открываем устройство через V4L2 или номер индекса
                dev_target = self.device_path if os.path.exists(self.device_path) else self.cam_id
                cap = cv2.VideoCapture(dev_target, cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY)
                if not cap.isOpened():
                    time.sleep(2.0)
                    continue

                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.is_active = True
                log.info("Камера %s подключена (%s)", self.cam_id, self.device_path)

                while not self._stop and cap.isOpened():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.04)
                        continue

                    # Кодируем кадр в JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
                    ret, buf = cv2.imencode(".jpg", frame, encode_param)
                    if ret:
                        with self.lock:
                            self.latest_jpeg = buf.tobytes()
                            self.latest_time = time.time()

                    time.sleep(1.0 / self.fps)

            except Exception as exc:  # noqa: BLE001
                log.debug("Ошибка захвата камеры %s: %s", self.cam_id, exc)
            finally:
                self.is_active = False
                if cap is not None:
                    cap.release()
                    cap = None
                time.sleep(2.0)


class CameraManager:
    """Управляет всеми камерами на борту (передняя id=0, задняя id=1)."""

    _instance: Optional[CameraManager] = None

    def __init__(self):
        self.cameras: dict[int, CameraDevice] = {}
        self.lock = threading.Lock()
        self._init_devices()

    @classmethod
    def get_instance(cls) -> CameraManager:
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance

    def _init_devices(self) -> None:
        """Поиск доступных V4L2 устройств (/dev/video*)."""
        video_devs = sorted(glob.glob("/dev/video*"))
        # Фильтруем четные или первые 2 устройства захвата
        cams_info = [
            (0, "/dev/video0", "Камера передняя (CAM 0)"),
            (1, "/dev/video1", "Камера задняя (CAM 1)"),
        ]
        for cam_id, dev_path, name in cams_info:
            dev = CameraDevice(cam_id, dev_path, name)
            self.cameras[cam_id] = dev
            if os.path.exists(dev_path):
                dev.start()

    def get_camera(self, cam_id: int) -> Optional[CameraDevice]:
        return self.cameras.get(cam_id)

    def feed_frame(self, cam_id: int, jpeg_bytes: bytes) -> bool:
        dev = self.get_camera(cam_id)
        if not dev:
            dev = CameraDevice(cam_id, f"/dev/video{cam_id}", f"Камера {cam_id}")
            self.cameras[cam_id] = dev
        dev.feed_frame(jpeg_bytes)
        return True

    def get_frame(self, cam_id: int, max_age_s: float = 2.0) -> Optional[bytes]:
        dev = self.get_camera(cam_id)
        if dev:
            frame = dev.get_frame(max_age_s)
            if frame:
                return frame
        return None

    def list_cameras(self) -> list[dict[str, Any]]:
        result = []
        for cid, dev in sorted(self.cameras.items()):
            active = dev.is_active or bool(dev.get_frame(max_age_s=2.0))
            result.append(
                {
                    "id": cid,
                    "name": dev.name,
                    "device": dev.device_path,
                    "exists": os.path.exists(dev.device_path),
                    "active": active,
                    "width": dev.width,
                    "height": dev.height,
                    "fps": dev.fps,
                    "stream_url": f"/api/camera/{cid}/stream",
                    "frame_url": f"/api/camera/{cid}/frame",
                }
            )
        return result

    def generate_mjpeg_stream(self, cam_id: int) -> Generator[bytes, None, None]:
        """Генератор multipart/x-mixed-replace для MJPEG стриминга в тег <img>."""
        dev = self.get_camera(cam_id)
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

        while True:
            frame = dev.get_frame() if dev else None
            if frame:
                yield boundary + frame + b"\r\n"
            time.sleep(0.04)  # ~25 fps
