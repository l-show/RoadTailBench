import json
import queue
import threading
from pathlib import Path


class RuntimeVideoRecorder:
    CAMERA_TRANSFORMS = {
        "CAM_FRONT": ((2.2, 0.0, 2.2), (-10.0, 0.0, 0.0)),
        "CAM_FRONT_RIGHT": ((1.2, 1.0, 2.2), (-10.0, 60.0, 0.0)),
        "CAM_FRONT_LEFT": ((1.2, -1.0, 2.2), (-10.0, -60.0, 0.0)),
        "CAM_BACK": ((-2.5, 0.0, 2.2), (-10.0, 180.0, 0.0)),
        "CAM_BACK_RIGHT": ((-1.5, 1.0, 2.2), (-10.0, 120.0, 0.0)),
        "CAM_BACK_LEFT": ((-1.5, -1.0, 2.2), (-10.0, -120.0, 0.0)),
    }

    def __init__(self, carla, world, ego_actor, output_dir, args):
        self.carla = carla
        self.world = world
        self.ego_actor = ego_actor
        self.output_dir = Path(output_dir) / "video"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.args = args
        self.sensors = []
        self.queues = {}
        self.threads = {}
        self.writers = {}
        self.frame_counts = {}
        self.errors = []
        self.cv2 = None
        self.np = None
        self.direct_mp4 = False

    def _load_image_libs(self):
        try:
            import cv2
            import numpy as np
            self.cv2 = cv2
            self.np = np
            self.direct_mp4 = not bool(getattr(self.args, "video_save_frames", False))
        except Exception as exc:
            self.errors.append(f"video image libraries unavailable: {exc}")
            self.direct_mp4 = False

    def _camera_blueprint(self):
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(int(getattr(self.args, "video_width", 1280))))
        bp.set_attribute("image_size_y", str(int(getattr(self.args, "video_height", 720))))
        bp.set_attribute("fov", str(float(getattr(self.args, "video_fov", 90.0))))
        fps = max(float(getattr(self.args, "video_fps", 10.0)), 0.1)
        bp.set_attribute("sensor_tick", str(1.0 / fps))
        return bp

    def _transform(self, location, rotation):
        return self.carla.Transform(
            self.carla.Location(x=location[0], y=location[1], z=location[2]),
            self.carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]),
        )

    def start(self):
        self._load_image_libs()
        mode = getattr(self.args, "record_video_mode", "spectator")
        bp = self._camera_blueprint()
        if mode in ("spectator", "both"):
            self._spawn_camera("spectator", bp, self._transform((-6.0, 0.0, 3.0), (-15.0, 0.0, 0.0)))
        if mode in ("ego_6cam", "both"):
            for name, (loc, rot) in self.CAMERA_TRANSFORMS.items():
                self._spawn_camera(name, bp, self._transform(loc, rot))

    def _spawn_camera(self, name, bp, transform):
        try:
            sensor = self.world.spawn_actor(
                bp,
                transform,
                attach_to=self.ego_actor,
                attachment_type=self.carla.AttachmentType.Rigid,
            )
        except Exception as exc:
            self.errors.append(f"{name}: failed to spawn camera: {exc}")
            return
        self.sensors.append(sensor)
        q = queue.Queue(maxsize=300)
        self.queues[name] = q
        self.frame_counts[name] = 0
        out_dir = self.output_dir / name
        out_dir.mkdir(parents=True, exist_ok=True)
        t = threading.Thread(target=self._writer_worker, args=(name, q, out_dir), daemon=True)
        t.start()
        self.threads[name] = t
        sensor.listen(self._make_callback(name, q))

    def _make_callback(self, name, q):
        def callback(image):
            if self.np is None:
                return
            try:
                img = self.np.frombuffer(image.raw_data, dtype=self.np.dtype("uint8"))
                img = self.np.reshape(img, (image.height, image.width, 4))[:, :, :3]
                q.put_nowait((int(image.frame), img.copy()))
            except queue.Full:
                self.errors.append(f"{name}: dropped frame {image.frame}; queue full")
            except Exception as exc:
                self.errors.append(f"{name}: callback error: {exc}")

        return callback

    def _writer_worker(self, name, q, out_dir):
        writer = None
        try:
            while True:
                item = q.get()
                if item is None:
                    break
                frame_id, img = item
                if self.cv2 is None:
                    q.task_done()
                    continue
                if self.direct_mp4:
                    if writer is None:
                        height, width = img.shape[:2]
                        fourcc = self.cv2.VideoWriter_fourcc(*"mp4v")
                        path = self.output_dir / f"{name}.mp4"
                        writer = self.cv2.VideoWriter(str(path), fourcc, float(getattr(self.args, "video_fps", 10.0)), (width, height))
                        self.writers[name] = str(path)
                    writer.write(img)
                else:
                    ext = "." + str(getattr(self.args, "video_image_format", "jpg")).lstrip(".")
                    self.cv2.imwrite(str(out_dir / f"{frame_id:06d}{ext}"), img)
                self.frame_counts[name] += 1
                q.task_done()
        except Exception as exc:
            self.errors.append(f"{name}: writer error: {exc}")
        finally:
            if writer is not None:
                writer.release()

    def close(self, carla_alive=True):
        for sensor in self.sensors:
            try:
                sensor.stop()
            except Exception:
                pass
        for q in self.queues.values():
            try:
                q.put_nowait(None)
            except Exception:
                pass
        for thread in self.threads.values():
            thread.join(timeout=3.0)
        if carla_alive:
            for sensor in self.sensors:
                try:
                    sensor.destroy()
                except Exception:
                    pass
        manifest = {
            "mode": getattr(self.args, "record_video_mode", "spectator"),
            "fps": float(getattr(self.args, "video_fps", 10.0)),
            "direct_mp4": self.direct_mp4,
            "frame_counts": self.frame_counts,
            "writers": self.writers,
            "errors": self.errors[-50:],
        }
        path = self.output_dir / "video_manifest.json"
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return {"video_manifest": str(path)}
