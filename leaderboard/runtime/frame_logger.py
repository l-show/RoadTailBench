import json
import math
from pathlib import Path

from leaderboard.core.io import save_json
from leaderboard.core.metrics_csv import save_metrics_csv
from leaderboard.metrics.evaluator import evaluate_leaderboard
from .carla_utils import actor_to_record, control_to_dict


class RuntimeFrameLogger:
    def __init__(self, output_dir, scenario, config, carla_module=None):
        self.output_dir = Path(output_dir)
        self.scenario = scenario
        self.config = config
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.frames_path = self.output_dir / "leaderboard_frame_log.jsonl"
        self.config_path = self.output_dir / "leaderboard_scenario_config.json"
        self.metrics_path = self.output_dir / "leaderboard_metrics.json"
        self.metrics_csv_path = self.output_dir / "leaderboard_metrics.csv"
        self.summary_path = self.output_dir / "leaderboard_run_summary.json"
        self._file = self.frames_path.open("w", encoding="utf-8")
        self._frames = []
        self._collisions = []
        self._collision_sensor = None
        self.carla = carla_module
        self._last_environment_proximity = {"raycast_available": False}
        self._last_environment_raycast_frame = None
        save_json(self.config_path, config)

    def attach_collision_sensor(self, carla, world, ego_actor):
        bp = world.get_blueprint_library().find("sensor.other.collision")
        self._collision_sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego_actor)

        def on_collision(event):
            other = event.other_actor
            location = None
            try:
                location = event.transform.location
            except AttributeError:
                try:
                    location = ego_actor.get_location()
                except RuntimeError:
                    location = None
            self._collisions.append({
                "frame": int(event.frame),
                "type": "collision",
                "other_actor_id": int(other.id) if other else None,
                "other_actor_type": other.type_id if other else "unknown",
                "role_name": other.attributes.get("role_name", "") if other else "",
                "location": [float(location.x), float(location.y), float(location.z)] if location else None,
                "normal_impulse": [
                    float(event.normal_impulse.x),
                    float(event.normal_impulse.y),
                    float(event.normal_impulse.z),
                ] if hasattr(event, "normal_impulse") else None,
            })

        self._collision_sensor.listen(on_collision)

    def collect_environment_proximity(self, carla, world, ego_actor, frame_id=None):
        if carla is None:
            return {"raycast_available": False}
        if not hasattr(world, "cast_ray"):
            return {"raycast_available": False}
        interval = max(1, int(self.config.get("environment_raycast_interval_frames", 5)))
        if frame_id is not None and self._last_environment_raycast_frame is not None:
            if (int(frame_id) - int(self._last_environment_raycast_frame)) < interval:
                reused = dict(self._last_environment_proximity)
                reused["raycast_reused"] = True
                reused["raycast_interval_frames"] = interval
                return reused
        try:
            tf = ego_actor.get_transform()
            center = tf.location + carla.Location(z=1.0)
            yaw = float(tf.rotation.yaw)
            max_distance = float(self.config.get("environment_raycast_distance_m", 30.0))
            min_hit_distance = float(self.config.get("environment_raycast_min_hit_distance_m", 1.0))
            angles = self.config.get("environment_raycast_angles_deg", [-90, -60, -30, 0, 30, 60, 90])
            extent_x = extent_y = 1.0
            try:
                extent_x = float(ego_actor.bounding_box.extent.x)
                extent_y = float(ego_actor.bounding_box.extent.y)
            except Exception:
                pass
            hits = []
            for rel in angles:
                heading = yaw + float(rel)
                rel_rad = math.radians(float(rel))
                self_clearance = abs(math.cos(rel_rad)) * extent_x + abs(math.sin(rel_rad)) * extent_y + 0.5
                forward = carla.Vector3D(
                    x=math.cos(math.radians(heading)),
                    y=math.sin(math.radians(heading)),
                    z=0.0,
                )
                origin = center + forward * self_clearance
                target = origin + forward * max_distance
                ray_hits = world.cast_ray(origin, target)
                if not ray_hits:
                    continue
                hit = None
                for candidate in ray_hits:
                    loc = candidate.location
                    if origin.distance(loc) >= min_hit_distance:
                        hit = candidate
                        break
                if hit is None:
                    continue
                loc = hit.location
                distance = origin.distance(loc)
                hits.append({
                    "relative_angle_deg": float(rel),
                    "distance_m": float(distance),
                    "location": [float(loc.x), float(loc.y), float(loc.z)],
                    "label": str(getattr(hit, "label", "")),
                })
            nearest = min(hits, key=lambda item: item["distance_m"]) if hits else None
            result = {
                "raycast_available": True,
                "raycast_reused": False,
                "raycast_interval_frames": interval,
                "raycast_max_distance_m": max_distance,
                "raycast_min_hit_distance_m": min_hit_distance,
                "environment_hits": hits,
                "nearest_environment_distance_m": nearest["distance_m"] if nearest else None,
                "nearest_environment_relative_angle_deg": nearest["relative_angle_deg"] if nearest else None,
            }
            self._last_environment_proximity = result
            self._last_environment_raycast_frame = frame_id
            return result
        except Exception as exc:
            result = {"raycast_available": False, "raycast_error": str(exc)}
            self._last_environment_proximity = result
            self._last_environment_raycast_frame = frame_id
            return result

    def log_tick(self, world, ego_actor, ego_control=None, actor_radius_m=120.0):
        snapshot = world.get_snapshot()
        frame_id = int(snapshot.frame)
        ego_loc = ego_actor.get_location()
        actors = []
        for actor in world.get_actors():
            try:
                if actor.id == ego_actor.id or actor.type_id.startswith("sensor."):
                    continue
                if not (actor.type_id.startswith("vehicle.") or actor.type_id.startswith("walker.") or actor.type_id.startswith("static.")):
                    continue
                if actor.get_location().distance(ego_loc) <= actor_radius_m:
                    actors.append(actor_to_record(actor))
            except (AttributeError, RuntimeError):
                continue
        ego_record = actor_to_record(ego_actor)
        if ego_control is not None:
            ego_record["control"] = control_to_dict(ego_control)
        try:
            waypoint = world.get_map().get_waypoint(ego_loc)
            ego_record["lane_width_m"] = float(getattr(waypoint, "lane_width", 0.0))
            ego_record["lane_id"] = int(getattr(waypoint, "lane_id", 0))
            ego_record["road_id"] = int(getattr(waypoint, "road_id", 0))
        except Exception:
            pass
        record = {
            "frame": frame_id,
            "time": float(snapshot.timestamp.elapsed_seconds),
            "ego": ego_record,
            "actors": actors,
            "collisions": [c for c in self._collisions if c.get("frame") == frame_id],
        }
        try:
            record["proximity"] = self.collect_environment_proximity(self.carla, world, ego_actor, frame_id) if self.carla else {"raycast_available": False}
        except Exception:
            record["proximity"] = {"raycast_available": False}
        self._frames.append(record)
        self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._file.flush()

    def close(self, run_summary=None, carla_alive=True):
        try:
            if self._collision_sensor:
                try:
                    self._collision_sensor.stop()
                except Exception:
                    pass
                if carla_alive:
                    try:
                        self._collision_sensor.destroy()
                    except Exception:
                        pass
        finally:
            self._file.close()
        metrics = evaluate_leaderboard(self._frames, self.config)
        save_json(self.metrics_path, metrics)
        save_metrics_csv(self.metrics_csv_path, metrics)
        if run_summary:
            save_json(self.summary_path, run_summary)
        return {
            "frames": str(self.frames_path),
            "config": str(self.config_path),
            "metrics": str(self.metrics_path),
            "metrics_csv": str(self.metrics_csv_path),
            "summary": str(self.summary_path),
        }
