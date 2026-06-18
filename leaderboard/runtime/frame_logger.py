import json
from pathlib import Path

from leaderboard.core.io import save_json
from leaderboard.metrics.evaluator import evaluate_leaderboard
from .carla_utils import actor_to_record, control_to_dict


class RuntimeFrameLogger:
    def __init__(self, output_dir, scenario, config):
        self.output_dir = Path(output_dir)
        self.scenario = scenario
        self.config = config
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.frames_path = self.output_dir / "leaderboard_frame_log.jsonl"
        self.config_path = self.output_dir / "leaderboard_scenario_config.json"
        self.metrics_path = self.output_dir / "leaderboard_metrics.json"
        self.summary_path = self.output_dir / "leaderboard_run_summary.json"
        self._file = self.frames_path.open("w", encoding="utf-8")
        self._frames = []
        self._collisions = []
        self._collision_sensor = None
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
            })

        self._collision_sensor.listen(on_collision)

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
        record = {
            "frame": frame_id,
            "time": float(snapshot.timestamp.elapsed_seconds),
            "ego": ego_record,
            "actors": actors,
            "collisions": [c for c in self._collisions if c.get("frame") == frame_id],
        }
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
        save_json(self.metrics_path, evaluate_leaderboard(self._frames, self.config))
        if run_summary:
            save_json(self.summary_path, run_summary)
        return {
            "frames": str(self.frames_path),
            "config": str(self.config_path),
            "metrics": str(self.metrics_path),
            "summary": str(self.summary_path),
        }
