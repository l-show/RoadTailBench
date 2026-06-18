import importlib
import math
import os
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

from leaderboard.core.io import save_json
from leaderboard.core.trajectory import reference_xy, trajectory_goal_xy
from .carla_utils import dict_to_transform, metadata_location
from .frame_logger import RuntimeFrameLogger
from .video_recorder import RuntimeVideoRecorder


class ScenarioTimeoutError(RuntimeError):
    pass


class CarlaUnavailableError(RuntimeError):
    pass


EGO_MODE_ALIASES = {
    "scene_ego": "scene_ego",
    "script_ego": "scene_ego",
    "agent_ego": "agent_ego",
    "external_ego": "agent_ego",
}


def normalize_ego_mode(value):
    if value not in EGO_MODE_ALIASES:
        raise ValueError(f"Unsupported ego mode: {value}")
    return EGO_MODE_ALIASES[value]


def import_carla():
    import carla
    return carla


class CodeScenarioRunner:
    def __init__(self, args):
        self.args = args
        self.args.ego_mode = normalize_ego_mode(args.ego_mode)
        self.carla = import_carla()
        self.client = self.carla.Client(args.host, args.port)
        self.client.set_timeout(args.carla_timeout)
        self.world = None
        self._carla_alive = True
        self._last_rpc = ""

    def mark_rpc(self, name):
        self._last_rpc = name

    def is_carla_error(self, exc):
        text = str(exc).lower()
        markers = (
            "time-out",
            "timeout",
            "connection failed",
            "connection refused",
            "actively refused",
            "simulator",
            "localhost",
        )
        return any(marker in text for marker in markers)

    def check_carla_port(self):
        timeout = max(0.2, float(getattr(self.args, "carla_health_timeout", 3.0)))
        try:
            with socket.create_connection((self.args.host, int(self.args.port)), timeout=timeout):
                return True
        except OSError:
            return False

    def probe_carla_alive(self):
        if not self.check_carla_port():
            self._carla_alive = False
            return False
        original_timeout = float(getattr(self.args, "carla_timeout", 180.0))
        try:
            self.client.set_timeout(max(0.5, float(getattr(self.args, "carla_health_timeout", 3.0))))
            self.mark_rpc("health_get_world")
            world = self.client.get_world()
            self.mark_rpc("health_get_snapshot")
            world.get_snapshot()
            self._carla_alive = True
            return True
        except Exception:
            self._carla_alive = False
            return False
        finally:
            try:
                self.client.set_timeout(original_timeout)
            except Exception:
                pass

    def raise_if_carla_unavailable(self, exc, rpc_name):
        if self.is_carla_error(exc):
            self._carla_alive = False
            raise CarlaUnavailableError(f"{rpc_name}: CARLA unavailable: {exc}") from exc
        raise exc

    def connect_world(self, scenario):
        metadata = scenario.metadata or {}
        town = self.args.town or metadata.get("town")
        if getattr(self.args, "skip_load_world", False):
            print("[leaderboard] --skip-load-world set; using current CARLA world", flush=True)
            self.mark_rpc("client.get_world")
            self.world = self.client.get_world()
        elif town:
            self.world = self.load_world(town, scenario.scene_id)
        else:
            print("[leaderboard] no town configured; using current CARLA world", flush=True)
            self.world = self.client.get_world()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(self.args.fixed_delta_seconds)
        self.mark_rpc("world.apply_settings_sync")
        self.world.apply_settings(settings)
        self.set_spectator_from_metadata(metadata)
        return self.world

    def set_spectator_from_metadata(self, metadata):
        if getattr(self.args, "spectator_mode", "ego_start") == "none":
            return
        ego_start = metadata.get("ego_start") or metadata.get("ego_spawn")
        loc = metadata_location(ego_start)
        if not loc:
            return
        rotation = ego_start.get("rotation", {}) if isinstance(ego_start, dict) else {}
        yaw = float(rotation.get("yaw", 0.0)) if isinstance(rotation, dict) else 0.0
        try:
            spectator = self.world.get_spectator()
            self.mark_rpc("spectator.set_transform_start")
            spectator.set_transform(
                self.carla.Transform(
                    self.carla.Location(x=loc[0], y=loc[1], z=loc[2] + 25.0),
                    self.carla.Rotation(pitch=-65.0, yaw=yaw, roll=0.0),
                )
            )
            print(f"[leaderboard] spectator set near ego_start ({loc[0]:.1f}, {loc[1]:.1f}, {loc[2]:.1f})", flush=True)
        except RuntimeError as exc:
            print(f"[leaderboard] spectator setup skipped: {exc}", flush=True)

    def map_candidates(self, town):
        candidates = [str(town)]
        if not str(town).startswith("/Game/"):
            candidates.append(f"/Game/Carla/Maps/{town}")
        return candidates

    def load_world(self, town, scene_id):
        candidates = self.map_candidates(town)
        if getattr(self.args, "map_load_mode", "api") == "helper":
            return self.load_world_with_helper(candidates, scene_id)
        return self.load_world_with_api(candidates, scene_id)

    def load_world_with_api(self, candidates, scene_id):
        original_timeout = float(getattr(self.args, "carla_timeout", 180.0))
        map_timeout = float(getattr(self.args, "map_load_timeout", original_timeout))
        self.client.set_timeout(map_timeout)
        try:
            last_error = None
            for candidate in candidates:
                try:
                    print(f"[leaderboard] loading CARLA map via API: {candidate}", flush=True)
                    self.mark_rpc(f"client.load_world:{candidate}")
                    world = self.client.load_world(candidate)
                    self.mark_rpc("world.get_map")
                    print(f"[leaderboard] loaded CARLA map: {world.get_map().name}", flush=True)
                    return world
                except RuntimeError as exc:
                    if self.is_carla_error(exc) and not self.check_carla_port():
                        self._carla_alive = False
                        raise CarlaUnavailableError(f"{scene_id}: CARLA disappeared while loading {candidate}: {exc}") from exc
                    last_error = exc
                    print(f"[leaderboard] load_world failed for {candidate}: {exc}", flush=True)
            raise RuntimeError(
                f"failed to load map for {scene_id}; tried {candidates}. "
                f"Last error: {last_error}"
            )
        finally:
            self.client.set_timeout(original_timeout)

    def load_world_with_helper(self, candidates, scene_id):
        helper = Path(__file__).resolve().parents[2] / "scripts" / "carla_control.py"
        last_error = None
        for candidate in candidates:
            cmd = [
                sys.executable,
                str(helper),
                "--host",
                str(self.args.host),
                "--port",
                str(self.args.port),
                "--timeout",
                str(float(self.args.map_load_timeout)),
                "--wait",
                "--map",
                str(candidate),
                "--sleep-after-load",
                str(float(self.args.map_load_sleep)),
                "--print-world",
            ]
            print(f"[leaderboard] loading CARLA map via helper: {candidate}", flush=True)
            result = subprocess.run(cmd, text=True, capture_output=True, timeout=float(self.args.map_load_timeout) + 10.0)
            if result.stdout:
                print(result.stdout.rstrip(), flush=True)
            if result.returncode == 0:
                world = self.client.get_world()
                print(f"[leaderboard] loaded CARLA map: {world.get_map().name}", flush=True)
                return world
            last_error = result.stderr.strip() or f"exit={result.returncode}"
            if last_error:
                print(f"[leaderboard] helper load failed for {candidate}: {last_error}", flush=True)
        raise RuntimeError(
            f"failed to load map for {scene_id}; tried {candidates}. "
            f"Last error: {last_error}"
        )

    def restore_world(self):
        if not self.world:
            return
        if not self._carla_alive and not self.probe_carla_alive():
            return
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.mark_rpc("world.apply_settings_async")
        self.world.apply_settings(settings)

    def set_spectator_follow_ego(self, ego):
        if getattr(self.args, "spectator_mode", "ego_start") != "ego_follow":
            return
        if not self.actor_alive(ego):
            return
        try:
            tf = ego.get_transform()
            forward = tf.get_forward_vector()
            spectator = self.world.get_spectator()
            loc = tf.location + self.carla.Location(z=3.0) - forward * 6.0
            self.mark_rpc("spectator.set_transform_follow")
            spectator.set_transform(
                self.carla.Transform(
                    loc,
                    self.carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw, roll=0.0),
                )
            )
        except RuntimeError as exc:
            if self.is_carla_error(exc):
                self._carla_alive = False
            else:
                print(f"[leaderboard] spectator follow skipped: {exc}", flush=True)

    def natural_goal_xy(self, config):
        return trajectory_goal_xy(config)

    def actor_alive(self, actor):
        try:
            return bool(actor and actor.is_alive)
        except RuntimeError:
            return False

    def check_natural_termination(
        self,
        ego,
        frame,
        goal_xy,
        goal_reached_ticks,
    ):
        if getattr(self.args, "disable_natural_end", False):
            return None, goal_reached_ticks
        if not self.actor_alive(ego):
            return "ego_destroyed", goal_reached_ticks

        threshold = float(getattr(self.args, "natural_end_distance_m", 5.0))
        min_ticks = max(1, int(getattr(self.args, "natural_end_min_ticks", 5)))
        if goal_xy and frame:
            loc = frame.get("ego", {}).get("location", [0.0, 0.0])
            dist = math.hypot(float(loc[0]) - goal_xy[0], float(loc[1]) - goal_xy[1])
            goal_reached_ticks = goal_reached_ticks + 1 if dist <= threshold else 0
            if goal_reached_ticks >= min_ticks:
                return "ego_reached_goal", goal_reached_ticks

        return None, goal_reached_ticks

    def find_scene_ego(self, scenario):
        metadata = scenario.metadata or {}
        role_values = []
        for key in ("ego_role_names", "ego_role_name"):
            value = metadata.get(key)
            if isinstance(value, list):
                role_values.extend(value)
            elif isinstance(value, str):
                role_values.extend(value.split(","))
        role_values.extend(self.args.ego_role_name.split(","))
        role_names = []
        for value in role_values:
            value = str(value).strip()
            if value and value not in role_names:
                role_names.append(value)

        try:
            self.mark_rpc("world.get_actors_find_ego")
            actors = list(self.world.get_actors().filter("vehicle.*"))
        except RuntimeError as exc:
            self.raise_if_carla_unavailable(exc, "find_scene_ego.get_actors")
        for role_name in role_names:
            for actor in actors:
                if actor.attributes.get("role_name") == role_name:
                    return actor

        ego_type_id = metadata.get("ego_type_id") or metadata.get("ego_blueprint") or self.args.ego_type_id
        ego_start = metadata_location(metadata.get("ego_start") or metadata.get("ego_spawn"))
        if ego_type_id:
            matches = [actor for actor in actors if actor.type_id == ego_type_id]
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1 and ego_start:
                start = self.carla.Location(x=ego_start[0], y=ego_start[1], z=ego_start[2])
                ranked = []
                for actor in matches:
                    try:
                        ranked.append((actor.get_location().distance(start), actor))
                    except RuntimeError:
                        continue
                ranked.sort(key=lambda item: item[0])
                radius = float(metadata.get("ego_start_match_radius_m", 8.0))
                close = [item for item in ranked if item[0] <= radius]
                if len(close) == 1:
                    return close[0][1]
                if ranked and (len(ranked) == 1 or ranked[0][0] + 1.0 < ranked[1][0]):
                    return ranked[0][1]
            if len(matches) > 1:
                raise RuntimeError(f"{scenario.scene_id}: ambiguous ego type_id={ego_type_id}; add role_name or ego_start metadata")

        if ego_start:
            start = self.carla.Location(x=ego_start[0], y=ego_start[1], z=ego_start[2])
            ranked = []
            for actor in actors:
                try:
                    ranked.append((actor.get_location().distance(start), actor))
                except RuntimeError:
                    continue
            ranked.sort(key=lambda item: item[0])
            radius = float(metadata.get("ego_start_match_radius_m", 8.0))
            close = [item for item in ranked if item[0] <= radius]
            if len(close) == 1:
                return close[0][1]

        if len(actors) == 1 and not (ego_type_id or ego_start or role_names):
            return actors[0]
        return None

    def advance_world_for_collection(self, world, wait_timeout=None):
        if self.args.ego_mode == "scene_ego" and getattr(self.args, "scene_drives_ticks", True):
            seconds = float(self.args.tick_wait_timeout) if wait_timeout is None else max(0.5, float(wait_timeout))
            try:
                self.mark_rpc("world.wait_for_tick")
                return world.wait_for_tick(seconds=seconds)
            except RuntimeError as exc:
                self.raise_if_carla_unavailable(exc, "world.wait_for_tick")
        try:
            self.mark_rpc("world.tick")
            return world.tick()
        except RuntimeError as exc:
            self.raise_if_carla_unavailable(exc, "world.tick")

    def spawn_agent_ego(self, scenario):
        metadata = scenario.metadata or {}
        ego_meta = metadata.get("ego_start") or metadata.get("ego_spawn")
        if not ego_meta:
            raise RuntimeError(f"{scenario.scene_id}: missing ego_start for agent_ego")
        bp_id = metadata.get("ego_blueprint") or metadata.get("ego_type_id") or self.args.ego_blueprint
        bp = self.world.get_blueprint_library().find(bp_id)
        bp.set_attribute("role_name", "hero")
        transform = dict_to_transform(self.carla, ego_meta)
        ego = self.world.try_spawn_actor(bp, transform)
        if not ego:
            raise RuntimeError(f"{scenario.scene_id}: failed to spawn agent ego {bp_id}")
        return ego

    def start_scene_process(self, scenario, output_dir):
        env = os.environ.copy()
        env["LEADERBOARD_SCENE_ID"] = scenario.scene_id
        env["LEADERBOARD_OUTPUT_DIR"] = str(output_dir)
        env["LEADERBOARD_EGO_MODE"] = self.args.ego_mode
        env["LEADERBOARD_CARLA_HOST"] = self.args.host
        env["LEADERBOARD_CARLA_PORT"] = str(self.args.port)
        env["ROADTAILBENCH_SCENE_ID"] = scenario.scene_id
        env["ROADTAILBENCH_OUTPUT_DIR"] = str(output_dir)
        env["ROADTAILBENCH_EGO_MODE"] = self.args.ego_mode
        env["ROADTAILBENCH_CARLA_HOST"] = self.args.host
        env["ROADTAILBENCH_CARLA_PORT"] = str(self.args.port)
        cmd = [sys.executable, str(scenario.script_path)]
        stdout_path = Path(output_dir) / "scenario_stdout.log"
        stdout_file = stdout_path.open("w", encoding="utf-8", errors="replace")
        proc = subprocess.Popen(
            cmd,
            cwd=str(scenario.script_path.parent),
            env=env,
            stdout=stdout_file,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        proc._leaderboard_stdout_file = stdout_file
        proc._leaderboard_stdout_path = stdout_path
        return proc

    def load_adapter(self):
        if not self.args.agent:
            return None
        module_name, class_name = self.args.agent.split(":", 1) if ":" in self.args.agent else (self.args.agent, "Adapter")
        module = importlib.import_module(module_name)
        adapter_cls = getattr(module, class_name)
        adapter = adapter_cls()
        config = {"config": self.args.agent_config}
        adapter.setup(config)
        return adapter

    def build_config(self, scenario):
        metadata = dict(scenario.metadata or {})
        config = {
            "schema_version": "leaderboard.runtime_config.v1",
            "scenario_id": scenario.scene_id,
            "route_id": scenario.scene_id,
            "script_path": str(scenario.script_path),
            "metadata_path": str(scenario.metadata_path) if scenario.metadata_path else None,
            "town": metadata.get("town") or self.args.town or self.world.get_map().name.split("/")[-1],
            "ego_mode": self.args.ego_mode,
            "reference_speed_kmh": 50.0,
            "reference_trajectory": [],
            "natural_end_distance_m": float(getattr(self.args, "natural_end_distance_m", 5.0)),
            "natural_end_min_ticks": int(getattr(self.args, "natural_end_min_ticks", 5)),
        }
        config.update(metadata)
        if not config.get("reference_trajectory"):
            config["reference_trajectory"] = config.get("route") or config.get("centerline_route") or []
        if not config.get("reference_trajectory") and config.get("ego_start") and config.get("ego_end"):
            config["reference_trajectory"] = [config["ego_start"]["location"], config["ego_end"]["location"]]
        config.setdefault("route", reference_xy(config))
        config.setdefault("centerline_route", config.get("route", []))
        return config

    def run_scenario(self, scenario):
        started_at = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(self.args.output_root) / f"{scenario.scene_id}_{started_at}"
        output_dir.mkdir(parents=True, exist_ok=True)
        proc = logger = ego = adapter = video = None
        status, error, ticks, script_exit_tick = "started", "", 0, None
        termination_reason = ""
        failure_class = ""
        started_wall = time.time()
        first_sim_time = last_sim_time = None
        scenario_timeout = float(getattr(self.args, "scenario_timeout", 0.0) or 0.0)
        scenario_deadline = time.time() + scenario_timeout if scenario_timeout > 0.0 else None
        goal_reached_ticks = 0
        carla_alive_before = self.probe_carla_alive()
        carla_alive_after = carla_alive_before

        def check_scenario_timeout():
            if scenario_deadline is not None and time.time() >= scenario_deadline:
                raise ScenarioTimeoutError(f"{scenario.scene_id}: scenario-timeout {scenario_timeout:.1f}s exceeded")

        def remaining_wait_timeout():
            if scenario_deadline is None:
                return None
            remaining = scenario_deadline - time.time()
            if remaining <= 0.0:
                raise ScenarioTimeoutError(f"{scenario.scene_id}: scenario-timeout {scenario_timeout:.1f}s exceeded")
            return min(float(self.args.tick_wait_timeout), remaining)

        try:
            if not carla_alive_before:
                raise CarlaUnavailableError(f"{scenario.scene_id}: CARLA is not reachable before scenario start")
            check_scenario_timeout()
            world = self.connect_world(scenario)
            if self.args.ego_mode == "agent_ego":
                ego = self.spawn_agent_ego(scenario)
                adapter = self.load_adapter()
            proc = self.start_scene_process(scenario, output_dir)
            deadline = time.time() + float(self.args.ego_wait_timeout)
            while time.time() < deadline:
                check_scenario_timeout()
                self.advance_world_for_collection(world, remaining_wait_timeout())
                ego = ego or self.find_scene_ego(scenario)
                if ego or (proc and proc.poll() is not None):
                    break
            check_scenario_timeout()
            if not ego:
                raise RuntimeError(f"{scenario.scene_id}: ego vehicle not found")
            config = self.build_config(scenario)
            goal_xy = self.natural_goal_xy(config)
            self.set_spectator_follow_ego(ego)
            logger = RuntimeFrameLogger(output_dir, scenario, config)
            logger.attach_collision_sensor(self.carla, world, ego)
            if getattr(self.args, "record_video", False):
                video = RuntimeVideoRecorder(self.carla, world, ego, output_dir, self.args)
                video.start()
            while ticks < int(self.args.max_ticks):
                check_scenario_timeout()
                self.advance_world_for_collection(world, remaining_wait_timeout())
                ticks += 1
                if not self.actor_alive(ego):
                    termination_reason = "ego_destroyed"
                    break
                control = None
                if adapter:
                    obs = {"frame": ticks, "ego": ego, "world": world, "config": config}
                    control = adapter.run_step(obs)
                    if hasattr(control, "to_carla"):
                        control = control.to_carla(self.carla)
                    ego.apply_control(control)
                else:
                    try:
                        control = ego.get_control()
                    except AttributeError:
                        control = None
                    except RuntimeError as exc:
                        self.raise_if_carla_unavailable(exc, "ego.get_control")
                logger.log_tick(world, ego, control, actor_radius_m=self.args.actor_log_radius_m)
                self.set_spectator_follow_ego(ego)
                latest_frame = logger._frames[-1] if logger._frames else None
                if latest_frame:
                    frame_time = float(latest_frame.get("time", 0.0))
                    if first_sim_time is None:
                        first_sim_time = frame_time
                    last_sim_time = frame_time
                    natural_reason, goal_reached_ticks = self.check_natural_termination(
                        ego,
                        latest_frame,
                        goal_xy,
                        goal_reached_ticks,
                    )
                    if natural_reason:
                        termination_reason = natural_reason
                        break
                if proc and proc.poll() is not None:
                    if script_exit_tick is None:
                        script_exit_tick = ticks
                    if ticks - script_exit_tick >= int(self.args.min_ticks_after_script_exit):
                        termination_reason = "script_exit"
                        break
            status = "completed"
        except ScenarioTimeoutError as exc:
            status = "completed_timeout"
            termination_reason = "scenario_timeout"
            failure_class = "scenario_timeout"
            error = str(exc)
        except CarlaUnavailableError as exc:
            status = "carla_crashed"
            termination_reason = "carla_unavailable"
            failure_class = "carla_unavailable"
            error = f"{exc}\n{traceback.format_exc()}"
        except Exception as exc:
            status = "failed"
            termination_reason = "exception"
            failure_class = "exception"
            error = f"{exc}\n{traceback.format_exc()}"
        finally:
            carla_alive_after = self._carla_alive and self.probe_carla_alive()
            if video:
                try:
                    video_outputs = video.close(carla_alive=carla_alive_after)
                except Exception as exc:
                    video_outputs = {"video_error": str(exc)}
            else:
                video_outputs = {}
            if carla_alive_after:
                try:
                    self.restore_world()
                except Exception:
                    pass
            if adapter:
                try:
                    adapter.destroy()
                except Exception:
                    pass
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=float(getattr(self.args, "process_exit_timeout", 2.0)))
                except subprocess.TimeoutExpired:
                    proc.kill()
            if proc and getattr(proc, "_leaderboard_stdout_file", None):
                try:
                    proc._leaderboard_stdout_file.close()
                except Exception:
                    pass
            elapsed_wall = time.time() - started_wall
            elapsed_sim = None
            if first_sim_time is not None and last_sim_time is not None:
                elapsed_sim = max(0.0, last_sim_time - first_sim_time)
            summary = {
                "scene_id": scenario.scene_id,
                "status": status,
                "error": error,
                "ticks": ticks,
                "output_dir": str(output_dir),
                "elapsed_wall_seconds": elapsed_wall,
                "elapsed_sim_seconds": elapsed_sim,
                "timeout_seconds": scenario_timeout or None,
                "termination_reason": termination_reason or status,
                "failure_class": failure_class,
                "last_rpc": self._last_rpc,
                "carla_alive_before": carla_alive_before,
                "carla_alive_after": carla_alive_after,
                "scenario_process_returncode": proc.poll() if proc else None,
            }
            if proc and getattr(proc, "_leaderboard_stdout_path", None):
                summary["scenario_stdout_log"] = str(proc._leaderboard_stdout_path)
            if logger:
                summary["outputs"] = logger.close(summary, carla_alive=carla_alive_after)
            else:
                save_json(output_dir / "leaderboard_run_summary.json", summary)
                summary["outputs"] = {"summary": str(output_dir / "leaderboard_run_summary.json")}
            if video_outputs:
                summary.setdefault("outputs", {}).update(video_outputs)
                save_json(output_dir / "leaderboard_run_summary.json", summary)
            if getattr(self.args, "video_synth_360", False) and getattr(self.args, "video_save_frames", False):
                try:
                    from leaderboard.cli.video import synth_360

                    path = synth_360(output_dir, fps=float(getattr(self.args, "video_fps", 10.0)))
                    summary.setdefault("outputs", {})["video_360"] = path
                    save_json(output_dir / "leaderboard_run_summary.json", summary)
                except Exception as exc:
                    summary.setdefault("outputs", {})["video_360_error"] = str(exc)
                    save_json(output_dir / "leaderboard_run_summary.json", summary)
            if carla_alive_after and self.args.cleanup_ego and ego and self.args.ego_mode == "agent_ego":
                try:
                    if ego.is_alive:
                        ego.destroy()
                except Exception:
                    pass
        return summary

    def run(self, scenarios):
        summaries = []
        batch_status = "completed"
        for scenario in scenarios:
            print(f"[leaderboard] running {scenario.scene_id}: {scenario.script_path}", flush=True)
            summary = self.run_scenario(scenario)
            summaries.append(summary)
            save_json(Path(self.args.output_root) / "leaderboard_batch_summary.json", summaries)
            print(f"[leaderboard] {scenario.scene_id} {summary['status']} ticks={summary['ticks']}", flush=True)
            if summary.get("status") not in ("completed", "completed_timeout") and summary.get("error"):
                first_line = summary["error"].splitlines()[0] if summary["error"].splitlines() else summary["error"]
                print(f"[leaderboard] error: {first_line}", flush=True)
            if summary.get("status") == "carla_crashed" and getattr(self.args, "abort_on_carla_crash", True):
                batch_status = "aborted_carla_crash"
                print("[leaderboard] CARLA is unavailable; aborting remaining scenarios", flush=True)
                break
        if batch_status != "completed":
            save_json(Path(self.args.output_root) / "leaderboard_batch_status.json", {
                "batch_status": batch_status,
                "completed_scenarios": len(summaries),
                "last_summary": summaries[-1] if summaries else None,
            })
        return summaries
