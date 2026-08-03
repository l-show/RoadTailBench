import carla
import time
import math
import os
import numpy as np

# ==========================================
# PID 控制器类 (保持不变)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, waypoint_location, vehicle_transform):
        v_begin, v_forward = vehicle_transform.location, vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint_location.x - v_begin.x, waypoint_location.y - v_begin.y, 0.0])
        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1: return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        if np.cross(v_vec, w_vec)[2] < 0: _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

# ==========================================
# 辅助函数 (保持不变)
# ==========================================
def get_target_waypoint(vehicle_loc, path_transforms, lookahead_dist=5.0):
    min_dist, closest_index = float('inf'), 0
    for i, t in enumerate(path_transforms):
        dist = vehicle_loc.distance(t.location)
        if dist < min_dist: min_dist, closest_index = dist, i
    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_transforms) - 1):
        current_dist += path_transforms[i].location.distance(path_transforms[i + 1].location)
        target_index = i + 1
        if current_dist > lookahead_dist: break
    return path_transforms[target_index].location

def build_ego_transforms(carla_map, raw_path_points):
    path_transforms = []
    for x, y, yaw in raw_path_points:
        loc = carla.Location(x=x, y=y, z=0.5)
        try:
            waypoint = carla_map.get_waypoint(loc)
            if waypoint:
                loc.z = waypoint.transform.location.z + 0.5
        except Exception:
            pass
        path_transforms.append(carla.Transform(loc, carla.Rotation(yaw=yaw)))
    return path_transforms

RAW_EGO_TRAJECTORY = [
    (183.292, -2.912, -174.977), (182.003, -3.025, -174.977), (179.573, -3.239, -174.977), (177.091, -3.461, -174.431),
    (174.560, -3.722, -173.450), (172.132, -4.022, -172.783), (169.804, -4.333, -171.765), (167.390, -4.742, -169.026),
    (164.944, -5.257, -167.426), (162.578, -5.886, -163.226), (160.235, -6.602, -161.803), (157.919, -7.401, -160.716),
    (155.606, -8.210, -160.716), (153.293, -9.019, -160.716), (151.033, -9.826, -158.304), (148.906, -10.823, -154.315),
    (146.789, -11.841, -154.315), (144.624, -12.875, -160.742), (142.196, -13.467, -167.508), (139.752, -13.992, -169.849),
    (137.335, -14.395, -171.057), (134.911, -14.748, -172.633), (132.476, -15.013, -175.887), (130.028, -15.122, -178.324),
    (127.529, -15.108, 177.753), (125.035, -14.947, 174.552), (122.564, -14.572, 168.117), (120.088, -13.968, 163.665),
    (117.683, -13.124, 157.871), (115.301, -12.081, 153.906), (113.034, -10.915, 152.371), (110.919, -9.781, 151.322),
    (108.770, -8.605, 151.322), (106.551, -7.250, 147.275), (104.406, -5.871, 147.275), (102.219, -4.466, 147.275),
    (99.990, -3.033, 147.275), (97.761, -1.601, 147.275), (95.446, -0.116, 147.415), (93.166, 1.329, 147.991),
    (90.830, 2.781, 148.411), (88.487, 4.220, 148.620), (86.096, 5.678, 148.620), (83.662, 7.160, 148.690),
    (81.138, 8.688, 148.830), (78.614, 10.215, 148.830), (76.090, 11.741, 148.830), (73.480, 13.320, 148.830),
    (70.821, 15.008, 145.700), (68.048, 16.975, 143.810), (65.183, 19.153, 141.313), (62.013, 21.754, 140.388),
    (58.689, 24.481, 141.664), (55.329, 26.828, 148.388), (51.701, 28.619, 156.526), (48.070, 30.037, 161.021),
    (44.461, 31.053, 166.196), (40.836, 31.788, 171.209), (37.272, 32.290, 172.038), (33.648, 32.709, 176.773),
    (29.953, 32.875, 179.005), (26.404, 32.928, 179.145), (22.705, 32.951, -179.186), (18.956, 32.862, -178.276),
    (15.213, 32.645, -175.263), (11.489, 32.213, -171.885), (7.847, 31.563, -168.292), (4.294, 30.731, -164.688),
    (1.083, 29.778, -162.516), (-2.431, 28.619, -160.737), (-5.778, 27.298, -154.840), (-7.213, 26.592, -153.662),
    (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662),
    (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662),
]

def is_actor_alive(actor):
    return bool(actor is not None and getattr(actor, "is_alive", False))

def find_existing_ego(world):
    actors = world.get_actors().filter("vehicle.*")
    for role_name in ("ego", "hero"):
        for actor in actors:
            if actor.attributes.get("role_name") == role_name:
                return actor

    start_x, start_y, _ = RAW_EGO_TRAJECTORY[0]
    start_loc = carla.Location(x=start_x, y=start_y, z=0.5)
    model3_matches = []
    for actor in actors:
        if actor.type_id != "vehicle.tesla.model3":
            continue
        try:
            model3_matches.append((actor.get_location().distance(start_loc), actor))
        except RuntimeError:
            continue
    model3_matches.sort(key=lambda item: item[0])
    if model3_matches and model3_matches[0][0] <= 8.0:
        return model3_matches[0][1]
    return None

def cleanup_actors(client, actors):
    commands = []
    seen = set()
    for actor in actors:
        if not is_actor_alive(actor):
            continue
        actor_id = getattr(actor, "id", id(actor))
        if actor_id in seen:
            continue
        seen.add(actor_id)
        commands.append(carla.command.DestroyActor(actor_id))
    if commands:
        client.apply_batch(commands)

# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    ego_transforms = build_ego_transforms(carla_map, RAW_EGO_TRAJECTORY)

    actor_list = []
    spawned_scene_ego = False
    vehicle_ego = None
    lon_ctrl = None
    lat_ctrl = None
    flyby_triggered = False
    active_props = []
    props_rel_x = []
    props_rel_y = []
    props_rel_z = []
    is_sticky = []
    goal_reached_ticks = 0
    stop_requested = False

    trigger_loc = carla.Location(x=117.856, y=-12.274, z=4.555)
    goal_x, goal_y, _ = RAW_EGO_TRAJECTORY[-1]
    ego_mode = os.environ.get("LEADERBOARD_EGO_MODE") or os.environ.get("ROADTAILBENCH_EGO_MODE") or "scene_ego"
    use_external_ego = ego_mode in ("agent_ego", "external_ego")

    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=100.0, precipitation_deposits=100.0, wind_intensity=100.0,
        sun_azimuth_angle=90, sun_altitude_angle=10, fog_density=10.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=50.0, scattering_intensity=11.0, mie_scattering_scale=0.13,
        rayleigh_scattering_scale=0.0331, dust_storm=0.0
    )
    world.set_weather(weather)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        # 2. 生成 Ego 车辆
        if use_external_ego:
            vehicle_ego = find_existing_ego(world)
            if vehicle_ego:
                print("[RTB083] Using external/agent ego actor.")
            else:
                print("[RTB083] Waiting for external/agent ego actor.")
        else:
            ego_bp = bp_lib.find('vehicle.tesla.model3')
            if ego_bp.has_attribute('role_name'):
                pass
            vehicle_ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
            spawned_scene_ego = bool(vehicle_ego)

        if spawned_scene_ego:
            lon_ctrl = PIDLongitudinalController()
            lat_ctrl = PIDLateralController()
            initial_lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position | \
                             carla.VehicleLightState.Fog | carla.VehicleLightState.Interior
            print("Ego 车辆已生成。")

        # 3. 生成 Auto 车辆
        start_auto = carla.Transform(carla.Location(x=-87.936, y=-35.091, z=5.138), carla.Rotation(yaw=24.305))
        vehicle_auto = world.try_spawn_actor(bp_lib.find('vehicle.audi.tt'), start_auto)
        if vehicle_auto:
            actor_list.append(vehicle_auto)
            vehicle_auto.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(vehicle_auto, -180.0)
            vehicle_auto.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position))

        # ==========================================
        # 4. 主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()

            if use_external_ego and not is_actor_alive(vehicle_ego):
                vehicle_ego = find_existing_ego(world)

            comp_time = time.time() - start_time
            if comp_time < 0.05: time.sleep(0.05 - comp_time)

    except Exception as e:
        print(f"异常: {e}")
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if actor_list and not stop_requested:
            cleanup_actors(client, actor_list)
        print("清理完成。")

if __name__ == '__main__':
    main()
