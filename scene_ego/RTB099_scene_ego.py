import carla
import time
import math
import numpy as np

# ==========================================
# 基础控制算法 (PID) - 保留并修复了缩进和拼写
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        _ie = np.clip(_ie, -2.0, 2.0)
        # 加速上限设为0.8，保证加速足够合理
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 0.8)

class PIDLateralController:
    def __init__(self, K_P=1.0, K_I=0.01, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)
        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        norm = np.linalg.norm(target_vector)
        if norm < 0.1: return 0.0
        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.7, 0.7)

def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_loc):
    target_speed_ms = target_speed_kmh / 3.6
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed_ms, current_speed_ms)
    steer_output = pid_lat.run_step(target_loc, tf)

    if abs(steer_output) < 0.02: steer_output = 0.0
    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)

# ==========================================
# 辅助函数
# ==========================================
def preprocess_trajectory(raw_trajectory, min_distance=0.5):
    """去除轨迹中的重复点或距离过近的点"""
    if not raw_trajectory:
        return []
    cleaned_traj = [raw_trajectory[0]]
    for point in raw_trajectory[1:]:
        last_point = cleaned_traj[-1]
        dist = math.hypot(point[0] - last_point[0], point[1] - last_point[1])
        if dist >= min_distance:
            cleaned_traj.append(point)
    print(f"轨迹优化: 原 {len(raw_trajectory)} 点 -> 现 {len(cleaned_traj)} 点")
    return cleaned_traj

def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None:
        print(f"[{vehicle.type_id}] 彻底脱离地图，被销毁！")
        vehicle.destroy()
        return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 6.0:
        print(f"[{vehicle.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        vehicle.destroy()
        return True
    return False

# ==========================================
# 轨迹数据定义
# ==========================================
RAW_V1_TRAJECTORY = [
    (66.975, -98.455, 129.504), (66.975, -98.455, 129.504), (66.975, -98.455, 129.504), (66.975, -98.455, 129.434),
    (66.187, -97.492, 129.289), (62.946, -93.61, 131.084), (59.675, -89.867, 131.154), (56.263, -86.023, 132.288),
    (52.866, -82.387, 134.08), (49.324, -78.659, 132.54), (45.966, -74.88, 131.325), (42.683, -71.146, 131.185),
    (39.321, -67.261, 130.829), (35.993, -63.343, 130.126), (32.784, -59.536, 130.126), (29.575, -55.728, 130.126),
    (26.259, -51.793, 130.126), (22.881, -47.909, 131.823), (19.494, -44.142, 132.248), (16.065, -40.278, 130.913),
    (12.945, -36.396, 125.056), (10.427, -31.923, 114.911), (8.441, -27.373, 111.064), (6.939, -22.562, 101.564),
    (6.213, -17.653, 95.975), (6.05, -12.527, 89.04), (6.356, -7.563, 82.549), (7.207, -2.478, 77.101),
    (8.792, 2.328, 63.889), (11.333, 6.613, 58.248), (13.956, 10.851, 58.248), (16.721, 15.194, 56.884),
    (19.496, 19.434, 56.458), (22.419, 23.673, 54.747), (25.391, 27.879, 54.817), (28.116, 32.146, 61.862),
    (30.458, 36.733, 64.665), (32.228, 41.549, 73.967), (33.389, 46.542, 80.614), (33.881, 51.644, 88.431),
    (33.825, 56.699, 93.041), (33.276, 61.815, 98.901), (32.454, 66.733, 100.844), (31.248, 71.745, 106.015),
    (29.613, 76.46, 112.529), (27.567, 81.106, 113.947), (25.523, 85.845, 111.701), (23.656, 90.573, 111.138),
    (21.811, 95.398, 110.502), (20.084, 100.09, 110.082), (18.339, 104.863, 110.082), (16.594, 109.638, 110.082),
    (14.82, 114.49, 110.082), (13.075, 119.263, 110.082), (11.358, 123.959, 110.082), (9.584, 128.812, 110.082),
    (7.811, 133.664, 110.082), (6.011, 138.421, 111.143), (4.119, 143.229, 111.857), (2.195, 148.024, 111.857),
    (1.699, 149.262, 111.857), (1.699, 149.262, 111.857), (1.699, 149.262, 111.857)
]

RAW_EGO_TRAJECTORY = [
    (28.192, 89.899, -63.699), (28.192, 89.899, -63.699), (28.192, 89.899, -63.699), (28.192, 89.899, -63.699),
    (28.192, 89.899, -63.699), (28.82, 88.607, -64.192), (30.384, 85.069, -68.61), (31.745, 81.512, -70.259),
    (32.984, 77.911, -71.834), (34.086, 74.265, -74.037), (35.095, 70.656, -77.415), (35.834, 66.854, -80.075),
    (36.429, 63.155, -80.944), (36.91, 59.316, -86.517), (36.936, 55.455, -91.375), (36.795, 51.666, -93.593),
    (36.441, 47.835, -97.195), (35.854, 44.033, -100.355), (35.011, 40.28, -104.855), (33.919, 36.722, -109.924),
    (32.492, 33.212, -114.184), (30.846, 29.796, -115.961), (28.961, 26.52, -130.459), (26.035, 24.136, -148.422),
    (22.867, 22.177, -142.465), (20.17, 19.448, -127.02), (18.246, 16.264, -116.165), (16.639, 12.747, -113.264),
    (15.15, 9.178, -112.338), (13.766, 5.634, -110.962), (12.429, 2.007, -110.036), (11.183, -1.649, -108.243),
    (9.812, -5.119, -115.277), (8.213, -8.629, -109.396), (7.265, -12.307, -95.624), (7.629, -16.12, -76.327),
    (8.844, -19.777, -69.07), (10.267, -23.233, -67.127), (11.727, -26.674, -66.912), (13.241, -30.227, -66.912),
    (15.057, -33.484, -55.01), (17.288, -36.633, -54.504), (19.581, -39.575, -49.176), (22.062, -42.364, -47.042),
    (24.714, -45.164, -46.045), (27.302, -47.854, -46.543), (29.924, -50.683, -47.894), (32.432, -53.535, -49.24),
    (34.872, -56.366, -49.24), (37.387, -59.297, -49.453), (39.92, -62.211, -48.386), (42.484, -65.098, -48.246),
    (45.016, -67.929, -48.176), (47.538, -70.769, -48.671), (50.046, -73.622, -48.671), (52.595, -76.521, -48.671),
    (55.145, -79.421, -48.671), (57.613, -82.226, -48.671), (60.08, -85.032, -48.671), (63.849, -89.318, -48.671),
    (67.961, -93.994, -48.741), (72.185, -98.844, -49.021)
]

# 预处理轨迹去除原数据中的连续重复点
V1_TRAJECTORY = preprocess_trajectory(RAW_V1_TRAJECTORY)
EGO_TRAJECTORY = preprocess_trajectory(RAW_EGO_TRAJECTORY)

# ==========================================
# 主程序
# ==========================================

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (72.185, -98.844)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.audi.tt'
_RTB_OPT_EGO_ROLE_NAMES = ['ego', 'hero']
_RTB_OPT_GOAL_RADIUS_M = 5.0
_RTB_OPT_GOAL_HITS = 0

def _rtb_opt_is_alive(actor):
    return bool(actor is not None and hasattr(actor, 'is_alive') and actor.is_alive)

def _rtb_opt_iter_actor_values(value, seen=None):
    if seen is None:
        seen = set()
    obj_id = id(value)
    if obj_id in seen:
        return
    seen.add(obj_id)
    if _rtb_opt_is_alive(value) and hasattr(value, 'get_location'):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _rtb_opt_iter_actor_values(item, seen)
    elif isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _rtb_opt_iter_actor_values(item, seen)

def _rtb_opt_actor_matches_ego(actor):
    if not _rtb_opt_is_alive(actor):
        return False
    try:
        role_name = actor.attributes.get('role_name', '')
        if role_name in _RTB_OPT_EGO_ROLE_NAMES:
            return True
    except Exception:
        pass
    try:
        if _RTB_OPT_EGO_TYPE_ID and actor.type_id == _RTB_OPT_EGO_TYPE_ID:
            return True
    except Exception:
        pass
    return False

def _rtb_opt_find_ego(local_vars):
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
    for name in preferred_names:
        if name in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[name]):
                if _rtb_opt_actor_matches_ego(actor) or 'ego' in name.lower():
                    return actor
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            if _rtb_opt_actor_matches_ego(actor):
                return actor
    return None

def _rtb_opt_collect_scene_actors(local_vars, world):
    actors = []
    seen = set()

    def add(actor):
        if not _rtb_opt_is_alive(actor):
            return
        try:
            actor_id = actor.id
        except Exception:
            actor_id = id(actor)
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for key in ('actor_list', 'actors', 'vehicles', 'spawned_actors'):
        if key in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[key]):
                add(actor)
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*', 'sensor.*', 'controller.*', 'static.prop.*', 'static.trigger.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass
    return actors

def _rtb_opt_cleanup_scene(local_vars, client, world):
    actors = _rtb_opt_collect_scene_actors(local_vars, world)
    try:
        commands = [carla.command.DestroyActor(actor.id) for actor in actors if _rtb_opt_is_alive(actor)]
        if commands:
            client.apply_batch(commands)
        return
    except Exception:
        pass
    for actor in actors:
        try:
            if _rtb_opt_is_alive(actor):
                actor.destroy()
        except Exception:
            pass

def _rtb_opt_goal_guard(local_vars, client, world):
    global _RTB_OPT_GOAL_HITS
    if _RTB_OPT_EGO_GOAL_XY is None:
        _RTB_OPT_GOAL_HITS = 0
        return False
    ego_actor = _rtb_opt_find_ego(local_vars)
    if not _rtb_opt_is_alive(ego_actor):
        _RTB_OPT_GOAL_HITS = 0
        return False
    try:
        loc = ego_actor.get_location()
        dist = ((loc.x - _RTB_OPT_EGO_GOAL_XY[0]) ** 2 + (loc.y - _RTB_OPT_EGO_GOAL_XY[1]) ** 2) ** 0.5
    except Exception:
        _RTB_OPT_GOAL_HITS = 0
        return False
    if dist <= _RTB_OPT_GOAL_RADIUS_M:
        _RTB_OPT_GOAL_HITS += 1
    else:
        _RTB_OPT_GOAL_HITS = 0
    if _RTB_OPT_GOAL_HITS >= 2:
        print('[RoadTailBench Opt] Ego reached trajectory endpoint; cleaning all scene actors and ending simulation.')
        _rtb_opt_cleanup_scene(local_vars, client, world)
        return True
    return False
# === End RoadTailBench Opt guard ===

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【天气修改点】完全对齐截图中的参数
    weather = carla.WeatherParameters(
        cloudiness=10.0,
        precipitation=35.0,
        precipitation_deposits=50.0,
        wind_intensity=75.0,
        sun_azimuth_angle=25.0,
        sun_altitude_angle=10.0,
        fog_density=30.0,  # 截图黄色高亮项
        fog_distance=0.0,
        fog_falloff=0.0,
        wetness=50.0,
        scattering_intensity=0.0,
        mie_scattering_scale=0.1500,
        rayleigh_scattering_scale=0.0500,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    v1_active = False
    ego_active = False

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_v1 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：V1 车 (Mercedes Sprinter) =================
        # 注: 原要求给的 AnimBP 是动画蓝图，无法生成载具实体。此处使用正确的 Sprinter 车辆蓝图
        bp_v1 = bp_lib.find('vehicle.mercedes.sprinter')
        v1_start_x, v1_start_y, v1_start_yaw = V1_TRAJECTORY[0]
        v1_loc = carla.Location(x=v1_start_x, y=v1_start_y, z=0.5)
        v1_loc.z = carla_map.get_waypoint(v1_loc).transform.location.z + 0.5
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(v1_loc, carla.Rotation(yaw=v1_start_yaw)))

        if v1:
            actor_list.append(v1)
            v1_active = True
            print("生成 V1 (Sprinter) 成功。")

        # ================= Actor 2：Ego车 (Audi TT) =================
        bp_ego = bp_lib.find('vehicle.audi.tt')
        if bp_ego.has_attribute('role_name'):
            bp_ego.set_attribute('role_name', 'ego')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '0,255,255')
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=ego_start_yaw)))

        if ego:
            actor_list.append(ego)
            ego_active = True
            print("生成 Ego (Audi TT) 成功。")

        # 【防飞天机制】先进行 10 帧物理预热，确保车辆落地稳定
        for _ in range(10):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

        # 【速度修改点】稳定后赋予物理初始速度 (避免从0加速引起瞬移或打滑)
        if v1_active:
            v1_init_speed_ms = 60.0 / 3.6
            yaw_rad = math.radians(v1_start_yaw)
            v1.set_target_velocity(carla.Vector3D(
                v1_init_speed_ms * math.cos(yaw_rad),
                v1_init_speed_ms * math.sin(yaw_rad),
                0.0))
            print("已赋予 V1 初始速度 60 km/h。")

        if ego_active:
            ego_init_speed_ms = 65.0 / 3.6
            yaw_rad = math.radians(ego_start_yaw)
            ego.set_target_velocity(carla.Vector3D(
                ego_init_speed_ms * math.cos(yaw_rad),
                ego_init_speed_ms * math.sin(yaw_rad),
                0.0))
            print("已赋予 Ego 初始速度 65 km/h。")

        v1_traj_idx = 0
        ego_traj_idx = 0
        print("\n仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

            # ==========================
            # V1 车辆 (Sprinter) 控制
            # ==========================
            if v1_active and v1.is_alive:
                if check_and_handle_out_of_bounds(v1, carla_map):
                    v1_active = False
                elif v1_traj_idx < len(V1_TRAJECTORY):
                    tx, ty, tyaw = V1_TRAJECTORY[v1_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v1.get_location().z)

                    if v1.get_location().distance(target_loc) < 3.5 and v1_traj_idx < len(V1_TRAJECTORY) - 1:
                        v1_traj_idx += 1

                    # V1 保持目标速度 40 km/h
                    apply_pid_control(v1, pid_v1['lon'], pid_v1['lat'], 40.0, target_loc)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))
                    v1_active = False
                    print("\nV1 已到达轨迹终点。")

            # ==========================
            # Ego 车 (Audi TT) 控制
            # ==========================
            if ego_active and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map):
                    ego_active = False
                elif ego_traj_idx < len(EGO_TRAJECTORY):
                    tx, ty, tyaw = EGO_TRAJECTORY[ego_traj_idx]
                    current_loc = ego.get_location()
                    target_loc = carla.Location(x=tx, y=ty, z=current_loc.z)

                    if current_loc.distance(target_loc) < 3.5 and ego_traj_idx < len(EGO_TRAJECTORY) - 1:
                        ego_traj_idx += 1

                    # 【速度动态逻辑】初始45，Y=25减速到20，Y=0加速到40
                    # 由于车辆是按轨迹行驶，通过判断当前坐标Y来决定PID的目标速度
                    ego_y = current_loc.y
                    if ego_y <= 0.0:
                        ego_target_speed = 40.0
                    elif ego_y <= 25.0:
                        ego_target_speed = 20.0
                    else:
                        ego_target_speed = 45.0

                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], ego_target_speed, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    ego_active = False
                    print("\nEgo 已到达轨迹终点。")

            # 如果两辆车都走完了，退出循环
            if not ego_active and not v1_active:
                print("所有车辆已完成测试。")
                break

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")

if __name__ == '__main__':
    main()
