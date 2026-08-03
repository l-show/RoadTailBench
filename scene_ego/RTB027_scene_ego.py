import carla
import time
import math
import numpy as np

# ================= 基础控制算法 (PID) =================
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.6)

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

def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()

    # 强制将坐标投影到最近的合法路面上（忽略高度/细微边界误差）
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    # 如果整个地图都找不到投影点（通常不可能，除非飞出世界边缘）
    if wp_nearest is None:
        print(f"[{vehicle.type_id}] 彻底脱离地图，被销毁！")
        vehicle.destroy()
        return True

    # 计算车辆当前物理位置与路网中心点的绝对距离
    distance = wp_nearest.transform.location.distance(loc)

    # 距离大于 6 米才算真正出界（相当于偏离道路中心线两条车道以上）
    if distance > 6.0:
        print(f"[{vehicle.type_id}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        vehicle.destroy()
        return True

    return False

def apply_initial_velocity(vehicle, speed_kmh, yaw_degrees):
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_degrees)
    vx = speed_ms * math.cos(yaw_rad)
    vy = speed_ms * math.sin(yaw_rad)
    vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))

# ================= 轨迹数据 (原卡车轨迹提供给警车) =================
POLICE_TRAJECTORY = [
    (86.177, 33.616, -109.583), (83.869, 27.148, -109.652), (81.354, 20.087, -109.582),
    (78.933, 12.99, -107.505), (76.669, 5.591, -106.431), (74.575, -1.832, -105.362),
    (72.596, -9.079, -105.649), (70.573, -16.282, -105.649), (68.576, -23.258, -106.075),
    (66.282, -30.598, -110.242), (63.332, -37.582, -117.579), (59.454, -44.129, -124.437),
    (54.909, -50.238, -129.113), (50.078, -56.113, -130.4), (44.862, -61.841, -134.099),
    (39.496, -67.072, -137.835), (33.741, -72.057, -140.039), (27.634, -76.821, -142.939),
    (21.426, -81.461, -143.362), (15.315, -86.022, -142.797), (9.341, -90.558, -142.797),
    (3.166, -95.245, -142.797), (-3.017, -99.919, -143.079), (-9.113, -104.5, -143.079),
    (-15.116, -108.998, -143.362), (-21.244, -113.536, -143.502), (-27.348, -118.106, -143.009),
    (-33.54, -122.771, -142.939), (-33.839, -122.997, -142.87), (-33.839, -122.997, -142.87)
]

# ================= 主程序 =================
EGO_TRAJECTORY = [
    (19.009, -77.257, 35.182), (19.009, -77.257, 35.182), (19.009, -77.257, 35.182),
    (19.009, -77.257, 35.182), (19.009, -77.257, 35.182), (19.009, -77.257, 35.182),
    (19.009, -77.257, 35.182), (20.643, -76.105, 35.182), (24.780, -73.154, 35.784),
    (28.814, -70.200, 37.286), (32.835, -67.091, 38.186), (36.732, -63.959, 39.776),
    (40.576, -60.635, 42.085), (44.181, -57.169, 46.053), (47.578, -53.504, 48.784),
    (50.344, -50.307, 49.194), (51.390, -49.092, 50.322), (52.193, -48.109, 50.800),
    (52.982, -47.141, 50.800), (53.772, -46.173, 50.800), (54.578, -45.195, 50.434),
    (55.388, -44.221, 50.069), (56.188, -43.266, 50.069), (56.978, -42.307, 51.824),
    (57.710, -41.306, 55.520), (58.391, -40.247, 58.988), (59.019, -39.180, 60.343),
    (59.641, -38.086, 60.652), (60.236, -36.976, 62.409), (60.795, -35.846, 65.011),
    (61.298, -34.708, 66.993), (61.737, -33.610, 69.194), (61.737, -33.610, 69.194),
    (61.737, -33.610, 69.194), (61.737, -33.610, 69.194), (61.737, -33.610, 69.194),
    (61.737, -33.610, 69.194), (61.737, -33.610, 69.194), (61.737, -33.610, 69.194),
    (61.853, -33.299, 70.049), (62.228, -32.107, 74.828), (62.546, -30.877, 75.640),
    (62.856, -29.666, 75.640), (63.171, -28.435, 75.640), (63.485, -27.225, 74.893),
    (63.813, -26.007, 74.893), (64.455, -23.681, 73.501), (65.147, -21.256, 75.049),
    (65.752, -18.868, 76.043), (66.357, -16.462, 75.793), (67.350, -12.541, 75.793),
    (68.571, -7.734, 75.350), (69.898, -2.924, 74.683), (71.246, 1.839, 73.187),
    (72.709, 6.683, 73.187), (74.149, 11.445, 73.187), (75.633, 16.204, 72.314),
    (77.144, 21.056, 73.282), (78.606, 25.924, 73.281), (80.065, 30.703, 72.406),
    (81.626, 35.529, 71.518), (83.247, 40.252, 70.536), (84.939, 45.040, 70.536),
    (86.659, 49.906, 70.536), (88.315, 54.619, 70.785), (89.966, 59.420, 71.035),
    (91.567, 64.152, 71.782), (93.180, 69.054, 71.782), (94.757, 73.881, 72.156),
    (96.317, 78.713, 71.781), (97.892, 83.445, 71.532), (99.492, 88.260, 72.154),
    (100.969, 93.035, 73.897)
]

def get_trajectory_target(vehicle_loc, trajectory, current_index, lookahead_m=8.0, max_search_ahead=30):
    if not trajectory:
        return None, current_index

    vx, vy = vehicle_loc.x, vehicle_loc.y
    closest_index = current_index
    min_dist_sq = float('inf')

    search_end = min(current_index + max_search_ahead, len(trajectory))
    for i in range(current_index, search_end):
        px, py, _ = trajectory[i]
        dist_sq = (px - vx) ** 2 + (py - vy) ** 2
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i

    if min_dist_sq > 25.0 ** 2:
        min_dist_sq = float('inf')
        for i, (px, py, _) in enumerate(trajectory):
            dist_sq = (px - vx) ** 2 + (py - vy) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i

    target_index = closest_index
    accumulated = 0.0
    for i in range(closest_index, len(trajectory) - 1):
        x1, y1, _ = trajectory[i]
        x2, y2, _ = trajectory[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        target_index = i + 1
        if seg_len < 1e-3:
            continue
        accumulated += seg_len
        if accumulated >= lookahead_m:
            break

    tx, ty, _ = trajectory[target_index]
    return carla.Location(x=tx, y=ty, z=vehicle_loc.z), closest_index

def get_ego_target_speed_kmh(ego_y):
    if ego_y >= 9.618:
        return 60.0
    if ego_y >= -40.0:
        return 15.0
    return 52.0

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (100.969, 93.035)
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

    # 极端天气设置（积水、大雾等）
    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=100.0, sun_azimuth_angle=90.0, sun_altitude_angle=10.0,
        fog_density=10.0, fog_distance=0.75, fog_falloff=0.1, wetness=100.0,
        scattering_intensity=11.5, mie_scattering_scale=0.21, rayleigh_scattering_scale=0.07
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # 准备 PID 控制器
        pid_police = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= 场景构建：生成超低摩擦力积水区 =================
        bp_friction = bp_lib.find('static.trigger.friction')
        bp_friction.set_attribute('friction', '0.0')

        # 生成 3x3 米的水坑
        bp_friction.set_attribute('extent_x', '10.0')
        bp_friction.set_attribute('extent_y', '10.0')
        bp_friction.set_attribute('extent_z', '10.0')

        # 将坐标偏移到侧边车轮能压到的地方
        friction_loc = carla.Location(x=65.293, y=-28.500, z=-5)
        friction_trigger = world.try_spawn_actor(bp_friction, carla.Transform(friction_loc))

        if friction_trigger:
            actor_list.append(friction_trigger)
            print("生成摩擦力触发器（单侧积水打滑区）成功。")

            # # === 修复的绘图代码 ===
            # box = carla.BoundingBox(friction_loc, carla.Vector3D(10.0, 10.0, 10.0))
            #     box=box,
            #     rotation=friction_trigger.get_transform().rotation,
            #     thickness=0.1,
            #     color=carla.Color(r=255, g=0, b=0),
            #     life_time=100.0
            # )

        # ================= Actor 1：警车 (代替原卡车) =================
        bp_police_car = bp_lib.find('vehicle.dodge.charger_police_2020')

        police_start_x, police_start_y, police_start_yaw = POLICE_TRAJECTORY[0]
        police_loc = carla.Location(x=police_start_x, y=police_start_y, z=0.5)
        police_loc.z = carla_map.get_waypoint(police_loc).transform.location.z + 0.5
        police_car = world.try_spawn_actor(bp_police_car,
                                           carla.Transform(police_loc, carla.Rotation(yaw=police_start_yaw)))

        police_active = False
        if police_car:
            actor_list.append(police_car)
            police_active = True

            # 开启警车的大灯阵列，制造炫目效果
            light_state = carla.VehicleLightState.HighBeam | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position | carla.VehicleLightState.Special1
            police_car.set_light_state(carla.VehicleLightState(light_state))
            print("生成 Dodge Charger 警车成功，大灯和警灯已开启。")

        # ================= Actor 2：Audi TT (Ego) =================
        bp_audi = bp_lib.find('vehicle.audi.tt')
        if bp_audi.has_attribute('role_name'):
            bp_audi.set_attribute('role_name', 'ego')
        if bp_audi.has_attribute('color'):
            bp_audi.set_attribute('color', '255,165,0')  # 橙色

        audi_start_x, audi_start_y, audi_start_yaw = EGO_TRAJECTORY[0]
        audi_start_loc = carla.Location(x=audi_start_x, y=audi_start_y, z=0.5)
        audi_start_wp = carla_map.get_waypoint(audi_start_loc, project_to_road=True)
        audi_start_loc.z = audi_start_wp.transform.location.z + 0.5
        audi = world.try_spawn_actor(bp_audi, carla.Transform(audi_start_loc, carla.Rotation(yaw=audi_start_yaw)))

        if audi:
            actor_list.append(audi)
            print("生成 Audi TT (Ego) 成功。")

        # ================= 稳定系统与初始速度 =================
        print("等待物理系统预热并稳定车辆底盘...")
        for _ in range(20):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

        print("赋予警车 50km/h、Ego 52km/h 的初始速度...")
        if police_car: apply_initial_velocity(police_car, 50.0, police_start_yaw)
        if audi: apply_initial_velocity(audi, 52.0, audi_start_yaw)

        world.tick()

        print("\n仿真正式开始！")
        police_traj_idx = 0
        ego_traj_idx = 0

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

            # ================= 警车：PID寻路 =================
            if police_active and police_car.is_alive:
                if check_and_handle_out_of_bounds(police_car, carla_map):
                    police_active = False
                elif police_traj_idx < len(POLICE_TRAJECTORY):
                    tx, ty, tyaw = POLICE_TRAJECTORY[police_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=police_car.get_location().z)

                    if police_car.get_location().distance(target_loc) < 3.0 and police_traj_idx < len(
                            POLICE_TRAJECTORY) - 1:
                        police_traj_idx += 1

                    # 警车也是50km/h，驶入摩擦力为0.1的触发器区域时，会产生剧烈打滑甩尾
                    apply_pid_control(police_car, pid_police['lon'], pid_police['lat'], 50.0, target_loc)
                else:
                    police_car.apply_control(carla.VehicleControl(brake=1.0))
                    police_active = False

            # ================= Audi (Ego): PID动态车道保持 =================
            if audi and audi.is_alive:
                if not check_and_handle_out_of_bounds(audi, carla_map):
                    ego_loc = audi.get_location()
                    ego_speed = get_ego_target_speed_kmh(ego_loc.y)
                    ego_target_loc, ego_traj_idx = get_trajectory_target(
                        ego_loc, EGO_TRAJECTORY, ego_traj_idx, lookahead_m=max(5.0, ego_speed / 3.6 * 0.6)
                    )

                    if ego_target_loc:
                        # Ego 也将面临打滑考验与警车的强光炫目干扰
                        apply_pid_control(audi, pid_ego['lon'], pid_ego['lat'], ego_speed, ego_target_loc)
                    else:
                        audi.apply_control(carla.VehicleControl(brake=1.0))

            # 帧率同步
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
