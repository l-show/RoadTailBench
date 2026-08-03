import carla
import time
import math
import numpy as np

# ==========================================
# 1. 基础控制算法 (PID) - 保留并复用
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

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

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.8)

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

# 动态获取前方路径点的辅助函数（用于自由巡航车辆）
def get_next_waypoint_loc(vehicle, carla_map, distance=5.0):
    loc = vehicle.get_location()
    current_wp = carla_map.get_waypoint(loc, project_to_road=True)
    next_wps = current_wp.next(distance)
    if next_wps:
        return next_wps[0].transform.location
    return loc

# ==========================================
# 2. 轨迹数据
# ==========================================
BUS_TRAJECTORY = [
    (-54.612, -32.109, 23.226), (-54.612, -32.109, 23.226), (-54.612, -32.109, 23.226), (-54.612, -32.109, 23.226),
    (-54.612, -32.109, 23.226), (-54.612, -32.109, 23.226), (-54.612, -32.109, 22.847), (-50.064, -30.244, 22.033),
    (-45.388, -28.254, 23.999), (-40.760, -26.152, 24.449), (-36.135, -24.043, 24.575), (-31.599, -21.942, 25.302),
    (-26.973, -19.639, 28.661), (-22.704, -17.039, 34.270), (-18.680, -14.074, 37.366), (-14.588, -10.922, 38.976),
    (-10.776, -7.561, 44.498), (-7.330, -3.940, 48.596), (-4.154, 0.028, 54.285), (-1.407, 4.203, 60.045),
    (0.975, 8.691, 63.676), (2.998, 13.262, 67.821), (4.894, 17.976, 69.364), (6.646, 22.658, 68.171),
    (8.505, 27.299, 69.329), (10.273, 32.065, 69.078), (12.107, 36.716, 68.089), (14.064, 41.407, 65.143),
    (16.297, 46.064, 64.314), (18.358, 50.619, 66.188), (20.377, 55.284, 67.349), (22.273, 59.911, 67.737),
    (24.185, 64.621, 68.382), (26.000, 69.279, 68.769), (27.959, 74.059, 65.378), (30.070, 78.591, 64.682),
    (32.243, 83.186, 64.682), (34.345, 87.723, 66.723), (36.290, 92.329, 67.120), (38.260, 96.924, 65.630),
    (40.453, 101.601, 64.777), (42.544, 106.234, 66.993), (44.525, 110.826, 66.219), (46.569, 115.569, 67.084),
    (48.548, 120.251, 67.084), (50.477, 124.863, 67.342), (50.766, 125.555, 67.342), (50.766, 125.555, 67.342),
    (50.766, 125.555, 67.342), (50.766, 125.555, 67.342)
]

# 新增：Ford Crown 的轨迹
FORD_TRAJECTORY = [
    (39.153, -64.378, 120.05), (39.153, -64.378, 117.394), (39.153, -64.378, 117.394), (39.153, -64.378, 117.394),
    (39.153, -64.378, 117.394), (39.153, -64.378, 117.394), (39.153, -64.378, 117.746), (34.592, -56.055, 122.348),
    (33.776, -54.797, 125.504), (33.776, -54.797, 125.504), (32.442, -52.934, 125.857), (28.672, -47.822, 126.217),
    (24.815, -42.641, 126.864), (20.994, -37.439, 125.444), (17.536, -32.118, 119.007), (14.809, -26.385, 110.477),
    (14.184, -24.399, 103.91), (14.184, -24.399, 103.91), (13.784, -22.781, 103.697), (13.006, -19.008, 98.653),
    (12.064, -12.724, 98.51), (11.117, -6.336, 97.873), (10.57, -0.112, 93.859), (10.137, 6.334, 93.138),
    (10.499, 12.767, 78.928), (12.357, 18.829, 71.261), (14.553, 24.678, 68.701), (16.865, 30.482, 67.926),
    (19.308, 36.341, 66.929), (21.755, 42.085, 66.859), (24.244, 47.818, 66.508), (26.809, 53.743, 66.648),
    (29.28, 59.483, 67.003), (31.779, 65.437, 67.286), (34.192, 71.202, 67.286), (36.658, 77.058, 67.077),
    (39.105, 82.843, 67.077), (41.544, 88.597, 66.937), (44.074, 94.538, 66.937), (46.525, 100.286, 66.36),
    (49.095, 106.098, 66.075), (51.621, 111.814, 66.496), (53.814, 116.878, 66.636), (53.814, 116.878, 66.636),
    (53.814, 116.878, 66.636), (53.814, 116.878, 66.636)
]

EGO_TRAJECTORY = [
    (42.303, 71.551, -114.624), (42.303, 71.551, -114.624), (42.137, 71.17, -113.546), (39.626, 65.465, -113.967),
    (37.162, 59.722, -113.033), (34.717, 53.971, -113.103), (32.138, 48.049, -113.598), (29.594, 42.226, -113.598),
    (27.009, 36.307, -113.598), (24.424, 30.389, -113.598), (21.842, 24.469, -113.528), (19.349, 18.624, -112.965),
    (16.847, 12.671, -112.256), (14.599, 6.836, -109.091), (12.733, 0.816, -106.445), (11.089, -5.428, -103.99),
    (9.545, -11.484, -104.702), (7.883, -17.725, -105.27), (6.139, -23.833, -106.119), (4.405, -30.053, -105.128),
    (2.801, -36.093, -104.848), (1.18, -42.237, -104.779), (-0.415, -48.28, -104.779), (-2.009, -54.323, -104.779),
    (-3.603, -60.367, -104.779), (-5.224, -66.511, -104.779), (-6.819, -72.554, -104.779), (-8.466, -78.799, -104.779),
    (-10.096, -84.978, -104.779), (-11.744, -91.223, -104.779), (-13.339, -97.266, -104.92),
    (-14.948, -103.303, -104.92), (-16.552, -109.342, -104.851), (-18.155, -115.386, -104.851),
    (-19.778, -121.528, -104.638),
    (-21.338, -127.576, -104.426), (-22.92, -133.726, -104.426), (-24.528, -139.977, -104.426),
    (-26.152, -146.228, -104.496),
    (-27.756, -152.483, -104.357), (-29.332, -158.639, -104.357), (-30.933, -164.895, -104.357),
    (-32.055, -171.028, -92.813),
    (-30.569, -177.11, -63.263), (-27.081, -182.266, -43.657), (-21.258, -184.858, -15.748),
    (-17.854, -185.818, -15.748),
    (-17.854, -185.818, -15.748), (-17.854, -185.818, -15.748)
]

# ==========================================
# 3. 辅助函数：安全生成车辆
# ==========================================
def spawn_vehicle(world, bp_name, loc_x, loc_y, yaw=None, color=None, role_name=None):
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(bp_name)
    if color and bp.has_attribute('color'):
        bp.set_attribute('color', color)
    if role_name and bp.has_attribute('role_name'):
        bp.set_attribute('role_name', role_name)

    carla_map = world.get_map()
    spawn_loc = carla.Location(x=loc_x, y=loc_y, z=0.5)
    wp = carla_map.get_waypoint(spawn_loc, project_to_road=True)
    if yaw is None or abs(yaw) < 1e-3:
        yaw = wp.transform.rotation.yaw

    # 使用投影后的高度保证不悬空
    spawn_transform = carla.Transform(
        carla.Location(x=loc_x, y=loc_y, z=wp.transform.location.z + 1.0),
        carla.Rotation(yaw=yaw)
    )
    return world.try_spawn_actor(bp, spawn_transform)

def check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=6.0):
    if not vehicle or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    if wp_nearest is None:
        return True
    if wp_exact is None:
        dist_to_road = wp_nearest.transform.location.distance(loc)
        return dist_to_road > threshold_dist
    return False

def destroy_actor(actor_list, actor, reason=""):
    if not actor:
        return None

    actor_id = getattr(actor, "id", "unknown")
    actor_type = getattr(actor, "type_id", "actor")
    try:
        if actor.is_alive:
            actor.destroy()
            suffix = f" ({reason})" if reason else ""
            print(f"Destroyed {actor_type} [{actor_id}]{suffix}")
    except RuntimeError:
        pass

    if actor in actor_list:
        actor_list.remove(actor)
    return None

# ==========================================
# 4. 主程序 (Main Loop)
# ==========================================
def destroy_all_scene_actors(actor_list, reason=""):
    print("Ending scene: {}".format(reason))
    for actor in list(actor_list):
        destroy_actor(actor_list, actor, reason)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    tm = client.get_trafficmanager(8000)

    # ---------------- 依据截图严格设置天气 ----------------
    weather = carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=15.0,
        fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    try:
        # 同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        # 控制器初始化
        pid_bus = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ford = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v3 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= 车1：Bus（循迹，y=32处停5秒） =================
        bus = spawn_vehicle(world, 'vehicle.mitsubishi.fusorosa', BUS_TRAJECTORY[0][0], BUS_TRAJECTORY[0][1],
                            BUS_TRAJECTORY[0][2])
        if bus: actor_list.append(bus)

        bus_traj_idx = 0

        # ================= 车2：Ford Crown (PID循迹，变速行驶) =================
        # 根据需求：出生在轨迹第一点
        ford = spawn_vehicle(world, 'vehicle.ford.crown', FORD_TRAJECTORY[0][0], FORD_TRAJECTORY[0][1],
                             FORD_TRAJECTORY[0][2], color='0,0,255')
        if ford: actor_list.append(ford)
        ford_traj_idx = 0
        ford_has_accelerated = False  # 记录是否打印过加速提示

        # ================= 车3：Tesla Cybertruck (PID循迹车道保持) =================
        tesla = spawn_vehicle(world, 'vehicle.tesla.cybertruck', -19.322, -101.638, 0.0, color='192,192,192')  # 银色
        if tesla: actor_list.append(tesla)

        # ================= 车4：Jeep Wrangler (TM 控制) =================
        jeep = spawn_vehicle(world, 'vehicle.jeep.wrangler_rubicon', 40.803, -6.342, 0.0)
        if jeep:
            actor_list.append(jeep)
            jeep.set_autopilot(True, tm.get_port())
            tm.ignore_vehicles_percentage(jeep, 0.0)
            tm.distance_to_leading_vehicle(jeep, 8.0)
            tm.auto_lane_change(jeep, False)
            tm.vehicle_percentage_speed_difference(jeep, 20.0)

        # ================= 车5：Ego - Audi TT (橙色, 循迹) =================
        ego = spawn_vehicle(world, 'vehicle.audi.tt', EGO_TRAJECTORY[0][0], EGO_TRAJECTORY[0][1], EGO_TRAJECTORY[0][2],
                            color='255,165,0', role_name='ego')
        if ego: actor_list.append(ego)
        ego_traj_idx = 0

        # 让物理引擎预热
        for _ in range(10): world.tick()
        print("\n仿真正式开始！长尾场景加载完毕。")

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # ----------------- 1. BUS logic -----------------
            if bus and bus.is_alive:
                if check_vehicle_out_of_bounds(bus, carla_map):
                    bus = destroy_actor(actor_list, bus, "out of bounds")
                elif bus_traj_idx >= len(BUS_TRAJECTORY) - 1 and bus.get_location().distance(
                        carla.Location(x=BUS_TRAJECTORY[-1][0], y=BUS_TRAJECTORY[-1][1], z=bus.get_location().z)) < 3.0:
                    bus = destroy_actor(actor_list, bus, "trajectory complete")
                else:
                    tx, ty, tyaw = BUS_TRAJECTORY[bus_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=bus.get_location().z)
                    if bus.get_location().distance(target_loc) < 2.5 and bus_traj_idx < len(BUS_TRAJECTORY) - 1:
                        bus_traj_idx += 1
                        tx, ty, tyaw = BUS_TRAJECTORY[bus_traj_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=bus.get_location().z)
                    apply_pid_control(bus, pid_bus['lon'], pid_bus['lat'], 60.0, target_loc)

            # ----------------- 2. Ford Crown logic -----------------
            if ford and ford.is_alive:
                if check_vehicle_out_of_bounds(ford, carla_map):
                    ford = destroy_actor(actor_list, ford, "out of bounds")
                elif ford_traj_idx >= len(FORD_TRAJECTORY) - 1 and ford.get_location().distance(
                        carla.Location(x=FORD_TRAJECTORY[-1][0], y=FORD_TRAJECTORY[-1][1], z=ford.get_location().z)) < 3.0:
                    ford = destroy_actor(actor_list, ford, "trajectory complete")
                else:
                    tx, ty, tyaw = FORD_TRAJECTORY[ford_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ford.get_location().z)

                    if ford.get_location().distance(target_loc) < 2.5 and ford_traj_idx < len(FORD_TRAJECTORY) - 1:
                        ford_traj_idx += 1
                        tx, ty, tyaw = FORD_TRAJECTORY[ford_traj_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ford.get_location().z)

                    ford_y = ford.get_location().y
                    if ford_y >= 20.0:
                        target_speed = 60.0
                        if not ford_has_accelerated:
                            print(f"[{sim_time:.1f}s] Ford Crown reaches Y={ford_y:.1f}, accelerating to 60 km/h")
                            ford_has_accelerated = True
                    else:
                        target_speed = 30.0

                    apply_pid_control(ford, pid_ford['lon'], pid_ford['lat'], target_speed, target_loc)

            # ----------------- 3. Tesla Cybertruck logic -----------------
            if tesla and tesla.is_alive:
                if check_vehicle_out_of_bounds(tesla, carla_map):
                    tesla = destroy_actor(actor_list, tesla, "out of bounds")
                else:
                    next_wp_loc = get_next_waypoint_loc(tesla, carla_map, distance=8.0)
                    apply_pid_control(tesla, pid_v3['lon'], pid_v3['lat'], 60.0, next_wp_loc)

            # ----------------- 4. Jeep Wrangler guard -----------------
            if jeep and jeep.is_alive and check_vehicle_out_of_bounds(jeep, carla_map):
                jeep = destroy_actor(actor_list, jeep, "out of bounds")

            # ----------------- 5. Ego Audi TT logic -----------------
            if ego and ego.is_alive:
                if check_vehicle_out_of_bounds(ego, carla_map):
                    ego = destroy_actor(actor_list, ego, "out of bounds")
                    destroy_all_scene_actors(actor_list, "ego destroyed out of bounds")
                    break
                elif ego_traj_idx >= len(EGO_TRAJECTORY) - 1 and ego.get_location().distance(
                        carla.Location(x=EGO_TRAJECTORY[-1][0], y=EGO_TRAJECTORY[-1][1], z=ego.get_location().z)) < 3.0:
                    ego = destroy_actor(actor_list, ego, "trajectory complete")
                    destroy_all_scene_actors(actor_list, "ego reached trajectory endpoint")
                    break
                else:
                    tx, ty, tyaw = EGO_TRAJECTORY[ego_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)

                    if ego.get_location().distance(target_loc) < 2.5 and ego_traj_idx < len(EGO_TRAJECTORY) - 1:
                        ego_traj_idx += 1
                        tx, ty, tyaw = EGO_TRAJECTORY[ego_traj_idx]
                        target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)

                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], 70.0, target_loc)
            elif ego is not None:
                destroy_all_scene_actors(actor_list, "ego is no longer alive")
                ego = None
                break

            # frame-rate sync
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n仿真结束。")
    finally:
        print("清理环境...")
        for actor in actor_list:
            if actor and actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        if tm: tm.set_synchronous_mode(False)
        print("清理完毕。")

if __name__ == '__main__':
    main()
