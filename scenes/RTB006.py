import carla
import time
import math
import numpy as np

# ==========================================
# 轨迹数据清洗 (自动去重，加入Z轴默认值0.5)
# ==========================================
RAW_DATA = [
    (-5.497, -65.884, 72.942), (-5.515, -60.946, 73.223), (-5.515, -57.946, 73.152),
    (-5.515, -57.946, 71.813), (-5.515, -57.946, 71.813), (-5.515, -57.946, 72.373),
    (-5.349, -57.283, 80.56), (-5.159, -56.053, 81.546), (-4.988, -54.817, 82.95),
    (-4.836, -53.535, 83.3), (-4.696, -52.251, 83.861), (-4.568, -51.007, 84.213),
    (-4.441, -49.764, 84.213), (-4.244, -48.509, 75.943), (-3.802, -47.321, 64.155),
    (-3.21, -46.196, 61.744), (-2.599, -45.058, 62.449), (-2.048, -43.936, 65.495),
    (-1.585, -42.775, 70.462), (-1.188, -41.547, 73.503), (-0.891, -40.333, 78.643),
    (-0.684, -39.1, 83.107), (-0.569, -37.835, 85.09), (-0.471, -36.568, 85.725),
    (-0.382, -35.279, 86.566), (-0.307, -34.031, 86.566), (-0.232, -32.765, 86.707),
    (-0.162, -31.496, 87.34), (-0.102, -30.205, 87.34), (-0.042, -28.915, 87.34),
    (0.016, -27.666, 87.41), (0.064, -26.396, 88.614), (0.095, -25.105, 88.614),
    (0.125, -23.855, 88.614), (0.155, -22.605, 88.614), (0.187, -21.314, 88.614),
    (0.217, -20.064, 88.614), (0.247, -18.815, 88.614), (0.277, -17.523, 88.684),
    (0.304, -16.272, 89.105), (0.319, -15.022, 89.316), (0.334, -13.731, 89.316),
    (0.349, -12.46, 89.316), (0.365, -11.168, 89.316), (0.363, -9.918, 90.658),
    (0.346, -8.648, 90.728), (0.329, -7.398, 91.65), (0.27, -6.149, 94.69),
    (0.119, -4.909, 98.98), (-0.115, -3.638, 101.239), (-0.368, -2.414, 101.872),
    (-0.634, -1.15, 101.872), (-0.895, 0.093, 101.872), (-1.161, 1.358, 101.872),
    (-1.41, 2.625, 98.861), (-1.595, 3.907, 98.084), (-1.769, 5.145, 97.308),
    (-1.878, 6.411, 94.147), (-1.967, 7.699, 93.796), (-2.043, 8.956, 92.883),
    (-2.098, 10.205, 92.321), (-2.139, 11.454, 91.197), (-2.158, 12.704, 90.846),
    (-2.177, 13.975, 90.846), (-2.198, 15.245, 91.056), (-2.221, 16.517, 91.056),
    (-2.245, 17.787, 91.056), (-2.282, 19.037, 93.099), (-2.361, 20.327, 93.59),
    (-2.374, 21.576, 87.882), (-2.308, 22.866, 86.962), (-2.24, 24.156, 86.962),
    (-2.174, 25.404, 86.962), (-2.107, 26.652, 86.962), (-2.039, 27.942, 86.962),
    (-1.971, 29.211, 86.962), (-1.903, 30.501, 86.962), (-1.843, 31.749, 87.382),
    (-1.784, 33.039, 87.382), (-1.726, 34.33, 87.452), (-1.676, 35.579, 87.732),
    (-1.626, 36.828, 87.732), (-1.578, 38.098, 88.507), (-1.568, 39.389, 90.695),
    (-1.591, 40.639, 91.047), (-1.615, 41.923, 91.117), (-1.636, 43.193, 90.128),
    (-1.631, 44.443, 89.495), (-1.61, 45.714, 88.509), (-1.563, 46.984, 87.668),
    (-1.51, 48.254, 87.456), (-1.453, 49.544, 87.456), (-1.397, 50.813, 87.456),
    (-1.33, 52.082, 86.891), (-1.256, 53.33, 85.902), (-1.164, 54.619, 85.902),
    (-1.071, 55.907, 85.902), (-0.981, 57.154, 85.832), (-0.889, 58.443, 85.971),
    (-0.805, 59.69, 86.252), (-0.745, 60.939, 87.985), (-0.703, 62.23, 88.125),
    (-0.664, 63.479, 88.406), (-0.629, 64.77, 88.476), (-0.596, 66.02, 88.617),
    (-0.578, 67.311, 90.315), (-0.59, 68.561, 90.666), (-0.604, 69.811, 90.666),
    (-0.603, 71.103, 89.192), (-0.568, 72.373, 87.716), (-0.504, 73.663, 86.872),
    (-0.43, 74.953, 86.661), (-0.364, 76.243, 87.368), (-0.306, 77.491, 87.368),
    (-0.248, 78.765, 87.368), (-0.19, 80.014, 87.368), (-0.131, 81.304, 87.368),
    (-0.088, 82.595, 89.281), (-0.086, 83.845, 90.554), (-0.099, 85.116, 90.624),
    (-0.113, 86.387, 90.624), (-0.126, 87.658, 90.413), (-0.125, 88.949, 89.637),
    (-0.111, 90.199, 88.647), (-0.077, 91.449, 87.731), (-0.026, 92.74, 87.731),
    (0.024, 93.989, 87.731), (0.075, 95.28, 87.731), (0.126, 96.571, 87.731),
    (0.176, 97.82, 87.731), (0.226, 99.09, 87.731), (0.277, 100.381, 87.731),
    (0.326, 101.631, 87.872), (0.338, 102.922, 90.671), (0.269, 104.19, 97.065),
    (0.012, 105.454, 107.372), (-0.456, 106.611, 117.679), (-1.095, 107.685, 123.509),
    (-1.862, 108.723, 130.117), (-2.724, 109.656, 136.622), (-3.682, 110.458, 143.237),
    (-4.768, 111.153, 153.406), (-5.919, 111.636, 163.331), (-7.179, 111.915, 171.722),
    (-8.442, 112.078, 173.417), (-9.689, 112.138, -178.615), (-10.98, 112.082, -176.065),
    (-12.266, 111.962, -173.796), (-13.551, 111.836, -175.705), (-14.799, 111.76, -176.902),
    (-16.089, 111.706, -177.749), (-17.358, 111.676, 179.069), (-18.604, 111.755, 172.955),
    (-19.84, 112.034, 163.657), (-21.033, 112.396, 162.591), (-21.768, 112.626, 162.591),
    (-21.768, 112.626, 162.591), (-21.768, 112.626, 162.591), (-21.768, 112.626, 162.591)
]

PATH_POINTS = []
if RAW_DATA:
    PATH_POINTS.append((RAW_DATA[0][0], RAW_DATA[0][1], 0.5, RAW_DATA[0][2]))
    for i in range(1, len(RAW_DATA)):
        if RAW_DATA[i] != RAW_DATA[i - 1]:
            PATH_POINTS.append((RAW_DATA[i][0], RAW_DATA[i][1], 0.5, RAW_DATA[i][2]))

RAW_EGO_DATA = [
    (4.853, 148.451, -91.737), (4.853, 148.451, -91.737), (4.853, 148.451, -91.737),
    (4.853, 148.451, -91.737), (4.853, 148.451, -91.737), (4.838, 147.745, -90.808),
    (4.813, 145.252, -90.555), (4.767, 142.754, -91.765), (4.691, 140.255, -91.512),
    (4.644, 137.714, -90.751), (4.610, 135.173, -91.005), (4.550, 132.673, -91.512),
    (4.484, 130.174, -91.512), (4.415, 127.571, -91.512), (4.315, 123.760, -91.512),
    (4.209, 120.011, -91.892), (4.085, 116.263, -91.892), (3.961, 112.516, -92.019),
    (3.826, 108.706, -92.272), (3.658, 104.897, -92.779), (3.473, 101.089, -92.779),
    (3.291, 97.281, -92.652), (3.124, 93.535, -92.399), (2.994, 89.788, -91.511),
    (2.895, 86.039, -91.511), (2.794, 82.228, -91.511), (2.693, 78.416, -91.511),
    (2.595, 74.666, -91.511), (2.491, 70.852, -91.638), (2.376, 67.104, -92.018),
    (2.256, 63.293, -91.258), (2.189, 59.544, -91.004), (2.107, 55.732, -91.384),
    (2.016, 51.983, -91.384), (1.919, 48.103, -91.638), (1.807, 44.292, -92.018),
    (1.673, 40.482, -92.018), (1.531, 36.735, -92.398), (1.379, 32.926, -92.018),
    (1.260, 29.115, -91.638), (1.152, 25.365, -91.638), (1.046, 21.617, -91.384),
    (0.954, 17.805, -91.384), (0.862, 13.992, -91.384), (0.770, 10.180, -91.384),
    (0.670, 6.306, -91.511), (0.568, 2.432, -91.511), (0.469, -1.316, -91.511),
    (0.383, -5.065, -91.131), (0.308, -8.877, -91.131), (0.269, -12.626, -90.243),
    (0.253, -16.376, -90.243), (0.212, -20.189, -91.003), (0.105, -23.937, -92.326),
    (-0.025, -27.810, -91.565), (-0.129, -31.621, -91.565), (-0.231, -35.369, -91.565),
    (-0.339, -39.180, -91.945), (-0.468, -42.990, -91.945), (-0.576, -46.801, -91.045),
    (-0.642, -50.550, -91.451), (-0.752, -54.361, -91.945), (-0.881, -58.108, -92.072),
    (-1.019, -61.917, -92.072), (-1.157, -65.724, -92.072), (-1.298, -69.470, -92.452),
    (-1.473, -73.280, -92.452), (-1.612, -77.027, -92.071), (-1.720, -80.838, -91.382),
    (-1.792, -83.462, -91.636), (-1.792, -83.462, -91.636), (-1.792, -83.462, -91.636),
    (-1.792, -83.462, -91.636), (-1.792, -83.462, -91.636), (-1.792, -83.462, -91.636)
]

EGO_PATH_POINTS = []
if RAW_EGO_DATA:
    EGO_PATH_POINTS.append((RAW_EGO_DATA[0][0], RAW_EGO_DATA[0][1], 0.5, RAW_EGO_DATA[0][2]))
    for i in range(1, len(RAW_EGO_DATA)):
        if RAW_EGO_DATA[i] != RAW_EGO_DATA[i - 1]:
            EGO_PATH_POINTS.append((RAW_EGO_DATA[i][0], RAW_EGO_DATA[i][1], 0.5, RAW_EGO_DATA[i][2]))


# ==========================================
# PID 控制器类 (自己接管，摆脱限速)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
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
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])

        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1: return 0.0

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0: _dot *= -1.0

        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


# ==========================================
# 辅助函数
# ==========================================
def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def get_target_waypoint(vehicle_loc, path_points, lookahead_dist=4.0):
    """根据给定的静态路径点进行循迹"""
    min_dist = float('inf')
    closest_index = 0
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist, closest_index = dist, i

    target_index, current_dist = closest_index, 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1, p2 = path_points[i], path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist: break
    return path_points[target_index]


def get_lane_keeping_waypoint(carla_map, vehicle_loc, lookahead_dist=6.0):
    """动态利用CARLA地图提取车道中心点进行车道保持"""
    current_wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    next_wps = current_wp.next(lookahead_dist)
    if next_wps:
        loc = next_wps[0].transform.location
        return (loc.x, loc.y, loc.z)
    return (current_wp.transform.location.x, current_wp.transform.location.y, current_wp.transform.location.z)


def get_proper_spawn_transform(world, x, y):
    loc = carla.Location(x=x, y=y, z=0.0)
    waypoint = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    trans = waypoint.transform
    trans.location.z += 0.5
    return trans


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp):
    """通用的PID控制器执行流程"""
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)  # 当前速度 km/h

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)

    vehicle.apply_control(control)


def destroy_actor(actor, reason):
    if actor and actor.is_alive:
        print(f"{actor.type_id} 销毁: {reason}")
        actor.destroy()
    return None


def check_and_destroy_out_of_bounds(actor, carla_map, threshold=6.0):
    if actor is None or not actor.is_alive:
        return None

    loc = actor.get_location()
    nearest_wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if nearest_wp is None:
        return destroy_actor(actor, "无法投影到道路")

    distance = nearest_wp.transform.location.distance(loc)
    if distance > threshold:
        return destroy_actor(actor, f"偏离道路中心 {distance:.2f} m")
    return actor


def has_reached_path_end(actor, path_points, threshold=3.0):
    if actor is None or not actor.is_alive or not path_points:
        return False
    loc = actor.get_location()
    end = path_points[-1]
    return math.hypot(loc.x - end[0], loc.y - end[1]) <= threshold


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # 按照严格要求的天气配置
    weather = carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
        sun_azimuth_angle=-1.0, sun_altitude_angle=45.0, fog_density=2.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=0.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
        rayleigh_scattering_scale=0.0331, dust_storm=0.0
    )
    world.set_weather(weather)
    bp_lib = world.get_blueprint_library()

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        actor_list = []

        # 定义四辆车的独立控制器字典（新增了audi）
        pids = {
            'prius': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()},
            'impala': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()},
            'amb': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()},
            'audi': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        }

        try:
            # 1. 车辆一: vehicle.toyota.prius (位置: x=11.931, y=96.543)
            bp_prius = bp_lib.find('vehicle.toyota.prius')
            trans_prius = get_proper_spawn_transform(world, x=11.931, y=96.543)
            prius = world.try_spawn_actor(bp_prius, trans_prius)
            if prius:
                actor_list.append(prius)
                print("1. Prius 生成成功 (使用PID接管车道保持)")

            # 2. 车辆二: vehicle.chevrolet.impala (位置: x=-4.970, y=-56.577)
            bp_impala = bp_lib.find('vehicle.chevrolet.impala')
            trans_impala = get_proper_spawn_transform(world, x=-4.970, y=-56.577)
            impala = world.try_spawn_actor(bp_impala, trans_impala)
            if impala:
                actor_list.append(impala)
                print("2. Impala 生成成功 (使用PID接管车道保持)")

            # 3. 车辆三: vehicle.ford.ambulance (根据静态路径点生成)
            bp_ambulance = bp_lib.find('vehicle.ford.ambulance')
            trans_ambulance = get_transform(x=PATH_POINTS[0][0], y=PATH_POINTS[0][1], z=0.5, yaw=PATH_POINTS[0][3])
            ambulance = world.try_spawn_actor(bp_ambulance, trans_ambulance)
            if ambulance:
                actor_list.append(ambulance)
                print("3. Ambulance 生成成功 (PID循迹)")

            # 4. 车辆四: vehicle.audi.tt (新增：橙色，位置 x=3.857, y=87.479)
            bp_audi = bp_lib.find('vehicle.audi.tt')
            if bp_audi.has_attribute('color'):
                bp_audi.set_attribute('color', '255,100,0')  # CARLA 标准橙色
            bp_audi.set_attribute('role_name', 'ego')
            initial_ego_point = EGO_PATH_POINTS[0]
            trans_audi = get_transform(
                x=initial_ego_point[0],
                y=initial_ego_point[1],
                z=0.5,
                yaw=initial_ego_point[3],
            )
            audi = world.try_spawn_actor(bp_audi, trans_audi)
            if audi:
                actor_list.append(audi)
                print("4. Audi TT 生成成功 (橙色EGO，PID轨迹点循迹，初始70km/h)")

            print("\n等待 1 秒物理系统稳定...")
            for _ in range(20): world.tick()

            # --- 为 Audi TT 赋予物理初速度 70km/h (约19.44m/s) ---
            if audi and audi.is_alive:
                forward_vec = audi.get_transform().get_forward_vector()
                initial_speed_mps = 70.0 / 3.6
                audi.set_target_velocity(carla.Vector3D(
                    forward_vec.x * initial_speed_mps,
                    forward_vec.y * initial_speed_mps,
                    forward_vec.z * initial_speed_mps
                ))

            print("\n场景正式运行...")

            # --- 平滑速度相关变量 ---
            # 车辆1 Prius 速度平滑
            prius_target_speed_actual = 20.0
            prius_accel_rate = 15.0  # 加速度 km/h per second

            # 车辆3 Ambulance 速度平滑
            amb_target_speed_actual = 100.0
            amb_accel_rate = 20.0
            amb_end_point = PATH_POINTS[-1]

            while True:
                start_time = time.time()
                world.tick()

                # ==========================
                # 控制 1: Prius (40 -> 70)
                # ==========================
                if prius and prius.is_alive:
                    prius = check_and_destroy_out_of_bounds(prius, carla_map)
                if prius and prius.is_alive:
                    prius_loc = prius.get_location()
                    prius_desired_speed = 40.0

                    # 到达 y=35 时目标速度变成 70km/h (从 96 往 35 跑，y变小)
                    if prius_loc.y <= 35.0:
                        prius_desired_speed = 70.0

                    # 模拟加速度，平滑加减速
                    if prius_target_speed_actual < prius_desired_speed:
                        prius_target_speed_actual = min(prius_target_speed_actual + prius_accel_rate * 0.05,
                                                        prius_desired_speed)

                    target_wp = get_lane_keeping_waypoint(carla_map, prius_loc)
                    apply_pid_control(prius, pids['prius']['lon'], pids['prius']['lat'], prius_target_speed_actual,
                                      target_wp)

                # ==========================
                # 控制 2: Impala (恒定 30)
                # ==========================
                if impala and impala.is_alive:
                    impala = check_and_destroy_out_of_bounds(impala, carla_map)
                if impala and impala.is_alive:
                    target_wp = get_lane_keeping_waypoint(carla_map, impala.get_location())
                    apply_pid_control(impala, pids['impala']['lon'], pids['impala']['lat'], 30.0, target_wp)

                # ==========================
                # 控制 3: Ambulance (150 -> 20 -> 停)
                # ==========================
                if ambulance and ambulance.is_alive:
                    ambulance = check_and_destroy_out_of_bounds(ambulance, carla_map)
                if ambulance and ambulance.is_alive:
                    tf = ambulance.get_transform()
                    dist_to_end = math.sqrt(
                        (tf.location.x - amb_end_point[0]) ** 2 + (tf.location.y - amb_end_point[1]) ** 2)

                    if dist_to_end < 2.5:
                        # 距离最后一点小于2.5米时强行刹停 (精确停在最后一点)
                        ambulance.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                    else:
                        amb_desired_speed = 150.0

                        # 到达 y=80 触发减速至 20km/h (从 -102 往 106 跑，y变大)
                        if tf.location.y >= 80.0:
                            amb_desired_speed = 20.0

                        # 模拟减速度，防止急刹导致打滑
                        if amb_target_speed_actual > amb_desired_speed:
                            amb_target_speed_actual = max(amb_target_speed_actual - amb_accel_rate * 0.05,
                                                          amb_desired_speed)

                        target_wp = get_target_waypoint(tf.location, PATH_POINTS, lookahead_dist=6.0)
                        apply_pid_control(ambulance, pids['amb']['lon'], pids['amb']['lat'], amb_target_speed_actual,
                                          target_wp)

                # ==========================
                # 控制 4: Audi TT (恒定 50km/h，自动搜索前方锚点循迹)
                # ==========================
                if audi and audi.is_alive:
                    audi = check_and_destroy_out_of_bounds(audi, carla_map)
                if audi and audi.is_alive:
                    if has_reached_path_end(audi, EGO_PATH_POINTS):
                        audi = destroy_actor(audi, "EGO 到达轨迹终点")
                        continue
                    target_wp = get_target_waypoint(audi.get_location(), EGO_PATH_POINTS, lookahead_dist=6.0)
                    apply_pid_control(audi, pids['audi']['lon'], pids['audi']['lat'], 50.0, target_wp)

                # --- 帧率同步补偿 ---
                compute_time = time.time() - start_time
                if compute_time < 0.05:
                    time.sleep(0.05 - compute_time)

        except Exception as e:
            print(f"发生异常: {e}")

    except KeyboardInterrupt:
        print("\n用户按 Ctrl+C 停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if actor_list:
            actors_to_destroy = [a for a in actor_list if a is not None and a.is_alive]
            client.apply_batch([carla.command.DestroyActor(a) for a in actors_to_destroy])
        print("清理完成。")


if __name__ == '__main__':
    main()
