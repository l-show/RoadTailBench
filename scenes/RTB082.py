import carla
import time
import math
import numpy as np
import pandas as pd


# ==========================================
# 辅助函数
# ==========================================

def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


def calculate_velocity_vector(speed, rotation):
    pitch_rad = math.radians(rotation.pitch)
    yaw_rad = math.radians(rotation.yaw)
    x = speed * math.cos(yaw_rad) * math.cos(pitch_rad)
    y = speed * math.sin(yaw_rad) * math.cos(pitch_rad)
    z = speed * math.sin(pitch_rad)
    return carla.Vector3D(x=x, y=y, z=z)


def get_target_waypoint(vehicle_loc, path_transforms, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0

    for i, t in enumerate(path_transforms):
        dist = vehicle_loc.distance(t.location)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_transforms) - 1):
        p1 = path_transforms[i].location
        p2 = path_transforms[i + 1].location
        d = p1.distance(p2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break

    return path_transforms[target_index].location


# ==========================================
# PID 控制器类
# ==========================================

class PIDLongitudinalController:
    """ 纵向控制 (油门/刹车) """

    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)

        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController:
    """ 横向控制 (转向) """

    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_location, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint_location.x - v_begin.x, waypoint_location.y - v_begin.y, 0.0])

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
# 行人模块类
# ==========================================


# ==========================================
# 轨迹数据定义和清洗
# ==========================================
def interpolate_path(path, interval=1.0):
    """将稀疏轨迹点进行线性插值，生成密集的航点"""
    dense_path = []
    for i in range(len(path) - 1):
        p1, p2 = np.array(path[i]), np.array(path[i+1])
        dist = np.linalg.norm(p2 - p1)
        num_points = max(2, int(dist / interval))
        for j in range(num_points):
            point = p1 + (p2 - p1) * (j / num_points)
            dense_path.append(tuple(point))
    dense_path.append(path[-1])
    return dense_path
# 消防车 (大货车) 轨迹数据
RAW_PATH_TRUCK = [
    (182.728, -12.900, 2.022), (180.081, -12.962, 1.608), (178.115, -13.022, 1.308),
    (178.115, -13.022, 1.308), (175.742, -13.046, 0.976), (173.613, -13.026, 0.725),
    (173.613, -13.026, 0.725), (173.613, -13.026, 0.725), (173.613, -13.026, 0.725),
    (173.613, -13.026, 0.725), (173.613, -13.026, 0.725), (170.975, -12.980, 0.621),
    (170.975, -12.980, 0.621), (169.942, -12.898, 0.622), (168.092, -12.750, 0.624),
    (165.676, -12.562, 0.621), (165.676, -12.562, 0.621), (162.811, -12.419, 0.617),
    (159.910, -12.295, 0.620), (159.910, -12.295, 0.620), (157.016, -12.214, 0.623),
    (156.757, -12.210, 0.624), (153.605, -12.164, 0.639), (150.650, -12.147, 0.653),
    (150.650, -12.147, 0.653), (147.644, -12.147, 0.667), (144.455, -12.152, 0.678),
    (143.664, -12.158, 0.681), (140.738, -12.180, 0.691), (137.502, -12.204, 0.702),
    (136.209, -12.234, 0.704), (134.082, -12.284, 0.706), (134.082, -12.284, 0.706),
    (130.356, -12.380, 0.724), (130.356, -12.380, 0.724), (130.356, -12.380, 0.724),
    (126.976, -12.454, 0.735), (123.749, -12.502, 0.735), (121.924, -12.511, 0.739),
    (120.334, -12.519, 0.743), (116.996, -12.536, 0.750), (116.996, -12.536, 0.750),
    (113.823, -12.536, 0.757), (111.298, -12.536, 0.763), (108.682, -12.537, 0.769),
    (106.115, -12.537, 0.775), (103.832, -12.537, 0.780), (101.044, -12.537, 0.786),
    (100.279, -12.537, 0.788), (97.471, -12.538, 0.794), (94.086, -12.538, 0.802),
    (94.086, -12.538, 0.802), (90.433, -12.534, 0.810), (90.433, -12.534, 0.810),
    (87.325, -12.568, 0.836), (83.929, -12.606, 0.865), (79.773, -12.652, 0.900),
    (78.468, -12.667, 0.911), (75.657, -12.698, 0.934), (75.657, -12.698, 0.934),
    (75.657, -12.698, 0.934), (73.882, -12.655, 0.949), (72.657, -12.625, 0.959),
    (71.383, -12.606, 0.972), (69.894, -12.585, 0.986), (69.894, -12.585, 0.986),
    (69.894, -12.585, 0.986), (69.894, -12.585, 0.986), (69.894, -12.585, 0.986),
    (69.894, -12.585, 0.986), (69.092, -12.573, 0.994), (66.095, -12.524, 1.037),
    (63.571, -12.471, 1.098), (63.571, -12.471, 1.098), (63.571, -12.471, 1.098),
    (61.049, -12.298, 1.141), (58.460, -11.715, 1.150), (55.484, -10.980, 1.135),
    (52.656, -10.277, 1.120), (52.656, -10.277, 1.120), (49.889, -9.752, 1.079),
    (49.889, -9.752, 1.079), (46.500, -9.405, 1.029), (45.242, -9.343, 1.007),
    (43.800, -9.277, 0.982), (40.477, -9.116, 0.923), (37.366, -9.025, 0.880),
    (34.529, -9.042, 0.838), (32.422, -9.076, 0.812), (31.499, -9.091, 0.801),
    (28.600, -9.137, 0.768), (27.446, -9.113, 0.787), (25.164, -9.066, 0.822),
    (21.611, -9.118, 0.900), (18.281, -9.167, 0.973), (15.519, -9.208, 1.033),
    (12.729, -9.211, 1.090), (9.461, -9.215, 1.157), (9.461, -9.215, 1.157),
    (9.461, -9.215, 1.157), (9.461, -9.215, 1.157), (9.461, -9.215, 1.157),
    (9.461, -9.215, 1.157), (9.461, -9.215, 1.157), (9.461, -9.215, 1.157),
    (-120.461, -9.215, 1.157)

]

# 基于货车轨迹动态生成 A2 协作变道超车轨迹
RAW_PATH_A2 = [(210.570, -9.220, 0.8), (179.570, -9.220, 0.8),
    (150.000, -9.220, 0.8), (50.000, -9.220, 0.8)]

RAW_PATH_A2_DENSE = interpolate_path(RAW_PATH_A2, interval=1.0)

# Ego 轨迹
RAW_PATH_ego = [
    (200.427, -12.666, 0.648), (197.589, -12.773, 0.361), (195.212, -12.801, 0.304),
    (192.614, -12.852, 0.428), (189.765, -12.931, 0.565), (187.630, -13.011, 0.633),
    (182.868, -13.194, 0.781), (182.162, -13.221, 0.802), (179.122, -13.323, 0.868),
    (174.375, -13.358, 0.954), (172.964, -13.368, 0.980), (170.167, -13.347, 1.017),
    (165.305, -13.245, 1.081), (162.470, -13.151, 1.119), (159.010, -13.057, 1.155),
    (154.293, -13.010, 1.177), (150.797, -13.047, 1.198), (145.906, -13.059, 1.232),
    (140.005, -13.066, 1.274), (136.935, -13.069, 1.296), (134.801, -13.071, 1.311),
    (132.425, -13.079, 1.326), (128.968, -13.107, 1.349), (124.774, -13.178, 1.384),
    (124.774, -13.178, 1.384), (121.825, -13.178, 1.412), (119.363, -13.090, 1.433),
    (116.860, -13.027, 1.457), (113.672, -13.022, 1.496), (110.524, -12.999, 1.541),
    (106.678, -12.949, 1.606), (103.429, -12.918, 1.661), (98.957, -12.879, 1.737),
    (96.080, -12.858, 1.786), (93.164, -12.838, 1.835), (87.813, -12.857, 1.933),
    (85.142, -12.867, 1.981), (80.091, -12.884, 2.073), (75.179, -12.902, 2.162),
    (70.435, -12.921, 2.248), (69.559, -12.925, 2.264), (68.898, -12.935, 2.165),
    (65.356, -12.991, 1.583), (59.268, -13.335, 1.053), (58.411, -13.381, 1.012),
    (52.656, -13.272, 1.028), (47.302, -13.198, 1.032), (41.287, -13.209, 1.042),
    (38.156, -13.230, 1.053), (32.622, -13.215, 1.082), (27.472, -13.174, 1.127),
    (20.037, -13.117, 1.192), (12.482, -13.060, 1.258), (9.668, -13.101, 1.279),
    (2.420, -13.285, 1.351), (-4.749, -13.375, 1.422), (-9.572, -13.391, 1.460),
    (-16.650, -13.280, 1.498), (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508),
    (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508),
    (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508),
    (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508), (-18.575, -13.239, 1.508),
    (-18.575, -13.239, 1.508)
]


def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )


# 【核心修复：自动计算角度】
def clean_and_convert_path(raw_path_points):
    path_transforms = []
    n = len(raw_path_points)
    for i in range(n):
        p = raw_path_points[i]
        yaw = 0.0
        found_next = False
        # 寻找下一个非重合点（距离大于0.1米）来计算正确的航向角
        for j in range(i + 1, n):
            next_p = raw_path_points[j]
            dx, dy = next_p[0] - p[0], next_p[1] - p[1]
            if math.sqrt(dx ** 2 + dy ** 2) > 0.1:
                yaw = math.degrees(math.atan2(dy, dx))
                found_next = True
                break
        # 如果后面全是重复点，沿用上一个点的角度
        if not found_next and i > 0:
            yaw = path_transforms[-1].rotation.yaw

        path_transforms.append(carla.Transform(
            carla.Location(x=p[0], y=p[1], z=p[2] + 0.5),
            carla.Rotation(yaw=yaw)
        ))
    return path_transforms

PATH_A2_TRANSFORMS = clean_and_convert_path(RAW_PATH_A2_DENSE) # 必须使用 DENSE
PATH_TRUCK_TRANSFORMS = clean_and_convert_path(RAW_PATH_TRUCK)
RAW_ego_TRANSFORMS = clean_and_convert_path(RAW_PATH_ego)


# ==========================================
# 主程序
# ==========================================

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # 设置天气为 ClearSunset
    weather = carla.WeatherParameters(
        cloudiness=15.0, precipitation=10.0, precipitation_deposits=50.0, wind_intensity=10.0,
        sun_azimuth_angle=1.0, sun_altitude_angle=6.0, fog_density=10.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=20.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
        rayleigh_scattering_scale=0.0331, dust_storm=0.0
    )
    world.set_weather(weather)
    print("天气已设置为 ClearSunset")


    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        # 固定每帧物理时间为 0.05秒 (即 20 FPS)
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(100.0)

        # ==========================================
        # 修复后的 A2 生成逻辑（替换 main 函数中的对应部分）
        # ==========================================

        # 在 main 函数开始生成 A2 的地方：
        bp_a2 = bp_lib.find('vehicle.audi.tt')

        # 定义一个高度重试列表 [0.5米, 1.5米, 3.0米]
        retry_heights = [0.5, 1.5, 3.0]
        spawn_success = False

        print("正在尝试生成 A2...")
        for h in retry_heights:
            # 获取轨迹起点并增加高度重试
            spawn_point_a2 = carla.Transform(
                carla.Location(
                    x=PATH_A2_TRANSFORMS[0].location.x,
                    y=PATH_A2_TRANSFORMS[0].location.y,
                    z=PATH_A2_TRANSFORMS[0].location.z + h
                ),
                PATH_A2_TRANSFORMS[0].rotation
            )

            vehicle_a2 = world.try_spawn_actor(bp_a2, spawn_point_a2)

            if vehicle_a2:
                actor_list.append(vehicle_a2)
                vehicle_a2.set_simulate_physics(True)
                print(f"A2 在高度偏移 {h}m 处生成成功！")
                spawn_success = True
                break
            else:
                # 如果高度重试失败，尝试向路中心（Y轴减小方向）偏移 1.5 米再试一次
                spawn_point_a2.location.y -= 1.5
                vehicle_a2 = world.try_spawn_actor(bp_a2, spawn_point_a2)
                if vehicle_a2:
                    actor_list.append(vehicle_a2)
                    vehicle_a2.set_simulate_physics(True)
                    print(f"A2 在位置偏移后生成成功！")
                    spawn_success = True
                    break

        if not spawn_success:
            print("严重错误：无法在任何重试点生成 A2。请检查坐标 (51, 8.7) 是否在路外。")
            # 如果实在不行，直接使用消防车前方的某个轨迹点生成
            print("尝试使用备选轨迹点生成...")
            vehicle_a2 = world.try_spawn_actor(bp_a2, PATH_A2_TRANSFORMS[5])
            if vehicle_a2:
                actor_list.append(vehicle_a2)
                print("A2 已在备选点 5 生成。")

        # --- 后续初始化 ---
        if vehicle_a2:
            lon_controller_a2 = PIDLongitudinalController()
            lat_controller_a2 = PIDLateralController()
            target_speed_a2_kmh = 80.0

        # --- 2. 生成 消防车 (被超车的大货车) ---
        bp_truck = bp_lib.find('vehicle.carlamotors.firetruck')
        bp_truck.set_attribute('color', '255,255,255')

        # 将消防车位置向前移：从索引 13 (x=69) 移到 索引 35 (x=43)
        # 此时 A2 在 x=72，消防车在 x=43，初始间距约 29 米，非常安全
        trans_truck = PATH_TRUCK_TRANSFORMS[0]

        vehicle_truck = world.try_spawn_actor(bp_truck, trans_truck)

        if vehicle_truck:
            actor_list.append(vehicle_truck)
            vehicle_truck.set_simulate_physics(True)
            print("消防车 (被超车辆) 生成成功")

            lon_controller_truck = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
            lat_controller_truck = PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
            target_speed_truck_kmh = 80.0
            current_target_speed_truck = target_speed_truck_kmh / 3.6
        else:
            print("消防车 生成失败！")

        # --- 3. 生成 Ego 车辆 (修复了复制粘贴导致覆盖 A2 的 bug) ---
        bp_ego = bp_lib.find('vehicle.tesla.model3')
        trans_ego = RAW_ego_TRANSFORMS[0]
        vehicle_ego = world.try_spawn_actor(bp_ego, trans_ego)
        if vehicle_ego:
            actor_list.append(vehicle_ego)
            vehicle_ego.set_simulate_physics(True)
            print("Ego 车辆生成成功")

            lon_controller_ego = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
            lat_controller_ego = PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
            target_speed_ego_kmh = 80.0
            current_target_speed_ego = target_speed_ego_kmh / 3.6
        else:
            print("Ego 车辆生成失败！")




        print("等待 1 秒，物理稳定中...")

        print("\n场景运行中... (车辆控制)")
        simulation_start_time = time.time()
        # ==========================================
        # 1. 状态与核心参数
        # ==========================================
        a2_state = "FOLLOW"  # 状态机: FOLLOW, RELEASE, INDEPENDENT
        release_start_time = 0
        a2_release_steer_start = 0.0
        a2_fixed_throttle = 0.35  # 这是一个经验值，足以维持 Audi 70km/h 的巡航动力

        TARGET_GAP = 5.0  # 保持 5 米车头对车尾净空
        SAFE_SPEED = 55.0
        BASE_SPEED = 80.0

        TRUCK_HALF_LEN, EGO_HALF_LEN = 5.0, 2.5

        print("\n场景运行：货车延后转向，A2 切换逻辑已重构...")
        # # 将视角绑定到 Ego 车后方以便观察
        # spectator = world.get_spectator()
        while True:
            start_time = time.time()
            world.tick()
            # # ---------------- 视角跟随 ----------------
            # if vehicle_ego and vehicle_ego.is_alive:
            #     tf = vehicle_ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))

            tf_e = vehicle_ego.get_transform()
            speed_e = math.sqrt(vehicle_ego.get_velocity().x ** 2 + vehicle_ego.get_velocity().y ** 2)

            # --- 1. 消防车 (Truck) 控制：延后 10m 转向 ---
            truck_is_turning = False
            if vehicle_truck:
                tf_t = vehicle_truck.get_transform()
                v_t = vehicle_truck.get_velocity()
                speed_t = math.sqrt(v_t.x ** 2 + v_t.y ** 2 + v_t.z ** 2)

                target_wp_t = get_target_waypoint(tf_t.location, PATH_TRUCK_TRANSFORMS, lookahead_dist=7.0)
                target_loc_t = carla.Location(target_wp_t)

                # 延后转向核心：原定转向在 x=55 附近。我们强制在 x > 45 之前保持 Y 轴不变
                if tf_t.location.x >42.0:
                    target_loc_t.y = PATH_TRUCK_TRANSFORMS[0].location.y

                steer_t = lat_controller_truck.run_step(target_loc_t, tf_t)
                if abs(steer_t) > 0.08: truck_is_turning = True

                # 【状态切换触发器】
                # 当货车走完弯路（x < 20）且车身正，释放 A2
                if a2_state == "FOLLOW" and tf_t.location.x < 0.0 and abs(steer_t) < 0.03:
                    a2_state = "RELEASE"
                    release_start_time = time.time()
                    a2_release_steer_start = vehicle_a2.get_control().steer
                    print(">>> A2 开始释放...")

                t_v_t = SAFE_SPEED if truck_is_turning else BASE_SPEED
                out_t = lon_controller_truck.run_step(t_v_t / 3.6, speed_t)
                vehicle_truck.apply_control(
                    carla.VehicleControl(throttle=max(0, out_t), steer=steer_t, brake=abs(min(0, out_t))))

            # --- 2. Ego 车辆控制 (ACC 10m) ---
            if vehicle_ego and vehicle_truck:
                tf_e = vehicle_ego.get_transform()
                v_e = vehicle_ego.get_velocity()
                speed_e = math.sqrt(v_e.x ** 2 + v_e.y ** 2 + v_e.z ** 2)

                center_d = tf_e.location.distance(tf_t.location)
                net_gap = center_d - TRUCK_HALF_LEN - EGO_HALF_LEN

                # ACC 逻辑
                t_v_e = speed_t + 1.2 * (net_gap - TARGET_GAP)
                limit_v = (SAFE_SPEED if truck_is_turning else BASE_SPEED) / 3.6
                t_v_e = min(t_v_e, limit_v)

                if net_gap < 3.0:  # AEB
                    t_out_e, b_out_e = 0.0, 1.0
                else:
                    out_e = lon_controller_ego.run_step(t_v_e, speed_e)
                    t_out_e, b_out_e = (out_e, 0.0) if out_e >= 0 else (0.0, abs(out_e))

                wp_e = get_target_waypoint(tf_e.location, RAW_ego_TRANSFORMS, lookahead_dist=7.0)
                st_e = lat_controller_ego.run_step(wp_e, tf_e)
                vehicle_ego.apply_control(carla.VehicleControl(throttle=t_out_e, steer=st_e, brake=b_out_e))

                # --- 3. A2 车辆控制：修正方向与摆动问题 ---
                if vehicle_a2 and vehicle_ego:
                    tf_a2 = vehicle_a2.get_transform()
                    loc_a2 = tf_a2.location
                    v_a2 = vehicle_a2.get_velocity()
                    speed_a2 = math.sqrt(v_a2.x ** 2 + v_a2.y ** 2 + v_a2.z ** 2)

                    ctrl_a2 = carla.VehicleControl()

                    if a2_state == "FOLLOW":
                        # 【并排行驶逻辑】
                        # 横向控制：预瞄点必须在车头前方（X轴负方向），所以是 loc_a2.x - 15.0
                        # 目标 Y 轴死锁在 -9.22
                        target_loc = carla.Location(x=loc_a2.x - 15.0, y=-9.22, z=loc_a2.z)

                        # 修复报错：直接传递 Location 对象，满足控制器内部 waypoint.x 的调用
                        ctrl_a2.steer = lat_controller_a2.run_step(target_loc, tf_a2)

                        # 纵向控制：对齐 Ego 的 X 坐标
                        # 在 X 负方向坐标系下：loc_a2.x > tf_e.location.x 表示 A2 落后了
                        x_error = loc_a2.x - tf_e.location.x

                        # 目标速度 = Ego速度 + 距离差补偿 (落后越多加越多速)
                        # 限制最大速度差，防止起步过猛摆动
                        t_v_a2 = speed_e + np.clip(1.5 * x_error, -5.0, 5.0)

                        # 最小保证速度 2.0m/s，防止卡死
                        t_v_a2 = max(t_v_a2, 2.0 / 3.6)

                        out_a2 = lon_controller_a2.run_step(t_v_a2, speed_a2)
                        ctrl_a2.throttle = np.clip(out_a2, 0.0, 0.8)
                        ctrl_a2.brake = np.clip(-out_a2, 0.0, 1.0)

                    elif a2_state == "RELEASE":
                        # 过渡阶段：平滑转向
                        ratio = min(1.0, (time.time() - release_start_time) / 0.5)
                        ctrl_a2.steer = a2_release_steer_start * (1.0 - ratio)
                        ctrl_a2.throttle = a2_fixed_throttle
                        if ratio >= 1.0:
                            a2_state = "INDEPENDENT"
                            lat_controller_a2._error_buffer.clear()

                    elif a2_state == "INDEPENDENT":
                        # 【独立巡航逻辑】
                        # 同样：预瞄点在车头前方（减去 15 米）
                        target_loc = carla.Location(x=loc_a2.x - 15.0, y=-9.22, z=loc_a2.z)
                        ctrl_a2.steer = lat_controller_a2.run_step(target_loc, tf_a2)

                        # 稳速巡航
                        error_v = (BASE_SPEED / 3.6) - speed_a2
                        ctrl_a2.throttle = np.clip(a2_fixed_throttle + 0.1 * error_v, 0.0, 1.0)
                        ctrl_a2.brake = 0.0

                    # 确保每一帧都下发控制
                    vehicle_a2.apply_control(ctrl_a2)
                    # --- 帧率同步补偿 ---
                    compute_time = time.time() - start_time
                    if compute_time < 0.05:
                        time.sleep(0.05 - compute_time)
    except Exception as e:
        print(f"发生异常：{e}")
        if actor_list:
            print("清理剩余 Actors...")
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        tm.set_synchronous_mode(False)

        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
            print(f"已销毁 {len(actor_list)} 个 Actor。")

        print("清理完成，Carla 已恢复正常。")


if __name__ == '__main__':
    main()