import carla
import time
import math
import numpy as np

# ==========================================
# 1. 轨迹数据清洗 (Sprinter 货车轨迹)
# ==========================================
RAW_Sprinter_TRAJECTORY = [
    (3.677, 107.183, -93.454), (3.677, 107.183, -93.454), (3.677, 107.183, -93.454),
    (3.677, 107.183, -93.314), (3.23, 99.573, -92.894), (3.163, 97.581, -90.375),
    (3.163, 97.581, -90.445), (3.163, 97.581, -89.885), (3.163, 97.581, -89.885),
    (3.295, 89.837, -87.504), (3.45, 86.441, -87.714), (3.641, 80.945, -88.764),
    (3.647, 78.395, -91.494), (3.512, 72.397, -89.814), (3.476, 71.098, -91.563),
    (3.323, 67.802, -93.173), (3.256, 65.704, -91.283), (3.192, 62.855, -91.283),
    (3.239, 55.106, -89.813), (3.163, 53.509, -93.523), (3.076, 51.511, -91.423),
    (3.037, 49.962, -91.423), (3.000, 48.262, -87.643), (3.016, 45.863, -90.512),
    (2.986, 43.863, -90.862), (2.971, 43.063, -92.612), (2.922, 41.665, -102.2),
    (2.294, 40.414, -121.798), (1.552, 39.347, -124.808), (0.181, 35.654, -101.011),
    (0.359, 33.513, -83.442), (0.793, 30.242, -81.762), (1.537, 24.393, -90.721),
    (1.549, 20.094, -87.85), (1.691, 15.997, -88.34), (1.866, 9.95, -88.34),
    (1.91, 5.751, -90.23), (1.898, 1.602, -89.81), (1.871, -4.748, -90.86),
    (1.784, -11.747, -90.509), (1.739, -17.347, -90.37), (1.682, -23.196, -90.579),
    (1.626, -28.746, -90.579), (1.562, -34.995, -90.579), (1.51, -41.044, -90.23),
    (1.512, -45.844, -89.39), (1.593, -51.743, -88.69), (1.734, -57.891, -88.76),
    (1.826, -64.14, -89.249), (1.853, -69.439, -90.509), (1.796, -73.937, -90.859),
    (1.765, -78.436, -89.109), (1.846, -83.335, -88.479), (1.978, -89.783, -89.669),
    (1.912, -94.432, -91.138), (1.834, -98.78, -90.648), (1.828, -104.18, -89.528),
    (1.932, -109.929, -88.548), (2.021, -115.277, -89.738), (2.038, -118.977, -89.668),
    (2.1, -125.672, -89.528), (2.133, -131.42, -89.668), (2.288, -143.068, -89.598),
    (2.296, -153.366, -89.878), (2.356, -160.615, -89.528), (2.476, -172.064, -89.317),
    (2.517, -179.764, -89.947), (2.528, -191.414, -89.947), (2.538, -203.014, -89.947),
    (2.541, -205.964, -89.947), (2.546, -210.914, -89.947), (2.546, -210.914, -89.947)
]

Sprinter_TRAJECTORY = []
for p in RAW_Sprinter_TRAJECTORY:
    if not Sprinter_TRAJECTORY or p != Sprinter_TRAJECTORY[-1]:
        Sprinter_TRAJECTORY.append(p)

# ==========================================
# 2. 轨迹数据清洗 (HGV 半挂车轨迹)
# ==========================================
RAW_HGV_TRAJECTORY = [
    (-2.186, -49.028, 88.272), (-2.186, -49.028, 88.622), (-2.186, -49.028, 89.042),
    (-2.186, -49.028, 89.252), (-2.186, -49.028, 89.252), (-2.134, -44.628, 89.392),
    (-2.130, -44.328, 89.532), (-2.124, -43.528, 89.532), (-2.065, -36.328, 89.602),
    (-2.065, -36.328, 88.131), (-1.925, -32.581, 87.991), (-1.804, -29.034, 89.251),
    (-1.823, -25.084, 91.071), (-1.823, -25.084, 88.901), (-1.780, -22.834, 88.901),
    (-1.714, -19.435, 88.761), (-1.597, -16.638, 87.571), (-1.368, -12.494, 86.801),
    (-1.187, -9.249, 86.801), (-1.187, -9.249, 86.801), (-1.187, -9.249, 87.710),
    (-1.125, -7.102, 87.850), (-1.018, -5.064, 87.010), (-0.878, -2.381, 87.010),
    (-0.878, -2.381, 86.870), (-0.805, -1.039, 86.730), (-0.699, 0.800, 86.660),
    (-0.522, 4.685, 93.380), (-1.008, 6.625, 104.509), (-1.443, 8.006, 138.385),
    (-2.949, 9.238, 141.885), (-2.949, 9.238, 158.053), (-2.949, 9.238, 150.704),
    (-4.337, 10.026, 150.424), (-7.953, 11.451, 165.962), (-11.073, 12.128, 170.090),
    (-11.713, 12.236, -178.991), (-15.235, 11.833, -161.982), (-20.004, 10.172, -160.022),
    (-24.062, 8.910, -163.171), (-29.250, 7.918, -175.208), (-35.391, 7.619, -177.867),
    (-45.332, 7.248, -177.657), (-54.447, 6.552, -173.457), (-61.029, 5.631, -173.036),
    (-71.595, 4.837, -178.775), (-82.142, 4.660, -178.495), (-92.781, 4.180, -177.235),
    (-102.569, 3.707, -177.235), (-112.807, 3.213, -177.235), (-115.953, 3.061, -177.235),
    (-115.953, 3.061, -177.235)
]

HGV_TRAJECTORY = []
for p in RAW_HGV_TRAJECTORY:
    if not HGV_TRAJECTORY or p != HGV_TRAJECTORY[-1]:
        HGV_TRAJECTORY.append(p)

# ==========================================
# PID 控制器类
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


class PIDLateralController2:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        wp_x = waypoint_loc.x if isinstance(waypoint_loc, carla.Location) else waypoint_loc[0]
        wp_y = waypoint_loc.y if isinstance(waypoint_loc, carla.Location) else waypoint_loc[1]

        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([wp_x - v_begin.x, wp_y - v_begin.y, 0.0])

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
# 核心路网/轨迹寻路逻辑
# ==========================================
def get_target_from_trajectory(vehicle_loc, trajectory, lookahead_dist=10.0):
    """ 基于固定 [X, Y, Yaw] 轨迹点的前瞻循迹寻找 """
    min_dist, closest_idx = float('inf'), 0
    for i, p in enumerate(trajectory):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist, closest_idx = dist, i

    target_idx = closest_idx
    current_dist = 0.0
    for i in range(closest_idx, len(trajectory) - 1):
        p1, p2 = trajectory[i], trajectory[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_idx = i + 1
        if current_dist >= lookahead_dist:
            break

    if target_idx >= len(trajectory) - 1 and current_dist < lookahead_dist:
        return None
    return trajectory[target_idx]


def get_next_waypoint_by_angle(current_wp, vehicle_transform, distance=5.0, action='straight'):
    """ 根据夹角动态寻找 Carla Map 锚点 """
    next_wps = current_wp.next(distance)
    if not next_wps:
        return None
    if len(next_wps) == 1:
        return next_wps[0]

    v_forward = vehicle_transform.get_forward_vector()
    best_wp = next_wps[0]

    if action == 'straight':
        min_angle = float('inf')
        for wp in next_wps:
            wp_forward = wp.transform.get_forward_vector()
            angle = math.degrees(math.acos(np.clip(
                v_forward.x * wp_forward.x + v_forward.y * wp_forward.y, -1.0, 1.0)))
            if angle < min_angle:
                min_angle = angle
                best_wp = wp

    elif action == 'left':
        min_cross_z = float('inf')
        for wp in next_wps:
            wp_forward = wp.transform.get_forward_vector()
            cross_z = v_forward.x * wp_forward.y - v_forward.y * wp_forward.x
            if cross_z < min_cross_z:
                min_cross_z = cross_z
                best_wp = wp
        return best_wp

    return best_wp


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 天气设置
        weather = carla.WeatherParameters(
            cloudiness=40.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=100.0, sun_azimuth_angle=140.0, sun_altitude_angle=60.0,
            fog_density=0.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=6.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.1, dust_storm=0.0
        )
        world.set_weather(weather)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        active_pid_vehicles = []  # 记录需要手动控制的 PID 车辆状态

        # ================= 1. Sprinter 货车 (使用对应轨迹) =================
        bp_Sprinter = bp_lib.find('vehicle.mercedes.sprinter')
        # 初始坐标对齐轨迹的第一个点，防止瞬移抖动
        start_x_s, start_y_s, start_yaw_s = Sprinter_TRAJECTORY[0]
        trans_Sprinter = carla.Transform(carla.Location(x=start_x_s, y=start_y_s, z=1.5), carla.Rotation(yaw=start_yaw_s))
        Sprinter = world.try_spawn_actor(bp_Sprinter, trans_Sprinter)
        if Sprinter:
            actor_list.append(Sprinter)
            active_pid_vehicles.append({
                'id': 'Sprinter', 'actor': Sprinter,
                'lon_pid': PIDLongitudinalController(), 'lat_pid': PIDLateralController2(),
                'target_speed': 60.0, 'mode': 'trajectory',
                'trajectory': Sprinter_TRAJECTORY  # 绑定专属轨迹
            })
            print("Sprinter 生成成功 (使用给定轨迹循迹, 60km/h)")

        # ================= 2. HGV 半挂车头 (使用对应轨迹) =================
        bp_hgv = bp_lib.find('vehicle.carlamotors.european_hgv')
        # 初始坐标对齐轨迹的第一个点
        start_x_h, start_y_h, start_yaw_h = HGV_TRAJECTORY[0]
        trans_hgv = carla.Transform(carla.Location(x=start_x_h, y=start_y_h, z=1.5), carla.Rotation(yaw=start_yaw_h))
        hgv = world.try_spawn_actor(bp_hgv, trans_hgv)
        if hgv:
            actor_list.append(hgv)
            active_pid_vehicles.append({
                'id': 'HGV', 'actor': hgv,
                'lon_pid': PIDLongitudinalController(), 'lat_pid': PIDLateralController2(),
                'target_speed': 100.0, 'mode': 'trajectory',
                'trajectory': HGV_TRAJECTORY  # 绑定专属轨迹
            })
            print("HGV 生成成功 (使用给定轨迹循迹, 100km/h)")

        # ================= 3. Ego Lincoln MKZ (TM 左转大弯) =================
        bp_lincoln = bp_lib.find('vehicle.lincoln.mkz_2017')
        bp_lincoln.set_attribute('color', '192,192,192')
        loc_lincoln = carla.Location(x=-55.383, y=9.395, z=1.5)
        trans_lincoln = carla.Transform(loc_lincoln, carla_map.get_waypoint(loc_lincoln).transform.rotation)
        ego = world.try_spawn_actor(bp_lincoln, trans_lincoln)

        if ego:
            actor_list.append(ego)
            ego.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(ego, -5)  # 维持约 60km/h

            # 设置忽略项
            tm.ignore_lights_percentage(ego, 100)
            tm.ignore_signs_percentage(ego, 100)
            tm.ignore_vehicles_percentage(ego, 100)
            tm.ignore_walkers_percentage(ego, 100)
            tm.distance_to_leading_vehicle(ego, 0.0)
            tm.auto_lane_change(ego, False)

            print("正在为 Ego 车辆规划左转路径...")
            ego_route = []
            current_wp = carla_map.get_waypoint(loc_lincoln)

            # 生成长达 100 次迭代的锚点（确保通过大路口）
            for _ in range(100):
                ego_route.append(current_wp.transform.location)
                next_wp = get_next_waypoint_by_angle(current_wp, current_wp.transform, distance=2.5, action='left')
                if next_wp is None:
                    break
                current_wp = next_wp

            tm.set_path(ego, ego_route)
            print("Lincoln MKZ Ego 生成成功 (TM控制左转大弯, 60km/h)")

        print("\n场景初始化完毕，开始仿真运行...")

        # 仿真主循环
        while True:
            start_time = time.time()
            world.tick()

            # 逆序遍历，安全删除结束任务的车辆
            for v_data in reversed(active_pid_vehicles):
                vehicle = v_data['actor']
                if not vehicle.is_alive:
                    active_pid_vehicles.remove(v_data)
                    continue

                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                target_loc = None

                if v_data['mode'] == 'trajectory':
                    # 【核心修改点】自动从字典中读取自己专属的轨迹数组
                    target_point = get_target_from_trajectory(tf.location, v_data['trajectory'], lookahead_dist=12.0)
                    if target_point is None:
                        print(f"[{v_data['id']}] 轨迹执行完毕，销毁车辆。")
                        vehicle.destroy()
                        active_pid_vehicles.remove(v_data)
                        continue
                    target_loc = target_point  # (X, Y) tuple

                elif v_data['mode'] == 'waypoint':
                    current_wp = carla_map.get_waypoint(tf.location)
                    target_wp = get_next_waypoint_by_angle(current_wp, tf, distance=12.0, action='straight')
                    if target_wp is None:
                        print(f"[{v_data['id']}] 行驶至地图尽头，销毁车辆。")
                        vehicle.destroy()
                        active_pid_vehicles.remove(v_data)
                        continue
                    target_loc = target_wp.transform.location

                # PID 控制下发
                if target_loc is not None:
                    throttle_out = v_data['lon_pid'].run_step(v_data['target_speed'], speed)
                    steer_out = v_data['lat_pid'].run_step(target_loc, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_out
                    if throttle_out >= 0.0:
                        control.throttle = throttle_out
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_out)

                    vehicle.apply_control(control)

            # 时间同步锁帧
            compute_time = time.time() - start_time
            if compute_time < 0.05:
                time.sleep(0.05 - compute_time)

    except KeyboardInterrupt:
        print("\n用户中断运行。")
    except Exception as e:
        print(f"\n发生异常: {e}")
    finally:
        print("\n清理场景及恢复 Carla 设置...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if 'tm' in locals():
            tm.set_synchronous_mode(False)

        # 清除活着的车辆
        for a in actor_list:
            if a.is_alive:
                a.destroy()
        print("清理完成！")


if __name__ == '__main__':
    main()