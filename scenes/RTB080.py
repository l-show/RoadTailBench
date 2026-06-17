import carla
import time
import math
import numpy as np

# ==========================================
# 1. 轨迹数据清洗 (Vehicle 1: Mustang)
# ==========================================
RAW_MUSTANG_TRAJECTORY = [
    (-63.57, -51.941, 36.926), (-63.57, -51.941, 36.926), (-63.57, -51.941, 37.281),
    (-63.57, -51.941, 37.846), (-59.903, -49.105, 37.633), (-55.873, -45.881, 39.567),
    (-52.015, -42.7, 39.284), (-48.137, -39.543, 39.144), (-44.131, -36.281, 39.214),
    (-40.264, -33.112, 39.354), (-36.332, -29.887, 39.354), (-32.474, -26.707, 39.709),
    (-28.564, -23.459, 39.709), (-24.717, -20.265, 39.709), (-20.768, -16.934, 40.639),
    (-16.892, -13.517, 41.501), (-13.085, -10.149, 41.501), (-9.321, -6.858, 40.011),
    (-5.12, -3.857, 32.479), (-0.759, -1.424, 23.386), (4.208, -0.02, 14.172),
    (9.195, 0.942, 7.507), (14.339, 1.394, 2.243), (19.502, 1.488, 0.736),
    (24.5, 1.519, -0.182), (29.667, 1.527, 0.614), (34.833, 1.582, 0.614),
    (39.998, 1.626, 0.049), (45.08, 1.619, -0.093), (50.245, 1.61, -0.093),
    (55.246, 1.598, -0.672), (60.329, 1.521, -0.882), (65.35, 1.444, -0.882),
    (70.516, 1.364, -0.882), (75.516, 1.341, 0.189), (80.681, 1.448, 1.89),
    (85.844, 1.619, 1.89), (90.926, 1.786, 1.89), (95.923, 1.95, 1.82),
    (101.088, 2.088, 0.975), (106.1, 2.149, 0.687), (111.102, 2.209, 0.687),
    (116.268, 2.271, 0.687), (121.435, 2.333, 0.687), (126.435, 2.393, 0.687),
    (131.518, 2.454, 0.687), (136.684, 2.516, 0.687), (141.85, 2.595, 1.833),
    (146.927, 2.854, 3.902), (151.996, 3.23, 4.749), (157.142, 3.708, 6.237),
    (162.108, 4.289, 7.233), (164.251, 4.603, 8.682), (164.251, 4.603, 8.682),
    (164.251, 4.603, 8.682)
]
MUSTANG_TRAJECTORY = []
for p in RAW_MUSTANG_TRAJECTORY:
    if not MUSTANG_TRAJECTORY or p != MUSTANG_TRAJECTORY[-1]:
        MUSTANG_TRAJECTORY.append(p)

# ==========================================
# 2. 轨迹数据清洗 (Vehicle 2: Jeep)
# ==========================================
RAW_JEEP_TRAJECTORY = [
    (-67.107, 58.988, -44.247), (-67.107, 58.988, -44.247), (-67.107, 58.988, -43.687),
    (-66.061, 58.007, -42.906), (-63.223, 55.465, -40.429), (-60.263, 52.965, -39.651),
    (-57.366, 50.583, -38.805), (-54.377, 48.217, -38.239), (-51.348, 46.007, -34.176),
    (-48.194, 43.866, -34.176), (-45.092, 41.757, -34.318), (-41.958, 39.588, -35.737),
    (-38.922, 37.385, -36.017), (-35.792, 35.1, -36.44), (-32.698, 32.768, -38.486),
    (-29.807, 30.38, -39.783), (-26.888, 27.928, -40.281), (-23.941, 25.412, -41.207),
    (-21.134, 22.925, -42.055), (-18.349, 20.413, -42.055), (-15.519, 17.859, -42.195),
    (-12.716, 15.274, -43.108), (-9.979, 12.711, -43.108), (-7.15, 10.063, -43.108),
    (-4.348, 7.573, -38.91), (-1.33, 5.349, -33.689), (1.94, 3.495, -26.915),
    (5.526, 2.038, -17.543), (9.148, 1.074, -12.945), (12.833, 0.385, -6.967),
    (16.698, 0.219, 2.854), (20.434, 0.532, 4.348), (24.303, 0.755, 3.873),
    (28.043, 1.03, 4.301), (31.657, 1.301, 2.128), (32.892, 1.114, -11.85),
    (36.704, 0.477, -2.935), (40.576, 0.515, 3.192), (44.379, 0.771, 3.981),
    (48.244, 1.04, 3.981), (52.05, 1.256, 2.563), (55.86, 1.379, 1.647),
    (59.733, 1.491, 1.647), (63.543, 1.551, 0.358), (67.353, 1.546, -2.394),
    (71.099, 1.39, -2.394), (74.908, 1.236, -2.041), (78.656, 1.158, -0.547),
    (82.531, 1.124, 0.176), (86.405, 1.163, 0.606), (90.218, 1.171, -0.174),
    (94.03, 1.147, -0.597), (97.731, 1.108, -0.597), (101.606, 1.077, -0.317),
    (105.481, 1.067, 0.109), (109.362, 1.1, 0.531), (113.236, 1.185, 1.672),
    (117.11, 1.298, 1.672), (120.92, 1.41, 1.672), (124.669, 1.519, 1.672),
    (128.479, 1.63, 1.672), (132.228, 1.74, 1.672), (136.101, 1.853, 1.672),
    (139.975, 1.966, 1.815), (143.719, 2.173, 3.821), (147.523, 2.442, 4.604),
    (151.259, 2.768, 5.242), (155.118, 3.122, 5.312), (158.911, 3.5, 5.737),
    (162.767, 3.888, 5.737), (166.56, 4.269, 5.737), (166.933, 4.306, 5.737),
    (166.933, 4.306, 5.737)
]
JEEP_TRAJECTORY = []
for p in RAW_JEEP_TRAJECTORY:
    if not JEEP_TRAJECTORY or p != JEEP_TRAJECTORY[-1]:
        JEEP_TRAJECTORY.append(p)


# ==========================================
# 3. 核心模块: PID 控制器与寻路逻辑
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


def get_target_from_trajectory(vehicle_loc, trajectory, lookahead_dist=10.0):
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


# ==========================================
# 4. 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # --- 天气与同步设置 ---
        weather = carla.WeatherParameters(
            cloudiness=25.0, precipitation=0.0, wind_intensity=10.0,
            sun_azimuth_angle=350.0, sun_altitude_angle=15.0, fog_density=6.0,
            scattering_intensity=1.0, rayleigh_scattering_scale=0.14
        )
        world.set_weather(weather)

        settings = world.get_settings()
        settings.synchronous_mode = True
        delta_seconds = 0.05
        settings.fixed_delta_seconds = delta_seconds
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        active_pid_vehicles = []

        # --- 车辆 1: Mustang (80km/h) ---
        bp_mustang = bp_lib.find('vehicle.ford.mustang')
        bp_mustang.set_attribute('color', '255,255,0')
        start_m = MUSTANG_TRAJECTORY[0]
        mustang = world.try_spawn_actor(bp_mustang, carla.Transform(carla.Location(x=start_m[0], y=start_m[1], z=1.5),
                                                                    carla.Rotation(yaw=start_m[2])))
        if mustang:
            actor_list.append(mustang)
            active_pid_vehicles.append({
                'id': 'Mustang', 'actor': mustang, 'lon_pid': PIDLongitudinalController(),
                'lat_pid': PIDLateralController2(), 'target_speed': 80.0,
                'mode': 'trajectory', 'trajectory': MUSTANG_TRAJECTORY, 'lookahead': 15.0
            })

        # --- 车辆 2: Jeep (60km/h) ---
        bp_jeep = bp_lib.find('vehicle.jeep.wrangler_rubicon')
        bp_jeep.set_attribute('color', '0,0,255')
        start_j = JEEP_TRAJECTORY[0]
        jeep = world.try_spawn_actor(bp_jeep, carla.Transform(carla.Location(x=start_j[0], y=start_j[1], z=1.5),
                                                              carla.Rotation(yaw=start_j[2])))
        if jeep:
            actor_list.append(jeep)
            active_pid_vehicles.append({
                'id': 'Jeep', 'actor': jeep, 'lon_pid': PIDLongitudinalController(),
                'lat_pid': PIDLateralController2(), 'target_speed': 60.0,
                'mode': 'trajectory', 'trajectory': JEEP_TRAJECTORY, 'lookahead': 10.0
            })

        # --- 车辆 3: Microlino (TM控制, 超速10%) ---
        bp_micro = bp_lib.find('vehicle.micro.microlino')
        bp_micro.set_attribute('color', '0,255,0')
        loc_micro = carla.Location(x=130.700, y=-1.919, z=1.5)
        micro = world.try_spawn_actor(bp_micro, carla.Transform(loc_micro, carla.Rotation(
            yaw=carla_map.get_waypoint(loc_micro).transform.rotation.yaw)))
        if micro:
            actor_list.append(micro)
            micro.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(micro, -10.0)

            # --- 车辆 4: Ego (TM控制, 初始速度 60km/h) ---
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2020')
        loc_ego = carla.Location(x=110.700, y=-1.919, z=1.5)
        yaw_ego = carla_map.get_waypoint(loc_ego).transform.rotation.yaw
        ego = world.try_spawn_actor(bp_ego, carla.Transform(loc_ego, carla.Rotation(yaw=yaw_ego)))
        if ego:
            actor_list.append(ego)
            ego.set_autopilot(True, tm.get_port())
            init_speed_ms = 60.0 / 3.6
            ego.set_target_velocity(carla.Vector3D(x=init_speed_ms * math.cos(math.radians(yaw_ego)),
                                                   y=init_speed_ms * math.sin(math.radians(yaw_ego)), z=0.0))
            print("Ego 生成成功 (TM控制, 初始60km/h, 避障全开)")

        # ---------------------------------------------------
        print("\n场景初始化完毕，开始仿真运行...")

        while True:
            start_time = time.time()
            world.tick()

            # 1. 车辆 PID 循迹更新
            for v_data in reversed(active_pid_vehicles):
                vehicle = v_data['actor']
                if not vehicle.is_alive:
                    active_pid_vehicles.remove(v_data)
                    continue

                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                look_dist = v_data.get('lookahead', 12.0)
                target_point = get_target_from_trajectory(tf.location, v_data['trajectory'], lookahead_dist=look_dist)

                if target_point is None:
                    control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
                    vehicle.apply_control(control)
                    active_pid_vehicles.remove(v_data)
                    continue

                throttle_out = v_data['lon_pid'].run_step(v_data['target_speed'], speed)
                steer_out = v_data['lat_pid'].run_step(target_point, tf)

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
            if compute_time < delta_seconds:
                time.sleep(delta_seconds - compute_time)

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

        # 遍历销毁所有的 Actor (包含所有车辆和鸡群)
        for a in actor_list:
            if a.is_alive:
                a.destroy()
        print("所有实体清理完成！")


if __name__ == '__main__':
    main()