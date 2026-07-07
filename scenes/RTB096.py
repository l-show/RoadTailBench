import carla
import time
import math
import numpy as np


# ==========================================
# 基础控制算法 (PID) - 保留核心逻辑
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
# 轨迹去重工具函数
# ==========================================
def remove_duplicate_waypoints(trajectory, dist_threshold=0.5):
    """剔除距离过近的重复轨迹点，防止车辆在原地打转或偏航角计算异常"""
    if not trajectory: return []
    cleaned_traj = [trajectory[0]]
    for pt in trajectory[1:]:
        last_pt = cleaned_traj[-1]
        dist = math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
        if dist > dist_threshold:
            cleaned_traj.append(pt)
    return cleaned_traj


# ==========================================
# 原始轨迹数据
# ==========================================
RAW_EGO_TRAJECTORY = [
    (25.488, 2.103, -1.69), (25.488, 2.103, -1.69), (25.488, 2.103, -1.76), (25.488, 2.103, -0.542),
    (26.57, 2.092, -0.542),
    (31.729, 2.08, -0.186), (36.809, 2.061, 0.108), (41.809, 2.092, 0.671), (46.892, 2.114, -0.315),
    (51.975, 2.056, -0.738),
    (57.143, 2.009, -0.175), (62.143, 1.993, -0.315), (67.31, 1.946, -0.525), (72.311, 1.9, -0.525),
    (77.311, 1.873, 0.041),
    (82.311, 1.892, 0.394), (87.394, 1.932, 0.464), (92.477, 1.973, 0.464), (97.534, 1.697, -16.685),
    (102.392, 0.24, -5.109),
    (107.531, 0.703, 6.728), (112.66, 1.327, 6.364), (117.644, 1.727, 3.697), (122.724, 1.888, 1.233),
    (127.723, 1.987, 0.878),
    (132.806, 2.055, 0.528), (137.89, 2.102, 0.528), (143.056, 2.15, 0.458), (148.223, 2.178, -0.178),
    (153.389, 2.154, -0.463),
    (158.55, 2.107, -0.326), (163.549, 2.084, -0.256), (168.633, 2.062, -0.256), (173.8, 2.039, -0.256),
    (178.885, 2.016, -0.256),
    (183.885, 1.994, -0.256), (189.052, 1.965, -0.326), (194.219, 1.935, -0.326), (199.22, 1.907, -0.326),
    (204.387, 1.877, -0.326),
    (209.388, 1.849, -0.326), (214.471, 1.82, -0.326), (219.555, 1.791, -0.326), (224.64, 1.762, -0.326),
    (229.64, 1.727, -0.897),
    (234.802, 1.531, -4.179), (239.855, 0.979, -8.349), (244.849, -0.281, -20.562), (249.534, -2.432, -29.714),
    (253.768, -5.378, -39.304),
    (257.432, -8.891, -49.158), (258.172, -9.792, -51.703), (258.172, -9.792, -51.703), (258.172, -9.792, -51.703)
]

# V2轨迹与Ego相同
RAW_V2_TRAJECTORY = RAW_EGO_TRAJECTORY.copy()

RAW_V3_TRAJECTORY = [
    (111.26, 76.816, -90.883), (111.26, 76.816, -90.883), (111.26, 76.816, -90.883), (111.261, 76.649, -89.75),
    (111.363, 67.988, -89.12), (111.491, 57.825, -90.053), (111.447, 47.524, -90.266), (111.4, 37.561, -90.266),
    (111.352, 27.609, -90.336), (111.333, 24.288, -90.758), (111.247, 15.627, -89.835), (111.264, 11.462, -89.765),
    (111.264, 11.462, -88.845), (111.332, 8.469, -88.632), (112.13, 4.228, -62.236), (113.977, 2.554, -41.38),
    (114.857, 1.793, -16.86), (122.259, -0.133, -8.453), (124.092, -0.097, 1.871), (134.235, 0.511, 2.812),
    (144.223, 0.989, 2.739), (154.22, 1.245, 1.224), (164.384, 1.459, 1.084), (174.55, 1.594, 0.659),
    (184.757, 1.664, 0.304), (194.757, 1.717, 0.234), (205.093, 1.739, 0.092), (215.087, 1.472, -4.646),
    (225.346, 0.237, -8.183), (235.244, -1.23, -10.318), (245.047, -3.867, -21.416), (253.607, -9.193, -46.661),
    (259.335, -17.341, -66.761), (261.367, -27.261, -87.822), (262.06, -37.396, -80.83), (262.735, -40.83, -78.074),
    (262.735, -40.83, -78.074), (262.735, -40.83, -78.074)
]

RAW_V4_TRAJECTORY = [
    (108.308, 104.328, -92.263), (108.308, 104.328, -92.263), (108.308, 104.328, -91.908), (108.015, 96.333, -91.76),
    (107.97, 86.003, -89.425), (108.183, 75.674, -88.93), (108.36, 65.343, -89.575), (108.279, 55.178, -90.641),
    (108.166, 45.013, -90.641), (108.069, 34.712, -90.361), (108.027, 24.58, -89.578), (108.423, 14.281, -81.192),
    (113.041, 5.314, -40.845), (119.59, 1.8, -9.14), (122.893, 1.357, -7.631), (133.144, 0.066, -6.993),
    (143.417, -1.049, -5.072), (153.736, -1.616, -2.335), (163.898, -1.791, 1.214), (173.879, -1.216, 4.527),
    (184.008, -0.351, 5.165), (193.967, 0.549, 5.165), (203.937, 1.302, 2.73), (214.266, 1.405, -0.953),
    (224.422, 1.07, -3.611), (234.693, 0.004, -8.355), (244.397, -2.335, -17.93), (253.399, -6.955, -36.592),
    (259.749, -14.577, -60.991), (263.323, -24.236, -77.366), (264.214, -34.337, -92.327), (262.561, -44.181, -98.289),
    (262.469, -49.341, -87.883), (262.469, -49.341, -87.883)
]

# 去重处理
EGO_TRAJ = remove_duplicate_waypoints(RAW_EGO_TRAJECTORY)
V2_TRAJ = remove_duplicate_waypoints(RAW_V2_TRAJECTORY)
V3_TRAJ = remove_duplicate_waypoints(RAW_V3_TRAJECTORY)
V4_TRAJ = remove_duplicate_waypoints(RAW_V4_TRAJECTORY)


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【修改点1】应用ClearNoon天气配置
    # 对应参数: [5.0, 0.0, 0.0, 10.0, -1.0, 45.0, 2.0, 0.75, 0.1, 0.0, 1.0, 0.03, 0.0331, 0.0]
    weather = carla.WeatherParameters(
        cloudiness=5.0,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=10.0,
        sun_azimuth_angle=0.0,  # -1.0在某些版本会报错，修正为0.0
        sun_altitude_angle=45.0,
        fog_density=2.0,
        fog_distance=0.75,
        fog_falloff=0.1,
        wetness=0.0,
        scattering_intensity=1.0,
        mie_scattering_scale=0.03,
        rayleigh_scattering_scale=0.0331,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 车辆状态记录字典
    vehicles_state = {
        'ego': {'actor': None, 'traj': EGO_TRAJ, 'idx': 0, 'active': False, 'pid_lon': PIDLongitudinalController(dt=dt),
                'pid_lat': PIDLateralController(dt=dt), 'init_speed': 60.0},
        'v2': {'actor': None, 'traj': V2_TRAJ, 'idx': 0, 'active': False, 'pid_lon': PIDLongitudinalController(dt=dt),
               'pid_lat': PIDLateralController(dt=dt), 'init_speed': 60.0},
        'v3': {'actor': None, 'traj': V3_TRAJ, 'idx': 0, 'active': False, 'pid_lon': PIDLongitudinalController(dt=dt),
               'pid_lat': PIDLateralController(dt=dt), 'init_speed': 70.0},
        'v4': {'actor': None, 'traj': V4_TRAJ, 'idx': 0, 'active': False, 'pid_lon': PIDLongitudinalController(dt=dt),
               'pid_lat': PIDLateralController(dt=dt), 'init_speed': 75.0}
    }

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # ================= Actor 1：Ego (lincoln.mkz_2020) =================
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2020')
        bp_ego.set_attribute('role_name', 'ego')
        bp_ego.set_attribute('color', '255,0,0')
        e_x, e_y, e_yaw = EGO_TRAJ[0]
        e_loc = carla.Location(x=e_x, y=e_y, z=0.5)
        e_loc.z = carla_map.get_waypoint(e_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(e_loc, carla.Rotation(yaw=e_yaw)))
        if ego:
            actor_list.append(ego)
            vehicles_state['ego']['actor'] = ego
            vehicles_state['ego']['active'] = True
            print("生成 Ego (MKZ 2020) 成功。")

        # ================= Actor 2：V2 (lincoln.mkz_2017) =================
        bp_v2 = bp_lib.find('vehicle.lincoln.mkz_2017')
        bp_v2.set_attribute('color', '0,0,255')
        v2_x, v2_y, v2_yaw = V2_TRAJ[0]
        # 【修改点4】防止V2与Ego出生在同一点导致物理引擎爆炸，将V2沿着偏航角向后偏移 10 米
        v2_x_offset = v2_x - 10.0 * math.cos(math.radians(v2_yaw))
        v2_y_offset = v2_y - 10.0 * math.sin(math.radians(v2_yaw))
        v2_loc = carla.Location(x=v2_x_offset, y=v2_y_offset, z=0.5)
        v2_loc.z = carla_map.get_waypoint(v2_loc).transform.location.z + 0.5
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=v2_yaw)))
        if v2:
            actor_list.append(v2)
            vehicles_state['v2']['actor'] = v2
            vehicles_state['v2']['active'] = True
            print("生成 V2跟随车 (MKZ 2017) 成功，已向后偏移10米防碰。")

        # ================= Actor 3：V3 (vespa.zx125 摩托) =================
        bp_v3 = bp_lib.find('vehicle.vespa.zx125')
        v3_x, v3_y, v3_yaw = V3_TRAJ[0]
        v3_loc = carla.Location(x=v3_x, y=v3_y, z=0.5)
        v3_loc.z = carla_map.get_waypoint(v3_loc).transform.location.z + 0.5
        v3 = world.try_spawn_actor(bp_v3, carla.Transform(v3_loc, carla.Rotation(yaw=v3_yaw)))
        if v3:
            actor_list.append(v3)
            vehicles_state['v3']['actor'] = v3
            vehicles_state['v3']['active'] = True
            print("生成 V3摩托 (vespa.zx125) 成功。")

        # ================= Actor 4：V4 (yamaha.yzf 摩托) =================
        bp_v4 = bp_lib.find('vehicle.yamaha.yzf')
        v4_x, v4_y, v4_yaw = V4_TRAJ[0]
        v4_loc = carla.Location(x=v4_x, y=v4_y, z=0.5)
        v4_loc.z = carla_map.get_waypoint(v4_loc).transform.location.z + 0.5
        v4 = world.try_spawn_actor(bp_v4, carla.Transform(v4_loc, carla.Rotation(yaw=v4_yaw)))
        if v4:
            actor_list.append(v4)
            vehicles_state['v4']['actor'] = v4
            vehicles_state['v4']['active'] = True
            print("生成 V4摩托 (yamaha.yzf) 成功。")

        # 【修改点3】物理引擎预热贴地
        for _ in range(10):
            world.tick()

        # 【修改点3】为所有存活的车辆赋予初始速度向量，防止原地起步或者速度突变飞天
        for key, state in vehicles_state.items():
            v = state['actor']
            if state['active'] and v.is_alive:
                speed_ms = state['init_speed'] / 3.6
                yaw_rad = math.radians(v.get_transform().rotation.yaw)
                v.set_target_velocity(carla.Vector3D(
                    speed_ms * math.cos(yaw_rad),
                    speed_ms * math.sin(yaw_rad),
                    0.0
                ))
        print("\n已为所有车辆赋予对应的初始速度，仿真正式开始！")

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        while True:
            start_time = time.time()
            world.tick()

            active_count = 0

            # 遍历控制所有车辆
            for key, state in vehicles_state.items():
                v_actor = state['actor']
                if state['active'] and v_actor.is_alive:
                    active_count += 1

                    if check_and_handle_out_of_bounds(v_actor, carla_map):
                        state['active'] = False
                        continue

                    traj = state['traj']
                    idx = state['idx']

                    if idx < len(traj):
                        tx, ty, tyaw = traj[idx]
                        target_loc = carla.Location(x=tx, y=ty, z=v_actor.get_location().z)

                        # 到达目标点附近，切换到下一个轨迹点
                        if v_actor.get_location().distance(target_loc) < 3.5 and idx < len(traj) - 1:
                            state['idx'] += 1

                        # 【修改点5】Ego 车特定路段减速逻辑
                        target_speed = state['init_speed']
                        if key == 'ego':
                            current_x = v_actor.get_location().x
                            # 到达位置x=97.247减速到20km/h，到达x=145.141恢复60km/h
                            if 97.247 <= current_x < 145.141:
                                target_speed = 20.0
                            else:
                                target_speed = 60.0

                        # 执行PID
                        apply_pid_control(v_actor, state['pid_lon'], state['pid_lat'], target_speed, target_loc)
                    else:
                        # 到达终点，踩刹车
                        v_actor.apply_control(carla.VehicleControl(brake=1.0))
                        state['active'] = False
                        print(f"\n[{key}] 已到达轨迹终点。")

            if active_count == 0:
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
            if actor is not None and actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕。")


if __name__ == '__main__':
    main()
