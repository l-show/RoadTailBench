import carla
import time
import math
import numpy as np


# ==========================================
# 基础控制算法 (PID) - 保持不变
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
# 轨迹数据清洗与定义
# ==========================================
# 函数：去除轨迹中相邻的重复点，防止车辆/行人循迹时卡顿
def clean_trajectory(raw_trajectory, min_distance=0.5):
    if not raw_trajectory: return []
    cleaned = [raw_trajectory[0]]
    for pt in raw_trajectory[1:]:
        last_pt = cleaned[-1]
        dist = math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
        if dist >= min_distance:
            cleaned.append(pt)
    return cleaned


RAW_TRUCK_TRAJ = [
    (-90.317, -68.574, -0.07), (-83.815, -68.582, 0.279), (-77.623, -68.551, -0.001), (-71.078, -68.566, -0.141),
    (-64.647, -68.578, -0.07), (-58.41, -68.572, 0.168), (-52.117, -68.54, 0.404), (-45.74, -68.464, 0.71),
    (-39.437, -68.386, 0.71), (-33.136, -68.307, 0.71), (-26.773, -68.238, 0.482), (-20.46, -68.185, 0.482),
    (-14.233, -68.133, 0.482), (-7.887, -68.117, 0.051), (-1.47, -68.071, 0.83), (4.801, -67.98, 0.83),
    (11.266, -67.915, 0.26), (17.413, -67.894, 0.19), (23.877, -67.866, 0.26), (30.262, -67.837, 0.26),
    (36.49, -67.809, 0.26), (42.735, -67.792, 0.12), (49.304, -67.794, -0.09), (55.721, -67.804, -0.09),
    (62.129, -67.843, -0.484), (68.329, -67.875, -0.134), (74.797, -67.857, 0.326), (81.05, -67.822, 0.326),
    (87.411, -67.792, 0.01), (93.571, -67.791, 0.01), (99.982, -67.79, 0.01), (106.544, -67.789, 0.01),
    (112.903, -67.801, -0.246), (119.362, -67.829, -0.246), (125.777, -67.885, -1.395), (132.134, -68.057, -1.57),
    (138.562, -68.423, -5.887), (145.014, -69.405, -11.234), (151.036, -70.799, -14.654), (156.993, -72.727, -21.744),
    (162.785, -75.559, -32.073), (167.97, -79.349, -42.382), (172.529, -83.624, -44.652), (176.822, -88.441, -52.581),
    (180.126, -93.743, -61.401), (182.896, -99.459, -65.641), (184.658, -103.35, -65.641), (184.658, -103.35, -65.641),
    (184.658, -103.35, -65.641)
]

RAW_PED_TRAJ = [
    (14.099, -50.407, 178.492), (14.099, -50.407, 178.492), (14.099, -50.407, 178.492), (14.099, -50.407, 178.492),
    (14.099, -50.407, 178.492), (14.099, -50.407, -179.794), (14.099, -50.407, 179.68), (11.268, -50.388, 179.61),
    (7.55, -50.364, 179.89), (3.791, -50.4, -179.25), (-0.025, -50.431, -179.962), (-3.834, -50.388, 178.141),
    (-7.706, -50.233, 175.744), (-11.394, -49.59, 163.367), (-15.027, -48.301, 155.717), (-18.496, -46.641, 155.865),
    (-18.993, -46.435, 165.612), (-18.993, -46.435, 165.612), (-18.993, -46.435, 165.612)
]

RAW_EGO_TRAJ = [
    (5.345, 13.513, -88.914), (5.345, 13.513, -88.914), (5.345, 13.513, -88.914), (5.345, 13.513, -88.914),
    (5.345, 13.513, -89.053), (5.371, 10.744, -89.497), (5.381, 5.627, -90.505), (5.337, 0.676, -90.505),
    (5.291, -4.526, -90.505), (5.264, -9.641, -90.197), (5.247, -14.671, -90.197), (5.229, -19.721, -90.197),
    (5.212, -24.704, -90.197), (5.195, -29.799, -90.197), (5.177, -34.809, -90.197), (5.136, -39.808, -90.83),
    (5.062, -44.975, -90.83), (4.987, -50.141, -90.83), (5.022, -55.139, -85.923), (5.819, -60.06, -73.031),
    (8.081, -64.481, -50.817), (12.229, -67.094, -16.575), (17.333, -68.114, -6.717), (22.356, -68.129, 3.414),
    (27.336, -67.832, 3.414), (32.386, -67.633, 0.422), (37.592, -67.597, 0.282), (42.621, -67.599, -0.389),
    (47.76, -67.663, -0.988), (52.887, -67.757, -1.058), (57.99, -67.851, -1.058), (63.223, -67.953, -1.128),
    (68.325, -68.054, -1.128), (73.29, -68.152, -1.128), (78.496, -68.254, -1.058), (83.442, -68.329, -0.671),
    (88.496, -68.327, 0.699), (93.661, -68.264, 0.699), (98.674, -68.203, 0.699), (103.791, -68.141, 0.699),
    (108.757, -68.081, 0.629), (113.82, -68.026, 0.629), (116.765, -67.993, 0.629), (116.765, -67.993, 0.629),
    (116.765, -67.993, 0.629)
]

TRUCK_TRAJ = clean_trajectory(RAW_TRUCK_TRAJ)
PED_TRAJ = clean_trajectory(RAW_PED_TRAJ)
EGO_TRAJ = clean_trajectory(RAW_EGO_TRAJ)


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【修改点1】严格按照截图设置天气
    weather = carla.WeatherParameters(
        cloudiness=30.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=10.0, sun_azimuth_angle=310.0, sun_altitude_angle=29.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0, wetness=0.0,
        scattering_intensity=2.5, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.0500,
        dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    # 初始化状态标记
    ego_active, truck_active, ped_active = False, False, False
    ego_idx, truck_idx, ped_idx = 0, 0, 0

    # 行人跳跃防卡死逻辑所需变量
    ped_stuck_time = 0.0
    ped_last_loc = None

    try:
        # 同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_truck = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # 【生成 Actor 1: Ego (Impala) 】
        bp_ego = bp_lib.find('vehicle.chevrolet.impala')
        bp_ego.set_attribute('role_name', 'ego')
        ex, ey, eyaw = EGO_TRAJ[0]
        ego_loc = carla.Location(x=ex, y=ey, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=eyaw)))
        if ego:
            actor_list.append(ego)
            ego_active = True

        # 【生成 Actor 2: Truck (Sprinter) 】
        # 匹配给定的Sprinter动画蓝图，如果没有严格对应的底层模型则选用常见的厢式货车
        bp_truck = bp_lib.filter('vehicle.*sprinter*')
        if not bp_truck: bp_truck = bp_lib.filter('vehicle.*carlacola*')  # 后备模型
        bp_truck = bp_truck[0]
        tx, ty, tyaw = TRUCK_TRAJ[0]
        truck_loc = carla.Location(x=tx, y=ty, z=0.5)
        truck_loc.z = carla_map.get_waypoint(truck_loc).transform.location.z + 0.5
        truck = world.try_spawn_actor(bp_truck, carla.Transform(truck_loc, carla.Rotation(yaw=tyaw)))
        if truck:
            actor_list.append(truck)
            truck_active = True

        # 【生成 Actor 3: 行人 (Pedestrian) 】
        bp_ped = bp_lib.filter('walker.pedestrian.*')[0]
        if bp_ped.has_attribute('is_invincible'):
            bp_ped.set_attribute('is_invincible', 'true')
        px, py, pyaw = PED_TRAJ[0]
        ped_loc = carla.Location(x=px, y=py, z=0.5)
        ped_loc.z = carla_map.get_waypoint(ped_loc).transform.location.z + 1.0
        ped = world.try_spawn_actor(bp_ped, carla.Transform(ped_loc, carla.Rotation(yaw=pyaw)))
        if ped:
            actor_list.append(ped)
            ped_active = True

        print("所有参与者生成完毕，等待物理引擎稳定...")

        # 【修改点2】预热Tick，防止车辆初始悬空导致给速度时起飞
        for _ in range(20):
            world.tick()

        # 【修改点3】车辆落稳后，直接赋予物理瞬时初速度 (60km/h -> 16.66m/s)
        init_speed_ms = 60.0 / 3.6
        if ego_active:
            yaw_rad = math.radians(EGO_TRAJ[0][2])
            ego.set_target_velocity(
                carla.Vector3D(init_speed_ms * math.cos(yaw_rad), init_speed_ms * math.sin(yaw_rad), 0.0))
        if truck_active:
            yaw_rad = math.radians(TRUCK_TRAJ[0][2])
            truck.set_target_velocity(
                carla.Vector3D(init_speed_ms * math.cos(yaw_rad), init_speed_ms * math.sin(yaw_rad), 0.0))

        print("已赋予车辆 60 km/h 的初始速度！")

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds - start_sim_time

            # ----------------------------------------
            # 1. Ego 循迹 (目标 60km/h)
            # ----------------------------------------
            if ego_active and ego.is_alive:
                if ego_idx < len(EGO_TRAJ):
                    t_x, t_y, _ = EGO_TRAJ[ego_idx]
                    target_loc = carla.Location(x=t_x, y=t_y, z=ego.get_location().z)

                    if ego.get_location().distance(target_loc) < 3.5 and ego_idx < len(EGO_TRAJ) - 1:
                        ego_idx += 1

                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], 60.0, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    ego_active = False

            # ----------------------------------------
            # 2. Truck 循迹 (目标 90km/h)
            # ----------------------------------------
            if truck_active and truck.is_alive:
                if truck_idx < len(TRUCK_TRAJ):
                    t_x, t_y, _ = TRUCK_TRAJ[truck_idx]
                    target_loc = carla.Location(x=t_x, y=t_y, z=truck.get_location().z)

                    if truck.get_location().distance(target_loc) < 3.5 and truck_idx < len(TRUCK_TRAJ) - 1:
                        truck_idx += 1

                    apply_pid_control(truck, pid_truck['lon'], pid_truck['lat'], 90.0, target_loc)
                else:
                    truck.apply_control(carla.VehicleControl(brake=1.0))
                    truck_active = False

            # ----------------------------------------
            # 3. 行人逻辑 (等待1s -> 跑步 -> 越障防卡死)
            # ----------------------------------------
            if ped_active and ped.is_alive:
                if sim_time < 1.0:
                    # 原地站立 1s
                    control = carla.WalkerControl()
                    control.speed = 0.0
                    ped.apply_control(control)
                else:
                    if ped_idx < len(PED_TRAJ):
                        t_x, t_y, _ = PED_TRAJ[ped_idx]
                        curr_loc = ped.get_location()

                        # 行人方向向量计算
                        direction = carla.Vector3D(t_x - curr_loc.x, t_y - curr_loc.y, 0.0)
                        dist = math.hypot(direction.x, direction.y)

                        if dist < 1.0 and ped_idx < len(PED_TRAJ) - 1:
                            ped_idx += 1

                        # 发送跑步控制指令 (速度设为 4.0 m/s)
                        if dist > 0:
                            direction.x /= dist
                            direction.y /= dist

                        control = carla.WalkerControl()
                        control.direction = direction
                        control.speed = 4.0
                        ped.apply_control(control)

                        # 【修改点4】行人防卡死跳跃逻辑 (跨过 <= 0.5m 障碍)
                        if ped_last_loc is not None:
                            moved_dist = curr_loc.distance(ped_last_loc)
                            if moved_dist < 0.05:  # 如果被卡住了
                                ped_stuck_time += dt
                            else:
                                ped_stuck_time = 0.0

                            # 卡住超过 0.5 秒，则强制提升 Z 轴高度 (相当于小跳跨过障碍物)
                            if ped_stuck_time > 0.5:
                                jump_loc = curr_loc
                                jump_loc.z += 0.55  # 提升0.55米跨越0.5米障碍
                                # 稍微向前移动一点防止垂直落下再次卡住
                                jump_loc.x += direction.x * 0.5
                                jump_loc.y += direction.y * 0.5
                                ped.set_location(jump_loc)
                                ped_stuck_time = 0.0  # 重置卡住计时

                        ped_last_loc = curr_loc
                    else:
                        # 终点停止
                        control = carla.WalkerControl()
                        control.speed = 0.0
                        ped.apply_control(control)
                        ped_active = False

            # 判断结束条件
            if not ego_active and not truck_active and not ped_active:
                print("\n所有 Actor 已完成轨迹，测试结束。")
                break

            # 帧率同步
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("正在清理环境...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()
        print("清理完毕。")


if __name__ == '__main__':
    main()
