import carla
import time
import math
import numpy as np


# ==========================
# 基础控制算法 (PID) - 保留
# ==========================
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


# ==========================
# 轨迹数据定义
# ==========================
# 1. Ego (Lincoln MKZ) 轨迹
EGO_TRAJECTORY = [
    (-1.281, -90.671, 89.965), (-1.281, -89.297, 89.328), (-1.281, -89.297, 89.328),
    (-1.212, -86.298, 88.685), (-1.028, -79.175, 88.475), (-0.919, -74.801, 88.615),
    (-0.766, -67.173, 89.043), (-0.645, -59.673, 89.112), (-0.621, -52.174, 90.46),
    (-0.711, -44.549, 90.742), (-0.809, -37.048, 90.742), (-0.893, -29.546, 90.316),
    (-0.907, -21.796, 90.033), (-0.886, -14.295, 89.821), (-0.862, -6.794, 89.821),
    (-0.838, 0.83, 89.821), (-0.82, 8.58, 89.891), (-0.806, 16.08, 89.891),
    (-0.762, 23.83, 89.608), (-0.676, 31.329, 89.186), (-0.569, 39.079, 89.326),
    (-0.508, 46.833, 89.888), (-0.514, 54.584, 91.172), (-0.791, 62.203, 92.737),
    (-1.25, 69.813, 93.939), (-1.785, 77.42, 93.722), (-2.119, 85.177, 91.711),
    (-2.167, 92.925, 89.566), (-2.018, 100.551, 88.504), (-1.802, 108.298, 88.292),
    (-1.579, 115.795, 88.292), (-1.504, 118.293, 88.292), (-1.504, 118.293, 88.292)
]


# 2. V2 (Nissan Micra) 轨迹 (已剔除导致瞬移的错误坐标点)
V2_TRAJECTORY = [
    (62.137, 80.244, -156.74), (62.137, 80.244, -158.016), (60.632, 79.632, -157.806),
    (58.277, 78.671, -157.806), (55.818, 77.883, -163.759), (53.419, 77.181, -163.476),
    (50.981, 76.458, -163.476), (48.499, 75.741, -165.713), (45.994, 75.111, -166.869),
    (43.489, 74.691, -174.633), (40.911, 74.519, -176.52), (38.414, 74.389, -177.163),
    (35.834, 74.261, -177.163), (33.337, 74.148, -177.588), (30.756, 74.048, -178.376),
    (28.256, 73.98, -178.869), (25.715, 73.951, -179.926), (23.215, 73.964, 178.435),
    (20.635, 74.083, 175.521), (18.059, 74.272, 176.444), (15.563, 74.404, 177.802),
    (12.981, 74.438, -179.402), (10.481, 74.388, -178.622), (7.903, 74.249, -173.571),
    (5.419, 73.734, -159.711), (3.174, 72.473, -141.426), (1.543, 70.594, -121.973),
    (0.568, 68.205, -105.253), (0.151, 65.705, -90.887), (0.375, 63.134, -81.993),
    (0.646, 60.648, -85.727), (0.815, 58.07, -86.44), (0.982, 55.492, -86.157),
    (1.155, 52.998, -85.442), (1.389, 50.511, -83.741), (1.648, 47.946, -86.715),
    (1.771, 45.454, -87.94), (1.825, 42.914, -89.428), (1.835, 40.373, -89.85),
    (1.857, 37.79, -89.283), (1.884, 35.248, -89.423), (1.91, 32.666, -89.423),
    (1.939, 30.084, -89.143), (1.989, 27.501, -88.79), (2.034, 25.002, -89.637),
    (2.047, 22.46, -90.062), (2.045, 19.96, -90.062), (2.042, 17.46, -90.062),
    (2.039, 14.877, -90.062), (2.036, 12.335, -90.062), (2.034, 9.834, -90.062),
    (2.028, 7.292, -90.275), (2.016, 4.751, -90.275), (1.999, 2.168, -90.417),
    (1.98, -0.374, -90.417), (1.969, -1.957, -90.417), (1.966, -2.374, -90.417),
    (1.919, -8.728, -90.417), (1.937, -15.082, -89.497), (1.992, -21.332, -89.497),
    (2.047, -27.688, -89.497), (2.102, -33.939, -89.497), (2.154, -40.295, -89.637),
    (2.173, -46.544, -89.85), (2.118, -53.002, -90.977), (2.012, -59.457, -90.624),
    (1.98, -65.705, -90.062), (2, -72.165, -89.572), (2.096, -78.413, -88.869),
    (2.212, -84.868, -89.507), (2.169, -91.22, -91.717), (1.979, -97.571, -91.717),
    (1.841, -102.152, -91.717)
]


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


# ==========================
# 主程序
# ==========================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # 【修改点1】严格依据截图设置精确天气
    weather = carla.WeatherParameters(
        cloudiness=20.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=10.0, sun_azimuth_angle=260.0, sun_altitude_angle=5.0,
        fog_density=4.0, fog_distance=0.0, fog_falloff=0.0, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    ego_active = False
    v2_active = False

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v2 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：Ego车 (Lincoln MKZ, 红色) =================
        bp_ego = bp_lib.find('vehicle.lincoln.mkz_2020')
        bp_ego.set_attribute('role_name', 'ego')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,0,0')  # 红色
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=ego_start_yaw)))
        if ego:
            actor_list.append(ego)
            ego_active = True
            # 打开行车灯
            ego.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
            print("生成 Ego (Lincoln MKZ) 成功。")

        # ================= Actor 2：NPC车 (Nissan Micra) =================
        bp_v2 = bp_lib.find('vehicle.nissan.micra')
        v2_start_x, v2_start_y, v2_start_yaw = V2_TRAJECTORY[0]
        v2_loc = carla.Location(x=v2_start_x, y=v2_start_y, z=0.5)
        v2_loc.z = carla_map.get_waypoint(v2_loc).transform.location.z + 0.5
        v2 = world.try_spawn_actor(bp_v2, carla.Transform(v2_loc, carla.Rotation(yaw=v2_start_yaw)))
        if v2:
            actor_list.append(v2)
            v2_active = True
            # 打开行车灯
            v2.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
            print("生成 NPC (Nissan Micra) 成功。")

        # 【修改点2】让物理引擎预热贴地，防止车辆腾空状态下给予速度导致失控飞天
        for _ in range(10):
            world.tick()

        # 【修改点3】Ego车赋予初始速度 30km/h (不用从0加速)
        if ego_active:
            ego_init_speed_ms = 30.0 / 3.6
            yaw_rad = math.radians(ego_start_yaw)
            # 使用 set_target_velocity 赋予物理初速度向量
            ego.set_target_velocity(carla.Vector3D(
                ego_init_speed_ms * math.cos(yaw_rad),
                ego_init_speed_ms * math.sin(yaw_rad),
                0.0))
            print("已赋予 Ego 初始速度 30 km/h。")

        # 记录仿真正式开始的时间
        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        ego_traj_idx = 0
        v2_traj_idx = 0

        print("\n仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()
            current_snap_time = world.get_snapshot().timestamp.elapsed_seconds
            sim_time = current_snap_time - start_sim_time  # 从车辆落稳后计算的相对时间

            # ==========================
            # Ego 车：循迹与控制 (目标速度 70km/h)
            # ==========================
            if ego_active and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map):
                    ego_active = False
                elif ego_traj_idx < len(EGO_TRAJECTORY):
                    tx, ty, tyaw = EGO_TRAJECTORY[ego_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego.get_location().z)

                    if ego.get_location().distance(target_loc) < 3.5 and ego_traj_idx < len(EGO_TRAJECTORY) - 1:
                        ego_traj_idx += 1

                    # Ego 目标速度 70 km/h
                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], 70.0, target_loc)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    ego_active = False
                    print("\nEgo 已到达轨迹终点。")

            # ==========================
            # V2 车辆 (Nissan Micra)：复杂的速度逻辑控制
            # ==========================
            if v2_active and v2.is_alive:
                if check_and_handle_out_of_bounds(v2, carla_map):
                    v2_active = False
                elif v2_traj_idx < len(V2_TRAJECTORY):
                    tx, ty, tyaw = V2_TRAJECTORY[v2_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v2.get_location().z)

                    if v2.get_location().distance(target_loc) < 3.5 and v2_traj_idx < len(V2_TRAJECTORY) - 1:
                        v2_traj_idx += 1

                    # 【修改点4】V2的速度逻辑：前3s静止，随后目标达到40，y<=35时目标达60
                    if sim_time < 0.1:
                        v2.apply_control(carla.VehicleControl(brake=1.0))  # 强制静止 3 秒
                    else:
                        v2_current_loc = v2.get_location()
                        # 根据位置判定目标速度
                        if v2_current_loc.y <= 35.0:
                            v2_target_speed = 60.0
                        else:
                            v2_target_speed = 40.0  # 初始起步冲向40km/h，保证 x=10 附近具有40km/h左右的速度

                        apply_pid_control(v2, pid_v2['lon'], pid_v2['lat'], v2_target_speed, target_loc)
                else:
                    v2.apply_control(carla.VehicleControl(brake=1.0))
                    v2_active = False
                    print("\nV2 已到达轨迹终点。")

            # 如果两辆车都走完了，可以退出循环
            if not ego_active and not v2_active:
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
