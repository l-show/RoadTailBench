import carla
import time
import math
import numpy as np


# ================= 1. 工具函数与控制器 =================

def calculate_yaw(p1, p2):
    """根据轨迹的前两个点计算初始航向角 (角度制)"""
    radians = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    return math.degrees(radians)


def filter_trajectory(raw_trajectory):
    """去除轨迹中的重复坐标点"""
    if not raw_trajectory:
        return []
    filtered = [(raw_trajectory[0][0], raw_trajectory[0][1])]
    for i in range(1, len(raw_trajectory)):
        dist = math.sqrt((raw_trajectory[i][0] - raw_trajectory[i - 1][0]) ** 2 +
                         (raw_trajectory[i][1] - raw_trajectory[i - 1][1]) ** 2)
        if dist > 0.1:
            filtered.append((raw_trajectory[i][0], raw_trajectory[i][1]))
    return filtered


def find_point_index(traj, target_xy, tol=0.05):
    """在轨迹中找到目标点索引；若未精确命中，则返回最近点索引。"""
    best_idx = 0
    best_dist = float("inf")
    tx, ty = target_xy
    for i, (x, y) in enumerate(traj):
        d = math.hypot(x - tx, y - ty)
        if d < best_dist:
            best_dist = d
            best_idx = i
        if d <= tol:
            return i
    return best_idx


class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 0.8)


class PIDLateralController:
    def __init__(self, K_P=1.2, K_I=0.02, K_D=0.1, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform, reverse_mode=False):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)
        if reverse_mode:
            # 倒车时用“车尾朝向”去对准目标点
            v_yaw += math.pi

        target_vector = np.array([waypoint_loc.x - v_loc.x, waypoint_loc.y - v_loc.y])
        if np.linalg.norm(target_vector) < 0.1:
            return 0.0

        target_yaw = math.atan2(target_vector[1], target_vector[0])
        error = target_yaw - v_yaw
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi

        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.7, 0.7)


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed_kmh, target_loc, reverse_mode=False):
    target_speed_ms = target_speed_kmh / 3.6
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    current_speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_cmd = pid_lon.run_step(target_speed_ms, current_speed_ms)
    steer = pid_lat.run_step(target_loc, tf, reverse_mode=reverse_mode)

    control = carla.VehicleControl(steer=steer, reverse=reverse_mode)
    if throttle_cmd >= 0:
        control.throttle, control.brake = throttle_cmd, 0.0
    else:
        control.throttle, control.brake = 0.0, abs(throttle_cmd)

    vehicle.apply_control(control)


# ================= 2. 数据准备 =================

RAW_TESLA_TRAJECTORY = [(84.47, -33.59), (84.47, -33.59), (84.47, -33.59), (82.819, -32.715),
                        (82.323, -32.452), (79.701, -31.439), (77.359, -30.762), (74.844, -30.01),
                        (74.662, -29.966), (70.841, -29.016), (68.127, -28.277), (65.403, -27.577),
                        (60.514, -26.266), (57.455, -25.372), (54.244, -24.328), (50.676, -23.174),
                        (46.926, -21.975), (43.769, -20.784), (40.924, -19.834), (40.924, -19.834),
                        (40.579, -19.689), (37.992, -18.6), (34.744, -17.154), (32.163, -16.04),
                        (29.76, -14.986), (27.065, -13.67), (24.599, -12.31), (22.149, -10.579),
                        (19.39, -8.639), (17.272, -6.789), (16.859, -6.408), (14.703, -4.061),
                        (12.191, -1.279), (11.788, -0.647), (10.652, 1.293), (9.515, 3.864),
                        (8.872, 6.601), (8.56, 9.395), (8.261, 12.755), (7.698, 16.651),
                        (7.428, 22.077), (7.466, 27.136), (7.923, 32.552), (8.476, 38.337),
                        (8.833, 42.635), (8.975, 49.195), (8.995, 54.82), (8.896, 60.256),
                        (8.726, 66.815), (8.619, 70.939), (8.389, 79.747), (8.215, 85.744),
                        (7.928, 91.924), (7.221, 102.588), (5.955, 114.138), (5.241, 125.739),
                        (4.719, 134.346), (4.007, 145.761), (3.407, 155.304)]

RAW_VAN_TRAJECTORY = [(11.351, 50.671), (11.087, 48.66), (10.358, 47.394), (9.275, 45.164), (8.163, 43.115),
                      (8.033, 42.808), (7.29, 40.953)]

VAN_STOP_POINT = (7.29, 40.953)
VAN_STOP_DURATION = 2.0


# ================= 3. 主程序 =================

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map, bp_lib = world.get_map(), world.get_blueprint_library()

    # 1. 雾天天气配置
    weather = carla.WeatherParameters(
        cloudiness=35.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=0.0, sun_azimuth_angle=90.0, sun_altitude_angle=10.0,
        fog_density=80.0, fog_distance=0.0, fog_falloff=0.0, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)

    dt = 0.05
    tesla_traj = filter_trajectory(RAW_TESLA_TRAJECTORY)
    van_traj = filter_trajectory(RAW_VAN_TRAJECTORY)
    actor_list = []

    # 货车的“停车点”和“倒车回原位路径”
    van_stop_idx = find_point_index(van_traj, VAN_STOP_POINT)
    van_reverse_traj = list(reversed(van_traj[:van_stop_idx + 1]))
    van_origin = van_traj[0]

    try:
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds = True, dt
        world.apply_settings(settings)

        # 2. 生成主车 Tesla
        bp_tesla = bp_lib.find('vehicle.tesla.model3')
        t_yaw = calculate_yaw(tesla_traj[0], tesla_traj[1])
        tesla = world.try_spawn_actor(
            bp_tesla,
            carla.Transform(
                carla.Location(x=tesla_traj[0][0], y=tesla_traj[0][1], z=6.2),
                carla.Rotation(yaw=t_yaw)
            )
        )

        if not tesla:
            print("Tesla 生成失败，请检查坐标冲突。")
            return

        actor_list.append(tesla)
        tesla.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Position | carla.VehicleLightState.Fog))

        # ================== 修改部分：提前生成货车 ==================
        bp_van = bp_lib.find('vehicle.mercedes.sprinter')
        bp_van.set_attribute('color', '255,255,255')
        v_yaw = calculate_yaw(van_traj[0], van_traj[1])
        van = world.try_spawn_actor(
            bp_van,
            carla.Transform(
                carla.Location(x=van_traj[0][0], y=van_traj[0][1], z=6.2),
                carla.Rotation(yaw=v_yaw)
            )
        )

        if not van:
            print("货车生成失败，请检查坐标冲突。")
            return

        actor_list.append(van)
        van.set_light_state(carla.VehicleLightState.NONE)
        print(">>> 车辆已全部生成完毕。")
        # ==========================================================

        # 3. TriggerBox 范围设置 (米)
        T_X = [10.11 - 1.5, 10.11 + 1.5]
        T_Y = [2.56 - 1.5, 2.56 + 1.5]

        # 4. 物理预热与 Tesla 速度注入 (由于货车已提前生成，预热也会让货车稳定落地)
        for _ in range(10):
            world.tick()
        init_v = 40.0 / 3.6
        tesla.set_target_velocity(
            carla.Vector3D(init_v * math.cos(math.radians(t_yaw)), init_v * math.sin(math.radians(t_yaw)), 0)
        )

        # 初始化控制器
        pid_t = {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        pid_v = {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        idx_t, idx_v = 0, 0
        idx_v_reverse = 0

        # ================== 修改部分：状态机初始状态 ==================
        # 货车状态机：等待触发 -> 前进 -> 停车等待 -> 倒车 -> 完成
        van_state = 'wait_for_trigger'
        van_triggered = False
        van_wait_start_s = None
        # ==========================================================

        print("仿真开始，等待 Tesla 进入触发区域...")
        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            t_loc = tesla.get_location()

            # --- 修改部分：触发逻辑，主车进入 TriggerBox 时启动货车 ---
            if not van_triggered:
                if T_X[0] < t_loc.x < T_X[1] and T_Y[0] < t_loc.y < T_Y[1]:
                    van_triggered = True
                    van_state = 'forward'  # 切换货车状态为前进
                    print(">>> 触发成功：主车已靠近，白色货车开始运动。")
            # --------------------------------------------------------

            # --- Tesla 控制 (30km/h) ---
            if tesla and tesla.is_alive:
                target_t = carla.Location(x=tesla_traj[idx_t][0], y=tesla_traj[idx_t][1], z=t_loc.z)
                if t_loc.distance(target_t) < 3.0 and idx_t < len(tesla_traj) - 1:
                    idx_t += 1
                apply_pid_control(tesla, pid_t['lon'], pid_t['lat'], 30.0, target_t)

            # --- 货车控制状态机 ---
            if van and van.is_alive:
                v_loc = van.get_location()
                stop_loc = carla.Location(x=VAN_STOP_POINT[0], y=VAN_STOP_POINT[1], z=v_loc.z)
                origin_loc = carla.Location(x=van_origin[0], y=van_origin[1], z=v_loc.z)

                # 新增等待状态：原地踩满刹车
                if van_state == 'wait_for_trigger':
                    van.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

                elif van_state == 'forward':
                    target_idx = min(idx_v, van_stop_idx)
                    target_xy = van_traj[target_idx]
                    target_v = carla.Location(x=target_xy[0], y=target_xy[1], z=v_loc.z)

                    if target_idx < van_stop_idx and v_loc.distance(target_v) < 1.2:
                        idx_v += 1
                        target_idx = min(idx_v, van_stop_idx)
                        target_xy = van_traj[target_idx]
                        target_v = carla.Location(x=target_xy[0], y=target_xy[1], z=v_loc.z)

                    if v_loc.distance(stop_loc) < 0.8 or target_idx >= van_stop_idx:
                        van.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                        van_state = 'stop_wait'
                        van_wait_start_s = sim_time
                        print(">>> 货车到达 (7.29, 40.953)，开始停车 2 秒。")
                    else:
                        apply_pid_control(van, pid_v['lon'], pid_v['lat'], 15.0, target_v, reverse_mode=False)

                elif van_state == 'stop_wait':
                    van.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    if sim_time - van_wait_start_s >= VAN_STOP_DURATION:
                        van_state = 'reverse'
                        idx_v_reverse = 0
                        # 倒车前重置 PID，避免前进阶段积分项残留
                        pid_v = {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
                        print(">>> 停车结束，货车开始倒车返回原位置。")

                elif van_state == 'reverse':
                    target_xy = van_reverse_traj[idx_v_reverse]
                    target_v = carla.Location(x=target_xy[0], y=target_xy[1], z=v_loc.z)

                    if v_loc.distance(target_v) < 1.0 and idx_v_reverse < len(van_reverse_traj) - 1:
                        idx_v_reverse += 1
                        target_xy = van_reverse_traj[idx_v_reverse]
                        target_v = carla.Location(x=target_xy[0], y=target_xy[1], z=v_loc.z)

                    apply_pid_control(van, pid_v['lon'], pid_v['lat'], 8.0, target_v, reverse_mode=True)

                    if idx_v_reverse >= len(van_reverse_traj) - 1 and v_loc.distance(origin_loc) < 1.0:
                        van.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, reverse=False))
                        van_state = 'done'
                        print(">>> 货车已倒车回到原始位置。")

                elif van_state == 'done':
                    van.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, reverse=False))

            # 仿真结束条件
            if idx_t >= len(tesla_traj) - 1:
                break

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except Exception as e:
        print(f"运行出错: {e}")
    finally:
        print("清理环境...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        for a in actor_list:
            if a and a.is_alive:
                a.destroy()
        print("清理完毕。")


if __name__ == '__main__':
    main()