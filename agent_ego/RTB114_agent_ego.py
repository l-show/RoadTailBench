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

RAW_TESLA_TRAJECTORY = [
    (97.789, -34.909, 176.922), (97.340, -34.885, 176.922), (88.780, -34.170, 173.881),
    (80.657, -33.061, 171.190), (72.663, -31.490, 166.236), (65.771, -29.591, 164.315),
    (58.457, -27.359, 161.006), (50.582, -24.585, 159.536), (42.337, -21.372, 158.626),
    (34.124, -18.079, 157.716), (26.200, -14.618, 153.757), (19.390, -10.449, 143.713),
    (12.688, -4.757, 136.261), (8.436, 1.323, 112.444), (7.632, 3.584, 108.382),
    (6.272, 8.235, 101.746), (5.257, 15.460, 95.356), (4.854, 22.748, 92.527),
    (4.808, 25.547, 90.108), (4.808, 25.547, 90.108), (4.808, 25.547, 90.108),
    (4.871, 27.195, 87.820), (5.443, 34.168, 83.393), (6.230, 41.074, 84.163),
    (6.741, 47.903, 87.750), (6.957, 54.849, 89.360), (6.912, 61.899, 90.830),
    (6.782, 68.797, 91.390), (6.555, 75.793, 92.019), (6.265, 82.887, 92.649),
    (5.925, 89.828, 92.929), (5.440, 97.111, 94.259), (4.824, 104.636, 95.239),
    (4.093, 112.603, 95.239), (3.346, 120.769, 95.169), (2.589, 129.135, 95.169),
    (2.035, 135.260, 95.169), (2.035, 135.260, 95.169), (2.035, 135.260, 95.169),
    (2.035, 135.260, 95.169), (2.035, 135.260, 95.169), (2.035, 135.260, 95.169)
]

RAW_VAN_TRAJECTORY = [(11.351, 50.671), (11.087, 48.66), (10.358, 47.394), (9.275, 45.164), (8.163, 43.115),
                      (8.033, 42.808), (7.29, 40.953)]

VAN_STOP_POINT = (7.29, 40.953)
VAN_START_DELAY = 11.0
VAN_STOP_DURATION = 2.0

EGO_CRUISE_SPEED_KMH = 30.0
EGO_STOP_TRIGGER_X = 4.8
EGO_STOP_TRIGGER_TOLERANCE = 0.10
EGO_STOP_DURATION = 10.0

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
        bp_tesla.set_attribute('color', '0,80,255')
        t_yaw = calculate_yaw(tesla_traj[0], tesla_traj[1])
        tesla = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        if not tesla:
            print("Tesla 生成失败，请检查坐标冲突。")
            return

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

        # 4. 物理预热与 Tesla 速度注入 (由于货车已提前生成，预热也会让货车稳定落地)
        for _ in range(10):
            world.tick()
        init_v = EGO_CRUISE_SPEED_KMH / 3.6

        # 初始化控制器
        pid_t = {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        pid_v = {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        idx_t, idx_v = 0, 0
        idx_v_reverse = 0

        # ================== 修改部分：状态机初始状态 ==================
        # 货车状态机：等待触发 -> 前进 -> 停车等待 -> 倒车 -> 完成
        van_state = 'wait_for_trigger'
        van_wait_start_s = None
        ego_state = 'cruise'
        ego_stop_start_s = None
        scenario_start_s = None
        # ==========================================================

        print("仿真开始，Van 生成后等待 11 秒自动启动。")
        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds
            if scenario_start_s is None:
                scenario_start_s = sim_time

            t_loc = tesla.get_location()

            if van_state == 'wait_for_trigger' and sim_time - scenario_start_s >= VAN_START_DELAY:
                van_state = 'forward'
                print(">>> Van 等待 11 秒结束，开始运行。")

            # --- Tesla 控制 (30km/h) ---

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
