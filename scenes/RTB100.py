import carla
import time
import math
import numpy as np


# ================= 1. 基础控制算法 (PID) =================
# （这部分与之前完全一致，保留不变）
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


def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    if wp_nearest is None: return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 6.0: return True
    return False


# ================= 2. 轨迹数据与处理 =================
def remove_duplicate_waypoints(trajectory, min_dist=0.5):
    if not trajectory: return []
    cleaned_traj = [trajectory[0]]
    for pt in trajectory[1:]:
        last_pt = cleaned_traj[-1]
        dist = math.sqrt((pt[0] - last_pt[0]) ** 2 + (pt[1] - last_pt[1]) ** 2)
        if dist >= min_dist:
            cleaned_traj.append(pt)
    return cleaned_traj


# (轨迹数据过长省略部分内容，直接使用你原有的 RAW_TRAJ 数据即可)
RAW_V4_VESPA_TRAJ = [(8.331, 108.862, -89.21), (8.379, 105.867, -89.07), (8.047, 90.88, -95.878),
                     (5.524, 75.599, -100.624), (4.1, 60.438, -92.803), (3.834, 45.444, -89.01),
                     (5.289, 30.555, -74.417), (13.882, 17.941, -41.299), (26.795, 10.562, -15.542),
                     (41.913, 9.772, 6.323), (56.841, 10.572, -0.75), (72.33, 10.763, 1.87), (87.568, 11.261, 1.87),
                     (102.816, 11.48, 0.017), (118.314, 11.566, 0.94), (133.563, 11.798, 0.229),
                     (149.068, 11.739, -0.271), (164.318, 11.678, 0.152), (179.568, 11.825, 0.649),
                     (194.818, 11.983, 0.297), (210.068, 12.121, 2.536), (225.385, 14.298, 17.186),
                     (238.925, 20.636, 33.719), (250.904, 30.408, 46.407), (260.125, 42.21, 56.344),
                     (268.337, 54.762, 57.336), (276.142, 67.615, 59.909), (283.856, 80.771, 58.327),
                     (287.269, 86.302, 58.327)]
RAW_V5_POLICE_TRAJ = [(-237.757, 0.647, 16.512), (-235.604, 1.285, 16.512), (-221.021, 4.692, 9.204),
                      (-205.794, 5.12, 0.748), (-190.3, 5.477, 1.676), (-175.305, 5.687, 0.025),
                      (-159.807, 5.669, 0.022), (-144.558, 5.629, -0.188), (-129.309, 5.586, -0.048),
                      (-114.059, 5.573, -0.048), (-98.559, 5.56, -0.048), (-83.059, 5.541, -0.33),
                      (-67.565, 5.521, -0.048), (-52.07, 5.546, 0.162), (-36.821, 5.589, 0.162),
                      (-21.818, 5.611, 0.022), (-6.318, 5.608, -0.048), (8.94, 5.566, -0.543), (24.439, 5.391, -0.613),
                      (39.939, 5.241, -0.543), (54.938, 5.214, 0.09), (70.188, 5.273, 0.23), (85.188, 5.353, 0.512),
                      (100.441, 5.489, 0.512), (115.452, 5.623, 0.512), (130.452, 5.73, 0.37), (145.702, 5.766, 0.087),
                      (160.702, 5.789, 0.087), (175.952, 5.812, 0.087), (191.452, 5.838, 0.157),
                      (206.943, 6.265, 3.946), (221.779, 8.382, 12.399), (236.265, 13.028, 25.027),
                      (249.383, 20.766, 37.258), (260.174, 31.505, 51.231), (268.98, 43.951, 57.166),
                      (276.961, 56.944, 59.452), (284.649, 70.115, 59.807), (292.192, 83.078, 59.807),
                      (293.575, 85.455, 59.807)]
RAW_V6_EGO_TRAJ = [(6.042, 167.375, -89.518), (6.041, 166.875, -90.011), (6.032, 156.555, -90.293),
                   (5.931, 146.392, -90.648), (5.844, 136.392, -90.298), (5.869, 126.059, -89.543),
                   (5.904, 115.725, -89.966), (5.843, 105.404, -90.883), (5.86, 95.245, -89.584),
                   (5.959, 84.913, -89.581), (5.947, 74.747, -91.005), (5.724, 64.751, -91.36), (5.557, 57.751, -91.36),
                   (5.531, 52.585, -90.299), (5.478, 42.585, -90.299), (5.47, 40.918, -90.299), (5.42, 31.414, -90.299),
                   (5.343, 21.414, -91.08), (5.215, 14.079, -88.764), (5.26, 12.08, -88.479), (9.77, 3.962, -22.348),
                   (19.617, 2.896, -5.249), (29.886, 2.109, -2.693), (39.887, 1.851, -1.035), (49.884, 1.903, 2.313),
                   (60.212, 2.185, 0.731), (70.543, 2.325, 0.658), (80.869, 2.292, -0.412), (91.023, 2.26, 0.08),
                   (101.185, 2.274, 0.08), (111.352, 2.288, 0.08), (121.685, 2.303, 0.08), (131.852, 2.317, 0.08),
                   (142.185, 2.313, -0.132), (152.185, 2.264, -0.342), (162.466, 2.292, 1.006), (172.461, 2.521, 1.726),
                   (182.456, 2.781, 1.016), (192.792, 2.965, 1.016), (202.957, 3.102, 0.526), (213.278, 3.522, 4.702),
                   (223.51, 4.925, 9.142), (233.218, 7.853, 21.107), (242.497, 12.37, 28.898),
                   (250.939, 18.012, 37.618), (258.325, 24.977, 46.716), (264.624, 32.944, 53.68),
                   (270.199, 41.24, 58.381), (275.568, 50.069, 59.246), (280.667, 58.673, 59.459),
                   (285.749, 67.287, 59.459)]

TRAJ_V4 = remove_duplicate_waypoints(RAW_V4_VESPA_TRAJ)
TRAJ_V5 = remove_duplicate_waypoints(RAW_V5_POLICE_TRAJ)
TRAJ_V6 = remove_duplicate_waypoints(RAW_V6_EGO_TRAJ)


def apply_initial_velocity(vehicle, speed_kmh, yaw_deg):
    speed_ms = speed_kmh / 3.6
    yaw_rad = math.radians(yaw_deg)
    vehicle.set_target_velocity(carla.Vector3D(speed_ms * math.cos(yaw_rad), speed_ms * math.sin(yaw_rad), 0.0))


# ================= 3. 主程序 =================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    print("\n" + "=" * 50)
    print("环境初始化开始...")
    print("=" * 50)

    weather = carla.WeatherParameters(
        cloudiness=60.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
        sun_azimuth_angle=-1.0, sun_altitude_angle=45.0, fog_density=3.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=0.0, scattering_intensity=1.0, mie_scattering_scale=0.0300,
        rayleigh_scattering_scale=0.0331, dust_storm=0.0
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    static_parked_vehicles = []  # 专门用于存放需要开门的三辆静止车
    active_flags = {'v4': False, 'v5': False, 'v6': False}
    traj_idx = {'v4': 0, 'v5': 0, 'v6': 0}

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_v4 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v5 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_v6 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        print("\n>>> 开始生成静态车辆 (修复坐标与物理状态) <<<")

        # 【核心修复1】将 UE 的厘米坐标除以 100，转换为 CARLA 的米单位
        # 【核心修复2】不要关闭物理，改为拉起手刹，否则车门动画无法播放

        static_configs = [
            # 车1: Sprinter. 原坐标 846,4375 -> 转换为 8.46, 43.75
            ('vehicle.mercedes.sprinter', carla.Transform(carla.Location(8.46440002, 43.7585693, 0.1), carla.Rotation(yaw=-90.037605))),
            # 车2: BMW. 原坐标 842,5195 -> 转换为 8.42, 51.95
            ('vehicle.bmw.grandtourer', carla.Transform(carla.Location(8.58647766, 58.0928222, 0.5), carla.Rotation(yaw=-90.267654))),
            # 车3: Audi E-Tron. 取代你手动放置的，放置在它们中间
            ('vehicle.audi.etron', carla.Transform(carla.Location(8.42328735, 51.9516552, 0.1), carla.Rotation(yaw=-89.650848)))
        ]

        for bp_name, transform in static_configs:
            bp = bp_lib.find(bp_name)
            # 设置车身颜色为惹眼的颜色（可选）
            if bp.has_attribute('color'):
                bp.set_attribute('color', '128,0,128')  # 对应你图片里的紫色

            veh = world.try_spawn_actor(bp, transform)
            if veh:
                print(f"  -> 成功生成 {bp_name}！(ID: {veh.id})")
                actor_list.append(veh)
                static_parked_vehicles.append(veh)

                # 关键：不要 set_simulate_physics(False)
                # 给车辆施加 1.0 的刹车并拉起手刹，让其稳稳停在原地
                veh.apply_control(carla.VehicleControl(hand_brake=True, steer=0.0, throttle=0.0, brake=1.0))
            else:
                print(f"  -> [失败] {bp_name} 生成失败！坐标碰撞。")

        print("\n>>> 开始生成动态测试车辆 <<<")
        bp_v4 = bp_lib.find('vehicle.vespa.zx125')
        v4_tf = carla.Transform(carla.Location(x=TRAJ_V4[0][0], y=TRAJ_V4[0][1], z=0.2),
                                carla.Rotation(yaw=TRAJ_V4[0][2]))
        v4_vespa = world.try_spawn_actor(bp_v4, v4_tf)
        if v4_vespa:
            actor_list.append(v4_vespa)
            active_flags['v4'] = True
            print("[成功] 生成 Vespa 摩托车。")

        bp_v5 = bp_lib.find('vehicle.dodge.charger_police')
        v5_tf = carla.Transform(carla.Location(x=TRAJ_V5[0][0], y=TRAJ_V5[0][1], z=0.2),
                                carla.Rotation(yaw=TRAJ_V5[0][2]))
        v5_police = world.try_spawn_actor(bp_v5, v5_tf)
        if v5_police:
            actor_list.append(v5_police)
            active_flags['v5'] = True
            v5_police.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.Position))
            print("[成功] 生成 警车。")

        bp_v6 = bp_lib.find('vehicle.lincoln.mkz_2020')
        v6_tf = carla.Transform(carla.Location(x=TRAJ_V6[0][0], y=TRAJ_V6[0][1], z=0.2),
                                carla.Rotation(yaw=TRAJ_V6[0][2]))
        v6_ego = world.try_spawn_actor(bp_v6, v6_tf)
        if v6_ego:
            actor_list.append(v6_ego)
            active_flags['v6'] = True
            print("[成功] 生成 Ego 测试车。")

        print("\n预热物理引擎中 (Tick 10次)...让车掉落到地面稳定")
        for _ in range(10):
            world.tick()

        # 赋予初始物理速度
        if active_flags['v4']: apply_initial_velocity(v4_vespa, 85.0, TRAJ_V4[0][2])
        if active_flags['v5']: apply_initial_velocity(v5_police, 90.0, TRAJ_V5[0][2])
        if active_flags['v6']: apply_initial_velocity(v6_ego, 60.0, TRAJ_V6[0][2])

        print("\n" + "=" * 50)
        print("仿真正式开始！(帧率锁定: 20FPS / 0.05s)")
        print("=" * 50)

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        doors_opened = False
        doors_closed = False
        last_print_time = -1

        while True:
            start_time = time.time()
            world.tick()

            sim_time = world.get_snapshot().timestamp.elapsed_seconds - start_sim_time

            if int(sim_time) > last_print_time:
                print(f"--- 仿真进行中: {sim_time:.1f} 秒 ---")
                last_print_time = int(sim_time)

            # =============== 定时事件：第8秒开门 ===============
            if sim_time >= 7.0 and not doors_opened:
                print(f"\n[触发事件] {sim_time:.1f}秒 - 模拟长尾场景：前车突然打开左前门（靠近主路）...")
                for veh in static_parked_vehicles:
                    if veh is not None and veh.is_alive:
                        try:
                            # 修复：长尾场景“鬼探头”或“开门杀”，最好只开靠路的门（前排左侧车门）
                            veh.open_door(carla.VehicleDoor.FL)
                            print(f"  -> {veh.type_id} 左前侧车门已开启！")
                        except Exception as e:
                            print(f"  -> {veh.type_id} 车门开启报错: {e}")
                doors_opened = True

            # =============== 定时事件：第10秒关门 ===============
            if sim_time >= 10.0 and doors_opened and not doors_closed:
                print(f"\n[触发事件] {sim_time:.1f}秒 - 尝试关闭所有静态车辆车门...")
                for veh in static_parked_vehicles:
                    if veh is not None and veh.is_alive:
                        try:
                            veh.close_door(carla.VehicleDoor.All)
                            print(f"  -> {veh.type_id} 车门已关闭！")
                        except Exception as e:
                            print(f"  -> {veh.type_id} 车门关闭报错: {e}")
                doors_closed = True

            # =============== 车辆循迹控制 (保持原逻辑不变) ===============
            if active_flags['v4'] and v4_vespa.is_alive:
                if check_and_handle_out_of_bounds(v4_vespa, carla_map):
                    active_flags['v4'] = False
                elif traj_idx['v4'] < len(TRAJ_V4):
                    tx, ty, tyaw = TRAJ_V4[traj_idx['v4']]
                    target_loc = carla.Location(x=tx, y=ty, z=v4_vespa.get_location().z)
                    if v4_vespa.get_location().distance(target_loc) < 3.5 and traj_idx['v4'] < len(TRAJ_V4) - 1:
                        traj_idx['v4'] += 1
                    apply_pid_control(v4_vespa, pid_v4['lon'], pid_v4['lat'], 85.0, target_loc)
                else:
                    v4_vespa.apply_control(carla.VehicleControl(brake=1.0))
                    active_flags['v4'] = False

            if active_flags['v5'] and v5_police.is_alive:
                if check_and_handle_out_of_bounds(v5_police, carla_map):
                    active_flags['v5'] = False
                elif traj_idx['v5'] < len(TRAJ_V5):
                    tx, ty, tyaw = TRAJ_V5[traj_idx['v5']]
                    target_loc = carla.Location(x=tx, y=ty, z=v5_police.get_location().z)
                    if v5_police.get_location().distance(target_loc) < 3.5 and traj_idx['v5'] < len(TRAJ_V5) - 1:
                        traj_idx['v5'] += 1
                    apply_pid_control(v5_police, pid_v5['lon'], pid_v5['lat'], 70.0, target_loc)
                else:
                    v5_police.apply_control(carla.VehicleControl(brake=1.0))
                    active_flags['v5'] = False

            if active_flags['v6'] and v6_ego.is_alive:
                if check_and_handle_out_of_bounds(v6_ego, carla_map):
                    active_flags['v6'] = False
                elif traj_idx['v6'] < len(TRAJ_V6):
                    tx, ty, tyaw = TRAJ_V6[traj_idx['v6']]
                    v6_curr_loc = v6_ego.get_location()
                    target_loc = carla.Location(x=tx, y=ty, z=v6_curr_loc.z)
                    if v6_curr_loc.distance(target_loc) < 3.5 and traj_idx['v6'] < len(TRAJ_V6) - 1: traj_idx['v6'] += 1

                    if v6_curr_loc.y > 66.0:
                        v6_target_speed = 60.0
                    elif 45.0 < v6_curr_loc.y <= 66.0:
                        v6_target_speed = 20.0
                    else:
                        v6_target_speed = 50.0

                    apply_pid_control(v6_ego, pid_v6['lon'], pid_v6['lat'], v6_target_speed, target_loc)
                else:
                    v6_ego.apply_control(carla.VehicleControl(brake=1.0))
                    active_flags['v6'] = False

            if not any(active_flags.values()) and sim_time > 4.5:
                print("\n所有动态车辆已完成测试且事件触发完毕，退出循环。")
                break

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[中断] 键盘强制终止。")
    finally:
        print("\n清理环境并恢复异步设置...")
        for actor in actor_list:
            if actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("清理完毕，脚本安全退出。")


if __name__ == '__main__':
    main()