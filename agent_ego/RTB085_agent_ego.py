import carla
import time
import math
import os
import numpy as np

# ==========================================
# 1. 基础控制算法 (PID) - 保留自参考代码
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -0.8, 0.6)

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
# 2. 轨迹数据
# ==========================================
# Impala 的轨迹 (Location_x, Location_y, Rotation_yaw)
IMPALA_TRAJECTORY = [
    (19.348, -103.288, 142.757), (19.348, -103.288, 142.757), (17.704, -101.992, 141.747),
    (16.64, -101.153, 138.055), (16.64, -101.153, 138.055), (14.61, -99.332, 136.743),
    (11.877, -96.596, 133.89), (9.323, -93.85, 132.46), (6.902, -90.829, 125.454),
    (4.78, -87.582, 121.113), (2.899, -84.342, 118.833), (1.298, -80.892, 110.07),
    (0.188, -77.188, 105.595), (-0.752, -73.436, 101.026), (-1.336, -69.735, 96.614),
    (-1.519, -65.994, 88.647), (-1.101, -62.214, 76.923), (-0.058, -58.616, 68.701),
    (0.011, -58.442, 59.012), (0.011, -58.442, 58.872), (1.991, -55.334, 58.649),
    (3.786, -52.046, 63.399), (5.477, -48.629, 63.688), (7.194, -45.155, 63.688),
    (8.923, -41.706, 62.697), (9.526, -40.54, 62.697), (11.139, -37.156, 66.394),
    (11.864, -35.495, 66.394), (13.355, -32.054, 66.82), (14.916, -28.542, 65.029),
    (16.665, -25.158, 61.059), (18.59, -21.881, 57.848), (20.72, -18.738, 54.422),
    (22.903, -15.697, 53.999), (25.163, -12.552, 54.498), (27.361, -9.44, 54.994),
    (29.576, -6.266, 55.313), (31.824, -3.12, 53.099), (34.143, -0.187, 49.777),
    (36.782, 2.644, 46.232), (39.427, 5.387, 45.877), (42.065, 8.048, 42.621),
    (45.123, 10.415, 34.322), (48.33, 12.582, 34.031), (51.564, 14.707, 31.864),
    (54.805, 16.581, 28.284), (58.225, 18.39, 27.287), (61.627, 20.098, 25.045),
    (65.225, 21.521, 19.266), (68.892, 22.755, 17.32), (72.595, 23.88, 16.192),
    (76.215, 24.843, 12.518), (79.968, 25.503, 7.766), (83.812, 25.979, 5.787),
    (87.67, 26.305, 4.214), (91.539, 26.443, 1.183), (94.787, 26.487, 0.688),
    (94.787, 26.487, 0.688), (94.787, 26.487, 0.688), (94.787, 26.487, 0.688),
    (94.787, 26.487, 0.688), (94.787, 26.487, 0.688)
]

AUDI_EGO_START_XY = (77.819, -31.250)
AUDI_EGO_END_XY = (13.614, -94.236)
AUDI_EGO_END_Z = 7.281
AUDI_EGO_END_RADIUS_M = 5.0

# ==========================================
# 3. 辅助函数：车辆出界检测
# ==========================================
def is_near_xy(actor, target_xy, threshold=AUDI_EGO_END_RADIUS_M):
    loc = actor.get_location()
    dx = loc.x - target_xy[0]
    dy = loc.y - target_xy[1]
    return math.sqrt(dx * dx + dy * dy) <= threshold

def check_and_handle_out_of_bounds(vehicle, carla_map):
    loc = vehicle.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False)

    is_out = False
    if wp_exact is None:
        is_out = True
    elif wp_nearest and wp_nearest.transform.location.distance(loc) > 4.0:
        is_out = True

    if is_out:
        vehicle.destroy()
        return True
    return False

def end_scene_now(client, world, tm, actor_list):
    try:
        commands = [
            carla.command.DestroyActor(actor.id)
            for actor in actor_list
            if actor and actor.is_alive
        ]
        if commands:
            client.apply_batch_sync(commands, True)
    except Exception:
        for actor in actor_list:
            try:
                if actor and actor.is_alive:
                    actor.destroy()
            except Exception:
                pass

    try:
        if tm:
            tm.set_synchronous_mode(False)
    except Exception:
        pass
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    except Exception:
        pass

    os._exit(0)

# ==========================================
# 4. 主程序 (Main Loop)
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    tm = client.get_trafficmanager(8000)

    # ---------------- 严格依据您的截图设置精确天气 ----------------
    weather = carla.WeatherParameters(
        cloudiness=50.0, precipitation=0.0, precipitation_deposits=25.0,
        wind_intensity=100.0, sun_azimuth_angle=0.0, sun_altitude_angle=32.0,
        fog_density=30.0, fog_distance=0.0, fog_falloff=0.0, wetness=25.0,
        scattering_intensity=10.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.05
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    rocks_spawned = False
    impala_active = False

    try:
        # 开启同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        pid_impala = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ================= Actor 1：Impala（黄色，走指定轨迹，带特殊灯光） =================
        bp_impala = bp_lib.find('vehicle.chevrolet.impala')
        if bp_impala.has_attribute('color'):
            bp_impala.set_attribute('color', '255,255,0')  # 黄色

        impala_start_x, impala_start_y, impala_start_yaw = IMPALA_TRAJECTORY[0]
        impala_loc = carla.Location(x=impala_start_x, y=impala_start_y, z=0.5)
        impala_loc.z = carla_map.get_waypoint(impala_loc).transform.location.z + 0.5

        impala = world.try_spawn_actor(bp_impala, carla.Transform(impala_loc, carla.Rotation(yaw=impala_start_yaw)))
        if impala:
            actor_list.append(impala)
            impala_active = True
            print("生成 Chevrolet Impala 成功。")

        # ================= Actor 2：Audi TT（橙色，TM控制） =================
        bp_audi = bp_lib.find('vehicle.audi.tt')
        if bp_audi.has_attribute('color'):
            bp_audi.set_attribute('color', '255,165,0')  # 橙色

        if bp_audi.has_attribute('role_name'):
            pass

        audi_loc = carla.Location(x=AUDI_EGO_START_XY[0], y=AUDI_EGO_START_XY[1], z=0.5)
        audi_wp = carla_map.get_waypoint(audi_loc, project_to_road=True)  # 投影到路面找准 Z 和朝向
        audi_loc.z = audi_wp.transform.location.z + 0.5

        audi = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if audi:
            audi.set_autopilot(True, tm.get_port())

            # TM 设置：避障打开(默认就是开启的，这里忽略百分比设为0确保绝对避障)
            tm.ignore_vehicles_percentage(audi, 0.0)
            # 减速50% (当前限速的-50%)
            tm.vehicle_percentage_speed_difference(audi, 50.0)
            # 跟车距离 10 米
            tm.distance_to_leading_vehicle(audi, 10.0)

            # 设置灯光：雾灯 + 双闪 (LeftBlinker | RightBlinker)
            print("生成 Audi TT 成功，已移交 TM 控制。")

        # 让物理引擎预热贴地
        for _ in range(10): world.tick()

        print("\n仿真正式开始！等待5秒后生成长尾障碍物...")
        impala_traj_idx = 0

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            if audi and audi.is_alive:
                if is_near_xy(audi, AUDI_EGO_END_XY):
                    print("TM Ego (Audi TT) reached scenario endpoint; cleaning actors and ending simulation.")
                    end_scene_now(client, world, tm, actor_list)
            elif audi:
                audi = None

            # ==========================
            # Impala 车：PID 循迹与特殊灯光控制
            # ==========================
            if impala_active and impala.is_alive:
                if check_and_handle_out_of_bounds(impala, carla_map):
                    impala_active = False
                elif impala_traj_idx < len(IMPALA_TRAJECTORY):
                    tx, ty, tyaw = IMPALA_TRAJECTORY[impala_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=impala.get_location().z)

                    # 距离目标点小于 2 米时切换到下一个点
                    if impala.get_location().distance(target_loc) < 2.0 and impala_traj_idx < len(
                            IMPALA_TRAJECTORY) - 1:
                        impala_traj_idx += 1

                    # PID 控制速度 15 km/h
                    apply_pid_control(impala, pid_impala['lon'], pid_impala['lat'], 15.0, target_loc)

                    # 💡灯光控制逻辑：双闪常开 + 雾灯常开 + 远光灯（闪3下，休1下）
                    # 定义周期为 2.0 秒：
                    # 0.0~0.2s 亮, 0.2~0.4s 灭 (闪1)
                    # 0.4~0.6s 亮, 0.6~0.8s 灭 (闪2)
                    # 0.8~1.0s 亮, 1.0~2.0s 灭 (闪3 + 休息)
                    cycle_time = sim_time % 2.0
                    high_beam = False
                    if (0.0 <= cycle_time < 0.2) or (0.4 <= cycle_time < 0.6) or (0.8 <= cycle_time < 1.0):
                        high_beam = True

                    light_state = carla.VehicleLightState.Fog | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker
                    if high_beam:
                        light_state |= carla.VehicleLightState.HighBeam

                    impala.set_light_state(carla.VehicleLightState(light_state))

                else:
                    # 轨迹走完，刹车停住
                    impala.apply_control(carla.VehicleControl(brake=1.0))
                    impala_active = False
                    print("\nImpala 已到达轨迹终点，车辆刹停。")

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
        if tm: tm.set_synchronous_mode(False)
        print("清理完毕。")

if __name__ == '__main__':
    main()
