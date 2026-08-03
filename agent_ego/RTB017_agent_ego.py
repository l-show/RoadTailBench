import carla
import time
import math
import os
import random
import sys
import numpy as np

SCENE_DIR = os.path.dirname(os.path.abspath(__file__))
if SCENE_DIR not in sys.path:
    sys.path.append(SCENE_DIR)

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 0. Ego 车指定轨迹数据
# ==========================================
EGO_TRAJECTORY = [
    (50.421, -42.438, 148.869), (50.421, -42.438, 148.869), (50.421, -42.438, 148.869),
    (50.421, -42.438, 148.659), (50.421, -42.438, 149.008), (48.281, -41.153, 148.867),
    (43.431, -38.224, 148.867), (42.719, -37.791, 148.444), (38.743, -35.348, 148.444),
    (38.743, -35.348, 148.444), (38.672, -35.305, 148.444), (36.542, -33.996, 148.231),
    (34.365, -32.605, 146.805), (32.24, -31.21, 146.593), (30.095, -29.771, 145.959),
    (27.989, -28.348, 145.959), (25.917, -26.948, 145.959), (23.811, -25.525, 145.959),
    (21.666, -24.086, 146.592), (19.499, -22.68, 147.088), (17.33, -21.277, 147.229),
    (15.227, -19.923, 147.229), (13.122, -18.575, 147.511), (10.977, -17.212, 147.722),
    (8.789, -15.838, 147.932), (6.6, -14.466, 147.932), (4.475, -13.135, 147.932),
    (2.357, -11.808, 147.932), (0.191, -10.562, 153.898), (-2.111, -9.591, 160.044),
    (-4.588, -8.868, 167.405), (-7.053, -8.464, 173.294), (-9.54, -8.214, 174.929),
    (-12.035, -8.062, 178.318), (-14.534, -8.062, -178.382), (-17.115, -8.192, -175.525),
    (-19.639, -8.485, -171.123), (-22.094, -8.951, -167.012), (-24.553, -9.591, -164.003),
    (-27.032, -10.317, -163.645), (-29.509, -11.049, -163.224), (-31.9, -11.779, -162.944),
    (-34.28, -12.52, -162.522), (-36.653, -13.267, -162.522), (-39.025, -14.018, -162.171),
    (-41.438, -14.812, -161.464), (-43.887, -15.634, -161.464), (-46.263, -16.413, -162.236),
    (-48.732, -17.174, -163.302), (-51.21, -17.903, -163.729), (-53.65, -18.615, -163.729),
    (-56.13, -19.339, -163.729), (-58.609, -20.062, -163.729), (-61.049, -20.774, -163.729),
    (-63.53, -21.499, -163.729), (-66.011, -22.223, -163.729), (-68.492, -22.947, -163.729),
    (-70.888, -23.658, -163.234), (-73.362, -24.403, -163.234), (-75.73, -25.117, -163.234),
    (-78.213, -25.825, -167.32), (-80.685, -26.188, -174.44), (-83.256, -26.439, -174.44),
    (-85.827, -26.689, -174.44), (-88.315, -26.931, -174.44), (-90.88, -27.239, -171.273),
    (-93.344, -27.658, -169.65), (-95.838, -28.147, -168.443), (-98.322, -28.691, -166.592),
    (-100.783, -29.331, -164.885), (-103.232, -30.012, -163.968), (-105.706, -30.756, -162.906),
    (-108.092, -31.502, -162.554), (-110.551, -32.295, -162.055), (-113.009, -33.091, -162.055),
    (-115.387, -33.861, -162.055), (-117.765, -34.631, -162.055), (-120.183, -35.414, -162.055),
    (-122.562, -36.184, -162.125), (-124.981, -36.964, -162.125), (-127.401, -37.744, -162.125),
    (-127.639, -37.821, -162.125), (-127.639, -37.821, -162.125)
]

# ==========================================
# 1. 基础控制算法 (PID)
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
# 2. 辅助函数：根据坐标生成车辆
# ==========================================
def spawn_vehicle_at_xy(world, blueprint_name, x, y, role_name="background", color=None, force_yaw=None):
    vehicle = RTB.spawn_vehicle(
        world,
        blueprint_name,
        x=x,
        y=y,
        yaw=force_yaw,
        color=color,
        role_name=role_name,
    )
    if vehicle:
        print(f"RoadTailBench spawn vehicle [{blueprint_name}] role={role_name}")
    return vehicle

def check_ego_out_of_road_strict(vehicle, carla_map, auto_destroy=True):
    if not vehicle or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()
    waypoint = carla_map.get_waypoint(
        loc,
        project_to_road=False,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is not None:
        return False

    print(f"[RTB017 guard] ego left Driving lane at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f}); destroying.")
    if auto_destroy:
        try:
            vehicle.destroy()
        except Exception:
            pass
    return True

    if not waypoint:
        print(f"警告: 无法在坐标 ({x}, {y}) 找到有效车道!")
        return None

    # Ego车需要强制使用轨迹首点的yaw，防止生成时朝向不对导致PID画龙
    spawn_yaw = force_yaw if force_yaw is not None else waypoint.transform.rotation.yaw

    spawn_transform = carla.Transform(
        carla.Location(x=waypoint.transform.location.x, y=waypoint.transform.location.y,
                       z=waypoint.transform.location.z + 0.5),
        carla.Rotation(yaw=spawn_yaw)
    )

    vehicle = world.try_spawn_actor(bp, spawn_transform)
    if vehicle:
        print(f"成功生成车辆[{blueprint_name}] 于方向 {role_name.split('_')[-1]}")
    return vehicle

# ==========================================
# 3. 主程序 (Main Loop)
# ==========================================



def _rtb_agent_is_alive(actor):
    return bool(actor is not None and getattr(actor, "is_alive", False))


def _rtb_agent_find_ego(world, role_names=("ego", "hero"), type_id=None, start_xy=None, radius_m=12.0):
    try:
        actors = list(world.get_actors().filter("vehicle.*"))
    except Exception:
        return None
    for actor in actors:
        try:
            if actor.attributes.get("role_name", "") in role_names:
                return actor
        except Exception:
            pass
    candidates = actors
    if type_id:
        candidates = [actor for actor in candidates if getattr(actor, "type_id", "") == type_id]
    if start_xy and candidates:
        sx, sy = start_xy
        def dist(actor):
            loc = actor.get_location()
            return ((loc.x - sx) ** 2 + (loc.y - sy) ** 2) ** 0.5
        candidates = sorted(candidates, key=dist)
        try:
            if dist(candidates[0]) <= radius_m:
                return candidates[0]
        except Exception:
            return None
    return candidates[0] if len(candidates) == 1 else None

_RTB_AGENT_EGO_TYPE_ID = 'vehicle.audi.tt'
_RTB_AGENT_EGO_START_XY = (50.421, -42.438)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    tm = client.get_trafficmanager(8000)

    # ---------------- 夜间大雨天气 ----------------
    weather = carla.WeatherParameters(
        cloudiness=60.0, precipitation=75.0, precipitation_deposits=75.0,
        wind_intensity=30.0, sun_azimuth_angle=-1.0, sun_altitude_angle=-90.0,
        fog_density=60.0, fog_distance=0.75, fog_falloff=0.10, wetness=80.0,
        scattering_intensity=1.0, mie_scattering_scale=0.0300, rayleigh_scattering_scale=0.0331
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    base_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        tm.global_percentage_speed_difference(30.0)

        # 车辆配置（Ego坐标改用轨迹首个点）
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        vehicle_configs = [
            {"id": 1, "dir": 1, "bp": "vehicle.lincoln.mkz_2020", "x": -18.343, "y": -85.337},
            {"id": 2, "dir": 1, "bp": "vehicle.tesla.model3", "x": -23.123, "y": -99.384},
            {"id": 3, "dir": 1, "bp": "vehicle.ford.mustang", "x": -24.235, "y": -69.137},
            {"id": 4, "dir": 2, "bp": "vehicle.chevrolet.impala", "x": -65.094, "y": -14.136},
            {"id": 5, "dir": 2, "bp": "vehicle.volkswagen.t2", "x": -41.486, "y": -3.480},
            {"id": 6, "dir": 3, "bp": "vehicle.yamaha.yzf", "x": -1.707, "y": 37.633},
            {"id": 7, "dir": 4, "bp": "vehicle.harley-davidson.low_rider", "x": 44.438, "y": 14.529},]

        dir2_vehicles = []
        pool_rule_breakers = []  # 用于筛选闯红灯的候选池
        ego_vehicle = None

        # ================= 剧本逻辑分配 =================
        for conf in vehicle_configs:
            force_yaw = conf.get("yaw", None)
            role_name = conf.get("role_name", f"dir_{conf['dir']}")
            veh = spawn_vehicle_at_xy(world, conf["bp"], conf["x"], conf["y"], role_name=role_name,
                                      color=conf.get("color"), force_yaw=force_yaw)
            if veh is None: continue
            actor_list.append(veh)

            # 1. Ego 车 (PID控制)
            if conf["id"] == "ego":
                ego_vehicle = veh
                # Ego车要求大雨夜开启: 远光灯+行车灯(位置)+近光+雾灯
                ego_light = base_light_state | carla.VehicleLightState.HighBeam
                print(" -> Ego奥迪TT已配置：将使用 PID 沿着轨迹行驶，速度设为 40km/h。")
                continue  # Ego不交给TM接管

            # 交给TM控制的背景车
            veh.set_autopilot(True, tm.get_port())

            # 2. 方向1 (必须最先通行)
            if conf["dir"] == 1:
                tm.vehicle_percentage_speed_difference(veh, 0.0)
                tm.ignore_lights_percentage(veh, 100.0)  # 无视红灯最先走
                veh.set_light_state(carla.VehicleLightState(base_light_state))
                print(f" -> 方向1车辆已配置：作为第一优先级，将直接驶入路口。")

            # 3. 方向2 (必须停在路口等红灯并频闪远光)
            elif conf["dir"] == 2:
                dir2_vehicles.append(veh)
                tm.ignore_lights_percentage(veh, 0.0)  # 100%遵守红灯
                print(f" -> 方向2车辆已配置：将在路口停车并闪烁远光灯。")

            # 4. 其他方向 (方向3、4进入破坏者候选池)
            else:
                tm.vehicle_percentage_speed_difference(veh, random.uniform(-10.0, 10.0))
                pool_rule_breakers.append(veh)

        # ================= 随机规则破坏者控制 (1到3辆) =================
        if pool_rule_breakers:
            # 保证至少 1 辆，且不超过 3 辆（同时也受限于候选池车辆总数）
            num_breakers = random.randint(1, min(3, len(pool_rule_breakers)))
            breakers = random.sample(pool_rule_breakers, num_breakers)

            for veh in pool_rule_breakers:
                if veh in breakers:
                    tm.ignore_lights_percentage(veh, 100.0)  # 闯红灯
                    print(
                        f" -> 警告：方向 {veh.attributes.get('role_name').split('_')[-1]} 的车辆被随机选为破坏者，将无视信号灯！")
                else:
                    tm.ignore_lights_percentage(veh, 0.0)  # 乖乖等红灯

                # 灯光随机
                if random.choice([True, False]):
                    veh.set_light_state(carla.VehicleLightState(base_light_state | carla.VehicleLightState.HighBeam))
                else:
                    veh.set_light_state(carla.VehicleLightState(base_light_state))

        # ================= 强制接管红绿灯系统 =================
        print("\n正在接管并强制冻结所有红绿灯为【红灯】...")
        tls = world.get_actors().filter('traffic.traffic_light')
        for tl in tls:
            tl.set_state(carla.TrafficLightState.Red)
            tl.freeze(True)

        # 预热环境
        for _ in range(10): world.tick()
        print("\n--> 仿真正式开始！长尾冲突场景已触发...")

        last_flash_change_time = 0.0

        # 初始化 Ego 的 PID 控制器
        ego_traj_idx = 0
        ego_active = True

        # 进入同步主循环
        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            for actor in actor_list:
                pass

            # ----------------------------------------
            # 1. Ego 车辆 PID 轨迹跟踪逻辑
            # ----------------------------------------
            if ego_vehicle and ego_active and check_ego_out_of_road_strict(ego_vehicle, carla_map, auto_destroy=True):
                ego_active = False

            # ----------------------------------------
            # 2. 长尾干扰：方向2 对向来车随机远光灯频闪
            # ----------------------------------------
            if sim_time - last_flash_change_time > random.uniform(0.1, 0.4):
                last_flash_change_time = sim_time
                for v in dir2_vehicles:
                    if v.is_alive:
                        # 60% 概率亮刺眼的远光灯
                        if random.random() > 0.4:
                            v.set_light_state(
                                carla.VehicleLightState(base_light_state | carla.VehicleLightState.HighBeam))
                        else:
                            v.set_light_state(carla.VehicleLightState(base_light_state))

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
