import carla
import time
import math
import numpy as np


# ==========================================
# PID 控制器类 (保持独立，无Traffic Manager)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])

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
# 辅助函数
# ==========================================
def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    )

def get_lane_keeping_waypoint(carla_map, vehicle_loc, lookahead_dist=6.0):
    """动态利用CARLA地图提取车道中心点进行车道保持"""
    current_wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    next_wps = current_wp.next(lookahead_dist)
    if next_wps:
        loc = next_wps[0].transform.location
        return (loc.x, loc.y, loc.z)
    return (current_wp.transform.location.x, current_wp.transform.location.y, current_wp.transform.location.z)


def get_proper_spawn_transform(world, x, y):
    loc = carla.Location(x=x, y=y, z=0.0)
    waypoint = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    trans = waypoint.transform
    trans.location.z += 0.5
    return trans


def get_safe_backward_transform(start_wp, target_distance):
    """安全地向后搜寻生成点，防止超出路线起点导致的 IndexError"""
    curr_wp = start_wp
    traveled = 0.0
    step = 2.0
    while traveled < target_distance:
        prev_wps = curr_wp.previous(step)
        if not prev_wps:
            break  # 如果后面没路了（比如到了路口），就停在这个极限安全位置
        curr_wp = prev_wps[0]
        traveled += step
    trans = curr_wp.transform
    trans.location.z += 0.5
    return trans


def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp):
    """通用的PID控制器执行流程"""
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)

    vehicle.apply_control(control)

def get_lane_keeping_waypoint(carla_map, vehicle_loc, lookahead_dist=6.0):
    """动态利用CARLA地图提取车道中心点进行车道保持"""
    current_wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    next_wps = current_wp.next(lookahead_dist)
    if next_wps:
        loc = next_wps[0].transform.location
        return (loc.x, loc.y, loc.z)
    return (current_wp.transform.location.x, current_wp.transform.location.y, current_wp.transform.location.z)

def set_vehicle_lights(vehicle, left_blinker=False, right_blinker=False):
    """设置车辆灯光：强制开启近光灯和示宽灯，并按需叠加转向灯"""
    # 基础灯光：示宽灯(Position) + 近光灯(LowBeam)
    light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
    if left_blinker:
        light_state |= carla.VehicleLightState.LeftBlinker
    if right_blinker:
        light_state |= carla.VehicleLightState.RightBlinker

    vehicle.set_light_state(carla.VehicleLightState(light_state))

def get_proper_spawn_transform(world, x, y):
    loc = carla.Location(x=x, y=y, z=0.0)
    waypoint = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    trans = waypoint.transform
    trans.location.z += 0.5
    return trans
def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp):
    """通用的PID控制器执行流程"""
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)  # 当前速度 km/h

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp, tf)

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
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # 天气配置
    weather = carla.WeatherParameters(
        cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
        sun_azimuth_angle=-1.0, sun_altitude_angle=45.0, fog_density=2.0, fog_distance=0.75
    )
    world.set_weather(weather)
    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        # 定义四辆车的PID控制器
        pids = {
            'v1': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController(K_P=1.3)},  # 大车转向偏柔和
            'v2': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController(K_P=1.3)},
            'v3': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController(K_P=2.2)},  # 轿车变道可稍激进
            'v4': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()},
            'jeep': {'lon': PIDLongitudinalController(), 'lat': PIDLateralController()}
        }

        print("正在生成车辆...")

        # 1. 消防车 (大卡车)
        bp_v1 = bp_lib.find('vehicle.carlamotors.firetruck')
        trans_v1 = get_proper_spawn_transform(world, x=12.554, y=-102.382)
        v1 = world.try_spawn_actor(bp_v1, trans_v1)
        if v1: actor_list.append(v1); print("1. Firetruck 生成成功")

        # 2. 货车 (可乐卡车)
        bp_v2 = bp_lib.find('vehicle.carlamotors.carlacola')
        trans_v2 = get_proper_spawn_transform(world, x=16.602, y=-101.076)
        v2 = world.try_spawn_actor(bp_v2, trans_v2)
        if v2: actor_list.append(v2); print("2. Carlacola 生成成功")

        # 3. 超车轿车 (Audi TT) - 采用安全算法生成在 V2 后面约 35 米处
        bp_v3 = bp_lib.find('vehicle.audi.tt')
        wp_v2 = carla_map.get_waypoint(trans_v2.location)
        trans_v3 = get_safe_backward_transform(wp_v2, target_distance=35.0)
        v3 = world.try_spawn_actor(bp_v3, trans_v3)
        if v3: actor_list.append(v3); print("3. Audi TT (超车车) 生成成功")

        # 4. 匝道汇入轿跑 (Mercedes)
        bp_v4 = bp_lib.find('vehicle.mercedes.coupe_2020')
        trans_v4 = get_proper_spawn_transform(world, x=86.783, y=-10.298)
        v4 = world.try_spawn_actor(bp_v4, trans_v4)
        if v4: actor_list.append(v4); print("4. Mercedes Coupe 生成成功")

        # 5. 车辆5: vehicle.jeep.wrangler_rubicon (新增：橙色)
        bp_jeep = bp_lib.find('vehicle.jeep.wrangler_rubicon')
        if bp_jeep.has_attribute('color'):
            bp_jeep.set_attribute('color', '255,100,0')  # CARLA 标准橙色
        trans_jeep = get_proper_spawn_transform(world, x=34.843, y=72.746)
        jeep = world.try_spawn_actor(bp_jeep, trans_jeep)
        if jeep:
            actor_list.append(jeep)
            print("5. jeep 生成成功 (橙色，PID自动搜寻前方锚点循迹，初始100km/h)")


        print("初始化物理系统...")
        for _ in range(20): world.tick()

        # --- 为 vehicle.jeep.wrangler_rubicon 赋予物理初速度 100km/h  ---
        if jeep and jeep.is_alive:
            forward_vec = jeep.get_transform().get_forward_vector()
            initial_speed_mps = 100.0 / 3.6
            jeep.set_target_velocity(carla.Vector3D(
                forward_vec.x * initial_speed_mps,
                forward_vec.y * initial_speed_mps,
                forward_vec.z * initial_speed_mps
            ))
        # ---------------------------------------------------------
        # 预计算 V3 的超车“主参考路径” (解决对向车道Waypoint反转的终极方案)
        # ---------------------------------------------------------
        v3_reference_path = []
        if v3 and v3.is_alive:
            current_wp = carla_map.get_waypoint(v3.get_location())
            for _ in range(1500):  # 提取 1500 米的参考路径保证够用
                v3_reference_path.append(current_wp)
                next_wps = current_wp.next(1.0)
                if next_wps:
                    current_wp = next_wps[0]
                else:
                    break

        print("\n场景正式运行，全车开启行车灯...")
        tick_count = 0

        while True:
            start_time = time.time()
            world.tick()
            tick_count += 1
            sim_time = tick_count * 0.05  # 仿真时间(秒)

            # ==========================
            # 控制 1: 消防车 (40 km/h)
            # ==========================
            if v1 and v1.is_alive:
                set_vehicle_lights(v1)  # 强制亮灯
                target_wp = get_lane_keeping_waypoint(carla_map, v1.get_location(), lookahead_dist=8.0)
                apply_pid_control(v1, pids['v1']['lon'], pids['v1']['lat'], 40.0, target_wp)

            # ==========================
            # 控制 2: 可乐卡车 (45 km/h)
            # ==========================
            if v2 and v2.is_alive:
                set_vehicle_lights(v2)  # 强制亮灯
                target_wp = get_lane_keeping_waypoint(carla_map, v2.get_location(), lookahead_dist=8.0)
                apply_pid_control(v2, pids['v2']['lon'], pids['v2']['lat'], 45.0, target_wp)

            # ==========================
            # 控制 3: Audi 超车车 (对向车道迅捷超车)
            # ==========================
            if v3 and v3.is_alive:
                v3_loc = v3.get_location()

                # 状态机时间轴定义 (时间更紧凑，动作更迅速)
                T_FOLLOW = 2.0  # 跟车观察时间
                T_OUT = 4.5  # 切出并加速阶段 (T_FOLLOW ~ T_OUT)
                T_PASS = 6.0  # 逆向超越阶段 (T_OUT ~ T_PASS) 仅用1.5秒超越
                T_BACK = 8.5  # 回归原车道阶段 (T_PASS ~ T_BACK)

                v3_target_speed = 45.0
                lateral_offset = 0.0
                LANE_WIDTH = 3.6

                if sim_time < T_FOLLOW:
                    # 阶段1: 跟随保持
                    lateral_offset = 0.0
                    v3_target_speed = 45.0
                    set_vehicle_lights(v3)

                elif sim_time < T_OUT:
                    # 阶段2: 打左灯，迅速向左变道进入对向车道，并急加速防追尾
                    progress = (sim_time - T_FOLLOW) / (T_OUT - T_FOLLOW)
                    lateral_offset = progress * LANE_WIDTH
                    v3_target_speed = 45.0 + (progress * 35.0)  # 提速到80 km/h
                    set_vehicle_lights(v3, left_blinker=True)

                elif sim_time < T_PASS:
                    # 阶段3: 全速逆向超车 (不打转向灯，直行)
                    lateral_offset = LANE_WIDTH
                    v3_target_speed = 80.0
                    set_vehicle_lights(v3)

                elif sim_time < T_BACK:
                    # 阶段4: 超车完成，打右灯迅速回归原车道
                    progress = (sim_time - T_PASS) / (T_BACK - T_PASS)
                    lateral_offset = LANE_WIDTH - (progress * LANE_WIDTH)
                    v3_target_speed = 80.0 - (progress * 15.0)  # 稍减速到 65 km/h
                    set_vehicle_lights(v3, right_blinker=True)

                else:
                    # 阶段5: 回归完成，关转向灯，平稳巡航
                    lateral_offset = 0.0
                    v3_target_speed = 55.0
                    set_vehicle_lights(v3)

                # --- 核心：基于偏移量计算前方轨迹点 ---
                min_dist = float('inf')
                closest_idx = 0
                for i, wp in enumerate(v3_reference_path):
                    d = v3_loc.distance(wp.transform.location)
                    if d < min_dist:
                        min_dist, closest_idx = d, i

                # 向前看 lookahead 米 (速度越快前瞻应该稍微远一点)
                lookahead_idx = min(closest_idx + 8, len(v3_reference_path) - 1)
                ref_wp = v3_reference_path[lookahead_idx]

                # 根据右向向量减去偏移量，在左侧生成一个朝向正确的虚拟点
                right_vector = ref_wp.transform.get_right_vector()
                target_loc = ref_wp.transform.location - (right_vector * lateral_offset)
                virtual_target_wp = (target_loc.x, target_loc.y, target_loc.z)

                apply_pid_control(v3, pids['v3']['lon'], pids['v3']['lat'], v3_target_speed, virtual_target_wp)

            # ==========================
            # 控制 4: Mercedes (70 km/h, 匝道汇入打灯)
            # ==========================
            if v4 and v4.is_alive:
                # 假设前 8 秒正处于匝道汇入主路的阶段，开启左转向灯+基础行车灯
                if sim_time < 8.0:
                    set_vehicle_lights(v4, left_blinker=True)
                else:
                    set_vehicle_lights(v4)  # 关转向灯，保留基础行车灯

                target_wp = get_lane_keeping_waypoint(carla_map, v4.get_location(), lookahead_dist=6.0)
                apply_pid_control(v4, pids['v4']['lon'], pids['v4']['lat'], 70.0, target_wp)

            # ==========================
            # 控制 5: jeep TT (恒定自动搜索前方锚点循迹)
            # ==========================
            if jeep and jeep.is_alive:
                # 动态提取前方 6 米的道路中心锚点进行车道保持
                target_wp = get_lane_keeping_waypoint(carla_map, jeep.get_location(), lookahead_dist=6.0)
                apply_pid_control(jeep, pids['jeep']['lon'], pids['jeep']['lat'], 73.0, target_wp)

            # --- 帧率同步 ---
            compute_time = time.time() - start_time
            if compute_time < 0.05:
                time.sleep(0.05 - compute_time)

    except KeyboardInterrupt:
        print("\n用户按 Ctrl+C 停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if actor_list:
            client.apply_batch([carla.command.DestroyActor(a) for a in actor_list])
        print("清理完成。")


if __name__ == '__main__':
    main()