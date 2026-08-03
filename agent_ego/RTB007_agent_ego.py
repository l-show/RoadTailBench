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

def try_spawn_with_fallback(world, bp_lib, blueprint_id, base_transform, name, color=None):
    bp = bp_lib.find(blueprint_id)
    if color and bp.has_attribute('color'):
        bp.set_attribute('color', color)

    candidates = [base_transform]
    base_wp = world.get_map().get_waypoint(
        base_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if base_wp:
        for dist in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0):
            for wp in base_wp.next(dist) + base_wp.previous(dist):
                trans = wp.transform
                trans.location.z += 0.5
                candidates.append(trans)

    for i, trans in enumerate(candidates):
        actor = world.try_spawn_actor(bp, trans)
        if actor:
            if i:
                print(f"{name} 生成成功: 原始点被占用，改用候选点 {i} ({trans.location.x:.3f}, {trans.location.y:.3f})")
            return actor

    print(f"{name} 生成失败: 已尝试 {len(candidates)} 个候选点，blueprint={blueprint_id}")
    return None

RAW_EGO_TRAJ = [
    (134.088, 45.904, 178.673), (134.088, 45.904, 178.673), (134.088, 45.904, 178.673),
    (134.088, 45.904, 178.673), (134.088, 45.904, 178.533), (134.088, 45.904, 178.463),
    (128.439, 46.048, 178.533), (118.293, 46.256, 179.734), (108.296, 46.202, -179.006),
    (98.306, 46.035, -179.216), (88.142, 45.929, -179.496), (78.146, 45.841, -179.496),
    (68.149, 45.750, -179.426), (57.984, 45.648, -179.426), (49.048, 45.559, -179.426),
    (41.383, 45.482, -179.426), (33.884, 45.407, -179.426), (27.468, 45.343, -179.426),
    (21.115, 45.265, -179.076), (14.762, 45.157, -179.006), (8.513, 45.048, -179.006),
    (2.161, 44.938, -179.006), (-4.088, 44.830, -179.006), (-10.544, 44.819, 178.169),
    (-16.886, 45.193, 176.201), (-23.126, 45.558, 178.619), (-29.373, 45.492, -177.994),
    (-35.718, 45.197, -176.489), (-40.394, 44.895, -176.279), (-40.394, 44.895, -176.279),
    (-40.394, 44.895, -176.279), (-40.394, 44.895, -176.279), (-40.394, 44.895, -176.279),
    (-40.394, 44.895, -176.279), (-40.394, 44.895, -176.279), (-40.394, 44.895, -176.279),
    (-40.394, 44.895, -176.279), (-40.914, 44.863, -176.489), (-47.157, 44.598, -178.482),
    (-53.405, 44.516, -179.469), (-59.760, 44.463, -179.539), (-66.114, 44.412, -179.539),
    (-72.572, 44.339, -179.259), (-79.030, 44.245, -179.119), (-85.278, 44.114, -178.489),
    (-91.733, 43.944, -178.489), (-97.980, 43.779, -178.489), (-104.229, 43.684, -179.620),
    (-110.687, 43.642, -179.620), (-117.145, 43.647, 178.826), (-123.365, 44.191, 169.221),
    (-129.640, 45.709, 161.730), (-135.297, 48.349, 149.595), (-140.432, 52.079, 137.069),
    (-144.537, 56.780, 126.750), (-147.691, 62.270, 112.152), (-147.847, 62.656, 111.934),
    (-147.847, 62.656, 111.934), (-147.847, 62.656, 111.934), (-147.847, 62.656, 111.934)
]

EGO_PATH_POINTS = []
if RAW_EGO_TRAJ:
    EGO_PATH_POINTS.append((RAW_EGO_TRAJ[0][0], RAW_EGO_TRAJ[0][1], 0.5, RAW_EGO_TRAJ[0][2]))
    for i in range(1, len(RAW_EGO_TRAJ)):
        if RAW_EGO_TRAJ[i] != RAW_EGO_TRAJ[i - 1]:
            EGO_PATH_POINTS.append((RAW_EGO_TRAJ[i][0], RAW_EGO_TRAJ[i][1], 0.5, RAW_EGO_TRAJ[i][2]))

def get_target_waypoint(vehicle_loc, path_points, lookahead_dist=6.0):
    min_dist = float('inf')
    closest_index = 0
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist, closest_index = dist, i

    target_index, current_dist = closest_index, 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1, p2 = path_points[i], path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]

def is_near_path_end(actor, path_points, threshold=3.0):
    if actor is None or not actor.is_alive or not path_points:
        return False
    loc = actor.get_location()
    end = path_points[-1]
    return math.hypot(loc.x - end[0], loc.y - end[1]) <= threshold

def destroy_actor(actor, reason):
    if actor and actor.is_alive:
        print(f"{actor.type_id} 销毁: {reason}")
        actor.destroy()
    return None

def destroy_all_actors(actor_list, reason):
    print(reason)
    for actor in actor_list[:]:
        if actor and actor.is_alive:
            actor.destroy()
        if actor in actor_list:
            actor_list.remove(actor)

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

        print("正在生成车辆...")

        # 先画出所有原始生成点与投影到道路后的点，便于排查车辆未生成原因。

        # 1. 消防车 (大卡车)
        trans_v1 = get_proper_spawn_transform(world, x=-48.306, y=55.167)
        v1 = try_spawn_with_fallback(world, bp_lib, 'vehicle.carlamotors.firetruck', trans_v1, "1. Firetruck")
        if v1: actor_list.append(v1); print("1. Firetruck 生成成功")

        # 2. 货车 (可乐卡车)
        trans_v2 = get_proper_spawn_transform(world, x=-47.762, y=51.454)
        v2 = try_spawn_with_fallback(world, bp_lib, 'vehicle.carlamotors.carlacola', trans_v2, "2. Carlacola")
        if v2: actor_list.append(v2); print("2. Carlacola 生成成功")

        # 3. 超车轿车 (Audi TT) - 采用安全算法生成在 V2 后面约 35 米处
        wp_v2 = carla_map.get_waypoint(trans_v2.location)
        trans_v3 = get_safe_backward_transform(wp_v2, target_distance=35.0)
        v3 = try_spawn_with_fallback(world, bp_lib, 'vehicle.audi.tt', trans_v3, "3. Audi TT")
        if v3: actor_list.append(v3); print("3. Audi TT (超车车) 生成成功")

        # 4. 匝道汇入轿跑 (Mercedes)
        trans_v4 = get_proper_spawn_transform(world, x=6.205, y=-9.597)
        v4 = try_spawn_with_fallback(world, bp_lib, 'vehicle.mercedes.coupe_2020', trans_v4, "4. Mercedes Coupe")
        if v4: actor_list.append(v4); print("4. Mercedes Coupe 生成成功")

        # 5. 车辆5: vehicle.jeep.wrangler_rubicon (新增：橙色)
        trans_jeep = get_transform(EGO_PATH_POINTS[0][0], EGO_PATH_POINTS[0][1], 0.5, yaw=EGO_PATH_POINTS[0][3])
        bp_jeep = bp_lib.find('vehicle.jeep.wrangler_rubicon')
        if bp_jeep.has_attribute('color'):
            bp_jeep.set_attribute('color', '255,100,0')
        jeep = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if jeep:
            print("5. jeep EGO 生成成功 (橙色，PID轨迹点循迹，初始60km/h)")

        print("初始化物理系统...")
        for _ in range(20): world.tick()

        # --- 为 vehicle.jeep.wrangler_rubicon EGO 赋予物理初速度 60km/h  ---
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

        if v1 and v1.is_alive:
            pass
        if v2 and v2.is_alive:
            pass
        if v3_reference_path:
            pass
        if v4 and v4.is_alive:
            pass

        print("\n场景正式运行，全车开启行车灯...")
        tick_count = 0
        jeep_target_speed = 60.0
        jeep_slowdown_triggered = False
        jeep_resume_time = None
        v4_target_speed_actual = 0.0
        v4_accel_rate = 12.0  # km/h per second

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

            # ==========================
            # 控制 2: 可乐卡车 (45 km/h)
            # ==========================
            if v2 and v2.is_alive:
                set_vehicle_lights(v2)  # 强制亮灯
                target_wp = get_lane_keeping_waypoint(carla_map, v2.get_location(), lookahead_dist=8.0)

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

            # ==========================
            # 控制 4: Mercedes (70 km/h, 匝道汇入打灯)
            # ==========================
            if v4 and v4.is_alive:
                # 假设前 8 秒正处于匝道汇入主路的阶段，开启左转向灯+基础行车灯
                if sim_time < 8.0:
                    set_vehicle_lights(v4, left_blinker=True)
                else:
                    set_vehicle_lights(v4)  # 关转向灯，保留基础行车灯

                v4_target_speed_actual = min(70.0, v4_target_speed_actual + v4_accel_rate * 0.02)
                target_wp = get_lane_keeping_waypoint(carla_map, v4.get_location(), lookahead_dist=6.0)

            # ==========================
            # 控制 5: Jeep EGO (轨迹点 PID 循迹)
            # ==========================

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
            actors_to_destroy = [a for a in actor_list if a is not None and a.is_alive]
            client.apply_batch([carla.command.DestroyActor(a) for a in actors_to_destroy])
        print("清理完成。")

if __name__ == '__main__':
    main()
