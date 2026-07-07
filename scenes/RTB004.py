# -*- coding: utf-8 -*-

import carla
import time
import math
import numpy as np
import random


# ==========================================
# 轨迹数据清洗 (自动去重)
# ==========================================
def clean_path_points(raw_points):
    cleaned_points = []
    if raw_points:
        cleaned_points.append(raw_points[0])
        for i in range(1, len(raw_points)):
            if raw_points[i] != raw_points[i - 1]:
                cleaned_points.append(raw_points[i])
    return cleaned_points


# 蓝车 (Nissan/Tesla) 轨迹数据
RAW_VEHICLE_PATH_POINTS = [
    (-23.595, 52.2, -89.396), (-23.595, 47.2, -89.096), (-23.595, 41, -94.096),
    (-23.595, 37, -94.096), (-23.595, 35, -94.166), (-23.595, 34.2, -94.339),
    (-23.861, 30.619, -93.826), (-24.094, 26.773, -92.652), (-24.234, 22.964, -91.925),
    (-24.363, 19.108, -91.558), (-24.429, 15.196, -90.109), (-24.424, 11.391, -89.872),
    (-24.343, 7.504, -87.562), (-24.101, 3.688, -85.596), (-23.711, -0.089, -80.814),
    (-22.989, -3.799, -78.13), (-22.136, -7.544, -74.397), (-21.118, -11.173, -74.327),
    (-20.075, -14.828, -73.327), (-18.929, -18.442, -71.683), (-17.423, -22.063, -61.694),
    (-15.515, -25.403, -60.861), (-13.752, -28.779, -63.729), (-12.018, -32.277, -63.008),
    (-10.252, -35.606, -60.477), (-8.232, -38.899, -57.583), (-6.121, -42.164, -55.897),
    (-3.834, -45.271, -52.278), (-1.462, -48.201, -50.749), (1.192, -51.064, -44.477),
    (3.933, -53.743, -43.158), (6.832, -56.316, -39.974), (9.749, -58.582, -36.317),
    (12.883, -60.707, -31.933), (16.175, -62.679, -29.216), (19.59, -64.437, -26.253),
    (23.074, -66.12, -25.071), (26.531, -67.711, -24.467), (29.935, -69.229, -23.54),
    (33.486, -70.707, -21.49), (37.073, -72.073, -19.433), (40.756, -73.319, -17.164),
    (44.466, -74.321, -13.442), (48.27, -75.125, -11.051), (51.909, -75.909, -14.959),
    (55.627, -77.08, -21.985), (58.993, -78.838, -33.343), (61.97, -81.177, -41.304),
    (64.633, -83.835, -48.421), (66.96, -86.87, -55.11), (69.103, -89.947, -55.809),
    (70.894, -93.377, -66.295), (72.347, -96.969, -68.358), (73.776, -100.571, -68.358),
    (75.101, -104.145, -71.47), (76.226, -107.787, -74.617), (77.245, -111.525, -74.756),
    (78.237, -115.206, -75.595), (79.118, -118.915, -78.008), (79.887, -122.649, -79.267),
    (80.603, -126.459, -79.408), (81.293, -130.146, -79.408), (81.983, -133.833, -79.408),
    (82.626, -137.275, -79.408), (82.626, -137.275, -79.408), (82.626, -137.275, -79.408),
    (82.626, -137.275, -79.408), (82.626, -137.275, -79.408)
]
VEHICLE_PATH_POINTS = clean_path_points(RAW_VEHICLE_PATH_POINTS)

# 行人随机漫游坐标点
PEDESTRIAN_LOCATIONS = [
    carla.Location(x=-19.568, y=-27.066, z=1.0),
    carla.Location(x=-17.682, y=-30.062, z=1.0),
    carla.Location(x=-16.603, y=-34.192, z=1.0),
    carla.Location(x=-20.078, y=-32.118, z=1.0)
]


# ==========================================
# PID 控制器类
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController2:
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
        if norm_w < 0.1:
            return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
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


def get_target_waypoint(actor_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - actor_loc.x) ** 2 + (p[1] - actor_loc.y) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_index = i

    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1 = path_points[i]
        p2 = path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]


def clamp(v, a, b):
    return max(a, min(b, v))


# 【新增】出界判定及销毁函数
def check_and_handle_out_of_bounds(actor, carla_map, threshold=6.0):
    """
    检查车辆是否垂直投影脱离了道路，如果超过距离阈值(如6米)则直接销毁该 actor。
    """
    if actor is None or not actor.is_alive:
        return True

    loc = actor.get_location()
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True)

    # 获取不到投影路点直接销毁
    if wp_nearest is None:
        print(f"[{actor.type_id} {actor.attributes.get('role_name', 'None')}] 无法投影到道路，判定出界被销毁！")
        actor.destroy()
        return True

    distance = wp_nearest.transform.location.distance(loc)
    # 大于允许的出轨阈值时销毁
    if distance > threshold:
        print(
            f"[{actor.type_id} {actor.attributes.get('role_name', 'None')}] 偏离道路中心 {distance:.2f} 米，判定出界被销毁！")
        actor.destroy()
        return True

    return False


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # 严格保持天气参数
    weather = carla.WeatherParameters(
        cloudiness=15.0, precipitation=100.0, precipitation_deposits=100.0,
        wind_intensity=10.0, sun_azimuth_angle=85.0, sun_altitude_angle=-90.0,
        fog_density=15.0, fog_distance=5.0, fog_falloff=0.0, wetness=60.0,
        scattering_intensity=8.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.10,
        dust_storm=0.0
    )
    world.set_weather(weather)
    print("天气参数已更新。")

    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substeps = 10
        world.apply_settings(settings)

        # 交通管理器 (Traffic Manager) 设置
        tm_port = 8000
        tm = client.get_trafficmanager(tm_port)
        tm.set_synchronous_mode(True)
        tm.set_hybrid_physics_mode(True)

        # ---------------------------------------------------------
        # 1. 生成 蓝车 (tesla.model3)
        # ---------------------------------------------------------
        bp_vehicle = bp_lib.find('vehicle.tesla.model3')
        bp_vehicle.set_attribute('color', '0,0,255')
        initial_vehicle_point = VEHICLE_PATH_POINTS[0]
        trans_vehicle = get_transform(x=initial_vehicle_point[0], y=initial_vehicle_point[1], z=1.0,
                                      yaw=initial_vehicle_point[2])
        vehicle = world.try_spawn_actor(bp_vehicle, trans_vehicle)
        if vehicle:
            actor_list.append(vehicle)
            vehicle.set_simulate_physics(True)
            print("tesla.model3 生成成功 (跟随蓝车)")

        lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0, dt=settings.fixed_delta_seconds)
        lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

        # ---------------------------------------------------------
        # 2. 生成 Agent 自动驾驶车辆 (红色的Audi)
        # ---------------------------------------------------------
        bp_agent_car = bp_lib.find('vehicle.audi.tt')
        bp_agent_car.set_attribute('color', '255,0,0')

        agent_spawn_loc = carla.Location(x=-27.166, y=40.632, z=0.0)
        agent_spawn_wp = carla_map.get_waypoint(agent_spawn_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        agent_spawn_transform = agent_spawn_wp.transform
        agent_spawn_transform.location.z += 1.0

        agent_vehicle = world.try_spawn_actor(bp_agent_car, agent_spawn_transform)
        if agent_vehicle:
            actor_list.append(agent_vehicle)
            agent_vehicle.set_simulate_physics(True)
            agent_vehicle.set_autopilot(True, tm_port)

            tm.auto_lane_change(agent_vehicle, False)
            tm.vehicle_percentage_speed_difference(agent_vehicle, -56.25)

            # 【关键防御】彻底禁止 TM 控制 Agent 车辆的灯光
            tm.ignore_lights_percentage(agent_vehicle, 100.0)
            try:
                tm.update_vehicle_lights(agent_vehicle, False)
            except Exception:
                pass
            print("Agent 车辆 (自动保持车道红车) 生成成功")

        # ---------------------------------------------------------
        # 3. [优化点] 生成橙色 Audi TT，设定为主控 Ego
        # ---------------------------------------------------------
        bp_orange_audi = bp_lib.find('vehicle.audi.tt')
        bp_orange_audi.set_attribute('color', '255,128,0')  # 橙色
        bp_orange_audi.set_attribute('role_name', 'ego')  # 【关键修改】设置actor名为ego

        # 目标位置，z轴设为0让API自动去贴近地面寻找
        orange_audi_loc = carla.Location(x=-23.344, y=30.983, z=20.0)
        # 自动获取道路锚点 (project_to_road=True 会把坐标映射到合法的道路中心或车道上)
        orange_audi_wp = carla_map.get_waypoint(orange_audi_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        orange_audi_transform = orange_audi_wp.transform
        orange_audi_transform.location.z += 0.5  # 略微抬高避免碰撞地面

        orange_audi = world.try_spawn_actor(bp_orange_audi, orange_audi_transform)
        if orange_audi:
            actor_list.append(orange_audi)
            orange_audi.set_simulate_physics(True)
            print(f"EGO 橙色 Audi TT 生成成功，吸附位置: {orange_audi_transform.location}")

            # 为 Ego 橙色 Audi 初始化独立的PID控制器
            orange_lon_controller = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0,
                                                              dt=settings.fixed_delta_seconds)
            orange_lat_controller = PIDLateralController2(K_P=1.95, K_I=0.05, K_D=0.2, dt=settings.fixed_delta_seconds)

        # ---------------------------------------------------------
        # 4. 生成 行人
        # ---------------------------------------------------------
        walker_bps = bp_lib.filter('walker.pedestrian.*')
        bp_walker = random.choice(walker_bps)

        initial_ped_loc = random.choice(PEDESTRIAN_LOCATIONS)
        trans_walker = get_transform(x=initial_ped_loc.x, y=initial_ped_loc.y, z=initial_ped_loc.z, yaw=0.0)
        walker = world.try_spawn_actor(bp_walker, trans_walker)
        if walker:
            actor_list.append(walker)
            print("行人 生成成功")

        # ==========================================
        # 等待物理稳定后，一次性设定车灯
        # ==========================================
        print("等待物理系统初始化...")
        for _ in range(5):
            world.tick()
            time.sleep(settings.fixed_delta_seconds)

        # 物理稳定后，强制一次性打开远光灯+位置灯
        base_light_state = carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam
        if vehicle and vehicle.is_alive:
            vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        if agent_vehicle and agent_vehicle.is_alive:
            agent_vehicle.set_light_state(carla.VehicleLightState(base_light_state))
        if orange_audi and orange_audi.is_alive:
            orange_audi.set_light_state(carla.VehicleLightState(base_light_state))

        print("\n=> 物理系统稳定，已下发车灯常亮指令！场景运行中...")

        # --- 车辆参数 ---
        initial_vehicle_speed = 85.0
        decelerate_vehicle_speed = 40.0
        decelerate_y_threshold = -1.0
        current_target_vehicle_speed = initial_vehicle_speed
        deceleration_rate = 15.0

        # 行人控制参数
        PED_SPEED_MPS = 5.0 / 3.6  # 5 km/h
        PED_ARRIVAL_DIST = 0.8
        current_ped_target = random.choice([loc for loc in PEDESTRIAN_LOCATIONS if loc != initial_ped_loc])
        ped_last_pos = walker.get_location() if walker else None
        ped_last_progress_time = time.time()

        # 主循环
        while True:
            start_time = time.time()
            world.tick()

            # ==============================
            # 1. 蓝车 (tesla.model3) PID 控制逻辑
            # ==============================
            if vehicle and vehicle.is_alive:
                # 【新增出界判定】
                if check_and_handle_out_of_bounds(vehicle, carla_map):
                    vehicle = None  # 防止报错，标记已销毁
                else:
                    tf = vehicle.get_transform()
                    vel = vehicle.get_velocity()
                    current_vehicle_speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                    if tf.location.y <= decelerate_y_threshold and current_target_vehicle_speed > decelerate_vehicle_speed:
                        speed_decrease = deceleration_rate * settings.fixed_delta_seconds
                        current_target_vehicle_speed = max(decelerate_vehicle_speed,
                                                           current_target_vehicle_speed - speed_decrease)
                    elif tf.location.y > decelerate_y_threshold and current_target_vehicle_speed < initial_vehicle_speed:
                        speed_increase = deceleration_rate * settings.fixed_delta_seconds
                        current_target_vehicle_speed = min(initial_vehicle_speed,
                                                           current_target_vehicle_speed + speed_increase)

                    target_wp = get_target_waypoint(tf.location, VEHICLE_PATH_POINTS, lookahead_dist=5.0)
                    throttle_output = lon_controller.run_step(current_target_vehicle_speed, current_vehicle_speed)
                    steer_output = lat_controller.run_step(target_wp, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_output
                    if throttle_output >= 0.0:
                        control.throttle = throttle_output
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_output)

                    vehicle.apply_control(control)

            # ==============================
            # 2. TM 自动驾驶车辆 (红色 Audi)出界判定
            # ==============================
            if agent_vehicle and agent_vehicle.is_alive:
                if check_and_handle_out_of_bounds(agent_vehicle, carla_map):
                    agent_vehicle = None

            # ==============================
            # 3. Ego 橙色 Audi TT 控制逻辑
            # ==============================
            if orange_audi and orange_audi.is_alive:
                # 【新增出界判定】
                if check_and_handle_out_of_bounds(orange_audi, carla_map):
                    orange_audi = None
                else:
                    o_tf = orange_audi.get_transform()
                    o_vel = orange_audi.get_velocity()
                    o_current_speed = 3.6 * math.sqrt(o_vel.x ** 2 + o_vel.y ** 2 + o_vel.z ** 2)

                    # 速度策略：初始70km/h，在y=-5减速到40km/h，y=-30恢复到90km/h
                    if o_tf.location.y > -5.0:
                        o_target_speed = 70.0
                    elif -30.0 < o_tf.location.y <= -5.0:
                        o_target_speed = 40.0
                    else:  # y <= -30.0
                        o_target_speed = 90.0

                    # 横向控制：动态获取车道中心前方锚点以实现车道保持
                    o_current_wp = carla_map.get_waypoint(o_tf.location)
                    o_next_wps = o_current_wp.next(4.0)  # 获取前方4米处的路点
                    if o_next_wps:
                        o_target_wp_loc = o_next_wps[0].transform.location
                        # 将路点转换为格式 (x, y, z) 传入控制器
                        o_target_wp = (o_target_wp_loc.x, o_target_wp_loc.y, o_target_wp_loc.z)
                        o_steer_output = orange_lat_controller.run_step(o_target_wp, o_tf)
                    else:
                        o_steer_output = 0.0

                    # 纵向PID控制计算
                    o_throttle_output = orange_lon_controller.run_step(o_target_speed, o_current_speed)

                    o_control = carla.VehicleControl()
                    o_control.steer = o_steer_output
                    if o_throttle_output >= 0.0:
                        o_control.throttle = o_throttle_output
                        o_control.brake = 0.0
                    else:
                        o_control.throttle = 0.0
                        o_control.brake = abs(o_throttle_output)

                    orange_audi.apply_control(o_control)

            # ==============================
            # 4. 行人随机漫游控制逻辑
            # ==============================
            if walker and walker.is_alive:
                ped_loc = walker.get_location()
                dx = current_ped_target.x - ped_loc.x
                dy = current_ped_target.y - ped_loc.y
                dist = math.hypot(dx, dy)

                if dist <= PED_ARRIVAL_DIST:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = time.time()
                    continue

                ctrl = carla.WalkerControl()
                if dist > 1e-6:
                    nx, ny = dx / dist, dy / dist
                    ctrl.direction = carla.Vector3D(nx, ny, 0.0)
                    slow_radius = 2.0
                    if dist < slow_radius:
                        desired_speed = PED_SPEED_MPS * (dist / slow_radius)
                        ctrl.speed = clamp(desired_speed, 0.25, PED_SPEED_MPS)
                    else:
                        ctrl.speed = PED_SPEED_MPS
                else:
                    ctrl.speed = 0.0
                    ctrl.direction = carla.Vector3D(0.0, 0.0, 0.0)

                walker.apply_control(ctrl)

                # 防卡死检测
                current_time = time.time()
                if math.hypot(ped_loc.x - ped_last_pos.x, ped_loc.y - ped_last_pos.y) > 0.05:
                    ped_last_progress_time = current_time
                    ped_last_pos = ped_loc

                if current_time - ped_last_progress_time > 8.0:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = current_time

            # ==============================
            # 严格保留的时间同步逻辑
            # ==============================
            compute_time = time.time() - start_time
            if compute_time < settings.fixed_delta_seconds:
                time.sleep(settings.fixed_delta_seconds - compute_time)

    except Exception as e:
        print(f"发生异常: {e}")
    except KeyboardInterrupt:
        print("\n用户停止运行。")
    finally:
        print("\n正在恢复环境并清理 Actors...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(False)

        # 【优化】判断 actor 是否存活再释放，防止释放已经被出界函数销毁的实体报错
        if actor_list:
            actors_to_destroy = [a for a in actor_list if a is not None and a.is_alive]
            client.apply_batch([carla.command.DestroyActor(a) for a in actors_to_destroy])

        print("清理完成，Carla 已恢复正常。")


if __name__ == '__main__':
    main()