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
    if wp_nearest is None:
        vehicle.destroy()
        return True
    distance = wp_nearest.transform.location.distance(loc)
    if distance > 6.0:
        vehicle.destroy()
        return True
    return False


# ==========================
# 轨迹数据定义
# ==========================
# 第一辆车 (Mercedes Sprinter) 的轨迹
V1_TRAJECTORY = [
    (-221.172, -0.776, -157.734), (-221.172, -0.776, -157.734), (-222.172, -1.186, -157.734),
    (-224.453, -2.186, -153.614), (-226.746, -3.354, -151.614), (-228.895, -4.605, -148.104),
    (-230.992, -5.939, -147.396), (-233.053, -7.325, -145.408), (-235.093, -8.742, -144.071),
    (-237.141, -10.287, -142.072), (-239.048, -11.878, -139.003), (-240.907, -13.526, -137.942),
    (-242.761, -15.299, -134.37), (-244.518, -17.169, -132.442), (-246.218, -19.036, -132.3),
    (-247.868, -20.893, -130.733), (-249.496, -22.876, -127.727), (-251.014, -24.943, -125.293),
    (-252.462, -27.057, -122.854), (-253.759, -29.171, -120.271), (-254.903, -31.371, -116.133),
    (-255.979, -33.606, -114.411), (-256.995, -35.959, -112.766), (-257.942, -38.251, -112.125),
    (-258.866, -40.553, -110.84), (-259.721, -42.881, -109.128), (-260.519, -45.273, -107.636),
    (-261.235, -47.69, -105.212), (-261.857, -50.135, -104.066), (-262.444, -52.545, -102.264),
    (-262.957, -55.057, -100.905), (-263.391, -57.5, -99.695), (-263.823, -60.027, -99.695),
    (-264.241, -62.472, -99.695), (-264.622, -65.007, -97.541), (-264.948, -67.467, -97.541),
    (-265.229, -69.973, -94.893), (-265.387, -72.49, -92.662), (-265.479, -75.052, -90.945),
    (-265.521, -77.615, -90.945), (-265.53, -80.096, -88.298), (-265.385, -82.611, -85.777),
    (-265.202, -85.083, -85.777), (-265.013, -87.638, -85.777), (-264.828, -90.151, -85.777),
    (-264.612, -92.582, -83.86), (-264.306, -95.127, -82.579), (-263.94, -97.665, -81.438),
    (-263.537, -100.199, -80.517), (-263.11, -102.645, -79.812), (-262.638, -105.125, -78.888),
    (-262.128, -107.556, -77.96), (-261.604, -110.07, -78.734), (-261.126, -112.552, -79.514),
    (-260.672, -115.08, -80.149), (-260.235, -117.611, -80.219), (-259.802, -120.101, -79.724),
    (-259.33, -122.541, -78.313), (-258.753, -125, -74.709), (-257.988, -127.406, -71.737),
    (-257.191, -129.846, -72.162), (-256.424, -132.297, -72.799), (-255.68, -134.668, -72.021),
    (-254.88, -137.059, -70.515), (-253.987, -139.461, -68.294), (-253.015, -141.741, -66),
    (-251.92, -144.011, -62.369), (-250.715, -146.178, -60.07), (-249.439, -148.304, -58.133),
    (-249.242, -148.62, -58.133), (-249.242, -148.62, -58.133), (-245.136, -154.519, -50.821),
    (-240.117, -160.171, -45.712), (-234.634, -165.383, -40.571), (-228.454, -169.956, -32.553),
    (-221.848, -173.913, -30.428), (-215.36, -177.59, -28.578), (-208.788, -181.123, -27.657),
    (-202.156, -184.55, -27.302), (-195.526, -187.981, -28.095), (-188.877, -191.642, -29.375),
    (-182.32, -195.456, -30.508), (-175.91, -199.213, -30.721), (-169.444, -203.112, -31.936),
    (-162.953, -207.22, -32.152), (-156.417, -211.298, -31.803), (-149.866, -215.36, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803),
    (-145.954, -217.786, -31.803), (-145.954, -217.786, -31.803)
]


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # ==========================
    # 【1】天气配置
    # ==========================
    weather = carla.WeatherParameters(
        cloudiness=100.0, precipitation=50.0, precipitation_deposits=100.0,
        wind_intensity=100.0, sun_azimuth_angle=180.0, sun_altitude_angle=20.0,
        fog_density=80.0, fog_distance=15.0, fog_falloff=0.2, wetness=100.0,
        scattering_intensity=10.0, mie_scattering_scale=0.1, rayleigh_scattering_scale=0.04
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []

    v1_active = False
    ego_active = False

    try:
        # 同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        pid_v1 = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_ego = {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)}

        # ==========================
        # 【2】超大范围湿滑路面生成与可视化
        # ==========================
        friction_bp = bp_lib.find('static.trigger.friction')
        # 设置摩擦力极低 (0.1)，范围 extent 是半尺寸 (即总长宽 40x40)
        extent_x, extent_y, extent_z = 20.0, 20.0, 5.0
        friction_bp.set_attribute('friction', '0.1')
        friction_bp.set_attribute('extent_x', str(extent_x))
        friction_bp.set_attribute('extent_y', str(extent_y))
        friction_bp.set_attribute('extent_z', str(extent_z))

        friction_loc = carla.Location(x=-255.862, y=-28.314, z=-2.634)
        friction_trigger = world.spawn_actor(friction_bp, carla.Transform(friction_loc))
        actor_list.append(friction_trigger)
        print("已生成超大范围湿滑区域。")

        # 【新增：画出摩擦力触发区域的红色边框】
        box_extent = carla.Vector3D(x=extent_x, y=extent_y, z=extent_z)
        bbox = carla.BoundingBox(friction_loc, box_extent)
        # 红色边框，存活时间 1000 秒，厚度 0.5
        world.debug.draw_box(box=bbox, rotation=carla.Rotation(), thickness=0.5, color=carla.Color(255, 0, 0),
                             life_time=1000.0)

        # ==========================
        # 【3】Actor 1：Mercedes Sprinter
        # ==========================
        bp_v1 = bp_lib.find('vehicle.mercedes.sprinter')
        if bp_v1.has_attribute('color'):
            bp_v1.set_attribute('color', '255,255,255')
        v1_x, v1_y, v1_yaw = V1_TRAJECTORY[0]
        v1_loc = carla.Location(x=v1_x, y=v1_y, z=0.5)
        v1_loc.z = carla_map.get_waypoint(v1_loc).transform.location.z + 0.5
        v1 = world.try_spawn_actor(bp_v1, carla.Transform(v1_loc, carla.Rotation(yaw=v1_yaw)))
        if v1:
            actor_list.append(v1)
            v1_active = True
            print("生成 Sprinter 成功。")

        # ==========================
        # 【4】Actor 2：Ego车 (Citroen C3)
        # ==========================
        bp_ego = bp_lib.find('vehicle.citroen.c3')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,255,0')  # 黄色

        ego_start_loc = carla.Location(x=-11.779, y=-71.222, z=0.5)
        ego_start_wp = carla_map.get_waypoint(ego_start_loc, project_to_road=True)
        ego_start_loc.z = ego_start_wp.transform.location.z + 0.5
        ego = world.try_spawn_actor(bp_ego, carla.Transform(ego_start_loc, ego_start_wp.transform.rotation))

        if ego:
            actor_list.append(ego)
            ego_active = True
            light_state = carla.VehicleLightState.HighBeam | carla.VehicleLightState.Fog
            ego.set_light_state(carla.VehicleLightState(light_state))
            print("生成 Ego (Citroen C3) 成功，已开启黄色雾灯及远光灯。")

        # 等待车辆落地贴面
        for _ in range(10):
            world.tick()

        # 【5】赋予初始物理速度
        if v1_active:
            v1_speed_ms = 10.0 / 3.6
            v1_yaw_rad = math.radians(v1_yaw)
            v1.set_target_velocity(
                carla.Vector3D(v1_speed_ms * math.cos(v1_yaw_rad), v1_speed_ms * math.sin(v1_yaw_rad), 0.0))

        if ego_active:
            ego_speed_ms = 60.0 / 3.6
            ego_yaw_rad = math.radians(ego_start_wp.transform.rotation.yaw)
            ego.set_target_velocity(
                carla.Vector3D(ego_speed_ms * math.cos(ego_yaw_rad), ego_speed_ms * math.sin(ego_yaw_rad), 0.0))

            current_target_wp = ego_start_wp
            ego_has_turned = False

        v1_traj_idx = 0
        print("\n仿真正式开始！红色线框代表极端湿滑区域。绿色点代表车辆正在追踪的锚点。")

        while True:
            start_time = time.time()
            world.tick()

            # ==========================
            # 车辆 1 (Sprinter) 固定轨迹
            # ==========================
            if v1_active and v1.is_alive:
                if check_and_handle_out_of_bounds(v1, carla_map):
                    v1_active = False
                elif v1_traj_idx < len(V1_TRAJECTORY):
                    tx, ty, tyaw = V1_TRAJECTORY[v1_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=v1.get_location().z)
                    if v1.get_location().distance(target_loc) < 2.5 and v1_traj_idx < len(V1_TRAJECTORY) - 1:
                        v1_traj_idx += 1
                    apply_pid_control(v1, pid_v1['lon'], pid_v1['lat'], 15.0, target_loc)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))
                    v1_active = False

            # ==========================
            # Ego车 (Citroen C3)：基于向量投影的最右侧车道判别法
            # ==========================
            if ego_active and ego.is_alive:
                if check_and_handle_out_of_bounds(ego, carla_map):
                    ego_active = False
                else:
                    ego_loc = ego.get_location()

                    # 可视化当前追踪的 Waypoint (画一个绿色的点，持续时间与物理帧一致)
                    world.debug.draw_point(current_target_wp.transform.location + carla.Location(z=1.0), size=0.1,
                                           color=carla.Color(0, 255, 0), life_time=0.1)

                    # 如果靠近锚点，就搜索下一个锚点
                    if ego_loc.distance(current_target_wp.transform.location) < 3.5:
                        next_wps = current_target_wp.next(4.0)

                        if len(next_wps) == 1:
                            # 只有一条路，照直走
                            current_target_wp = next_wps[0]
                        elif len(next_wps) > 1:
                            # 【核心修正：利用 Right Vector 判别右侧岔道】
                            # 获取当前道路的向右的法向量 (Right Vector)
                            right_vector = current_target_wp.transform.get_right_vector()

                            best_right_wp = None
                            max_right_projection = -999.0

                            for wp in next_wps:
                                # 计算从当前锚点指向候选岔路锚点的方向向量
                                dir_vector = wp.transform.location - current_target_wp.transform.location

                                # 归一化方向向量 (求单位向量)
                                length = math.sqrt(dir_vector.x ** 2 + dir_vector.y ** 2)
                                if length > 0.0:
                                    dir_vector.x /= length
                                    dir_vector.y /= length

                                # 点乘运算：计算候选岔路在车辆右侧方向上的投影长度
                                # 投影值越大（正数最大），说明该路点在几何上越偏向右侧
                                projection = (dir_vector.x * right_vector.x) + (dir_vector.y * right_vector.y)

                                if projection > max_right_projection:
                                    max_right_projection = projection
                                    best_right_wp = wp

                            current_target_wp = best_right_wp
                            ego_has_turned = True  # 标记已经遇到了岔路并执行了右转逻辑

                    # 速度逻辑：遇到岔路右转并完成一次决策后，速度降至 40km/h
                    ego_target_speed = 40.0 if ego_has_turned else 60.0
                    apply_pid_control(ego, pid_ego['lon'], pid_ego['lat'], ego_target_speed,
                                      current_target_wp.transform.location)

            if not v1_active and not ego_active:
                print("所有车辆任务完成或已被清理。")
                break

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