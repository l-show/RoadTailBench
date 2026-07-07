import carla
import time
import math
import numpy as np


# ==========================================
# PID 控制器类 (保持不变)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
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
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, waypoint_location, vehicle_transform):
        v_begin, v_forward = vehicle_transform.location, vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint_location.x - v_begin.x, waypoint_location.y - v_begin.y, 0.0])
        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1: return 0.0
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        if np.cross(v_vec, w_vec)[2] < 0: _dot *= -1.0
        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


# ==========================================
# 辅助函数 (保持不变)
# ==========================================
def get_target_waypoint(vehicle_loc, path_transforms, lookahead_dist=5.0):
    min_dist, closest_index = float('inf'), 0
    for i, t in enumerate(path_transforms):
        dist = vehicle_loc.distance(t.location)
        if dist < min_dist: min_dist, closest_index = dist, i
    target_index = closest_index
    current_dist = 0.0
    for i in range(closest_index, len(path_transforms) - 1):
        current_dist += path_transforms[i].location.distance(path_transforms[i + 1].location)
        target_index = i + 1
        if current_dist > lookahead_dist: break
    return path_transforms[target_index].location


def clean_and_convert_path(raw_path_points):
    path_transforms = []
    for i in range(len(raw_path_points)):
        p, yaw = raw_path_points[i], 0.0
        for j in range(i + 1, len(raw_path_points)):
            dx, dy = raw_path_points[j][0] - p[0], raw_path_points[j][1] - p[1]
            if math.sqrt(dx ** 2 + dy ** 2) > 0.1:
                yaw = math.degrees(math.atan2(dy, dx));
                break
        if yaw == 0.0 and i > 0: yaw = path_transforms[-1].rotation.yaw
        path_transforms.append(carla.Transform(carla.Location(x=p[0], y=p[1], z=p[2] + 0.5), carla.Rotation(yaw=yaw)))
    return path_transforms


RAW_PATH_left = [(190.690, -2.997, 3.907), (189.114, -2.772, 4.060), (186.339, -2.516, 4.297), (183.481, -2.417, 4.456),
                 (180.901, -2.423, 4.485), (178.057, -2.555, 4.504), (175.338, -2.735, 4.534), (172.579, -3.018, 4.591),
                 (169.806, -3.458, 4.627), (167.154, -4.004, 4.643), (166.825, -4.056, 4.647), (162.495, -4.960, 4.679),
                 (159.228, -6.101, 4.612), (156.661, -7.224, 4.594), (154.214, -8.302, 4.648), (151.095, -9.657, 4.503),
                 (147.657, -11.134, 4.421), (145.197, -12.183, 4.397), (142.906, -13.057, 4.394),
                 (140.291, -13.960, 4.391), (137.958, -14.542, 4.375), (135.630, -15.093, 4.364),
                 (133.096, -15.448, 4.367), (130.516, -15.686, 4.377), (128.147, -15.584, 4.393),
                 (125.697, -15.086, 4.420), (123.318, -14.321, 4.463), (120.822, -13.382, 4.512),
                 (117.856, -12.274, 4.555), (115.115, -11.054, 4.619), (114.061, -10.555, 4.647),
                 (112.261, -9.686, 4.696), (109.898, -8.448, 4.765), (107.145, -6.886, 4.830), (104.599, -5.299, 4.881),
                 (102.204, -3.783, 4.940), (99.463, -2.197, 4.978), (96.304, -0.334, 5.008), (91.891, 2.360, 5.025),
                 (89.188, 4.010, 5.035), (85.774, 6.095, 5.048), (82.929, 7.832, 5.059), (79.087, 10.177, 5.074),
                 (75.389, 12.435, 5.088), (70.467, 15.242, 5.057), (66.177, 17.812, 5.029), (61.884, 20.374, 4.997),
                 (59.307, 21.910, 4.977), (56.300, 23.701, 4.953), (50.924, 26.712, 4.919), (45.256, 29.138, 4.906),
                 (42.743, 30.030, 4.875), (37.647, 31.601, 4.836), (36.032, 32.011, 4.817)]
RAW_ego_TRANSFORMS = clean_and_convert_path(RAW_PATH_left)


# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    actor_list = []
    flyby_triggered = False
    active_props = []
    props_rel_x = []
    props_rel_y = []
    props_rel_z = []
    is_sticky = []

    trigger_loc = carla.Location(x=117.856, y=-12.274, z=4.555)

    weather = carla.WeatherParameters(
        cloudiness=20.0, precipitation=10.0, precipitation_deposits=45.0, wind_intensity=20.0,
        sun_azimuth_angle=0, sun_altitude_angle=0, fog_density=10.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=50.0, scattering_intensity=11.0, mie_scattering_scale=0.03,
        rayleigh_scattering_scale=0.0331, dust_storm=0.0
    )
    world.set_weather(weather)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)

        # 2. 生成 Ego 车辆
        bp_ego = bp_lib.find('vehicle.tesla.model3')
        bp_ego.set_attribute('role_name', 'ego')
        vehicle_ego = world.try_spawn_actor(bp_ego, RAW_ego_TRANSFORMS[0])
        if vehicle_ego:
            actor_list.append(vehicle_ego)
            lon_ctrl = PIDLongitudinalController()
            lat_ctrl = PIDLateralController()
            initial_lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position | \
                             carla.VehicleLightState.Fog | carla.VehicleLightState.Interior
            vehicle_ego.set_light_state(carla.VehicleLightState(initial_lights))
            print("Ego 车辆已生成。")

        # 3. 生成 Auto 车辆
        start_auto = carla.Transform(carla.Location(x=-87.936, y=-35.091, z=5.138), carla.Rotation(yaw=24.305))
        vehicle_auto = world.try_spawn_actor(bp_lib.find('vehicle.audi.tt'), start_auto)
        if vehicle_auto:
            actor_list.append(vehicle_auto)
            vehicle_auto.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(vehicle_auto, -180.0)
            vehicle_auto.set_light_state(
                carla.VehicleLightState(carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position))

        # ==========================================
        # 4. 主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()

            if vehicle_ego:
                tf_ego = vehicle_ego.get_transform()
                dist_to_trigger = tf_ego.location.distance(trigger_loc)

                # --- [优化] 提前计算主车速度 (单位: m/s) ---
                vel = vehicle_ego.get_velocity()
                curr_spd_mps = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                # --- [优化] 到达指定坐标(x=62)，解除摄像头遮挡 ---
                # 因为行驶轨迹的X坐标是在不断变小的(向负方向开)，所以判断是 <= 62.0
                if tf_ego.location.x <= 62.0:
                    for j in range(len(is_sticky)):
                        if is_sticky[j]:
                            is_sticky[j] = False  # 取消所有袋子的粘滞状态
                            print("到达X=62节点，解除摄像头遮挡干扰！")

                # --- 触发生成干扰物 ---
                if not flyby_triggered and dist_to_trigger < 5.0:
                    spawn_offsets = [
                        (30.0, 0.0, 2.20),  # 前4个针对摄像头
                        (31.5, 0.25, 2.35),
                        (30.5, -0.25, 2.05),
                        (32.5, -0.25, 2.35),
                        (25.0, 1.5, 2.20),  # 后4个正常飞过
                        (28.0, -1.8, 1.80),
                        (35.0, 0.5, 3.50),
                        (40.0, -0.5, 0.50)
                    ]
                    sticky_mask = [True, True, True, True, False, False, False, False]
                    for i, (ox, oy, oz) in enumerate(spawn_offsets):
                        prop = world.try_spawn_actor(bp_lib.find('static.prop.shoppingbag'),
                                                     carla.Transform(carla.Location(x=ox, y=oy, z=oz)),
                                                     attach_to=vehicle_ego)
                        if prop:
                            prop.set_collisions(False)
                            prop.set_simulate_physics(False)
                            active_props.append(prop)
                            props_rel_x.append(ox)
                            props_rel_y.append(oy)
                            props_rel_z.append(oz)
                            is_sticky.append(sticky_mask[i])
                            actor_list.append(prop)
                    flyby_triggered = True

                # --- 干扰物运动逻辑 ---
                for i in range(len(active_props) - 1, -1, -1):
                    prop = active_props[i]

                    # 1. 如果还在粘滞状态，且正好抵达到前置摄像头位置
                    if is_sticky[i] and props_rel_x[i] <= 2.8:
                        props_rel_x[i] = 2.8  # 锁死相对X坐标
                        shake = 0.005 * math.sin(time.time() * 20 + i)  # 保持风吹微抖动
                        new_tf = carla.Transform(
                            carla.Location(x=2.8, y=props_rel_y[i] + shake, z=props_rel_z[i] + shake),
                            carla.Rotation(pitch=0, yaw=90, roll=0)  # 宽面正对摄像头
                        )
                    else:
                        # 2. 飞行状态（还没撞上，或者已经被解除了粘滞）
                        # [优化] 根据主车的实际车速动态计算相对运动速度，使视觉冲击更真实
                        # curr_spd_mps * 0.05 恰好是主车每帧驶过的距离
                        # 我们加上一定的常数(基础风速)，确保车停下时袋子也会动
                        if is_sticky[i]:
                            # 迎面飞来：车速 + 相对风速 (使得粘滞必定能命中)
                            m_speed = (curr_spd_mps * 0.05) + 0.3
                        else:
                            # 飞走(或路过)：加上向后吹的物理风力，使其迅速被吹落
                            m_speed = (curr_spd_mps * 0.05) + 0.8

                        props_rel_x[i] -= m_speed

                        # 飞行伴随随机疯狂翻滚
                        new_tf = carla.Transform(carla.Location(x=props_rel_x[i], y=props_rel_y[i], z=props_rel_z[i]),
                                                 carla.Rotation(pitch=time.time() * 200, yaw=time.time() * 150,
                                                                roll=time.time() * 120))
                    try:
                        prop.set_transform(new_tf)
                    except:
                        pass

                    # 3. 如果是非粘滞状态(路过的，或解绑后的)，并且飞到了车后方 20 米，进行销毁清理
                    if not is_sticky[i] and props_rel_x[i] < -20.0:
                        prop.destroy()
                        active_props.pop(i)
                        props_rel_x.pop(i)
                        props_rel_y.pop(i)
                        props_rel_z.pop(i)
                        is_sticky.pop(i)

                # --- Ego PID 循迹控制逻辑 ---
                # curr_spd 已统一换算为 km/h 用于 PID
                curr_spd_kmh = 3.6 * curr_spd_mps
                target_wp = get_target_waypoint(tf_ego.location, RAW_ego_TRANSFORMS)
                throt = lon_ctrl.run_step(70.0 / 3.6, curr_spd_kmh / 3.6)  # 目标速度70km/h
                steer = lat_ctrl.run_step(target_wp, tf_ego)

                control = carla.VehicleControl(steer=steer)
                if throt >= 0:
                    control.throttle, control.brake = throt, 0.0
                else:
                    control.throttle, control.brake = 0.0, abs(throt)
                vehicle_ego.apply_control(control)

                # --- 动态灯光系统逻辑 ---
                current_lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position | \
                                 carla.VehicleLightState.Fog | carla.VehicleLightState.Interior
                if control.brake > 0.1:
                    current_lights |= carla.VehicleLightState.Brake
                vehicle_ego.set_light_state(carla.VehicleLightState(current_lights))

            comp_time = time.time() - start_time
            if comp_time < 0.05: time.sleep(0.05 - comp_time)

    except Exception as e:
        print(f"异常: {e}")
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if actor_list: client.apply_batch([carla.command.DestroyActor(a) for a in actor_list if a.is_alive])
        print("清理完成。")


if __name__ == '__main__':
    main()
