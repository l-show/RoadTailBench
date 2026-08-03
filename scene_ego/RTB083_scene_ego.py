import carla
import time
import math
import os
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

def build_ego_transforms(carla_map, raw_path_points):
    path_transforms = []
    for x, y, yaw in raw_path_points:
        loc = carla.Location(x=x, y=y, z=0.5)
        try:
            waypoint = carla_map.get_waypoint(loc)
            if waypoint:
                loc.z = waypoint.transform.location.z + 0.5
        except Exception:
            pass
        path_transforms.append(carla.Transform(loc, carla.Rotation(yaw=yaw)))
    return path_transforms

RAW_EGO_TRAJECTORY = [
    (183.292, -2.912, -174.977), (182.003, -3.025, -174.977), (179.573, -3.239, -174.977), (177.091, -3.461, -174.431),
    (174.560, -3.722, -173.450), (172.132, -4.022, -172.783), (169.804, -4.333, -171.765), (167.390, -4.742, -169.026),
    (164.944, -5.257, -167.426), (162.578, -5.886, -163.226), (160.235, -6.602, -161.803), (157.919, -7.401, -160.716),
    (155.606, -8.210, -160.716), (153.293, -9.019, -160.716), (151.033, -9.826, -158.304), (148.906, -10.823, -154.315),
    (146.789, -11.841, -154.315), (144.624, -12.875, -160.742), (142.196, -13.467, -167.508), (139.752, -13.992, -169.849),
    (137.335, -14.395, -171.057), (134.911, -14.748, -172.633), (132.476, -15.013, -175.887), (130.028, -15.122, -178.324),
    (127.529, -15.108, 177.753), (125.035, -14.947, 174.552), (122.564, -14.572, 168.117), (120.088, -13.968, 163.665),
    (117.683, -13.124, 157.871), (115.301, -12.081, 153.906), (113.034, -10.915, 152.371), (110.919, -9.781, 151.322),
    (108.770, -8.605, 151.322), (106.551, -7.250, 147.275), (104.406, -5.871, 147.275), (102.219, -4.466, 147.275),
    (99.990, -3.033, 147.275), (97.761, -1.601, 147.275), (95.446, -0.116, 147.415), (93.166, 1.329, 147.991),
    (90.830, 2.781, 148.411), (88.487, 4.220, 148.620), (86.096, 5.678, 148.620), (83.662, 7.160, 148.690),
    (81.138, 8.688, 148.830), (78.614, 10.215, 148.830), (76.090, 11.741, 148.830), (73.480, 13.320, 148.830),
    (70.821, 15.008, 145.700), (68.048, 16.975, 143.810), (65.183, 19.153, 141.313), (62.013, 21.754, 140.388),
    (58.689, 24.481, 141.664), (55.329, 26.828, 148.388), (51.701, 28.619, 156.526), (48.070, 30.037, 161.021),
    (44.461, 31.053, 166.196), (40.836, 31.788, 171.209), (37.272, 32.290, 172.038), (33.648, 32.709, 176.773),
    (29.953, 32.875, 179.005), (26.404, 32.928, 179.145), (22.705, 32.951, -179.186), (18.956, 32.862, -178.276),
    (15.213, 32.645, -175.263), (11.489, 32.213, -171.885), (7.847, 31.563, -168.292), (4.294, 30.731, -164.688),
    (1.083, 29.778, -162.516), (-2.431, 28.619, -160.737), (-5.778, 27.298, -154.840), (-7.213, 26.592, -153.662),
    (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662),
    (-7.213, 26.592, -153.662), (-7.213, 26.592, -153.662),
]

def is_actor_alive(actor):
    return bool(actor is not None and getattr(actor, "is_alive", False))

def find_existing_ego(world):
    actors = world.get_actors().filter("vehicle.*")
    for role_name in ("ego", "hero"):
        for actor in actors:
            if actor.attributes.get("role_name") == role_name:
                return actor

    start_x, start_y, _ = RAW_EGO_TRAJECTORY[0]
    start_loc = carla.Location(x=start_x, y=start_y, z=0.5)
    model3_matches = []
    for actor in actors:
        if actor.type_id != "vehicle.tesla.model3":
            continue
        try:
            model3_matches.append((actor.get_location().distance(start_loc), actor))
        except RuntimeError:
            continue
    model3_matches.sort(key=lambda item: item[0])
    if model3_matches and model3_matches[0][0] <= 8.0:
        return model3_matches[0][1]
    return None

def cleanup_actors(client, actors):
    commands = []
    seen = set()
    for actor in actors:
        if not is_actor_alive(actor):
            continue
        actor_id = getattr(actor, "id", id(actor))
        if actor_id in seen:
            continue
        seen.add(actor_id)
        commands.append(carla.command.DestroyActor(actor_id))
    if commands:
        client.apply_batch(commands)

# ==========================================
# 主程序
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    ego_transforms = build_ego_transforms(carla_map, RAW_EGO_TRAJECTORY)

    actor_list = []
    spawned_scene_ego = False
    vehicle_ego = None
    lon_ctrl = None
    lat_ctrl = None
    flyby_triggered = False
    active_props = []
    props_rel_x = []
    props_rel_y = []
    props_rel_z = []
    is_sticky = []
    goal_reached_ticks = 0
    stop_requested = False

    trigger_loc = carla.Location(x=117.856, y=-12.274, z=4.555)
    goal_x, goal_y, _ = RAW_EGO_TRAJECTORY[-1]
    ego_mode = os.environ.get("LEADERBOARD_EGO_MODE") or os.environ.get("ROADTAILBENCH_EGO_MODE") or "scene_ego"
    use_external_ego = ego_mode in ("agent_ego", "external_ego")

    weather = carla.WeatherParameters(
        cloudiness=40.0, precipitation=100.0, precipitation_deposits=100.0, wind_intensity=100.0,
        sun_azimuth_angle=90, sun_altitude_angle=10, fog_density=10.0, fog_distance=0.75,
        fog_falloff=0.1, wetness=50.0, scattering_intensity=11.0, mie_scattering_scale=0.13,
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
        if use_external_ego:
            vehicle_ego = find_existing_ego(world)
            if vehicle_ego:
                print("[RTB083] Using external/agent ego actor.")
            else:
                print("[RTB083] Waiting for external/agent ego actor.")
        else:
            ego_bp = bp_lib.find('vehicle.tesla.model3')
            if ego_bp.has_attribute('role_name'):
                ego_bp.set_attribute('role_name', 'ego')
            vehicle_ego = world.try_spawn_actor(ego_bp, ego_transforms[0])
            spawned_scene_ego = bool(vehicle_ego)

        if spawned_scene_ego:
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

            if use_external_ego and not is_actor_alive(vehicle_ego):
                vehicle_ego = find_existing_ego(world)

            if is_actor_alive(vehicle_ego):
                tf_ego = vehicle_ego.get_transform()
                dist_to_trigger = tf_ego.location.distance(trigger_loc)
                dist_to_goal = math.sqrt((tf_ego.location.x - goal_x) ** 2 + (tf_ego.location.y - goal_y) ** 2)
                if dist_to_goal <= 5.0:
                    goal_reached_ticks += 1
                else:
                    goal_reached_ticks = 0
                if goal_reached_ticks >= 2:
                    print("[RTB083] Ego reached trajectory endpoint; destroying scene actors and ending.")
                    cleanup_actors(client, actor_list + [vehicle_ego])
                    stop_requested = True
                    break

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
                if spawned_scene_ego:
                    curr_spd_kmh = 3.6 * curr_spd_mps
                    target_wp = get_target_waypoint(tf_ego.location, ego_transforms)
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
        if actor_list and not stop_requested:
            cleanup_actors(client, actor_list)
        print("清理完成。")

if __name__ == '__main__':
    main()
