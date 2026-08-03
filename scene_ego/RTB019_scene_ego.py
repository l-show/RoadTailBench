import carla
import time
import math
import numpy as np

# ==========================================
# 辅助函数：赋予车辆指定初始速度
# ==========================================
def set_initial_velocity(vehicle, speed_kmh):
    speed_ms = speed_kmh / 3.6
    yaw = vehicle.get_transform().rotation.yaw
    yaw_rad = math.radians(yaw)
    vel_x = speed_ms * math.cos(yaw_rad)
    vel_y = speed_ms * math.sin(yaw_rad)
    vehicle.set_target_velocity(carla.Vector3D(x=vel_x, y=vel_y, z=0.0))

# ==========================================
# 基础控制算法 (PID)
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.5, K_I=0.05, K_D=0.1, dt=0.05):
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
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

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
# 轨迹数据
# ==========================================
EGO_TRAJECTORY = [
    (-20.928, -80.782, 88.165), (-20.928, -80.782, 88.305), (-20.402, -70.822, 85.029),
    (-20.215, -68.665, 85.029), (-20.213, -68.649, 85.169), (-19.731, -62.17, 85.745),
    (-18.99, -52.202, 85.745), (-18.242, -41.899, 86.027), (-17.682, -31.722, 87.015),
    (-17.123, -21.571, 86.592), (-16.461, -11.425, 86.167), (-15.779, -1.282, 86.097),
    (-15.024, 8.855, 85.397), (-14.161, 18.815, 84.906), (-13.298, 29.112, 85.326),
    (-12.483, 39.075, 85.256), (-11.611, 49.37, 85.116), (-10.797, 59.671, 85.751),
    (-10.044, 69.809, 85.751), (-9.279, 80.112, 85.751), (-8.513, 90.415, 85.751),
    (-7.727, 100.719, 85.471)
]

NPC_TRAJECTORY = [
    (-74.403, -17.073, 18.25), (-74.403, -17.073, 18.25), (-74.403, -17.073, 18.25),
    (-73.896, -16.89, 19.898), (-72.72, -16.475, 19.121), (-71.505, -16.041, 19.621),
    (-70.307, -15.617, 19.338), (-69.085, -15.198, 18.494), (-68.354, -14.953, 18.494),
    (-67.367, -14.623, 18.494), (-65.667, -14.055, 18.494), (-63.415, -13.301, 18.494),
    (-59.722, -12.129, 17.092), (-56.139, -11.028, 17.022), (-52.494, -9.915, 16.952),
    (-48.789, -8.786, 16.882), (-45.198, -7.709, 16.672), (-41.487, -6.597, 16.672),
    (-37.835, -5.504, 16.672), (-34.244, -4.428, 16.672), (-30.592, -3.335, 16.672),
    (-27.001, -2.259, 16.672), (-23.349, -1.165, 16.672), (-19.63, -0.081, 14.111),
    (-15.81, 0.506, 0.695), (-12.084, 0.124, -8.24), (-8.427, -0.685, -17.357),
    (-4.861, -2.029, -24.014), (-1.466, -3.759, -29.767), (1.835, -5.789, -32.401),
    (4.987, -7.821, -34.417), (8.147, -10.063, -35.567), (11.249, -12.281, -35.567),
    (14.402, -14.535, -35.567), (17.452, -16.716, -35.567), (20.503, -18.898, -35.567),
    (23.664, -21.139, -34.056), (26.937, -23.211, -31.764), (30.181, -25.213, -30.98),
    (33.503, -27.207, -30.98), (36.825, -29.202, -30.98), (40.147, -31.196, -30.98),
    (43.363, -33.127, -30.98), (46.625, -35.099, -31.475), (49.876, -37.089, -31.475),
    (53.071, -39.054, -31.828), (56.309, -41.066, -32.04), (59.56, -43.173, -34.247),
    (62.743, -45.38, -34.884), (65.355, -47.187, -34.602), (65.355, -47.187, -34.602),
    (65.355, -47.187, -34.602)
]

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

# ==========================================
# 主程序 (Main Loop)
# ==========================================

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-7.727, 100.719)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.chevrolet.impala'
_RTB_OPT_EGO_ROLE_NAMES = ['ego', 'hero']
_RTB_OPT_GOAL_RADIUS_M = 5.0
_RTB_OPT_GOAL_HITS = 0

def _rtb_opt_is_alive(actor):
    return bool(actor is not None and hasattr(actor, 'is_alive') and actor.is_alive)

def _rtb_opt_iter_actor_values(value, seen=None):
    if seen is None:
        seen = set()
    obj_id = id(value)
    if obj_id in seen:
        return
    seen.add(obj_id)
    if _rtb_opt_is_alive(value) and hasattr(value, 'get_location'):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _rtb_opt_iter_actor_values(item, seen)
    elif isinstance(value, (list, tuple, set)):
        for item in value:
            yield from _rtb_opt_iter_actor_values(item, seen)

def _rtb_opt_actor_matches_ego(actor):
    if not _rtb_opt_is_alive(actor):
        return False
    try:
        role_name = actor.attributes.get('role_name', '')
        if role_name in _RTB_OPT_EGO_ROLE_NAMES:
            return True
    except Exception:
        pass
    try:
        if _RTB_OPT_EGO_TYPE_ID and actor.type_id == _RTB_OPT_EGO_TYPE_ID:
            return True
    except Exception:
        pass
    return False

def _rtb_opt_find_ego(local_vars):
    preferred_names = ('ego', 'ego_vehicle', 'vehicle_ego', 'v3_ego', 'v2_ego', 'agent_ego', 'audi', 'tesla', 'moto', 'truck', 'firetruck')
    for name in preferred_names:
        if name in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[name]):
                if _rtb_opt_actor_matches_ego(actor) or 'ego' in name.lower():
                    return actor
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            if _rtb_opt_actor_matches_ego(actor):
                return actor
    return None

def _rtb_opt_collect_scene_actors(local_vars, world):
    actors = []
    seen = set()

    def add(actor):
        if not _rtb_opt_is_alive(actor):
            return
        try:
            actor_id = actor.id
        except Exception:
            actor_id = id(actor)
        if actor_id in seen:
            return
        seen.add(actor_id)
        actors.append(actor)

    for key in ('actor_list', 'actors', 'vehicles', 'spawned_actors'):
        if key in local_vars:
            for actor in _rtb_opt_iter_actor_values(local_vars[key]):
                add(actor)
    for value in local_vars.values():
        for actor in _rtb_opt_iter_actor_values(value):
            add(actor)
    try:
        world_actors = world.get_actors()
        for pattern in ('vehicle.*', 'walker.*', 'sensor.*', 'controller.*', 'static.prop.*', 'static.trigger.*'):
            for actor in world_actors.filter(pattern):
                add(actor)
    except Exception:
        pass
    return actors

def _rtb_opt_cleanup_scene(local_vars, client, world):
    actors = _rtb_opt_collect_scene_actors(local_vars, world)
    try:
        commands = [carla.command.DestroyActor(actor.id) for actor in actors if _rtb_opt_is_alive(actor)]
        if commands:
            client.apply_batch(commands)
        return
    except Exception:
        pass
    for actor in actors:
        try:
            if _rtb_opt_is_alive(actor):
                actor.destroy()
        except Exception:
            pass

def _rtb_opt_goal_guard(local_vars, client, world):
    global _RTB_OPT_GOAL_HITS
    if _RTB_OPT_EGO_GOAL_XY is None:
        _RTB_OPT_GOAL_HITS = 0
        return False
    ego_actor = _rtb_opt_find_ego(local_vars)
    if not _rtb_opt_is_alive(ego_actor):
        _RTB_OPT_GOAL_HITS = 0
        return False
    try:
        loc = ego_actor.get_location()
        dist = ((loc.x - _RTB_OPT_EGO_GOAL_XY[0]) ** 2 + (loc.y - _RTB_OPT_EGO_GOAL_XY[1]) ** 2) ** 0.5
    except Exception:
        _RTB_OPT_GOAL_HITS = 0
        return False
    if dist <= _RTB_OPT_GOAL_RADIUS_M:
        _RTB_OPT_GOAL_HITS += 1
    else:
        _RTB_OPT_GOAL_HITS = 0
    if _RTB_OPT_GOAL_HITS >= 2:
        print('[RoadTailBench Opt] Ego reached trajectory endpoint; cleaning all scene actors and ending simulation.')
        _rtb_opt_cleanup_scene(local_vars, client, world)
        return True
    return False
# === End RoadTailBench Opt guard ===

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    tm = client.get_trafficmanager(8000)

    weather = carla.WeatherParameters(
        cloudiness=30.0, precipitation=0.0, precipitation_deposits=0.0,
        wind_intensity=10.0, sun_azimuth_angle=255.0, sun_altitude_angle=11.0,
        fog_density=2.0, fog_distance=0.75, fog_falloff=0.0, wetness=0.0,
        scattering_intensity=1.0, mie_scattering_scale=0.05, rayleigh_scattering_scale=0.0831
    )
    world.set_weather(weather)

    dt = 0.05
    actor_list = []
    ego_active, npc_active = False, False

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        # ================= 长尾场景预处理：替换静态垃圾袋 =================
        trash_actors = []
        trash_state = 0  # 0:未触发 1:风吹滚动中 2:结束
        wind_start_time = 0.0

        # 1. 查找并隐藏地图上烘焙的无物理静态垃圾袋 SM_CreasedBox01_Opt
        env_objects = world.get_environment_objects(carla.CityObjectLabel.Any)
        target_bag_names = {"SM_TrasdhBag_Opt", "SM_TrasdhBag_Opt4", "SM_TrasdhBag_Opt5"}
        bags_to_hide = [obj.id for obj in env_objects if obj.name in target_bag_names]

        if bags_to_hide:
            world.enable_environment_objects(bags_to_hide, False)
            print(f"检测到 {len(bags_to_hide)} 个静态垃圾袋模型，已隐藏。")

        # 将 UE 坐标 (cm) 转为 CARLA 坐标 (m)
        base_trash_loc = carla.Location(x=-24.7, y=-34.6, z=0.2)

        # 2. 挑选一个动态的道具蓝图代替
        # 如果你自己在UE中打包装了名为 'static.prop.trashbag' 的蓝图，可以替换。这里使用内置物品降级。
        prop_bp = None
        for bp in bp_lib.filter('static.prop.*'):
            if 'bag' in bp.id.lower() or 'trash' in bp.id.lower() or 'box' in bp.id.lower():
                prop_bp = bp
                break
        if not prop_bp: prop_bp = bp_lib.filter('static.prop.*')[0]

        # 3. 在原位置附近生成3个具有物理特性的动态垃圾袋
        for i in range(3):
            spawn_loc = carla.Location(
                x=base_trash_loc.x + np.random.uniform(-0.5, 0.5),
                y=base_trash_loc.y + np.random.uniform(-0.5, 0.5),
                z=base_trash_loc.z + 0.5  # 抬高0.5米防止穿模掉出地图
            )
            bag_actor = world.try_spawn_actor(prop_bp,
                                              carla.Transform(spawn_loc, carla.Rotation(yaw=np.random.uniform(0, 360))))
            if bag_actor:
                bag_actor.set_simulate_physics(True)
                trash_actors.append(bag_actor)
                actor_list.append(bag_actor)

        print(f"成功在目标区域生成了 {len(trash_actors)} 个可滚动的动态垃圾袋。")

        # ================= 车辆初始化 =================
        pid_ego = {'lon': PIDLongitudinalController(K_P=1.5, dt=dt), 'lat': PIDLateralController(dt=dt)}
        pid_npc = {'lon': PIDLongitudinalController(K_P=1.5, dt=dt), 'lat': PIDLateralController(dt=dt)}

        bp_ego = bp_lib.find('vehicle.chevrolet.impala')
        bp_ego.set_attribute('role_name', 'ego')
        ego_start_x, ego_start_y, ego_start_yaw = EGO_TRAJECTORY[0]
        ego_loc = carla.Location(x=ego_start_x, y=ego_start_y, z=0.5)
        ego_loc.z = carla_map.get_waypoint(ego_loc).transform.location.z + 0.5
        ego_vehicle = world.try_spawn_actor(bp_ego, carla.Transform(ego_loc, carla.Rotation(yaw=ego_start_yaw)))
        if ego_vehicle:
            actor_list.append(ego_vehicle)
            ego_active = True

        bp_npc = bp_lib.find('vehicle.mercedes.sprinter')
        npc_start_x, npc_start_y, npc_start_yaw = NPC_TRAJECTORY[0]
        npc_loc = carla.Location(x=npc_start_x, y=npc_start_y, z=0.5)
        npc_loc.z = carla_map.get_waypoint(npc_loc).transform.location.z + 0.5
        npc_vehicle = world.try_spawn_actor(bp_npc, carla.Transform(npc_loc, carla.Rotation(yaw=npc_start_yaw)))
        if npc_vehicle:
            actor_list.append(npc_vehicle)
            npc_active = True

        # 让物理引擎预热贴地
        for _ in range(10): world.tick()

        # 赋予初始物理速度
        if ego_active: set_initial_velocity(ego_vehicle, speed_kmh=10.0)
        if npc_active: set_initial_velocity(npc_vehicle, speed_kmh=30.0)

        print("\n仿真正式开始！车辆将按照指定轨迹行驶...")
        ego_traj_idx, npc_traj_idx = 0, 0

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            # ==========================
            # 长尾场景：触发垃圾袋滚动逻辑
            # ==========================
            if trash_state == 0 and ego_active and ego_vehicle.is_alive:
                dist_to_trash = ego_vehicle.get_location().distance(base_trash_loc)
                # 当主车距离垃圾袋位置小于 25 米时触发（可按需调整）
                if dist_to_trash < 25.0:
                    trash_state = 1
                    wind_start_time = sim_time
                    print("\n[⚠️长尾场景触发] 一阵大风吹过，垃圾袋被吹向路中间！")

            elif trash_state == 1:
                # 持续吹风 2.5 秒，制造翻滚感
                if (sim_time - wind_start_time) < 2.5:
                    # 计算目标方向: 终点 - 起点
                    # 终点: x=-13.838, y=-35.219 (路中间)
                    # 起点: x=-24.7, y=-34.6 (路边)
                    dx = -13.838 - (-24.7)
                    dy = -35.219 - (-34.6)
                    norm_vec = math.sqrt(dx ** 2 + dy ** 2)
                    nx, ny = dx / norm_vec, dy / norm_vec

                    for bag in trash_actors:
                        # 加入微小的随机偏差，让3个袋子滚得不一样
                        rx = nx + np.random.uniform(-0.1, 0.1)
                        ry = ny + np.random.uniform(-0.1, 0.1)

                        # 向特定方向施加力 (force) 和向上的微力让它蹦跶
                        # 注意：力的大小(这里设为300)与物品蓝图的质量（Mass）有关，如果吹不动可以加大到 1000 甚至 5000
                        force_mag = 300.0
                        bag.add_force(
                            carla.Vector3D(x=rx * force_mag, y=ry * force_mag, z=np.random.uniform(10.0, 30.0)))

                        # 施加扭矩 (Torque) 让它真的在"滚"而不是平移
                        bag.add_torque(carla.Vector3D(
                            x=np.random.uniform(-50, 50),
                            y=np.random.uniform(-50, 50),
                            z=np.random.uniform(-50, 50)
                        ))
                else:
                    trash_state = 2
                    print("[模拟结束] 风停了，垃圾袋靠惯性滑行。")

            # ==========================
            # Ego 车：PID 循迹
            # ==========================
            if ego_active and ego_vehicle.is_alive:
                if check_and_handle_out_of_bounds(ego_vehicle, carla_map):
                    ego_active = False
                elif ego_traj_idx < len(EGO_TRAJECTORY):
                    tx, ty, tyaw = EGO_TRAJECTORY[ego_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=ego_vehicle.get_location().z)
                    if ego_vehicle.get_location().distance(target_loc) < 3.0 and ego_traj_idx < len(EGO_TRAJECTORY) - 1:
                        ego_traj_idx += 1
                    apply_pid_control(ego_vehicle, pid_ego['lon'], pid_ego['lat'], 70.0, target_loc)
                else:
                    ego_vehicle.apply_control(carla.VehicleControl(brake=1.0))
                    ego_active = False

            # ==========================
            # NPC 车：PID 循迹
            # ==========================
            if npc_active and npc_vehicle.is_alive:
                if check_and_handle_out_of_bounds(npc_vehicle, carla_map):
                    npc_active = False
                elif npc_traj_idx < len(NPC_TRAJECTORY):
                    tx, ty, tyaw = NPC_TRAJECTORY[npc_traj_idx]
                    target_loc = carla.Location(x=tx, y=ty, z=npc_vehicle.get_location().z)
                    if npc_vehicle.get_location().distance(target_loc) < 2.0 and npc_traj_idx < len(NPC_TRAJECTORY) - 1:
                        npc_traj_idx += 1
                    apply_pid_control(npc_vehicle, pid_npc['lon'], pid_npc['lat'], 50.0, target_loc)
                else:
                    npc_vehicle.apply_control(carla.VehicleControl(brake=1.0))
                    npc_active = False

            # 帧率同步控制
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

            if not ego_active and not npc_active:
                print("\n所有车辆轨迹结束，停止仿真。")
                break

    except KeyboardInterrupt:
        print("\n键盘中断，终止运行。")
    finally:
        print("\n清理环境并恢复异步设置...")
        # 恢复被隐藏的静态地图物体
        if 'bags_to_hide' in locals() and bags_to_hide:
            world.enable_environment_objects(bags_to_hide, True)

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