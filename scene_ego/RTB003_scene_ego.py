import sys
import carla
import time
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库 (根据你提供的库内容，版本为V10)
import RoadTailBenchInitV9 as RTB

# ==========================================
# 原始轨迹数据硬编码
# ==========================================
RAW_SUV_PATH_POINTS = [
    (-27.131, 74.994, -94.951), (-27.131, 70.994, -94.951), (-27.131, 66.994, -94.951),
    (-27.131, 62.994, -94.951), (-27.131, 58.994, -94.951), (-27.131, 54.994, -94.951),
    (-27.131, 54.994, -94.951), (-27.131, 54.994, -94.461), (-27.178, 54.282, -93.804),
    (-27.304, 51.802, -91.941), (-27.353, 49.27, -90.812), (-27.333, 46.686, -89.437),
    (-27.311, 44.177, -89.507), (-27.29, 41.678, -89.507), (-27.268, 39.122, -89.507),
    (-27.257, 36.654, -89.786), (-27.247, 34.082, -89.786), (-27.25, 31.513, -90.539),
    (-27.284, 28.911, -91.085), (-27.331, 26.424, -91.085), (-27.379, 23.88, -91.085),
    (-27.44, 21.285, -91.459), (-27.511, 18.807, -91.669), (-27.582, 16.223, -91.364),
    (-27.619, 13.711, -90.571), (-27.639, 11.156, -90.431), (-27.666, 8.556, -90.871),
    (-27.697, 6.081, -90.521), (-27.72, 3.489, -90.521), (-27.743, 0.904, -90.311),
    (-27.756, -1.603, -90.15), (-27.756, -4.106, -90.01), (-27.757, -6.617, -89.94),
    (-27.749, -9.121, -89.709), (-27.731, -11.647, -89.479), (-27.71, -14.218, -89.549),
    (-27.703, -16.74, -90.199), (-27.728, -19.332, -90.649), (-27.777, -21.924, -91.435),
    (-27.849, -24.437, -91.645), (-27.901, -26.918, -90.913), (-27.936, -29.434, -90.774),
    (-27.97, -31.927, -90.703), (-27.995, -34.458, -90.201), (-27.993, -37.042, -90.106),
    (-28.004, -39.579, -90.316), (-28.022, -42.149, -90.526), (-28.055, -44.704, -90.806),
    (-28.076, -47.238, -90.011), (-28.073, -49.83, -89.941), (-28.081, -52.377, -90.442),
    (-28.1, -54.962, -90.372), (-28.113, -57.444, -90.302), (-28.121, -60.054, -90.083),
    (-28.125, -62.556, -90.153), (-28.15, -65.101, -91.001), (-28.196, -67.598, -91.141),
    (-28.243, -70.18, -91.001), (-28.287, -72.701, -91.071), (-28.333, -75.201, -91.071),
    (-28.373, -77.704, -90.854), (-28.41, -80.209, -90.854), (-28.451, -82.786, -90.923),
    (-28.446, -85.255, -89.392), (-28.418, -87.865, -89.392), (-28.397, -90.355, -89.609),
    (-28.396, -92.951, -90.467), (-28.417, -95.502, -90.467), (-28.44, -98.106, -90.536),
    (-28.466, -100.599, -90.676), (-28.507, -103.147, -91.119), (-28.554, -105.654, -90.839),
    (-28.574, -108.24, -90.27), (-28.583, -110.767, -90.2), (-28.591, -113.325, -90.13),
    (-28.598, -115.833, -90.347), (-28.62, -118.399, -90.557), (-28.636, -120.935, -90.06),
    (-28.644, -123.447, -90.273), (-28.66, -125.944, -90.486), (-28.682, -128.448, -90.556),
    (-28.707, -131.033, -90.556), (-28.732, -133.564, -90.556), (-28.757, -136.124, -90.556),
    (-28.781, -138.638, -90.556), (-28.798, -141.214, -90.277), (-28.81, -143.688, -90.277),
    (-28.812, -146.271, -89.862), (-28.795, -148.855, -89.206), (-28.763, -151.187, -89.206),
    (-28.763, -151.187, -89.206), (-28.763, -151.187, -89.206), (-28.763, -151.187, -89.206)
]

RAW_TRUCK_PATH_POINTS = [
    (-30.587, 30.319, -88.672), (-30.538, 27.781, -89.022), (-30.495, 25.183, -89.091),
    (-30.478, 22.646, -89.722), (-30.492, 20.112, -91.793), (-30.587, 17.515, -91.331),
    (-30.619, 15.046, -89.138), (-30.553, 12.533, -88.382), (-30.483, 9.983, -88.592),
    (-30.445, 7.437, -89.736), (-30.496, 4.955, -91.895), (-30.58, 2.38, -91.685),
    (-30.645, -0.186, -91.031), (-30.692, -2.748, -91.031), (-30.736, -5.24, -91.031),
    (-30.784, -7.783, -91.267), (-30.84, -10.316, -91.267), (-30.926, -12.922, -92.191),
    (-31.002, -15.395, -90.973), (-30.963, -17.9, -86.667), (-30.783, -20.483, -86.707),
    (-30.734, -23.025, -90.022), (-30.712, -25.533, -89.404), (-30.739, -28.101, -91.687),
    (-30.82, -30.687, -91.633), (-30.889, -33.261, -91.353), (-30.943, -35.826, -90.979),
    (-30.959, -38.382, -89.716), (-30.924, -40.936, -89.2), (-30.908, -43.475, -90.236),
    (-30.933, -46.062, -90.796), (-30.987, -48.569, -91.406), (-31.006, -51.056, -90.226),
    (-31.004, -53.641, -89.784), (-30.981, -56.167, -89.41), (-30.957, -58.642, -89.771),
    (-30.986, -61.155, -91.204), (-31.037, -63.733, -91.114), (-31.039, -66.239, -89.338),
    (-31.009, -68.777, -89.338), (-31.002, -71.292, -89.999), (-31.02, -73.841, -90.853),
    (-31.057, -76.34, -90.853), (-31.112, -78.9, -91.437), (-31.189, -81.495, -91.735),
    (-31.266, -84.035, -91.735), (-31.339, -86.625, -91.229), (-31.372, -89.175, -90.126),
    (-31.347, -91.751, -88.827), (-31.294, -94.321, -88.827), (-31.243, -96.847, -88.897),
    (-31.214, -99.366, -90.012), (-31.221, -101.88, -90.444), (-31.243, -104.408, -90.514),
    (-31.266, -106.996, -90.514), (-31.29, -109.581, -90.514), (-31.312, -112.076, -90.514),
    (-31.337, -114.666, -90.801), (-31.373, -117.15, -90.871), (-31.412, -119.74, -90.871),
    (-31.447, -122.229, -90.591), (-31.473, -124.8, -90.591), (-31.493, -127.321, -90.162),
    (-31.501, -129.891, -90.162), (-31.507, -132.476, -89.881), (-31.494, -134.958, -89.67),
    (-31.479, -137.484, -89.67), (-31.465, -139.98, -89.67), (-31.453, -142.476, -89.947),
    (-31.465, -144.979, -90.362), (-31.492, -147.562, -91.461), (-31.577, -150.061, -92.633),
    (-31.696, -152.642, -92.633), (-32.067, -155.105, -103.889), (-32.775, -157.502, -107.675),
    (-33.483, -159.725, -107.675), (-33.483, -159.725, -107.675)
]

RAW_SUV_PATH_POINTS = [
    (-27.317, 52.186, -92.098), (-27.317, 52.186, -92.098), (-27.317, 52.186, -92.098),
    (-27.317, 52.186, -92.098), (-27.317, 52.186, -92.098), (-27.319, 52.120, -92.098),
    (-27.413, 49.560, -92.098), (-27.496, 47.022, -91.468), (-27.549, 44.517, -90.838),
    (-27.573, 41.952, -90.418), (-27.604, 37.705, -90.418), (-27.563, 32.637, -89.331),
    (-27.503, 27.520, -89.331), (-27.437, 21.597, -89.691), (-27.417, 15.210, -89.984),
    (-27.489, 8.809, -90.926), (-27.591, 2.504, -90.926), (-27.692, -3.725, -90.926),
    (-27.793, -10.123, -90.856), (-27.888, -16.453, -90.856), (-27.985, -22.941, -90.856),
    (-28.077, -29.105, -90.856), (-28.165, -35.557, -90.646), (-28.204, -41.822, -90.226),
    (-28.229, -48.242, -90.226), (-28.254, -54.586, -90.226), (-28.280, -61.006, -90.226),
    (-28.305, -67.367, -90.226), (-28.354, -73.764, -91.421), (-28.602, -80.084, -92.414),
    (-28.824, -86.387, -91.686), (-28.908, -92.784, -89.860), (-28.871, -99.083, -89.650),
    (-28.805, -105.530, -89.094), (-28.707, -111.743, -89.094), (-28.608, -118.012, -89.094),
    (-28.517, -124.410, -89.764), (-28.504, -130.786, -90.042), (-28.534, -137.162, -90.567),
    (-28.597, -143.485, -90.567), (-28.654, -149.274, -90.567), (-28.654, -149.274, -90.567),
    (-28.654, -149.274, -90.567), (-28.654, -149.274, -90.567), (-28.654, -149.274, -90.567),
    (-28.654, -149.274, -90.567)
]

RAW_EGO_PATH_POINTS = [
    (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743),
    (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743),
    (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743), (-27.081, 34.240, -90.743),
    (-27.081, 34.240, -90.883), (-27.095, 33.838, -92.083), (-27.233, 30.043, -92.013),
    (-27.327, 26.277, -91.160), (-27.406, 22.401, -91.160), (-27.491, 18.199, -91.160),
    (-27.616, 12.020, -91.160), (-27.733, 5.581, -90.740), (-27.807, -0.790, -90.250),
    (-27.819, -7.106, -90.110), (-27.832, -13.483, -90.110), (-27.844, -19.820, -90.110),
    (-27.856, -26.261, -90.110), (-27.930, -33.926, -91.563), (-28.093, -41.535, -90.890),
    (-28.152, -49.144, -90.216), (-28.181, -56.828, -90.216), (-28.210, -64.439, -90.216),
    (-28.249, -71.938, -90.631), (-28.377, -79.658, -90.971), (-28.505, -87.192, -90.971),
    (-28.580, -94.777, -90.014), (-28.597, -102.360, -90.775), (-28.683, -109.963, -90.374),
    (-28.693, -117.631, -89.967), (-28.689, -125.367, -89.967), (-28.729, -132.893, -90.770),
    (-28.833, -140.641, -90.770), (-28.924, -148.238, -90.396), (-28.964, -155.743, -90.270),
    (-28.987, -160.737, -90.270), (-28.987, -160.737, -90.270), (-28.987, -160.737, -90.270),
    (-28.987, -160.737, -90.270), (-28.987, -160.737, -90.270), (-28.987, -160.737, -90.270)
]

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-28.987, -160.737)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.citroen.c3'
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
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 复杂天气直接利用 RTB 构建，保留原始设置
        weather = RTB.build_weather(
            cloudiness=25.0, precipitation=35.0, precipitation_deposits=80.0,
            wind_intensity=20.0, sun_azimuth_angle=95.0, sun_altitude_angle=10.0,
            fog_density=4.0, wetness=100.0, scattering_intensity=2.5,
            mie_scattering_scale=0.1, rayleigh_scattering_scale=0.07
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置")

        # 挂载 TM (保留 Ego 车的 TM 配置需求环境)
        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 过滤原点原地抽搐噪点，然后插值稠密化到 0.5m 标准间隔
        traj_suv = RTB.clean_trajectory(RAW_SUV_PATH_POINTS, min_dist=0.01)
        traj_suv = RTB.interpolate_trajectory(traj_suv, interval=0.5)

        traj_truck = RTB.clean_trajectory(RAW_TRUCK_PATH_POINTS, min_dist=0.01)
        traj_truck = RTB.interpolate_trajectory(traj_truck, interval=0.5)

        traj_ego = RTB.clean_trajectory(RAW_EGO_PATH_POINTS, min_dist=0.01)
        traj_ego = RTB.interpolate_trajectory(traj_ego, interval=0.5)

        # ==========================================
        # 3. 车辆、行人、模型实体安全生成
        # ==========================================
        # 使用 RTB 的智能生成器，防止卡地与穿模
        suv = RTB.spawn_vehicle(world, 'vehicle.tesla.cybertruck',
                                x=traj_suv[0][0], y=traj_suv[0][1], z_offset=1.5)
        actor_list.append(suv)

        truck = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter',
                                  x=traj_truck[0][0], y=traj_truck[0][1], color='255,0,0', z_offset=1.5)
        actor_list.append(truck)

        # Ego 朝向 Y轴负方向 (yaw=-90.0)
        ego = RTB.spawn_vehicle(world, 'vehicle.citroen.c3',
                                x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                color='255,255,0', role_name='ego', z_offset=1.5)
        actor_list.append(ego)

        # ==========================================
        # 4. 车辆PID与行人控制器挂载
        # ==========================================
        # 严格遵守：每辆车必须拥有独立专属的控制实例
        pid_lon_suv = RTB.PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
        pid_lat_suv = RTB.PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
        idx_suv = 0  # 预瞄点索引游标

        pid_lon_truck = RTB.PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.0)
        pid_lat_truck = RTB.PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
        idx_truck = 0

        pid_lon_ego = RTB.PIDLongitudinalController(K_P=1.5, K_I=0.05, K_D=0.1)
        pid_lat_ego = RTB.PIDLateralController(K_P=1.95, K_I=0.05, K_D=0.2)
        idx_ego = 0

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        lights_ego = RTB.VehicleLightManager(ego)
        lights_ego.set_static_lights(low_beam=True)  # 默认开个近光灯

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # SUV剧本：起步 101km/h，到达 y<=15 时减速至 80km/h
        sm_suv = RTB.MultiStageBehaviorMachine(initial_speed=85.0)
        sm_suv.add_stage(trigger_type='y_less', trigger_val=15.0, target_speed=70.0, accel=15.0)

        # Truck剧本：保持 98km/h 匀速
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=80.0)

        # Ego剧本：起步 80km/h，y<=27 时急刹到 40km/h，y<=-30 时再加速到 120km/h
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        sm_ego.add_stage(trigger_type='y_less', trigger_val=27.0, target_speed=40.0, accel=40.0)  # 急刹车设置大加速度
        sm_ego.add_stage(trigger_type='y_less', trigger_val=-30.0, target_speed=120.0, accel=20.0)  # 重新加速

        # ==========================================
        # 7. 预热与初始状态注入
        # ==========================================
        print("[场景预热] 等待 1 秒，让所有车辆落地，物理悬挂稳定中...")
        for _ in range(20):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

        print("[初始注入] 瞬间赋予物理极速，消除起步迟缓...")
        RTB.set_vehicle_initial_speed(suv, target_speed_kmh=85.0)
        RTB.set_vehicle_initial_speed(truck, target_speed_kmh=98.0)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=80.0, yaw_deg=traj_ego[0][2])

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        print("\n🚀 场景正式启动运行...")
        sim_time = 0.0

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # 【核心安全机制】 垂直投影离开道路判定销毁 Actor
            # 注意: 如果出界，函数会自动调用 destroy 且保证不会触发野指针崩溃
            if RTB.check_vehicle_out_of_bounds(suv, carla_map, threshold_dist=6.0, auto_destroy=True): suv = None
            if RTB.check_vehicle_out_of_bounds(truck, carla_map, threshold_dist=6.0, auto_destroy=True): truck = None
            if RTB.check_vehicle_out_of_bounds(ego, carla_map, threshold_dist=6.0, auto_destroy=True): ego = None

            # --- 车辆控制逻辑: SUV ---
            if suv and suv.is_alive:
                # 状态机平滑输出当前目标速度
                speed_suv = sm_suv.tick(suv.get_location(), sim_time, dt)
                # 动态获取预瞄点
                target_wp_suv, idx_suv = RTB.get_target_waypoint(suv.get_location(), traj_suv, idx_suv, speed_suv)
                if target_wp_suv:
                    RTB.apply_pid_control(suv, pid_lon_suv, pid_lat_suv, speed_suv, target_wp_suv)

            # --- 车辆控制逻辑: Truck ---
            if truck and truck.is_alive:
                speed_truck = sm_truck.tick(truck.get_location(), sim_time, dt)
                target_wp_truck, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck,
                                                                     speed_truck)
                if target_wp_truck:
                    RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, speed_truck, target_wp_truck)

            # --- 车辆控制逻辑: Ego ---
            if ego and ego.is_alive:
                # 触发傻瓜式动态自动车灯(刹车灯与转向灯自动亮起)
                lights_ego.auto_update_from_control()

                # 状态机处理急刹与重新加速
                speed_ego = sm_ego.tick(ego.get_location(), sim_time, dt)

                # Ego 的动态循迹模式：调用地图 API 追踪车道中心点 (无固定轨迹)
                target_wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, speed_ego)
                if target_wp_ego:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, speed_ego, target_wp_ego)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[发生异常] {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
