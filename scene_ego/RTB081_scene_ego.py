import sys
import carla
import time
import math

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 2. 轨迹数据硬编码 (已去除纯文本的表头)
# ==========================================
STR_V1_IMPALA = """
-140.179 -0.65 1.431
-140.179 -0.65 1.431
-139.888 -0.645 1.011
-131.006 -0.329 3.309
-122.133 0.296 4.221
-113.251 0.783 2.723
-104.36 1.051 1.286
-95.466 1.25 1.286
-86.572 1.434 0.936
-77.677 1.571 0.866
-68.782 1.685 0.586
-59.886 1.731 0.164
-50.99 1.757 0.164
-42.095 1.762 -0.466
-33.2 1.689 -0.466
-24.304 1.641 -0.256
-15.408 1.633 0.304
-6.512 1.695 0.654
2.377 1.998 3.814
11.222 2.921 8.63
19.97 4.531 11.369
28.661 6.424 13.934
37.261 8.7 15.213
45.795 11.193 21.743
53.112 15.861 46.394
57.43 23.568 69.158
60.369 31.964 70.865
63.333 40.506 70.865
66.249 48.91 70.865
67.763 53.786 76.289
67.763 53.786 76.289
67.763 53.786 76.289
"""

STR_V2_MODEL3 = """
-32.3 53.831 -92.29
-32.3 53.831 -92.29
-32.3 53.831 -92.29
-32.32 53.144 -91.52
-32.368 49.332 -90.418
-32.378 45.583 -90.068
-32.374 41.771 -89.718
-32.339 37.963 -89.088
-32.263 34.155 -88.449
-32.152 30.413 -87.889
-31.963 26.626 -86.118
-31.677 22.881 -85.278
-31.35 19.157 -84.787
-31 15.385 -84.647
-30.608 11.747 -81.782
-29.597 8.193 -64.518
-27.708 4.978 -55.787
-25.265 2.196 -40.523
-22.062 0.17 -21.984
-18.363 -0.478 -2.47
-14.615 -0.314 7.106
-10.875 0.28 9.072
-7.103 0.87 8.72
-3.391 1.362 0.783
0.419 1.403 2.794
4.155 1.714 6.111
7.936 2.192 7.909
11.698 2.806 10.104
15.513 3.486 10.104
19.186 4.24 13.749
22.889 5.146 13.749
26.592 6.052 13.749
31.571 7.27 13.749
37.743 8.781 13.749
43.811 10.276 15.053
49.728 12.548 30.429
54.547 16.635 51.514
57.762 22.096 66.054
60.226 27.954 68.04
62.494 33.888 70.619
64.561 39.787 70.689
66.663 45.783 70.689
68.764 51.78 70.689
70.101 57.86 85.043
70.469 64.099 88.147
70.491 70.452 91.789
70.046 76.79 96.248
69.071 82.96 101.807
67.478 89.11 106.405
65.493 95.036 110.851
63.198 100.961 111.2
60.944 106.902 110.568
58.729 112.856 110.147
56.792 118.137 110.147
56.792 118.137 110.147
56.792 118.137 110.147
"""

STR_EGO_TT = """
62.101 22.03 -117.037
62.101 22.03 -117.037
62.101 22.03 -117.037
62.101 22.03 -117.037
60.115 18.18 -117.672
54.073 10.316 -142.462
44.742 6.519 -164.883
34.884 4.036 -166.661
25.154 1.728 -166.661
15.257 -0.608 -168.173
5.159 -1.668 -177.693
-4.999 -2.078 -177.693
-10.39 -2.295 -178.272
-10.39 -2.295 179.987
-11.223 -2.295 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 179.987
-13.723 -2.294 -179.943
-18.619 -2.299 -179.943
-24.973 -2.305 -179.943
-31.327 -2.363 -179.033
-37.679 -2.504 -178.683
-44.031 -2.65 -178.683
-50.385 -2.748 -179.453
-56.739 -2.788 -179.803
-63.093 -2.805 -179.873
-69.447 -2.819 -179.873
-75.801 -2.829 179.917
-82.051 -2.82 179.917
-88.405 -2.81 179.917
-94.76 -2.801 179.917
-101.114 -2.792 179.917
-107.469 -2.804 -179.733
"""

STR_V4_TRUCK = """
74.394 62.17 -104.896
74.394 62.17 -104.896
74.276 61.728 -104.896
73.633 59.315 -104.896
72.99 56.814 -104.121
72.37 54.349 -104.121
71.589 51.174 -103.702
70.004 45.021 -106.574
68.08 38.966 -109.232
65.914 32.992 -110.591
63.679 27.044 -110.591
62.038 22.675 -110.591
60.968 19.828 -110.806
60.466 18.66 -113.636
59.715 16.988 -117.035
58.31 14.874 -127.412
56.669 12.936 -135.818
54.757 11.329 -143.948
52.585 9.933 -151.387
50.285 8.853 -155.96
47.999 7.839 -158.025
45.56 7.127 -165.067
43.104 6.472 -165.067
40.649 5.817 -165.067
37.81 5.06 -165.067
32.899 3.75 -165.067
27.987 2.44 -165.067
23.068 1.16 -167.388
18.1 0.077 -167.712
13.114 -0.912 -171.21
8.074 -1.579 -172.997
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
6.833 -1.727 -175.147
5.172 -1.868 -175.147
0.098 -2.151 -178.83
-4.984 -2.254 -178.83
-8.817 -2.333 -178.83
-8.817 -2.333 -178.83
-8.817 -2.333 -178.83
-8.817 -2.333 -178.83
-8.817 -2.333 -178.83
-13.318 -2.425 -178.83
-18.401 -2.48 -179.605
-23.401 -2.513 -179.745
-28.484 -2.535 -179.815
-33.567 -2.537 179.905
-38.651 -2.529 179.905
-43.734 -2.531 -179.955
-48.817 -2.535 -179.955
-53.9 -2.539 -179.955
-58.983 -2.578 -178.542
-64.064 -2.721 -178.816
-69.147 -2.775 -179.516
-74.23 -2.836 -179.303
-82.251 -2.934 -179.303
-91.146 -3.012 -179.796
-100.042 -3.044 -179.796
-108.646 -3.073 -179.866
-108.646 -3.073 -179.866
-108.646 -3.073 -179.866
-108.646 -3.073 -179.866
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-107.469 ,-2.804)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.audi.tt'
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
        dt = 0.05
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照用户截图要求配置暗夜积水暴风雨天气
        weather_kwargs = {
            'cloudiness': 5.0,
            'precipitation': 35.0,  # Rain: 35
            'precipitation_deposits': 60.0,  # Puddles: 60
            'wind_intensity': 60.0,  # Wind: 60
            'sun_azimuth_angle': -1.0,  # Azim: -1
            'sun_altitude_angle': -90.0,  # Alt: -90 (Pitch Black Night)
            'fog_density': 44.0,
            'fog_distance': 75.0,
            'fog_falloff': 1.0,
            'wetness': 75.0,  # Wetness: 75
            'scattering_intensity': 1.0,
            'mie_scattering_scale': 0.0300,
            'rayleigh_scattering_scale': 0.0331,
            'dust_storm': 0.0
        }
        RTB.set_static_weather(world, **weather_kwargs)
        print("[场景配置] 暗夜暴雨环境系统已按要求设置完成")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (间隔0.5米)
        # ==========================================
        raw_v1 = RTB.parse_string_trajectory(STR_V1_IMPALA, min_dist=0.1)
        traj_v1 = RTB.interpolate_trajectory(raw_v1, interval=0.5)

        raw_v2 = RTB.parse_string_trajectory(STR_V2_MODEL3, min_dist=0.1)
        traj_v2 = RTB.interpolate_trajectory(raw_v2, interval=0.5)

        raw_ego = RTB.parse_string_trajectory(STR_EGO_TT, min_dist=0.1)
        traj_ego = RTB.interpolate_trajectory(raw_ego, interval=0.5)

        raw_v4 = RTB.parse_string_trajectory(STR_V4_TRUCK, min_dist=0.1)
        traj_v4 = RTB.interpolate_trajectory(raw_v4, interval=0.5)

        # ==========================================
        # 3. 实体安全生成
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=raw_v1[0][0], y=raw_v1[0][1], yaw=raw_v1[0][2],
                               role_name='v1')
        v2 = RTB.spawn_vehicle(world, 'vehicle.tesla.model3', x=raw_v2[0][0], y=raw_v2[0][1], yaw=raw_v2[0][2],
                               role_name='v2')
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=raw_ego[0][0], y=raw_ego[0][1], yaw=raw_ego[0][2],
                                role_name='ego')
        # 卡车需要稍微抬高一点防止底盘卡死，这里使用 z_offset=1.0
        v4 = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x=raw_v4[0][0], y=raw_v4[0][1], yaw=raw_v4[0][2],
                               role_name='v4', z_offset=1.0)

        actor_list.extend([v1, v2, ego, v4])
        if not all([v1, v2, ego, v4]):
            print("[警告] 有车辆生成失败，请检查坐标点。")
            return

        # 🏎️ 【物理优化】修改卡车的默认重量，使其突破原生物理限制，保证能快速响应 PID 提速
        if v4:
            physics_ctrl = v4.get_physics_control()
            physics_ctrl.mass = 3500.0  # 默认卡车质量超1万，直接减重到 3.5吨
            v4.apply_physics_control(physics_ctrl)
            print("[物理调优] 消防卡车已完成物理减重！")

        # ==========================================
        # 4 & 6. 剧本状态机与控制器挂载
        # ==========================================
        # V1 (Impala): 维持初始速度 50km/h
        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=50.0)

        # V2 (Model3)
        sm_v2 = RTB.MultiStageBehaviorMachine(initial_speed=10.0)
        sm_v2.add_stage('time', target_speed=40.0, trigger_val=3.0, accel=15.0)

        # Ego (Audi TT): 初始 50km/h。在 x=-13 减速到 10km/h，过 3s 恢复 50km/h
        # Ego 轨迹 X 轴从 62 向 -141 运动，所以是变小 (x_less)
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=50.0)
        sm_ego.add_stage('x_less', target_speed=5.0, trigger_val=-8.0, accel=30.0)
        sm_ego.add_stage('time', target_speed=50.0, trigger_val=3.0, accel=15.0)

        # V4 (Firetruck): 初始 50km/h。x=6.8 减速到 20km/h，x=-9 停车，过 3s 恢复 40km/h
        # Truck 轨迹 X 轴从 74 向 -108 运动，同样是变小 (x_less)
        sm_v4 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_v4.add_stage('x_less', target_speed=30.0, trigger_val=6.8, accel=15.0)
        sm_v4.add_stage('x_less', target_speed=0.0, trigger_val=-9.0, accel=20.0)
        sm_v4.add_stage('time', target_speed=40.0, trigger_val=3.0, accel=15.0)

        # 为每一辆车独立挂载 PID。
        # 【Bug已修复】: 之前给 pid_lon_4 少传了 K_D 导致计算出现 None * Float。已补齐全部参数！
        pid_lon_1, pid_lat_1 = RTB.PIDLongitudinalController(dt=dt), RTB.PIDLateralController(dt=dt)
        pid_lon_2, pid_lat_2 = RTB.PIDLongitudinalController(dt=dt), RTB.PIDLateralController(dt=dt)
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(dt=dt), RTB.PIDLateralController(dt=dt)
        pid_lon_4 = RTB.PIDLongitudinalController(K_P=3.0, K_I=0.1, K_D=0.1, dt=dt)
        pid_lat_4 = RTB.PIDLateralController(preset='truck', dt=dt)

        # ==========================================
        # 5. 灯光管理器配置 (黑夜环境，开启车灯)
        # ==========================================
        lm_v1 = RTB.VehicleLightManager(v1)
        lm_v1.set_static_lights(low_beam=False, high_beam=True)  # V1 开启行车灯、远光灯

        lm_v2 = RTB.VehicleLightManager(v2)
        lm_v2.turn_on(carla.VehicleLightState.Position)  # V2 开启基础行车位置灯

        lm_ego = RTB.VehicleLightManager(ego)
        lm_ego.set_static_lights(low_beam=False, high_beam=True)  # Ego 开启行车灯、远光灯

        lm_v4 = RTB.VehicleLightManager(v4)
        lm_v4.set_static_lights(low_beam=False, high_beam=True)  # 卡车开启行车灯、远光灯

        # ==========================================
        # 7. 预热与初始速度瞬间注入
        # ==========================================
        RTB.set_vehicle_initial_speed(v1, 50.0, yaw_deg=raw_v1[0][2])
        RTB.set_vehicle_initial_speed(v2, 20.0, yaw_deg=raw_v2[0][2])
        RTB.set_vehicle_initial_speed(ego, 50.0, yaw_deg=raw_ego[0][2])
        RTB.set_vehicle_initial_speed(v4, 50.0, yaw_deg=raw_v4[0][2])

        # 寻路索引记录
        idx_v1, idx_v2, idx_ego, idx_v4 = 0, 0, 0, 0

        print("[仿真启动] 暗夜暴雨 - 复杂长尾剧本场景开始执行...")

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 车辆1 (Impala) 控制 ----------------
            if v1 and v1.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    speed_v1 = sm_v1.tick(v1.get_location(), sim_time, dt)
                    target_wp_1, idx_v1 = RTB.get_target_waypoint(v1.get_location(), traj_v1, idx_v1, speed_v1)
                    if target_wp_1:
                        RTB.apply_pid_control(v1, pid_lon_1, pid_lat_1, speed_v1, target_wp_1)
                        lm_v1.auto_update_from_control()
                    else:
                        v1.apply_control(carla.VehicleControl(brake=1.0))

            # ---------------- 车辆2 (Model3) 控制 ----------------
            if v2 and v2.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v2, carla_map, auto_destroy=True):
                    speed_v2 = sm_v2.tick(v2.get_location(), sim_time, dt)
                    target_wp_2, idx_v2 = RTB.get_target_waypoint(v2.get_location(), traj_v2, idx_v2, speed_v2)
                    if target_wp_2:
                        RTB.apply_pid_control(v2, pid_lon_2, pid_lat_2, speed_v2, target_wp_2)
                        lm_v2.auto_update_from_control()
                    else:
                        v2.apply_control(carla.VehicleControl(brake=1.0))

            # ---------------- 车辆3 (Ego TT) 控制 ----------------
            if ego and ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    speed_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                    target_wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, speed_ego)
                    if target_wp_ego:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, speed_ego, target_wp_ego)
                        lm_ego.auto_update_from_control()

                    else:
                        ego.apply_control(carla.VehicleControl(brake=1.0))

            # ---------------- 车辆4 (Firetruck 卡车) 控制 ----------------
            if v4 and v4.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v4, carla_map, auto_destroy=True):
                    speed_v4 = sm_v4.tick(v4.get_location(), sim_time, dt)
                    target_wp_4, idx_v4 = RTB.get_target_waypoint(v4.get_location(), traj_v4, idx_v4, speed_v4)
                    if target_wp_4:
                        RTB.apply_pid_control(v4, pid_lon_4, pid_lat_4, speed_v4, target_wp_4)
                        lm_v4.auto_update_from_control()
                    else:
                        # 卡车抵达终点强制拉起手刹
                        v4.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[错误异常] 仿真出现异常: {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)

if __name__ == '__main__':
    main()