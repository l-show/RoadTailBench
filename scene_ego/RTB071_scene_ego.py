import sys
import carla
import time

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码区
# ==========================================
TRAJ_STR_V1 = """
-131.375	1.528	-0.267
-131.375	1.528	-0.267
-130.044	1.532	0.227
-120.055	1.6	0.506
-109.892	1.707	0.716
-99.727	1.8	0.367
-89.515	1.877	0.506
-79.516	2.007	0.93
-69.351	2.185	1.07
-59.185	2.38	1.07
-49.02	2.569	0.93
-38.688	2.631	0.144
-28.523	2.648	0.074
-18.524	2.635	-0.206
-8.357	2.599	-0.206
1.81	2.59	0.074
11.976	2.633	0.424
21.976	2.697	0.284
32.142	2.705	0.004
42.308	2.656	-1.357
52.467	2.398	-1.43
62.631	2.184	-1.01
72.655	2.124	0.494
82.821	2.212	0.494
92.82	2.314	0.777
102.985	2.382	-0.431
113.149	2.255	-1.134
123.314	2.08	-0.781
133.314	2.009	-0.081
143.313	2.009	0.129
153.48	2.055	0.339
163.644	1.912	-0.764
173.814	1.867	-0.409
"""

TRAJ_STR_V2 = """
-69.351	2.185	1.07
-59.185	2.38	1.07
-49.02	2.569	0.93
-38.688	2.631	0.144
-28.523	2.648	0.074
-18.524	2.635	-0.206
-8.357	2.599	-0.206
1.81	2.59	0.074
11.976	2.633	0.424
21.976	2.697	0.284
32.142	2.705	0.004
42.308	2.656	-1.357
52.467	2.398	-1.43
62.631	2.184	-1.01
72.655	2.124	0.494
82.821	2.212	0.494
92.82	2.314	0.777
102.985	2.382	-0.431
113.149	2.255	-1.134
123.314	2.08	-0.781
133.314	2.009	-0.081
143.313	2.009	0.129
153.48	2.055	0.339
163.644	1.912	-0.764
173.814	1.867	-0.409
183.812	1.686	-1.193
193.975	1.412	-2.325
204.12	0.781	-4.591
214.163	-0.709	-12.854
221.479	-2.357	-14.091
229.231	-4.329	-14.301
238.919	-6.799	-14.301
245.252	-8.251	-8.346
247.238	-8.487	-5.579
247.238	-8.487	-5.579
"""

TRAJ_STR_V3 = """
106.279	-2.836	-178.874
106.279	-2.836	-178.874
106.279	-2.836	-179.154
104.279	-2.814	179.287
100.468	-2.749	179.007
96.656	-2.683	179.007
90.97	-2.585	179.007
84.618	-2.475	179.007
78.265	-2.364	179.007
69.559	-2.214	179.007
60.665	-2.059	179.007
53.458	-1.934	179.007
47.126	-1.825	179.007
44.294	-1.776	179.007
41.753	-1.732	179.007
39.212	-1.655	177.316
36.68	-1.458	172.379
34.179	-1.029	168.213
31.723	-0.391	163.586
29.315	0.405	159.413
27.024	1.486	150.074
24.922	2.893	141.136
23.095	4.636	131.697
21.58	6.652	119.777
20.507	8.891	112.516
19.562	11.28	111.449
18.744	13.671	104.153
18.215	16.146	98.556
17.979	18.712	93.288
17.875	21.249	92.15
17.813	23.079	91.59
17.813	23.079	91.59
17.813	23.079	91.59
17.813	23.079	91.59
17.813	23.079	91.59
17.813	23.079	91.59
17.813	23.079	91.59
17.802	23.495	91.59
17.733	26.034	91.17
17.703	28.617	90.675
17.672	31.158	90.745
17.607	36.198	90.745
17.541	42.655	90.249
17.531	49.014	89.899
17.539	55.369	89.969
17.544	61.724	89.899
17.555	68.078	89.899
17.567	74.432	89.899
17.578	80.791	89.899
17.591	87.145	89.829
17.61	93.606	89.829
17.643	104.814	89.829
17.543	116.251	91.185
17.355	127.686	89.901
17.909	139.107	84.076
20.096	150.128	74.05
24.221	160.759	62.189
26.06	164.023	59.14
26.06	164.023	59.14
26.06	164.023	59.14
"""

TRAJ_STR_EGO = """
21.213	113.803	-90.687
21.213	113.803	-90.687
21.213	113.803	-91.527
21.195	112.915	-91.174
21.123	109.106	-90.684
21.095	105.297	-89.911
21.16	101.486	-88.361
21.275	97.675	-88.151
21.396	93.863	-88.571
21.444	90.047	-89.903
21.447	86.235	-89.973
21.448	82.485	-90.043
21.417	78.671	-90.603
21.376	74.858	-90.673
21.332	71.046	-90.673
21.294	67.234	-90.533
21.278	63.422	-90.183
21.266	59.547	-90.183
21.257	55.797	-90.113
21.246	51.985	-90.393
21.217	48.11	-90.182
21.216	44.299	-89.832
21.224	40.487	-90.185
21.184	36.675	-90.815
21.13	32.863	-90.815
21.077	29.051	-90.745
21.056	27.488	-90.745
21.056	27.488	-90.745
21.056	27.488	-90.745
21.056	27.488	-90.745
21.056	27.488	-90.745
21.032	25.613	-90.745
20.983	21.864	-90.745
20.936	18.26	-90.745
20.926	17.489	-90.745
20.92	16.989	-90.745
20.913	16.472	-90.745
20.907	15.972	-90.745
20.9	15.464	-90.605
20.895	14.948	-90.535
20.891	14.441	-90.535
20.885	13.933	-90.744
20.867	13.426	-93.974
20.813	12.93	-100.214
20.668	12.444	-110.116
20.519	12.064	-111.493
20.519	12.064	-111.493
20.519	12.064	-111.493
20.519	12.064	-111.493
20.379	11.708	-111.493
20.186	11.239	-113.975
19.961	10.795	-120.587
19.687	10.369	-123.366
19.402	9.952	-127.931
18.704	9.067	-128.289
16.28	6.155	-136.257
13.281	3.846	-149.752
9.883	2.166	-158.039
6.293	0.92	-163.982
2.625	-0.098	-165.622
-1.035	-0.908	-169.999
-4.818	-1.362	-175.246
-8.623	-1.559	-178.142
-12.434	-1.665	-178.844
-16.246	-1.742	-178.844
-20.057	-1.795	-179.907
-23.81	-1.784	179.811
-30.831	-1.761	179.811
-40.913	-1.728	179.811
-50.913	-1.706	-179.979
-61.246	-1.708	179.881
-71.245	-1.658	179.741
-81.41	-1.616	179.951
-91.576	-1.679	-179.209
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-91.576	,-1.679)
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
        dt = 0.05

        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 修正：直接把截图里的参数作为 kwargs 塞进一键静态天气配置器里，不再构建外部变量
        RTB.set_static_weather(
            world,
            cloudiness=40.0,
            precipitation=100.0,
            precipitation_deposits=75.0,  # Puddles
            wind_intensity=100.0,
            sun_azimuth_angle=100.0,
            sun_altitude_angle=12.0,
            fog_density=10.0,
            fog_distance=0.75,
            fog_falloff=0.1000,
            wetness=75.0,
            scattering_intensity=1.5,
            mie_scattering_scale=0.21,
            rayleigh_scattering_scale=0.07,
            dust_storm=0.0
        )
        print("[场景配置] 长尾截屏指定极端恶劣天气系统已接管！")

        # 2. 轨迹数据清洗与稠密化 (去噪并插值到0.5m)
        # ==========================================
        # 库的串流处理器会自动剔除头部文本，并清洗原地驻留的重复点
        raw_v1 = RTB.parse_string_trajectory(TRAJ_STR_V1, min_dist=0.5)
        path_v1 = RTB.interpolate_trajectory(raw_v1, interval=0.5)

        raw_v2 = RTB.parse_string_trajectory(TRAJ_STR_V2, min_dist=0.5)
        path_v2 = RTB.interpolate_trajectory(raw_v2, interval=0.5)

        raw_v3 = RTB.parse_string_trajectory(TRAJ_STR_V3, min_dist=0.5)
        path_v3 = RTB.interpolate_trajectory(raw_v3, interval=0.5)

        raw_ego = RTB.parse_string_trajectory(TRAJ_STR_EGO, min_dist=0.5)
        path_ego = RTB.interpolate_trajectory(raw_ego, interval=0.5)

        # 3. 车辆实体安全生成与初速度注入
        # ==========================================
        # 提取各个轨迹的初始锚点
        v1_start, v2_start, v3_start, ego_start = path_v1[0], path_v2[0], path_v3[0], path_ego[0]

        vehicle_v1 = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', v1_start[0], v1_start[1], yaw=v1_start[2],
                                       role_name='npc_truck')
        vehicle_v2 = RTB.spawn_vehicle(world, 'vehicle.harley-davidson.low_rider', v2_start[0], v2_start[1],
                                       yaw=v2_start[2], role_name='npc_moto')
        vehicle_v3 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', v3_start[0], v3_start[1], yaw=v3_start[2],
                                       role_name='npc_car')
        vehicle_ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', ego_start[0], ego_start[1], yaw=ego_start[2],
                                        role_name='ego')

        actor_list.extend([vehicle_v1, vehicle_v2, vehicle_v3, vehicle_ego])

        # 利用库函数瞬间注入物理初速度 (无卡顿发射)
        RTB.set_vehicle_initial_speed(vehicle_v1, 50.0, yaw_deg=v1_start[2])
        RTB.set_vehicle_initial_speed(vehicle_v2, 80.0, yaw_deg=v2_start[2])
        RTB.set_vehicle_initial_speed(vehicle_v3, 60.0, yaw_deg=v3_start[2])
        RTB.set_vehicle_initial_speed(vehicle_ego, 70.0, yaw_deg=ego_start[2])

        # 4. 车辆灯光管理器
        # ==========================================
        # V2, V3 开启行车灯
        light_v2 = RTB.VehicleLightManager(vehicle_v2)
        light_v2.set_static_lights(low_beam=False, high_beam=False)
        light_v3 = RTB.VehicleLightManager(vehicle_v3)
        light_v3.set_static_lights(low_beam=False, high_beam=False)
        # Ego 开启行车灯与近光灯
        light_ego = RTB.VehicleLightManager(vehicle_ego)
        light_ego.set_static_lights(low_beam=True, high_beam=False)

        # 5. 车辆PID控制器挂载 (针对极大雨水的环境，采用 wet_road 预设防滑)
        # ==========================================
        pid_lon_v1, pid_lat_v1 = RTB.PIDLongitudinalController(preset='truck', dt=dt), RTB.PIDLateralController(
            preset='truck', dt=dt)
        pid_lon_v2, pid_lat_v2 = RTB.PIDLongitudinalController(preset='motorcycle', dt=dt), RTB.PIDLateralController(
            preset='motorcycle', dt=dt)
        pid_lon_v3, pid_lat_v3 = RTB.PIDLongitudinalController(preset='wet_road', dt=dt), RTB.PIDLateralController(
            preset='wet_road', dt=dt)
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(preset='wet_road', dt=dt), RTB.PIDLateralController(
            preset='wet_road', dt=dt)

        # 轨迹寻路索引锚点缓存
        idx_v1 = idx_v2 = idx_v3 = idx_ego = 0

        # 6. 剧本状态机编排
        # ==========================================
        # V1: 货车，恒定 50km/h
        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=50.0)

        # V2: 摩托车，恒定 90km/h
        sm_v2 = RTB.MultiStageBehaviorMachine(initial_speed=90.0)

        # V3: 小轿车，初始40。在 y=1 (向南开，所以触发条件是 y_less 1.0) 减速到30，等5s，加速到60
        sm_v3 = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_v3.add_stage('y_less', target_speed=30.0, trigger_val=1.0, accel=15.0)
        sm_v3.add_stage('time', target_speed=30.0, trigger_val=5.0, accel=15.0)
        sm_v3.add_stage('immediate', target_speed=60.0, accel=15.0)

        # V4 (Ego): 小轿车，初始70。在 y=30 减速到30，在 y=12 减速到5，等5s，加速到60
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm_ego.add_stage('y_less', target_speed=30.0, trigger_val=50.0, accel=25.0)
        sm_ego.add_stage('y_less', target_speed=5.0, trigger_val=30.0, accel=35.0)
        sm_ego.add_stage('time', target_speed=5.0, trigger_val=5.0, accel=25.0)
        sm_ego.add_stage('immediate', target_speed=60.0, accel=20.0)

        # 7. 仿真主循环
        # ==========================================
        print("[主循环] 开始高精度同步推演...")
        sim_time = 0.0

        while True:
            # 记录本帧开始的时间，用于补齐硬件时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # --- V1 (Truck) ---
            if vehicle_v1 and vehicle_v1.is_alive:
                if not RTB.check_vehicle_out_of_bounds(vehicle_v1, world.get_map(), auto_destroy=True):
                    speed = sm_v1.tick(vehicle_v1.get_location(), sim_time, dt)
                    wp, idx_v1 = RTB.get_target_waypoint(vehicle_v1.get_location(), path_v1, idx_v1, speed)
                    if wp: RTB.apply_pid_control(vehicle_v1, pid_lon_v1, pid_lat_v1, speed, wp)

            # --- V2 (Motorcycle) ---
            if vehicle_v2 and vehicle_v2.is_alive:
                if not RTB.check_vehicle_out_of_bounds(vehicle_v2, world.get_map(), auto_destroy=True):
                    speed = sm_v2.tick(vehicle_v2.get_location(), sim_time, dt)
                    wp, idx_v2 = RTB.get_target_waypoint(vehicle_v2.get_location(), path_v2, idx_v2, speed)
                    if wp: RTB.apply_pid_control(vehicle_v2, pid_lon_v2, pid_lat_v2, speed, wp)
                    light_v2.auto_update_from_control()  # 尾灯物理联动

            # --- V3 (Impala Car) ---
            if vehicle_v3 and vehicle_v3.is_alive:
                if not RTB.check_vehicle_out_of_bounds(vehicle_v3, world.get_map(), auto_destroy=True):
                    speed = sm_v3.tick(vehicle_v3.get_location(), sim_time, dt)
                    wp, idx_v3 = RTB.get_target_waypoint(vehicle_v3.get_location(), path_v3, idx_v3, speed)
                    if wp: RTB.apply_pid_control(vehicle_v3, pid_lon_v3, pid_lat_v3, speed, wp)
                    light_v3.auto_update_from_control()

                    # --- V4 (Ego Audi TT) ---
            if vehicle_ego and vehicle_ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(vehicle_ego, world.get_map(), auto_destroy=True):
                    speed = sm_ego.tick(vehicle_ego.get_location(), sim_time, dt)
                    wp_ego, idx_ego = RTB.get_target_waypoint(vehicle_ego.get_location(), path_ego, idx_ego, speed)
                    if wp_ego:
                        RTB.apply_pid_control(vehicle_ego, pid_lon_ego, pid_lat_ego, speed, wp_ego)

                    light_ego.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()