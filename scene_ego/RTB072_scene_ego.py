import sys
import carla
import time
import math

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码区 (已剔除表头)
# ==========================================
TRAJ_TRUCK_STR = """
27.283	128	-84.591
27.283	128	-84.591
27.283	128	-84.591
27.283	128	-85.37
27.352	127.254	-84.731
27.702	123.461	-84.731
28.017	119.663	-85.899
28.269	115.859	-86.613
28.494	112.053	-86.613
28.558	110.971	-86.613
28.588	110.464	-86.613
28.618	109.956	-86.613
28.648	109.457	-86.613
28.678	108.949	-86.613
28.708	108.442	-86.613
28.728	108.109	-86.613
28.728	108.109	-86.613
28.744	107.826	-86.613
28.775	107.319	-86.543
28.808	106.811	-85.983
28.844	106.304	-85.28
28.902	105.799	-82.362
28.982	105.298	-78.322
29.102	104.804	-73.343
29.269	104.324	-68.405
29.475	103.859	-63.567
29.716	103.412	-60.267
29.99	102.975	-54.551
30.301	102.573	-50.372
30.641	102.196	-45.053
31.016	101.854	-40.016
31.419	101.53	-38.45
31.825	101.224	-34.97
32.251	100.948	-30.437
32.69	100.707	-27.521
33.146	100.483	-24.805
33.615	100.287	-20.633
34.094	100.118	-18.076
34.581	99.971	-16.096
35.071	99.836	-13.968
35.567	99.725	-10.408
36.062	99.658	-7.131
36.574	99.595	-6.201
37.081	99.554	-4.437
40.18	99.313	-4.437
44.051	99.152	-1.559
47.821	99.065	-1.207
54.028	98.957	0.588
60.382	99.052	0.874
66.736	99.128	-0.048
73.089	99.017	-1.957
79.439	98.796	-1.745
85.792	98.646	-0.901
92.145	98.546	-0.901
98.582	98.445	-0.901
108.248	98.293	-0.901
118.412	98.073	-1.461
128.741	97.77	-1.741
138.894	97.46	-1.881
149.056	97.126	-1.881
159.215	96.734	-2.583
169.352	95.983	-5.873
179.412	94.541	-11.128
189.318	92.265	-15.728
199.118	89.002	-21.393
208.224	84.879	-27.45
217.029	79.801	-31.743
225.716	74.206	-33.509
234.181	68.573	-34.659
242.397	62.878	-34.729
243.219	62.308	-34.729
243.219	62.308	-34.729
"""

TRAJ_MOTO_STR = """
29.217	152.768	-86.24
29.73	144.953	-86.24
30.055	140.006	-86.24
30.305	136.201	-86.73
29.793	132.442	-105.149
28.762	128.771	-106.146
27.821	125.078	-101.758
27.294	121.306	-94.022
27.255	117.497	-87.267
27.543	113.695	-85.316
27.849	109.958	-85.316
28.526	101.695	-85.386
29.038	92.96	-86.888
29.533	83.932	-86.535
30.071	75.052	-86.535
30.6	66.317	-86.535
31.137	57.434	-86.535
31.675	48.555	-86.535
32.214	39.675	-86.465
32.927	28.134	-86.465
33.688	15.445	-87.095
34.304	2.752	-87.235
34.917	-9.941	-87.235
35.52	-22.426	-87.235
36.133	-35.122	-87.235
36.789	-48.711	-87.235
37.586	-65.212	-87.235
38.458	-81.714	-86.675
39.576	-97.925	-86.25
40.504	-114.692	-86.957
41.367	-130.92	-86.957
42.244	-147.418	-86.957
43.122	-163.921	-86.957
43.61	-173.117	-86.957
43.61	-173.117	-86.957
43.61	-173.117	-86.957
"""

TRAJ_CAR_STR = """
-57.528	102.657	-5.73
-57.528	102.657	-5.73
-52.944	102.239	-4.961
-36.983	101.127	-3.606
-24.113	100.316	-3.606
-11.629	99.691	-1.709
-0.758	99.391	-1.569
7.135	99.174	-1.569
11.155	99.064	-1.569
14.964	99.148	4.429
18.688	99.898	20.877
21.973	101.813	42.892
23.877	105.008	69.42
24.507	108.804	92.477
24.125	112.597	95.235
23.848	116.337	93.172
23.649	120.144	92.962
23.452	123.952	92.962
23.058	131.568	92.962
22.346	145.32	92.962
21.207	167.332	92.962
19.872	193.132	92.962
18.658	216.6	92.962
17.571	237.599	92.962
16.58	256.743	92.962
15.673	274.282	92.962
14.893	289.351	92.962
14.146	304.582	91.755
14.782	319.819	83.156
17.324	334.853	79.27
20.672	349.726	74.977
20.672	349.726	74.977
20.672	349.726	74.977
20.672	349.726	74.977
"""

TRAJ_EGO_STR = """
23.69	197.865	-85.545
23.69	197.865	-85.545
23.69	197.865	-85.545
23.828	196.037	-85.825
24.185	190.883	-86.385
24.476	185.808	-87.155
24.702	180.814	-88.075
24.873	175.735	-88.075
25.068	170.656	-87.375
25.318	165.662	-87.025
25.582	160.586	-87.025
25.85	155.425	-87.025
26.109	150.432	-87.025
26.373	145.355	-87.025
26.664	140.28	-86.247
26.997	135.208	-86.247
27.329	130.135	-86.247
27.662	125.063	-86.247
27.897	119.985	-87.687
28.029	116.737	-87.687
28.033	115.466	-92.101
27.984	114.134	-92.101
27.876	111.594	-94.737
27.523	109.121	-102.451
26.725	106.711	-112.927
25.609	104.476	-120.755
24.138	102.441	-130.973
22.312	100.677	-138.66
20.322	99.1	-147.142
18.115	97.841	-152.666
15.779	96.851	-164.888
13.357	96.233	-168.799
10.838	95.906	-175.785
8.175	95.785	-177.599
3.095	95.62	-179.33
-1.985	95.723	176.919
-6.983	95.878	179.553
-13.587	95.93	179.553
-24.672	96.017	179.553
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-24.672, 96.0173)
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
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按照截图参数设定逼真的阴雨大雾天
        RTB.set_static_weather(world,
                               cloudiness=40.0,
                               precipitation=100.0,
                               precipitation_deposits=100.0,
                               wind_intensity=100.0,
                               sun_azimuth_angle=90.0,
                               sun_altitude_angle=4.0,
                               fog_density=10.0,
                               fog_distance=0.75,
                               fog_falloff=0.1,
                               wetness=100.0,
                               scattering_intensity=0.0,
                               mie_scattering_scale=0.04,
                               rayleigh_scattering_scale=0.13,
                               dust_storm=0.0
                               )
        print("[场景配置] 天气系统已设置 (大雾阴雨傍晚)")

        # ==========================================
        # 2. 轨迹数据解析与清洗稠密化
        # ==========================================
        raw_traj_truck = RTB.parse_string_trajectory(TRAJ_TRUCK_STR, min_dist=0.5)
        raw_traj_moto = RTB.parse_string_trajectory(TRAJ_MOTO_STR, min_dist=0.5)
        raw_traj_car = RTB.parse_string_trajectory(TRAJ_CAR_STR, min_dist=0.5)
        raw_traj_ego = RTB.parse_string_trajectory(TRAJ_EGO_STR, min_dist=0.5)

        # 进行 0.5 米密度的插值稠密化
        dense_traj_truck = RTB.interpolate_trajectory(raw_traj_truck, interval=0.5)
        dense_traj_moto = RTB.interpolate_trajectory(raw_traj_moto, interval=0.5)
        dense_traj_car = RTB.interpolate_trajectory(raw_traj_car, interval=0.5)
        dense_traj_ego = RTB.interpolate_trajectory(raw_traj_ego, interval=0.5)

        # ==========================================
        # 3. 车辆生成与初始状态赋予
        # ==========================================

        # 【车辆1】小货车 (标准行车道吸附)
        truck = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', x=raw_traj_truck[0][0], y=raw_traj_truck[0][1],
                                  yaw=raw_traj_truck[0][2], role_name="truck")
        actor_list.append(truck)
        RTB.set_vehicle_initial_speed(truck, 30.0, yaw_deg=raw_traj_truck[0][2])

        # 【车辆2】摩托车 (⭐ 关键修改：取消行车道吸附，支持人行道生成)
        moto_bp = bp_lib.find('vehicle.harley-davidson.low_rider')
        moto_bp.set_attribute('role_name', 'moto')

        # 探测当前坐标下的人行道高程 (Sidewalk)
        moto_search_loc = carla.Location(x=raw_traj_moto[0][0], y=raw_traj_moto[0][1], z=0.0)
        moto_wp = carla_map.get_waypoint(moto_search_loc, project_to_road=True, lane_type=carla.LaneType.Sidewalk)

        # 如果获取到了人行道，用人行道的高程+0.5防穿模；否则默认给1.0米高让其自己掉落
        moto_z = (moto_wp.transform.location.z + 0.5) if moto_wp else 1.0

        moto_transform = carla.Transform(
            carla.Location(x=raw_traj_moto[0][0], y=raw_traj_moto[0][1], z=moto_z),
            carla.Rotation(yaw=raw_traj_moto[0][2])
        )
        moto = world.try_spawn_actor(moto_bp, moto_transform)
        if moto:
            actor_list.append(moto)
            RTB.set_vehicle_initial_speed(moto, 70.0, yaw_deg=raw_traj_moto[0][2])
            light_moto = RTB.VehicleLightManager(moto)
            light_moto.set_static_lights(low_beam=False)  # 仅开行车灯
        else:
            print("[警告] 摩托车生成失败！人行道该位置可能已被占据。")

        # 【车辆3】小轿车 (标准行车道吸附)
        car = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=raw_traj_car[0][0], y=raw_traj_car[0][1],
                                yaw=raw_traj_car[0][2], role_name="car")
        actor_list.append(car)
        RTB.set_vehicle_initial_speed(car, 60.0, yaw_deg=raw_traj_car[0][2])
        light_car = RTB.VehicleLightManager(car)
        light_car.set_static_lights(low_beam=False)

        # 【车辆4】EGO小轿车 (标准行车道吸附)
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=raw_traj_ego[0][0], y=raw_traj_ego[0][1],
                                yaw=raw_traj_ego[0][2], role_name="ego")
        actor_list.append(ego)
        RTB.set_vehicle_initial_speed(ego, 60.0, yaw_deg=raw_traj_ego[0][2])
        light_ego = RTB.VehicleLightManager(ego)
        light_ego.set_static_lights(low_beam=True)

        # ==========================================
        # 4. 车辆 PID 控制器独立挂载
        # ==========================================
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_truck = RTB.PIDLateralController(preset='truck')

        pid_lon_moto = RTB.PIDLongitudinalController(preset='motorcycle')
        pid_lat_moto = RTB.PIDLateralController(preset='motorcycle')

        pid_lon_car = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_car = RTB.PIDLateralController(preset='default_car')

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================
        sm_truck = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        sm_truck.add_stage('time', target_speed=65.0, trigger_val=3.0, accel=15.0)

        sm_moto = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm_car = RTB.MultiStageBehaviorMachine(initial_speed=60.0)

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_ego.add_stage('y_less', target_speed=20.0, trigger_val=150.0, accel=35.0)
        sm_ego.add_stage('time', target_speed=60.0, trigger_val=6.0, accel=15.0)

        # ---------------- 预热与初始状态变量 ----------------
        idx_truck, idx_moto, idx_car, idx_ego = 0, 0, 0, 0
        sim_time = 0.0

        # ==========================================
        # 6. 仿真主循环 (带帧率同步守护)
        # ==========================================
        print("[主循环] 开始仿真长尾剧本...")
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 车辆控制逻辑分发 ----------------

            # 【车辆1】小货车处理
            if truck and truck.is_alive:
                if not RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    speed_kmh = 3.6 * math.hypot(truck.get_velocity().x, truck.get_velocity().y)
                    tgt_speed = sm_truck.tick(truck.get_location(), sim_time, dt)
                    tgt_wp, idx_truck = RTB.get_target_waypoint(truck.get_location(), dense_traj_truck, idx_truck,
                                                                speed_kmh)
                    if tgt_wp: RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, tgt_speed, tgt_wp)

            # 【车辆2】摩托车处理 (⭐ 关键修改：取消出界守护，允许在人行道野区行驶)
            if moto and moto.is_alive:
                # 屏蔽 check_vehicle_out_of_bounds 检测，防止因为离开 Driving Lane 被销毁
                speed_kmh = 3.6 * math.hypot(moto.get_velocity().x, moto.get_velocity().y)
                tgt_speed = sm_moto.tick(moto.get_location(), sim_time, dt)
                tgt_wp, idx_moto = RTB.get_target_waypoint(moto.get_location(), dense_traj_moto, idx_moto, speed_kmh)
                if tgt_wp: RTB.apply_pid_control(moto, pid_lon_moto, pid_lat_moto, tgt_speed, tgt_wp)

            # 【车辆3】小轿车处理
            if car and car.is_alive:
                if not RTB.check_vehicle_out_of_bounds(car, carla_map, auto_destroy=True):
                    speed_kmh = 3.6 * math.hypot(car.get_velocity().x, car.get_velocity().y)
                    tgt_speed = sm_car.tick(car.get_location(), sim_time, dt)
                    tgt_wp, idx_car = RTB.get_target_waypoint(car.get_location(), dense_traj_car, idx_car, speed_kmh)
                    if tgt_wp: RTB.apply_pid_control(car, pid_lon_car, pid_lat_car, tgt_speed, tgt_wp)

            # 【车辆4】EGO车处理
            if ego and ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    speed_kmh = 3.6 * math.hypot(ego.get_velocity().x, ego.get_velocity().y)
                    tgt_speed = sm_ego.tick(ego.get_location(), sim_time, dt)
                    tgt_wp, idx_ego = RTB.get_target_waypoint(ego.get_location(), dense_traj_ego, idx_ego, speed_kmh)

                    if tgt_wp:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, tgt_speed, tgt_wp)

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