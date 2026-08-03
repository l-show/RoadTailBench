# -*- coding: utf-8 -*-
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
# 轨迹数据硬编码区域
# ==========================================
RAW_TRAJ_TRUCK1 = """
-251.16	333.15	11.087
-251.16	333.15	11.087
-242.163	334.676	9.706
-232.383	336.73	12.701
-222.488	339.058	13.278
-212.734	341.263	11.869
-202.948	343.319	11.799
-192.988	345.355	11.303
-187.922	346.367	11.303
-177.299	348.49	11.303
-175.336	348.875	11.09
-165.361	350.842	11.96
-155.593	352.986	12.463
-145.511	355.25	12.676
-135.746	357.403	12.323
-125.965	359.494	11.9
-115.84	361.557	11.261
-105.665	363.358	8.521
-95.608	364.847	7.657
-85.358	366.158	7.228
-75.108	367.459	7.228
-65.022	368.738	7.158
-55.098	369.964	6.945
-44.8	370.788	2.773
-34.806	371.117	1.475
-24.808	371.295	0.769
-14.64	371.422	0.629
-4.474	371.444	-0.716
5.522	371.167	-2.729
15.834	370.493	-3.89
25.961	369.603	-5.407
36.249	368.629	-5.407
46.532	367.61	-6.043
56.466	366.45	-7.759
66.354	364.897	-10.352
76.191	363.099	-10.352
86.35	361.208	-10.775
96.138	359.152	-12.643
105.919	356.816	-14.631
115.939	354.293	-13.484
125.807	351.839	-14.622
135.625	349.197	-15.473
145.559	346.353	-16.253
155.324	343.526	-15.971
165.291	340.799	-14.258
175.32	338.311	-13.904
185.011	335.846	-14.624
194.657	333.21	-15.63
204.601	330.4	-15.912
214.523	327.512	-16.55
219.955	325.898	-16.55
"""

RAW_TRAJ_FIRETRUCK = """
-303.428	322.086	11.92
-293.654	324.204	12.84
-283.745	326.476	12.981
-273.987	328.659	11.773
-264.182	330.627	10.775
-254.192	332.544	11.341
-244.08	334.668	12.122
-233.978	336.845	12.192
-223.89	339.083	12.757
-213.974	341.327	12.687
-204.041	343.495	12.262
-194.086	345.561	11.265
-186.07	347.121	10.982
-174.794	349.377	11.688
-164.838	351.431	11.618
-154.716	353.511	11.618
-144.921	355.525	11.618
-134.962	357.572	11.618
-124.841	359.652	11.618
-114.876	361.674	11.053
-104.715	363.541	7.8
-94.804	364.862	7.585
-84.872	366.014	5.437
-74.585	366.992	5.297
-64.618	367.942	6.155
-54.676	369.014	6.155
-44.402	370.105	5.442
-34.104	370.961	3.146
-24.109	371.238	1.335
-13.778	371.253	-1.188
-3.447	371.04	-1.188
6.55	370.822	-1.761
16.539	370.38	-3.913
26.847	369.675	-3.913
37.151	368.903	-4.76
47.103	367.953	-6.395
57.327	366.529	-11.347
67.272	364.444	-11.787
77.065	362.439	-11.227
87.2	360.427	-11.228
97.335	358.414	-11.228
107.45	356.318	-12.436
117.199	354.111	-13.001
127.205	351.543	-15.495
133.784	349.694	-15.708
"""

RAW_TRAJ_EGO = """
-275.216	328.462	11.089
-274.717	328.560	11.089
-273.993	328.702	11.089
-271.499	329.191	11.089
-267.697	329.937	11.089
-263.883	330.707	11.886
-260.214	331.481	11.887
-256.483	332.267	11.887
-252.745	333.052	11.649
-249.072	333.809	11.649
-245.397	334.566	11.649
-241.730	335.348	12.602
-238.011	336.180	12.602
-234.352	336.997	12.602
-230.688	337.796	12.079
-225.841	338.834	12.079
-219.628	340.164	12.079
-213.412	341.484	11.721
-207.287	342.722	11.364
-201.057	343.973	11.364
-194.929	345.204	11.364
-188.699	346.455	11.365
-182.453	347.730	11.722
-176.232	349.021	11.722
-170.111	350.287	11.484
-168.046	350.917	16.383
-167.406	351.104	16.383
-166.206	351.452	15.758
-164.998	351.774	11.702
-163.738	351.930	2.378
-162.467	351.934	-0.633
-161.197	351.919	-0.633
-159.949	351.856	-4.796
-158.682	351.749	-4.796
-157.432	351.641	-5.271
-156.187	351.526	-5.271
-154.900	351.417	-3.534
-153.652	351.388	1.811
-152.404	351.461	4.054
-151.138	351.565	4.813
-148.708	351.772	5.203
-146.232	352.110	9.388
-143.724	352.520	8.965
-141.176	352.948	10.360
-138.717	353.398	10.360
-136.220	353.870	11.305
-133.769	354.364	11.435
-131.319	354.863	11.695
-128.832	355.383	11.826
-126.385	355.896	11.826
-123.939	356.408	11.697
-121.406	356.913	11.209
-118.954	357.398	11.209
-116.461	357.892	11.209
-114.019	358.426	14.339
-111.607	359.220	21.345
-109.291	360.124	21.345
-106.926	361.053	21.609
-104.561	361.985	21.081
-102.132	362.726	14.574
-99.712	363.353	14.310
-97.237	363.927	11.724
-94.789	364.435	11.724
-92.342	364.944	11.724
-89.889	365.429	10.075
-87.386	365.869	9.547
-84.919	366.276	9.282
-82.410	366.680	8.886
-79.898	367.065	8.490
-77.423	367.419	7.565
-74.860	367.747	7.168
-72.338	368.064	7.168
-69.857	368.376	7.168
-66.260	368.828	7.168
-61.299	369.451	6.863
-56.237	369.908	4.349
-51.168	370.289	4.084
-46.181	370.646	4.084
-41.110	371.008	4.084
-36.119	371.300	1.990
-31.122	371.473	1.990
-26.123	371.581	1.097
-21.124	371.674	0.302
-16.042	371.571	-1.688
-11.044	371.457	-0.892
-5.961	371.382	-0.627
-0.961	371.341	-0.627
4.037	371.222	-1.556
9.119	371.084	-1.696
14.196	370.815	-4.029
19.182	370.442	-4.575
24.166	370.043	-4.575
29.231	369.613	-6.246
34.193	369.001	-7.178
39.319	368.354	-7.178
44.280	367.730	-7.046
49.327	367.115	-6.514
54.379	366.561	-6.248
59.348	365.997	-7.768
64.378	365.267	-8.566
69.302	364.400	-10.970
74.293	363.432	-10.970
79.365	362.449	-10.970
84.270	361.476	-11.236
89.338	360.468	-11.236
94.238	359.474	-11.674
99.277	358.336	-14.590
104.103	357.026	-15.388
109.019	355.720	-14.457
113.863	354.482	-14.058
118.812	353.322	-12.866
123.692	352.232	-12.601
128.632	351.033	-14.984
133.461	349.737	-14.914
138.374	348.429	-14.914
143.274	347.079	-15.979
148.161	345.680	-15.979
153.132	344.271	-15.713
157.544	343.029	-15.713
158.506	342.758	-15.713
163.400	341.381	-15.713
167.250	340.297	-15.713
167.250	340.297	-15.713
167.250	340.297	-15.713
167.250	340.297	-15.713
167.250	340.297	-15.713
172.585	339.155	-15.037
178.618	337.521	-15.166
184.747	335.841	-15.551
190.768	334.165	-15.551
196.990	332.434	-15.551
203.011	330.758	-15.551
209.032	329.082	-15.552
215.150	327.379	-15.552
221.166	325.698	-15.937
225.566	324.424	-16.323
225.566	324.424	-16.323
225.566	324.424	-16.323
225.566	324.424	-16.323
225.566	324.424	-16.323
"""

RAW_TRAJ_IMPALA = """
-0.549	367.395	-178.309
-4.369	367.431	178.596
-9.365	367.563	178.591
-14.365	367.602	-179.908
-19.531	367.587	-179.626
-24.615	367.553	-179.626
-29.781	367.537	-179.05
-34.777	367.364	-177.617
-39.938	367.13	-177.334
-45.099	366.885	-177.264
-50.175	366.625	-176.266
-55.321	366.169	-174.252
-60.296	365.667	-174.321
-65.438	365.156	-174.321
-70.579	364.644	-174.251
-75.632	364.089	-173.678
-80.599	363.521	-173.253
-85.718	362.818	-171.684
-90.813	361.965	-168.804
-95.805	361.005	-169.368
-100.888	360.082	-169.936
-105.975	359.178	-169.936
-111.063	358.281	-170.006
-116.074	357.41	-170.148
-121.157	356.487	-168.517
-126.051	355.464	-168.094
-131.109	354.406	-168.374
-136.17	353.363	-168.163
-141.056	352.302	-167.595
-146.102	351.193	-167.595
-151.067	350.102	-167.595
-156.031	349.01	-167.595
-160.997	347.919	-167.595
-165.881	346.845	-167.665
-170.848	345.767	-167.808
-175.902	344.695	-168.515
-180.808	343.732	-169.15
-185.807	342.811	-169.717
-190.78	341.761	-167.342
-195.821	340.63	-167.342
-200.788	339.537	-168.71
-205.694	338.575	-168.923
-210.683	337.598	-168.923
-214.772	336.797	-168.923
-217.716	336.221	-168.923
-222.785	335.226	-168.853
-227.854	334.228	-168.853
-232.842	333.247	-168.783
-237.907	332.227	-168.57
-242.794	331.186	-167.364
-247.832	330.046	-167.011
-252.867	328.889	-167.151
-257.745	327.794	-167.433
-262.71	326.701	-167.788
-267.603	325.673	-168.562
-272.667	324.647	-168.349
-277.728	323.603	-168.349
-282.707	322.576	-168.349
-287.686	321.549	-168.349
-292.665	320.522	-168.349
-297.566	319.53	-168.704
-302.466	318.531	-168.421
-307.528	317.494	-168.421
-312.428	316.5	-168.704
-317.488	315.455	-167.78
-322.534	314.344	-167.568
-327.417	313.267	-167.568
-332.466	312.17	-167.78
-337.356	311.125	-168.062
-342.411	310.056	-168.062
-345.346	309.435	-168.062
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (225.566, 324.424)
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
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 精确应用图片截图中的天气要求
        weather = carla.WeatherParameters(
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=-90.0,
            fog_density=60.0, fog_distance=75.0, fog_falloff=1.0,
            wetness=0.0, scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已设置 (带雾、无光照)")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 自动调用库函数解析文本、去重清洗
        traj_truck1 = RTB.parse_string_trajectory(RAW_TRAJ_TRUCK1, min_dist=0.5)
        traj_firetruck = RTB.parse_string_trajectory(RAW_TRAJ_FIRETRUCK, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)
        traj_impala = RTB.parse_string_trajectory(RAW_TRAJ_IMPALA, min_dist=0.5)

        # ==========================================
        # 3. 车辆、行人、模型实体安全生成
        # ==========================================
        # 车辆1：大卡车 (说明：如果你的环境中包含了自定义的 Anim_Truck 蓝图，将此处替换即可。这里使用官方大卡车确保代码通用)
        truck1 = RTB.spawn_vehicle(world, 'vehicle.carlamotors.carlacola', x=traj_truck1[0][0], y=traj_truck1[0][1],
                                   yaw=traj_truck1[0][2], z_offset=1.5)
        actor_list.append(truck1)

        # 车辆2：消防卡车
        firetruck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x=traj_firetruck[0][0],
                                      y=traj_firetruck[0][1], yaw=traj_firetruck[0][2], z_offset=1.5)
        actor_list.append(firetruck)

        # 车辆3：Ego (奥迪TT)
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                role_name="ego,color=255,0,0")
        actor_list.append(ego)

        # 车辆4：Impala 小轿车
        impala = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj_impala[0][0], y=traj_impala[0][1],
                                   yaw=traj_impala[0][2])
        actor_list.append(impala)

        # 行人生成
        walker_bp = bp_lib.filter('walker.pedestrian.*')[0]
        ped_spawn_pt = carla.Transform(carla.Location(x=19.721, y=-91.630, z=1.0))
        walker = world.try_spawn_actor(walker_bp, ped_spawn_pt)
        if walker:
            actor_list.append(walker)

        # ==========================================
        # 4. 车辆PID与行人控制器挂载
        # ==========================================
        # 车辆PID控制器初始化
        pid_lon_t1 = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_t1 = RTB.PIDLateralController(preset='truck')

        pid_lon_f = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_f = RTB.PIDLateralController(preset='truck')

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')

        pid_lon_imp = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_imp = RTB.PIDLateralController(preset='default_car')

        # 轨迹寻路索引缓存
        idx_t1, idx_f, idx_ego, idx_imp = 0, 0, 0, 0

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        lights_ego = RTB.VehicleLightManager(ego)
        lights_impala = RTB.VehicleLightManager(impala)
        # Ego 和 Impala 要求开启远光灯
        lights_ego.set_static_lights(high_beam=True)
        lights_impala.set_static_lights(high_beam=True)

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # 卡车1状态机：初始60km/h，x>-177减速到0并静止10秒，随后恢复60
        sm_truck1 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_truck1.add_stage('x_greater', target_speed=0.0, trigger_val=-163.0, accel=20.0)
        sm_truck1.add_stage('time', target_speed=60.0, trigger_val=10.0, accel=10.0)

        # 消防车状态机：初始90km/h，过3s减速到20，再过15s加速到50
        sm_firetruck = RTB.MultiStageBehaviorMachine(initial_speed=90.0)
        sm_firetruck.add_stage('time', target_speed=20.0, trigger_val=8.0, accel=15.0)
        sm_firetruck.add_stage('time', target_speed=50.0, trigger_val=15.0, accel=10.0)

        # Ego状态机
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=45.0)
        sm_ego.add_stage('x_greater', target_speed=20.0, trigger_val=-235.0, accel=20.0)
        sm_ego.add_stage('x_greater', target_speed=70.0, trigger_val=-168.0, accel=30.0)

        # Impala状态机：初始70km/h，过3s减速到60
        sm_impala = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm_impala.add_stage('time', target_speed=60.0, trigger_val=3.0, accel=10.0)

        # ==========================================
        # 7. 预热与初始状态注入
        # ==========================================
        RTB.set_vehicle_initial_speed(truck1, target_speed_kmh=60.0)
        RTB.set_vehicle_initial_speed(firetruck, target_speed_kmh=90.0)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=60.0)
        RTB.set_vehicle_initial_speed(impala, target_speed_kmh=70.0)

        # # 将视角绑定到 Ego 车后方以便观察
        # spectator = world.get_spectator()

        # ==========================================
        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        print("[场景运行] 仿真开始，按 Ctrl+C 退出。")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # # ---------------- 视角跟随 ----------------
            # if ego and ego.is_alive:
            #     tf = ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))

            # ---------------- 车辆控制与出界守护 ----------------
            # 卡车1 控制
            if not RTB.check_vehicle_out_of_bounds(truck1, carla_map, auto_destroy=True):
                spd = sm_truck1.tick(truck1.get_location(), sim_time, dt)
                wp, idx_t1 = RTB.get_target_waypoint(truck1.get_location(), traj_truck1, idx_t1, speed_kmh=spd)
                RTB.apply_pid_control(truck1, pid_lon_t1, pid_lat_t1, spd, wp)

            # 消防车 控制
            if not RTB.check_vehicle_out_of_bounds(firetruck, carla_map, auto_destroy=True):
                spd = sm_firetruck.tick(firetruck.get_location(), sim_time, dt)
                wp, idx_f = RTB.get_target_waypoint(firetruck.get_location(), traj_firetruck, idx_f, speed_kmh=spd)
                RTB.apply_pid_control(firetruck, pid_lon_f, pid_lat_f, spd, wp)

            # Ego 控制 (要求画出 Ego 的预瞄点与牵引线)
            if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                spd = sm_ego.tick(ego.get_location(), sim_time, dt)
                wp, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, speed_kmh=spd)
                RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, spd, wp)
                lights_ego.auto_update_from_control()  # 车灯根据刹车联动

            # Impala 控制
            if not RTB.check_vehicle_out_of_bounds(impala, carla_map, auto_destroy=True):
                spd = sm_impala.tick(impala.get_location(), sim_time, dt)
                wp, idx_imp = RTB.get_target_waypoint(impala.get_location(), traj_impala, idx_imp, speed_kmh=spd)
                RTB.apply_pid_control(impala, pid_lon_imp, pid_lat_imp, spd, wp)
                lights_impala.auto_update_from_control()  # 车灯根据刹车联动

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
