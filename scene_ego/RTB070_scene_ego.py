# -*- coding: utf-8 -*-
import sys
import carla
import time
import random

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码区域
# ==========================================
RAW_TRAJ_1 = """
144.357	-5.044	155.316
144.357	-5.044	155.316
144.357	-5.044	155.316
144.357	-5.044	155.106
138.177	-2.125	155.404
128.605	1.247	166.088
118.559	2.673	177.295
108.407	2.409	-173.693
98.364	0.86	-170.586
94.913	0.284	-170.516
94.913	0.284	-170.166
92.454	-0.161	-169.745
90.65	-0.487	-169.675
90.65	-0.487	-169.675
90.65	-0.487	-169.675
89.667	-0.666	-169.675
89.667	-0.666	-169.675
89.667	-0.666	-169.675
89.667	-0.666	-169.675
89.667	-0.666	-169.675
89.667	-0.666	-169.675
"""

RAW_TRAJ_2 = """
193.27	-37.171	145.337
193.27	-37.171	145.337
193.099	-37.053	145.337
191.008	-35.607	145.337
188.917	-34.162	145.337
186.826	-32.716	145.337
184.735	-31.271	145.337
182.644	-29.825	145.337
181.602	-29.104	145.337
181.602	-29.104	145.337
181.602	-29.104	145.337
181.602	-29.104	145.337
181.602	-29.104	145.337
181.602	-29.104	145.337
179.888	-27.92	145.337
177.797	-26.474	145.337
175.709	-25.026	145.267
173.62	-23.578	145.267
171.533	-22.128	145.197
169.445	-20.678	145.267
167.311	-19.22	145.967
165.193	-17.815	146.946
163.053	-16.442	147.785
160.862	-15.074	148.135
158.738	-13.754	148.135
157.393	-12.918	148.135
157.393	-12.918	148.135
157.393	-12.918	148.135
149.45	-7.476	153.264
149.45	-7.476	153.264
149.45	-7.476	153.264
147.591	-6.536	153.194
145.324	-5.388	152.914
143.06	-4.233	153.551
140.743	-3.19	157.273
138.382	-2.25	158.973
136.001	-1.358	159.815
133.604	-0.514	161.725
131.178	0.243	163.416
128.678	0.896	167.015
126.232	1.408	169.42
124.294	1.572	-178.876
124.294	1.572	-178.876
124.294	1.572	-178.876
123.404	1.554	-178.876
119.644	1.48	-178.876
118.812	1.464	-178.876
118.304	1.454	-178.876
117.795	1.444	-178.946
117.278	1.435	-178.809
116.768	1.424	-178.809
116.26	1.413	-178.809
114.992	1.387	-178.809
113.725	1.361	-178.809
112.451	1.334	-178.809
111.178	1.308	-178.809
109.907	1.278	-177.808
108.646	1.218	-177.051
107.354	1.15	-176.98
106.08	1.087	-177.676
104.809	1.058	-179.573
103.535	1.081	177.452
102.263	1.169	174.057
101	1.349	169.515
99.755	1.622	166.22
98.521	1.931	165.94
97.292	2.237	166.573
96.032	2.505	169.354
94.798	2.69	173.41
93.516	2.798	177.181
92.234	2.831	-179.832
90.974	2.774	-175.078
89.704	2.61	-170.295
88.469	2.352	-165.411
87.254	1.993	-162.564
86.041	1.599	-161.805
84.814	1.196	-161.805
83.613	0.802	-162.218
82.396	0.45	-165.829
81.148	0.178	-169.998
79.887	0.016	-175.602
78.615	-0.028	179.923
77.335	0.038	173.91
76.088	0.197	172.213
74.831	0.413	168.261
73.588	0.736	161.766
72.403	1.148	159.499
71.218	1.63	156.873
70.051	2.133	156.662
68.886	2.637	156.313
67.722	3.153	155.756
66.557	3.69	154.989
65.403	4.258	152.445
64.28	4.846	152.375
63.143	5.441	152.375
62.022	6.027	152.375
60.881	6.624	152.375
59.77	7.23	150.054
58.68	7.889	148.425
57.616	8.575	145.504
56.579	9.307	144.595
55.539	10.069	142.185
54.564	10.877	138.897
53.606	11.718	138.547
52.658	12.563	138.011
51.716	13.413	137.732
50.765	14.28	137.384
49.852	15.132	136.896
48.922	16.002	136.896
48.681	16.228	136.896
48.681	16.228	136.896
48.681	16.228	136.896
45.804	19.163	134.303
42.457	22.986	129.548
39.299	26.969	126.912
36.496	31.207	121.986
33.852	35.547	117.642
31.78	40.094	110.641
30.246	44.938	104.733
29.202	49.911	99.192
28.591	54.955	93.618
28.522	60.035	87.633
28.982	65.095	82.676
29.799	70.111	78.861
30.821	75.09	78.085
31.919	80.053	77.099
33.126	84.993	75.623
34.401	89.918	75.343
35.692	94.835	75.343
36.994	99.835	75.412
38.27	104.755	75.411
39.549	109.674	75.481
40.805	114.599	75.9
42.044	119.53	75.9
43.283	124.46	75.83
44.547	129.47	75.9
45.726	134.328	76.464
"""

RAW_TRAJ_3 = """
32.371	56.304	-88.828
32.783	51.24	-82.254
33.736	46.25	-76.005
35.282	41.411	-69.104
37.311	36.753	-62.932
39.831	32.34	-57.267
42.769	28.193	-52.485
45.998	24.268	-48.3
49.68	20.647	-43.251
53.383	17.164	-43.251
57.301	13.8	-37.339
61.488	10.921	-31.383
65.925	8.446	-26.05
70.588	6.425	-20.963
75.423	4.865	-14.535
80.407	3.878	-8.292
85.462	3.338	-2.164
90.626	3.398	3.358
95.604	3.859	6.876
100.645	4.515	7.579
105.689	5.147	6.879
110.751	5.585	2.185
115.915	5.585	-1.997
120.977	5.148	-7.09
126.014	4.46	-9.012
130.985	3.405	-14.154
135.805	2.078	-17.395
140.611	0.425	-20.77
145.375	-1.572	-24.667
149.912	-3.862	-28.442
154.235	-6.532	-33.265
158.468	-9.345	-33.965
162.682	-12.184	-33.966
166.894	-15.021	-33.897
171.122	-17.843	-33.688
175.438	-20.676	-32.278
179.73	-23.39	-32.558
183.988	-26.165	-33.539
188.136	-28.958	-34.099
191.651	-31.344	-34.17
191.651	-31.344	-34.17
191.651	-31.344	-34.17
"""

RAW_TRAJ_4 = """
42.917	105.992	-104.207
41.669	101.064	-104.207
40.422	96.137	-104.207
39.174	91.209	-104.207
37.906	86.2	-104.207
36.658	81.273	-104.207
35.411	76.345	-104.207
34.214	71.405	-102.577
33.227	66.419	-99.728
32.551	61.382	-95.291
32.371	56.304	-88.828
32.783	51.24	-82.254
33.736	46.25	-76.005
35.282	41.411	-69.104
37.311	36.753	-62.932
39.831	32.34	-57.267
42.769	28.193	-52.485
45.998	24.268	-48.3
49.68	20.647	-43.251
53.383	17.164	-43.251
57.301	13.8	-37.339
61.488	10.921	-31.383
65.925	8.446	-26.05
70.588	6.425	-20.963
75.423	4.865	-14.535
80.407	3.878	-8.292
85.462	3.338	-2.164
90.626	3.398	3.358
95.604	3.859	6.876
100.645	4.515	7.579
105.689	5.147	6.879
110.751	5.585	2.185
115.915	5.585	-1.997
120.977	5.148	-7.09
126.014	4.46	-9.012
130.985	3.405	-14.154
135.805	2.078	-17.395
140.611	0.425	-20.77
145.375	-1.572	-24.667
149.912	-3.862	-28.442
154.235	-6.532	-33.265
158.468	-9.345	-33.965
"""

RAW_TRAJ_5 = """
53.094	145.015	-105.824
53.094	145.015	-105.824
53.094	145.015	-105.824
51.809	140.532	-105.821
50.477	135.628	-104.837
49.175	130.628	-104.557
47.91	125.704	-104.277
46.66	120.775	-104.207
45.412	115.848	-104.207
44.165	110.919	-104.207
42.917	105.992	-104.207
41.669	101.064	-104.207
40.422	96.137	-104.207
39.174	91.209	-104.207
37.906	86.2	-104.207
36.658	81.273	-104.207
35.411	76.345	-104.207
34.214	71.405	-102.577
33.227	66.419	-99.728
32.551	61.382	-95.291
32.371	56.304	-88.828
32.783	51.24	-82.254
33.736	46.25	-76.005
35.282	41.411	-69.104
37.311	36.753	-62.932
39.831	32.34	-57.267
42.769	28.193	-52.485
45.998	24.268	-48.3
49.68	20.647	-43.251
53.383	17.164	-43.251
57.301	13.8	-37.339
61.488	10.921	-31.383
65.925	8.446	-26.05
70.588	6.425	-20.963
75.423	4.865	-14.535
80.407	3.878	-8.292
85.462	3.338	-2.164
90.626	3.398	3.358
95.604	3.859	6.876
100.645	4.515	7.579
105.689	5.147	6.879
110.751	5.585	2.185
115.915	5.585	-1.997
120.977	5.148	-7.09
126.014	4.46	-9.012
130.985	3.405	-14.154
135.805	2.078	-17.395
"""

RAW_TRAJ_6 = """
208.247	-47.224	144.229
208.247	-47.224	144.229
208.247	-47.224	144.229
206.833	-46.204	144.229
203.727	-44	144.894
200.571	-41.751	144.334
197.454	-39.557	145.838
194.302	-37.413	145.768
189.533	-34.168	145.768
184.271	-30.607	145.977
179.011	-27.042	145.837
173.752	-23.474	145.837
168.58	-19.964	145.837
163.322	-16.396	145.977
158.037	-12.868	146.886
152.681	-9.45	148.518
147.118	-6.383	154.128
141.316	-3.796	157.955
135.371	-1.555	160.885
129.294	0.296	165.446
123.184	1.598	171.674
116.862	2.172	179.138
110.513	1.904	-175.123
105.232	1.329	-173.11
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (45.726,	134.328)
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

        # 严格按照截图参数设置极其恶劣的暗夜大雨天气
        RTB.set_static_weather(
            world,
            cloudiness=5.0,
            precipitation=75.0,
            precipitation_deposits=55.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=-90.0,
            fog_density=60.0,
            fog_distance=75.0,
            fog_falloff=1.0,
            wetness=75.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 暗夜大雨大雾天气系统已完美复刻")

        # ==========================================
        # 2. 轨迹数据解析、去重与插值稠密化 (0.5m)
        # ==========================================
        trajectories = []
        for raw_str in [RAW_TRAJ_1, RAW_TRAJ_2, RAW_TRAJ_3, RAW_TRAJ_4, RAW_TRAJ_5, RAW_TRAJ_6]:
            parsed = RTB.parse_string_trajectory(raw_str, min_dist=0.1)
            dense_traj = RTB.interpolate_trajectory(parsed, interval=0.5)
            trajectories.append(dense_traj)

        # ==========================================
        # 4. 生成车辆实体并注入初速度
        # ==========================================
        vehicle_configs = [
            {'id': 1, 'bp': 'vehicle.tesla.model3', 'traj': trajectories[0], 'is_truck': False},
            {'id': 2, 'bp': 'vehicle.audi.tt', 'traj': trajectories[1], 'is_truck': False},  # Ego
            {'id': 3, 'bp': 'vehicle.chevrolet.impala', 'traj': trajectories[2], 'is_truck': False},
            {'id': 4, 'bp': 'vehicle.chevrolet.impala', 'traj': trajectories[3], 'is_truck': False},
            {'id': 5, 'bp': 'vehicle.chevrolet.impala', 'traj': trajectories[4], 'is_truck': False},
            {'id': 6, 'bp': 'vehicle.carlamotors.european_hgv', 'traj': trajectories[5], 'is_truck': True},
        ]

        # 车辆控制中枢：专门存放运行时所需的所有 PID与状态机对象
        fleet_manager = []

        for conf in vehicle_configs:
            start_pt = conf['traj'][0]
            z_off = 1.5 if conf['is_truck'] else 0.5
            yaw = start_pt[2] if len(start_pt) > 2 else None

            veh_actor = RTB.spawn_vehicle(world, conf['bp'], x=start_pt[0], y=start_pt[1], yaw=yaw, z_offset=z_off)
            if veh_actor:
                actor_list.append(veh_actor)
                # 注入初速度 60km/h
                RTB.set_vehicle_initial_speed(veh_actor, target_speed_kmh=60.0, yaw_deg=yaw)

                # 分配专用的 PID 控制器
                pid_preset = 'truck' if conf['is_truck'] else 'default_car'
                pid_lon = RTB.PIDLongitudinalController(preset=pid_preset)
                pid_lat = RTB.PIDLateralController(preset=pid_preset)

                # 分配车辆灯光管理器
                light_mgr = RTB.VehicleLightManager(veh_actor)

                # 分配剧本状态机
                sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)

                fleet_manager.append({
                    'id': conf['id'],
                    'actor': veh_actor,
                    'traj': conf['traj'],
                    'current_idx': 0,
                    'pid_lon': pid_lon,
                    'pid_lat': pid_lat,
                    'light_mgr': light_mgr,
                    'sm': sm
                })

        # ==========================================
        # 5. 剧本编排与车灯初始化
        # ==========================================
        for v_data in fleet_manager:
            vid = v_data['id']
            sm = v_data['sm']
            lm = v_data['light_mgr']
            traj = v_data['traj']

            # --- 车灯设置 ---
            if vid == 1:
                lm.set_static_lights(low_beam=True, high_beam=False)
            elif vid == 2:
                lm.set_static_lights(low_beam=False, high_beam=True)
            elif vid == 5:
                lm.set_static_lights(low_beam=True, high_beam=False)
            else:
                lm.set_static_lights(low_beam=False, high_beam=False)  # 仅开启行车灯

            # --- 状态机剧本编排 ---
            if vid == 1:
                # 车辆1：走到轨迹最后（木桩处）急刹车
                sm.add_stage(trigger_type='point', trigger_val=traj[-1], target_speed=0.0, accel=40.0, tolerance=2.0)

            elif vid == 2:
                # 车辆2(Ego)：在第一次 x < 109.6(因为车辆是往X变小的方向开) 时刹停
                sm.add_stage(trigger_type='x_less', trigger_val=130, target_speed=20.0, accel=35.0)
                sm.add_stage(trigger_type='x_less', trigger_val=110, target_speed=0.0, accel=25.0)
                # 等待 1 秒后起步恢复到 60km/h
                sm.add_stage(trigger_type='time', trigger_val=1.5, target_speed=20.0, accel=20.0)
                sm.add_stage(trigger_type='time', trigger_val=0.5, target_speed=60.0, accel=20.0)
            elif vid == 5:
                sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=36.0, accel=10.0)

            elif vid == 6:
                # 车辆6(大货车)：在终点急刹车
                sm.add_stage(trigger_type='point', trigger_val=traj[-1], target_speed=0.0, accel=35.0, tolerance=3.0)

            # 车辆 3, 4无需额外编排，维持 60km/h 一直跑

        print("[场景配置] 车辆剧本、灯光与PID挂载完毕，开始主循环推演。")

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        sim_time = 0.0

        # # 🚀 [新增] 获取上帝视角相机对象
        # spectator = world.get_spectator()

        # 🚀 [新增] 从车队中找出 ego 车辆(ID为2的小轿车)的引用，方便下面视角绑定
        ego = None
        for v_data in fleet_manager:
            if v_data['id'] == 2:
                ego = v_data['actor']
                break

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # 遍历所有车辆更新状态
            for v_data in fleet_manager:
                vehicle = v_data['actor']

                # 1. 越界销毁守护机制 (由于设置了 auto_destroy=True，出界车辆会自动销毁)
                if RTB.check_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=6.0, auto_destroy=True):
                    continue
                if not vehicle.is_alive:
                    continue

                v_loc = vehicle.get_location()

                # 2. 从状态机获取当前帧的目标速度
                target_speed = v_data['sm'].tick(v_loc, sim_time, dt)

                # 3. 轨迹预瞄搜寻
                target_wp, closest_idx = RTB.get_target_waypoint(
                    v_loc,
                    v_data['traj'],
                    v_data['current_idx'],
                    speed_kmh=target_speed,
                    min_lookahead=5.0,
                    lookahead_ratio=0.4
                )

                # 更新索引防止局部搜索丢失
                v_data['current_idx'] = closest_idx

                if target_wp:
                    # 画出预瞄点与牵引线

                    # 提交 PID 控制
                    RTB.apply_pid_control(vehicle, v_data['pid_lon'], v_data['pid_lat'], target_speed, target_wp)

                # 5. 车灯物理联动（如果发生了刹车、转向，灯光会自动变化）
                v_data['light_mgr'].auto_update_from_control()

            # # ---------------- 视角跟随 ----------------
            # # 🚀 [新增] 动态将镜头绑定到 ego 车辆后方 6 米、高 3 米处，俯角 -15 度
            # if ego and ego.is_alive:
            #     tf = ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # ==========================================
        # 7. 环境安全清理
        # ==========================================
        try:
            RTB.disable_synchronous_mode(world)
        except Exception:
            pass
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()