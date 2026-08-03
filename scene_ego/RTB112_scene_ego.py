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
# 长尾场景：轨迹硬编码区 (去除表头，纯数值)
# ==========================================
VAN_TRAJ_STR = """
37.697	69.108	88.431
37.697	69.108	88.431
37.697	69.108	88.431
37.79	72.348	88.291
37.941	77.415	88.361
38.081	82.485	88.431
38.13	87.557	90.738
37.937	92.544	92.544
37.631	97.608	95.535
36.987	102.64	98.818
36.049	107.708	103.504
34.386	112.49	114.938
31.901	116.808	123.581
29.048	121.104	123.868
25.89	125.069	131.128
22.353	128.702	136.882
18.532	132.039	142.217
14.551	135.045	143.076
14.219	135.294	143.076
14.219	135.294	144.616
11.128	136.872	168.641
8.313	136.419	-156.73
8.085	136.321	-156.59
3.774	133.753	-135.194
1.121	129.514	-107.953
0.227	124.531	-93.547
0.118	119.453	-91
0.031	114.459	-91
-0.012	112.794	-92.4
-0.012	112.794	-92.4
0.076	108.141	-87.085
0.33	103.161	-87.155
0.534	98.1	-87.788
0.725	92.957	-87.928
0.882	87.981	-88.418
0.99	82.839	-89.05
1.123	77.771	-88.197
1.283	72.697	-88.197
1.405	67.705	-89.327
1.464	62.628	-89.397
1.506	57.535	-89.607
1.531	52.457	-89.817
1.547	47.377	-89.817
1.563	42.298	-89.817
1.545	37.22	-91.236
1.122	32.247	-98.869
0.203	27.168	-100.536
-0.71	22.257	-100.536
-1.397	17.229	-93.953
-1.606	12.154	-91.509
-1.516	7.077	-85.77
-0.972	2.112	-82.233
-0.185	-2.905	-80.311
0.67	-7.911	-80.311
1.525	-12.917	-80.311
2.18	-17.949	-85.757
2.264	-23.104	-91.252
2.153	-28.179	-91.252
2.113	-33.254	-89.763
2.129	-36.999	-89.763
2.129	-36.999	-89.763
2.129	-36.999	-89.763
2.129	-36.999	-89.763
2.129	-36.999	-89.763
2.129	-37.024	-89.763
2.15	-37.531	-86.715
2.178	-38.029	-86.715
2.208	-38.535	-86.715
2.237	-39.042	-86.715
2.266	-39.548	-86.715
2.295	-40.055	-86.715
2.324	-40.561	-86.715
2.396	-41.811	-86.715
2.575	-45.611	-90.851
2.218	-49.391	-98.695
1.424	-53.036	-107.108
1.314	-53.394	-107.108
1.257	-53.576	-108.193
1.068	-54.046	-114.64
0.828	-54.502	-120.807
0.568	-54.938	-120.807
0.308	-55.374	-120.877
-0.001	-55.761	-138.596
-0.387	-56.09	-139.681
-0.774	-56.419	-139.681
-1.732	-57.231	-140.005
-2.784	-57.934	-153.237
-3.945	-58.494	-154.328
-5.097	-58.969	-162.353
-6.337	-59.224	-172.132
-7.593	-59.398	-172.132
-8.851	-59.552	-176.959
-10.116	-59.48	172.327
-11.367	-59.258	166.478
-12.537	-58.828	153.242
-13.648	-58.212	150.806
-14.754	-57.587	147.248
-15.711	-56.787	134.866
-16.559	-55.843	129.96
-17.341	-54.869	128.233
-18.091	-53.845	121.877
-18.662	-52.713	115.512
-20.849	-48.131	115.512
-23.337	-42.297	110.307
-25.449	-36.318	109.304
-27.447	-30.302	106.833
-29.289	-24.234	106.973
-31.105	-18.262	106.473
-32.753	-12.133	103.673
-34.164	-5.947	102.123
-35.468	0.26	101.773
-36.71	6.479	100.432
-37.721	12.74	98.492
-38.589	19.127	97.017
-39.262	25.329	95.309
-39.734	31.653	93.526
-40.057	37.987	92.464
-40.279	44.223	91.904
-40.489	50.564	91.834
-40.679	56.905	91.409
-40.824	63.249	90.986
-40.883	69.595	90.356
-40.923	75.941	90.356
-41.01	82.286	90.989
-41.184	88.628	91.635
-41.365	94.97	91.635
-41.54	101.104	91.635
-41.692	106.412	91.635
-41.706	106.923	91.635
-41.765	108.965	91.635
-41.837	111.502	91.635
-41.879	112.958	91.635
-41.916	114.226	92.279
-42.087	115.482	100.861
-42.393	116.712	107.817
-42.854	117.893	116.592
-43.489	118.992	121.071
-44.173	120.032	131.035
-45.142	120.879	141.887
-46.127	121.645	143.988
-47.19	122.332	151.804
-48.328	122.889	156.369
-49.491	123.334	163.203
-50.74	123.647	171.459
-52.004	123.715	-178.348
-53.243	123.585	-172.886
-54.489	123.367	-165.042
-55.692	122.972	-158.467
-56.83	122.464	-152.867
-57.934	121.842	-147.889
-58.512	121.473	-147.311
-58.512	121.473	-147.311
-58.512	121.473	-147.311
"""

EGO_TRAJ_STR = """
-59.045	126.448	6.703
-59.045	126.448	6.703
-59.045	126.448	6.703
-59.045	126.448	6.703
-59.045	126.448	6.703
-58.428	126.52	6.563
-55.955	126.776	4.205
-53.378	126.834	-1.601
-50.886	126.688	-7.196
-48.395	126.21	-16.844
-46.097	125.15	-29.605
-43.908	123.87	-33.635
-42.003	122.204	-45.867
-40.567	120.132	-63.777
-39.447	117.856	-64.931
-38.664	115.449	-77.347
-38.237	112.908	-83.345
-38.012	110.381	-85.554
-37.837	107.85	-86.464
-37.701	105.356	-87.799
-37.642	102.817	-89
-37.598	100.278	-89.07
-37.548	97.745	-88.72
-37.437	92.766	-88.72
-37.329	86.428	-89.146
-37.213	80.18	-88.656
-37.049	73.827	-88.516
-36.884	67.483	-88.516
-36.673	61.148	-87.816
-36.437	54.923	-87.956
-36.222	48.593	-88.166
-35.989	42.257	-87.463
-35.701	35.921	-87.393
-35.361	29.588	-86.113
-34.911	23.26	-85.48
-34.336	16.943	-83.926
-33.401	10.67	-79.581
-32.207	4.44	-78.74
-30.953	-1.778	-78.32
-29.673	-7.885	-77.9
-28.273	-14.072	-76.208
-26.639	-20.2	-72.599
-24.642	-26.221	-71.012
-22.524	-32.199	-70.303
-20.354	-38.16	-69.205
-18.315	-43.455	-68.92
-17.706	-45.034	-68.92
-17.557	-45.422	-68.92
-17.557	-45.422	-68.92
-17.557	-45.422	-68.92
-17.557	-45.422	-68.92
-17.557	-45.422	-68.92
-17.452	-45.693	-68.92
-17.27	-46.167	-68.92
-17.087	-46.64	-68.92
-16.905	-47.113	-68.92
-16.722	-47.587	-68.92
-16.54	-48.06	-68.92
-16.358	-48.533	-68.92
-16.175	-49.006	-68.92
-15.989	-49.491	-68.92
-15.54	-50.655	-68.92
-15.058	-51.828	-65.669
-14.497	-52.966	-62.446
-13.91	-54.091	-62.446
-13.304	-55.205	-53.728
-12.333	-56.008	-31.955
-11.258	-56.637	-20.391
-10.013	-56.847	-3.537
-8.768	-56.796	5.346
-7.506	-56.67	8.066
-6.266	-56.405	15.34
-5.049	-55.98	23.018
-3.939	-55.372	35.723
-3.002	-54.522	50.283
-2.207	-53.56	50.426
-1.468	-52.532	59.546
-0.947	-51.377	69.518
-0.585	-50.162	74.977
-0.278	-48.932	81.808
-0.245	-47.687	93.047
-0.346	-46.423	94.855
-0.474	-45.161	96.194
-0.611	-43.899	96.264
-0.751	-42.638	96.404
-0.761	-42.555	96.404
-0.761	-42.555	96.404
-0.761	-42.555	96.404
-0.761	-42.555	96.404
-0.761	-42.555	96.404
-0.768	-42.493	96.404
-0.905	-41.253	94.665
-0.98	-39.986	91.259
-1.008	-38.716	91.259
-1.102	-34.096	90.032
-1.062	-29.099	89.525
-1.038	-24.02	90.018
-1.073	-19.028	90.788
-1.142	-13.953	90.788
-1.21	-9.046	90.788
-1.281	-3.867	90.788
-1.351	1.207	90.718
-1.414	6.281	90.718
-1.478	11.355	90.718
-1.532	16.43	90.225
-1.505	21.511	89.522
-1.505	26.505	90.155
-1.519	31.58	90.155
-1.559	36.657	90.647
-1.626	41.734	91.137
-1.741	46.726	91.347
-1.875	51.8	91.627
-2.037	56.873	91.767
-2.163	61.948	91.067
-2.25	67.024	90.787
-2.313	72.017	90.717
-2.377	77.094	90.717
-2.421	82.17	90.224
-2.417	87.247	89.379
-2.361	92.24	89.239
-2.3	97.318	90.241
-2.354	102.389	90.955
-2.394	104.715	91.025
-2.394	104.715	91.025
-2.394	104.715	91.025
"""

JEEP_TRAJ_STR = """
-37.469	105.048	-85.81
-37.469	105.048	-85.81
-37.469	105.048	-85.81
-37.469	105.048	-85.81
-37.469	105.048	-85.81
-36.768	95.417	-86.461
-36.368	85.29	-89.835
-36.305	75.124	-88.976
-36.288	64.961	-89.962
-36.139	54.812	-88.754
-35.947	44.677	-88.824
-35.651	34.544	-87.613
-35.178	24.412	-86.551
-34.367	14.302	-83.899
-32.918	4.43	-78.535
-30.662	-5.645	-77.041
-28.382	-15.548	-76.9
-25.655	-25.326	-71.436
-22.074	-34.816	-67.921
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.373
-18.869	-42.478	-66.443
-18.774	-42.708	-67.656
-18.579	-43.183	-67.656
-18.387	-43.651	-67.656
-18.194	-44.119	-67.656
-17.478	-45.863	-67.656
-16.517	-48.201	-67.656
-15.587	-50.462	-67.656
-15.13	-51.574	-67.656
-14.934	-52.049	-66.383
-14.708	-52.494	-58.123
-14.406	-52.902	-48.549
-14.054	-53.269	-44.476
-13.685	-53.631	-44.476
-13.329	-53.981	-44.476
-12.966	-54.338	-44.476
-12.282	-55.009	-44.476
-10.367	-56.596	-28.802
-8	-57.445	-3.09
-5.476	-57.427	19.374
-3.175	-56.278	41.068
-1.726	-54.238	59.601
-0.55	-51.998	69.468
0.144	-49.563	77.505
0.512	-47.055	84.849
0.61	-44.519	90.211
0.58	-41.98	91.773
0.441	-39.445	94.562
0.223	-36.916	95.262
-0.02	-34.388	95.542
-0.274	-31.821	95.752
-0.528	-29.295	95.323
-0.749	-26.766	94.968
-0.958	-24.236	94.338
-1.112	-21.744	92.501
-1.153	-19.206	89.842
-1.143	-15.585	89.842
-1.148	-10.508	90.688
-1.23	-5.434	90.968
-1.315	-0.359	90.968
-1.389	4.009	90.968
-1.452	7.752	90.968
-1.5	10.31	92.122
-1.651	12.842	94.066
-1.761	15.335	90.125
-1.735	17.81	89.271
-1.705	20.223	89.271
-1.646	24.799	89.271
-1.55	32.391	89.271
-1.494	40.129	89.761
-1.515	47.618	90.601
-1.604	55.23	90.671
-1.656	62.844	89.335
-1.574	69.941	89.335
-1.521	74.498	89.335
-1.546	78.306	92.152
-1.707	82.111	92.714
-1.896	85.914	92.854
-2.174	91.483	92.854
-2.58	100.358	91.219
-2.676	109.09	90.443
-2.741	117.492	90.443
-2.772	121.484	90.443
-2.747	125.286	86.515
-2.299	129.057	79.055
-1.166	132.673	66.676
1.014	135.733	42.948
3.927	138.144	27.147
7.462	139.479	7.776
11.232	139.754	-8.738
14.823	138.836	-27.426
18.226	137.012	-30.537
21.328	134.923	-37.761
24.262	132.406	-43.223
27.023	129.793	-45.916
29.657	127.046	-46.204
32.118	124.147	-53.068
34.19	121.034	-58.391
35.889	117.988	-63.424
35.889	117.988	-63.424
35.889	117.988	-63.424
"""

def prepare_trajectory(raw_str):
    """
    【内部轨迹预处理】
    功能：解析纯数值文本，防止yaw当成高度被错误插值，最后将其还原为贴地的 0.5m 间距密集锚点。
    """
    # 1. 字符串清洗并提取 (x, y, yaw)
    raw_tuples = RTB.parse_string_trajectory(raw_str, min_dist=0.1)
    initial_yaw = raw_tuples[0][2] if len(raw_tuples[0]) > 2 else 0.0

    # 2. 剥离 yaw，仅保留 XY 并传入 Z=0.0 给插值器，防止插值混乱
    xy_tuples = [(p[0], p[1], 0.0) for p in raw_tuples]

    # 3. 0.5米稠密化
    dense_tuples = RTB.interpolate_trajectory(xy_tuples, interval=0.5)

    # 4. 转化为符合 RTB 工具链的标准化 carla.Location 对象 (默认抬高 0.5 米)
    path_locations = [carla.Location(x=p[0], y=p[1], z=0.5) for p in dense_tuples]

    return path_locations, initial_yaw

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-2.394, 104.715)
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
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照截图数值定制的长尾大雾降雨天气
        weather = RTB.build_weather(
            cloudiness=40.0,
            precipitation=80.0,
            precipitation_deposits=90.0,
            wind_intensity=100.0,
            sun_azimuth_angle=135.0,
            sun_altitude_angle=10.0,
            fog_density=30.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=75.0,
            scattering_intensity=3.5,
            mie_scattering_scale=0.21,
            rayleigh_scattering_scale=0.07,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 长尾极端天气系统已设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析与绘制
        # ==========================================
        path_van, yaw_van = prepare_trajectory(VAN_TRAJ_STR)
        path_ego, yaw_ego = prepare_trajectory(EGO_TRAJ_STR)
        path_jeep, yaw_jeep = prepare_trajectory(JEEP_TRAJ_STR)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        # 第一辆：Sprinter 小货车 (Van)
        van = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', path_van[0].x, path_van[0].y, yaw=yaw_van,
                                role_name="van")
        actor_list.append(van)

        # 第二辆：Impala Ego 小轿车
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', path_ego[0].x, path_ego[0].y, yaw=yaw_ego,
                                role_name="ego")
        actor_list.append(ego)

        # 第三辆：Wrangler Rubicon (Jeep SUV)。由于底盘高，可选加点 z_offset 防卡地
        jeep = RTB.spawn_vehicle(world, 'vehicle.jeep.wrangler_rubicon', path_jeep[0].x, path_jeep[0].y, yaw=yaw_jeep,
                                 role_name="jeep", z_offset=0.8)
        actor_list.append(jeep)

        # 赋予无视物理阻塞的瞬间初速度
        RTB.set_vehicle_initial_speed(van, 40.0)
        RTB.set_vehicle_initial_speed(ego, 60.0)
        RTB.set_vehicle_initial_speed(jeep, 60.0)

        # ==========================================
        # 4. 车辆PID控制器挂载与灯光设置
        # ==========================================
        # Van 控制器 (使用卡车预设防止刹不住)
        pid_lon_van = RTB.PIDLongitudinalController(preset='truck')
        pid_lat_van = RTB.PIDLateralController(preset='truck')
        idx_van = 0

        # Ego 控制器
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        idx_ego = 0

        # Jeep 控制器
        pid_lon_jeep = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_jeep = RTB.PIDLateralController(preset='default_car')
        idx_jeep = 0

        # Ego 灯光系统：开启行车灯、近光灯
        light_ego = RTB.VehicleLightManager(ego)
        light_ego.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        # Jeep 灯光系统：仅开启行车灯
        light_jeep = RTB.VehicleLightManager(jeep)
        light_jeep.turn_on(carla.VehicleLightState.Position)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================

        # 【Van 剧本】：初始40 -> 过15s变60 -> y < -37 时减到30 -> 等待5s恢复60。
        sm_van = RTB.MultiStageBehaviorMachine(initial_speed=20.0)
        sm_van.add_stage('time', trigger_val=15.0, target_speed=60.0, accel=15.0)
        sm_van.add_stage('y_less', trigger_val=-37.0, target_speed=30.0, accel=25.0)
        sm_van.add_stage('time', trigger_val=5.0, target_speed=60.0, accel=15.0)

        # 【Ego 剧本】：初始60 -> x > -17 减到10 -> x > -10 恢复50 -> x > -0.761 减到20 -> 等5s恢复60。
        # 逻辑注意：Ego 的 X 坐标从 -59 往大走，所以用 x_greater 跨越触发。
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_ego.add_stage('x_greater', trigger_val=-17.0, target_speed=10.0, accel=25.0)
        sm_ego.add_stage('x_greater', trigger_val=-10.0, target_speed=50.0, accel=20.0)
        sm_ego.add_stage('x_greater', trigger_val=-0.761, target_speed=20.0, accel=25.0)
        sm_ego.add_stage('time', trigger_val=5.0, target_speed=60.0, accel=15.0)

        # 【Jeep 剧本】：初始60 -> 第一次 y < -20 减到40 -> y < -42 减到0停车 -> 等5s恢复20 -> 等3s恢复60。
        # 逻辑注意：这段时间内轨迹一直往下走，所以用 y_less 触发。
        sm_jeep = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_jeep.add_stage('y_less', trigger_val=-20.0, target_speed=40.0, accel=20.0)
        sm_jeep.add_stage('y_less', trigger_val=-30.0, target_speed=0.0, accel=25.0)
        sm_jeep.add_stage('time', trigger_val=5.0, target_speed=20.0, accel=10.0)
        sm_jeep.add_stage('time', trigger_val=3.0, target_speed=70.0, accel=25.0)

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        sim_time = 0.0
        print("[RoadTailBench] 🚀 长尾仿真正式开始！")

        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ------------- 货车 Van 控制 -------------
            if van and van.is_alive:
                if RTB.check_vehicle_out_of_bounds(van, carla_map, auto_destroy=True):
                    van = None  # 出界直接销毁，并释放空指针
                else:
                    target_spd = sm_van.tick(van.get_location(), sim_time, dt)
                    target_wp, idx_van = RTB.get_target_waypoint(van.get_location(), path_van, idx_van,
                                                                 speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(van, pid_lon_van, pid_lat_van, target_spd, target_wp)

            # ------------- 轿车 Ego 控制 -------------
            if ego and ego.is_alive:
                if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    ego = None
                else:
                    target_spd = sm_ego.tick(ego.get_location(), sim_time, dt)
                    target_wp, idx_ego = RTB.get_target_waypoint(ego.get_location(), path_ego, idx_ego,
                                                                 speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd, target_wp)

                    # 更新 Ego 刹车/转向灯联动
                    light_ego.auto_update_from_control()

            # ------------- 越野车 Jeep 控制 -------------
            if jeep and jeep.is_alive:
                if RTB.check_vehicle_out_of_bounds(jeep, carla_map, auto_destroy=True):
                    jeep = None
                else:
                    target_spd = sm_jeep.tick(jeep.get_location(), sim_time, dt)
                    target_wp, idx_jeep = RTB.get_target_waypoint(jeep.get_location(), path_jeep, idx_jeep,
                                                                  speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(jeep, pid_lon_jeep, pid_lat_jeep, target_spd, target_wp)

                    # 更新 Jeep 刹车/转向灯联动
                    light_jeep.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键安全清理场景内的所有残留实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
