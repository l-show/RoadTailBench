# -*- coding: utf-8 -*-
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
# 轨迹数据硬编码
# ==========================================
RAW_TRAJ_V1 = """
77.8	-44.692	85.049
77.844	-44.194	85.049
77.882	-43.679	86.587
77.901	-43.18	88.962
77.909	-42.664	89.102
77.917	-42.165	89.102
77.925	-41.665	89.102
77.932	-41.166	90.289
77.91	-40.659	94.617
77.854	-40.164	97.48
77.777	-39.662	99.856
77.677	-39.173	102.511
77.555	-38.681	104.748
77.408	-38.187	108.312
77.238	-37.718	112.083
77.042	-37.241	112.921
76.825	-36.773	115.501
76.587	-36.316	120.284
76.339	-35.891	120.284
76.039	-35.494	131.771
75.7	-35.118	132.051
75.36	-34.742	132.471
74.978	-34.395	141.883
74.578	-34.081	141.883
74.185	-33.772	141.883
73.778	-33.453	141.883
73.372	-33.134	141.883
72.965	-32.815	141.883
72.558	-32.496	141.883
72.148	-32.21	150.536
71.686	-31.999	158.699
71.209	-31.852	164.21
70.713	-31.712	164.21
70.113	-31.542	164.21
68.912	-31.203	164.21
67.684	-30.808	160.787
66.481	-30.399	163.227
65.241	-30.037	163.786
64.041	-29.688	163.786
62.783	-29.397	167.799
61.562	-29.129	166.192
60.328	-28.825	165.563
59.125	-28.487	163.677
57.783	-28.094	163.677
53.104	-26.724	163.677
48.142	-25.271	163.677
43.336	-23.887	164.794
38.329	-22.591	165.912
33.337	-21.254	163.188
28.559	-19.787	162.908
23.779	-18.323	163.188
18.988	-16.895	163.467
14.039	-15.411	162.208
9.359	-13.439	154.113
4.721	-11.164	153.275
0.106	-8.84	153.275
-4.434	-6.554	153.275
-9.049	-4.231	153.275
-13.161	-2.16	153.275
-15.431	-1.017	153.275
-17.738	0.144	153.275
-20.009	1.288	153.275
-22.306	2.47	151.458
-24.508	3.818	145.802
-26.552	5.329	142.312
-28.57	6.942	139.658
-30.513	8.644	138.4
-32.379	10.309	137.561
-34.191	12.032	135.045
-36.019	13.857	135.045
-37.816	15.596	137.349
-39.654	17.289	137.349
-41.554	19.04	137.349
-43.454	20.79	137.349
-45.324	22.512	137.349
-47.193	24.234	137.349
-49.063	25.956	137.349
-50.901	27.65	137.349
-52.847	29.282	145.516
-54.476	30.291	149.354
-54.476	30.291	149.354
-54.476	30.291	149.354
-54.476	30.291	149.354
-54.476	30.291	149.354

"""

RAW_TRAJ_EGO = """
-69.53 -3.881 -4.837
-68.412 -3.961 -4.105
-64.685 -4.244 -4.677
-60.886 -4.556 -4.747
-57.027 -4.899 -5.385
-53.171 -5.294 -5.961
-48.674 -5.764 -5.961
-43.543 -6.364 -8.269
-38.519 -7.137 -9.269
-33.61 -8.081 -12.912
-28.609 -9.375 -15.178
-23.655 -10.841 -18.918
-18.78 -12.549 -20.135
-14.115 -14.348 -22.298
-9.502 -16.274 -23.752
-4.943 -18.327 -24.817
-0.407 -20.431 -24.887
4.28 -22.605 -24.887
8.817 -24.71 -24.887
13.382 -26.948 -26.633
18 -29.264 -26.633
22.618 -31.58 -26.633
27.039 -33.914 -28.984
31.42 -36.492 -31.882
35.751 -39.308 -34.498
39.964 -42.299 -35.583
44.03 -45.209 -35.653
48.163 -48.17 -35.501
52.261 -51.033 -34.502
56.379 -53.866 -36.575
60.211 -57.2 -38.726
64.673 -59.113 -5.129
69.738 -58.335 17.19
74.29 -55.966 34.74
77.815 -52.47 54.427
79.879 -47.765 74.111
80.63 -42.753 91.028
79.869 -37.655 104.603
78.198 -32.776 114.051
75.427 -28.64 130.931
71.561 -25.541 153.975
66.993 -23.545 157.029
62.143 -21.777 160.452
57.276 -20.049 160.452
52.482 -18.374 161.194
47.678 -16.487 156.052
43.022 -14.252 155.995
38.385 -12.384 160.05
33.669 -10.729 161.27
28.738 -9.202 164.167
23.842 -7.849 164.668
19.029 -6.499 163.96
14.089 -4.986 162.33
9.166 -3.42 162.4
4.398 -1.913 162.54
-0.355 -0.363 160.514
-5.176 1.493 158.024
-9.768 3.47 155.623
-14.319 5.727 150.657
-18.535 8.406 145.83
-22.679 11.493 139.491
-26.325 14.911 135.744
-29.695 18.708 127.693
-32.76 22.868 124.792
-35.346 27.142 117.41
-37.476 31.664 113.001
-39.073 36.572 103.592
-39.773 41.681 94.746
-40.172 46.66 94.396
-40.375 51.812 90.049
-40.01 56.782 81.981
-38.863 61.799 72.012
-37.177 66.493 68.069
-35.145 71.047 65.133
-32.622 76.157 63.224
-29.708 81.747 61.806
-26.678 87.399 61.736
-24.265 91.742 59.645
"""

RAW_TRAJ_V3 = """
-25.847	-75.92	69.279
-25.847	-75.92	69.279
-25.847	-75.92	69.279
-25.847	-75.92	69.279
-25.847	-75.92	69.279
-25.847	-75.92	69.279
-25.399	-74.734	69.279
-24.957	-73.567	69.279
-24.479	-72.418	66.771
-23.972	-71.236	66.771
-23.481	-70.093	66.771
-22.966	-68.937	65.713
-22.434	-67.766	65.009
-21.886	-66.606	64.656
-21.354	-65.486	64.657
-20.812	-64.324	65.245
-20.293	-63.198	65.245
-19.756	-62.034	65.245
-19.237	-60.907	65.245
-18.708	-59.762	65.245
-18.172	-58.598	65.245
-17.638	-57.429	65.715
-17.11	-56.258	65.715
-16.598	-55.123	65.716
-16.07	-53.949	65.951
-15.554	-52.792	66.186
-15.042	-51.61	66.774
-14.537	-50.425	67.479
-14.072	-49.269	68.537
-13.616	-48.065	69.595
-13.17	-46.856	69.948
-12.742	-45.685	69.948
-12.353	-44.619	69.948
-12.181	-44.151	69.477
-11.995	-43.692	66.64
-11.795	-43.234	66.052
-11.589	-42.779	65.345
-11.38	-42.325	65.345
-11.164	-41.855	65.346
-10.949	-41.385	65.346
-10.737	-40.923	65.346
-10.521	-40.454	65.464
-10.314	-39.999	65.464
-10.106	-39.545	65.464
-9.892	-39.075	65.464
-9.681	-38.612	65.464
-9.466	-38.143	65.464
-9.252	-37.673	65.464
-9.037	-37.203	65.464
-8.823	-36.733	65.464
-8.615	-36.278	65.464
-8.407	-35.824	65.228
-8.192	-35.364	64.64
-7.966	-34.899	63.006
-7.717	-34.447	59.175
-7.445	-34.008	57.157
-7.153	-33.582	54.063
-6.846	-33.187	50.854
-6.527	-32.802	50.136
-6.196	-32.427	45.826
-5.823	-32.07	41.874
-5.436	-31.754	38.026
-5.033	-31.458	34.055
-4.601	-31.175	31.462
-4.159	-30.941	25.512
-3.69	-30.725	23.343
-3.212	-30.553	17.33
-2.713	-30.419	12.08
-2.218	-30.352	4.373
-1.703	-30.314	3.33
-1.195	-30.298	-0.543
-0.681	-30.347	-8.633
-0.18	-30.434	-11.044
0.318	-30.536	-12.368
0.822	-30.648	-12.62
1.31	-30.758	-12.62
1.822	-30.875	-13.376
2.316	-30.993	-13.376
2.819	-31.112	-13.376
3.305	-31.228	-13.376
3.809	-31.348	-13.376
4.306	-31.457	-11.739
4.812	-31.562	-11.739
5.301	-31.664	-11.739
5.799	-31.767	-11.739
6.297	-31.87	-11.739
6.786	-31.972	-11.854
7.292	-32.078	-11.854
7.789	-32.183	-11.728
8.294	-32.295	-12.939
8.781	-32.407	-12.939
9.277	-32.521	-12.939
9.777	-32.647	-15.269
10.276	-32.783	-15.269
11.361	-33.079	-15.269
12.545	-33.478	-20.06
13.739	-33.914	-20.06
14.933	-34.349	-20.06
16.107	-34.778	-20.06
17.281	-35.207	-20.06
18.455	-35.636	-20.06
19.649	-36.072	-20.06
20.823	-36.501	-20.06
21.998	-36.929	-20.06
23.171	-37.361	-20.566
24.353	-37.827	-22.036
25.547	-38.32	-23.037
26.69	-38.827	-24.556
27.822	-39.356	-25.315
28.948	-39.898	-27.031
30.092	-40.499	-27.847
31.197	-41.083	-27.847
32.339	-41.687	-27.847
33.463	-42.28	-27.847
34.605	-42.884	-27.847
35.692	-43.499	-31.443
36.794	-44.172	-31.443
37.879	-44.836	-31.443
38.981	-45.51	-31.443
40.083	-46.184	-31.373
41.169	-46.845	-31.824
42.24	-47.566	-35.748
43.265	-48.32	-36.51
44.27	-49.064	-36.51
45.278	-49.837	-38.071
46.279	-50.621	-38.071
47.294	-51.419	-38.198
48.293	-52.205	-38.198
49.292	-52.991	-38.198
50.291	-53.777	-38.198
50.71	-54.106	-38.198
51.162	-54.462	-38.198
52.177	-55.261	-38.198
53.164	-56.028	-37.181
54.204	-56.793	-33.887
55.28	-57.469	-31.449
56.407	-58.098	-26.638
57.531	-58.646	-25.013
58.705	-59.13	-20.42
59.928	-59.552	-16.421
61.18	-59.862	-10.171
62.418	-60.031	-5.971
63.685	-60.049	4.093
64.93	-59.947	5.693
66.163	-59.649	15.764
67.405	-59.297	16.023
68.657	-59.089	6.622
69.919	-58.942	6.622
71.159	-58.783	7.917
72.388	-58.558	12.082
73.608	-58.285	13.921
74.815	-57.959	15.639
76.014	-57.544	23.414
77.108	-56.945	34.73
78.109	-56.13	42.822
78.944	-55.175	53.641
79.658	-54.1	57.648
80.324	-52.995	60.809
80.842	-51.837	69.179
81.3	-50.63	69.179
81.75	-49.419	69.857
82.182	-48.201	70.514
82.61	-46.983	71.17
82.969	-45.786	76.262
83.224	-44.563	80.37
83.424	-43.308	81.135
83.568	-42.067	87.638
83.553	-40.818	92.652
83.491	-39.529	92.784
83.429	-38.239	92.652
83.311	-36.995	98.748
83.046	-35.753	104.451
82.724	-34.503	104.451
82.341	-33.293	111.835
81.841	-32.125	113.746
81.299	-30.975	116.011
80.733	-29.814	116.011
80.187	-28.69	116.011
79.574	-27.553	119.608
78.9	-26.501	124.623
78.167	-25.437	124.49
77.411	-24.442	129.943
76.568	-23.518	135.126
75.634	-22.626	136.34
74.692	-21.742	138.812
73.718	-20.959	143.084
72.694	-20.208	146.737
71.583	-19.548	150.083
70.499	-18.927	150.614
69.36	-18.318	152.783
68.209	-17.732	153.846
67.052	-17.207	157.78
65.888	-16.753	158.888
63.964	-16.01	158.888
61.573	-15.152	161.762
59.143	-14.411	163.877
56.661	-13.693	163.877
54.177	-12.985	164.143
51.732	-12.291	164.143
49.246	-11.589	164.808
46.778	-10.949	165.872
44.272	-10.318	165.872
41.848	-9.708	165.872
39.343	-9.077	165.872
36.923	-8.449	165.206
34.466	-7.8	165.206
32.009	-7.147	165.073
29.593	-6.503	165.073
27.138	-5.848	165.073
24.727	-5.188	164.274
22.329	-4.479	163.209
19.856	-3.733	163.209
17.383	-2.987	163.209
14.99	-2.265	163.209
12.597	-1.543	163.209
10.125	-0.796	162.81
7.67	0.006	161.612
5.258	0.808	161.612
2.807	1.623	161.612
0.452	2.461	158.743
-1.953	3.406	158.484
-4.352	4.363	157.818
-6.699	5.338	156.318
-9.039	6.432	153.282
-11.319	7.647	151.109
-13.486	8.893	148.281
-15.577	10.261	145.633
-17.229	11.458	143.373
-17.229	11.458	143.373
-17.229	11.458	143.373
-17.229	11.458	143.373
-17.229	11.458	143.373

"""

RAW_TRAJ_V4 = """
-51.921 -9.085 -7.537
-48.575 -9.511 -6.28
-44.847 -9.923 -6.35
-41.064 -10.395 -8.452
-37.312 -11.067 -11.397
-33.599 -11.928 -14.463
-29.884 -13.028 -20.804
-26.441 -14.513 -24.019
-22.959 -16.065 -24.019
-19.462 -17.732 -27.257
-16.22 -19.61 -34.092
-13.26 -22.002 -46.067
-10.996 -25.046 -62.477
-9.777 -28.704 -78.253
-9.848 -32.523 -107.384
-11.233 -36.132 -110.732
-12.564 -39.699 -110.653
-13.922 -43.182 -111.65
-15.396 -46.752 -111.968
-16.834 -50.334 -111.758
-18.22 -53.801 -111.968
-19.694 -57.367 -113.184
-21.223 -60.841 -114.185
-22.785 -64.232 -115.112
-24.372 -67.612 -115.545
-26.082 -71.069 -116.555
-27.769 -74.398 -117.635
-29.625 -77.707 -120.482
-31.595 -80.877 -123.541
-33.686 -83.943 -126.096
-36.05 -86.99 -128.531
-38.524 -89.971 -131.571
-41.077 -92.698 -133.961
-43.679 -95.379 -134.389
-46.375 -98.055 -135.894
-49.247 -100.633 -139.699
-52.171 -102.956 -143.744
-55.287 -105.235 -144.548
-58.392 -107.421 -145.047
-61.601 -109.552 -147.049
-64.742 -111.552 -148.045
-67.98 -113.511 -149.334
-71.241 -115.438 -149.404
"""
# ==========================================
# 核心修复工具 1：多层道路精准吸附处理器 (防穿透版)
# ==========================================
def process_and_snap_trajectory(carla_map, raw_traj_str, interval=1.0, search_z=100.0):
    """
    【修复核心】：增加 search_z 参数。
    如果遇到立交桥/多层山路，传入一个较高的 search_z (比如 50.0)，
    强迫 Carla 从高空往下探测，优先吸附到最上层的路面！
    """
    raw_pts = RTB.parse_string_trajectory(raw_traj_str, min_dist=0.5)
    if not raw_pts: return [], 0.0

    initial_yaw = raw_pts[0][2]
    dense_pts = RTB.interpolate_trajectory(raw_pts, interval=interval)

    snapped_path = []
    for p in dense_pts:
        x, y = p[0], p[1]
        # 【关键修改】：从指定的 search_z 高度开始向下探测最近的路面
        wp = carla_map.get_waypoint(carla.Location(x=x, y=y, z=search_z), project_to_road=True)
        real_z = wp.transform.location.z if wp else search_z
        snapped_path.append(carla.Location(x=x, y=y, z=real_z))

    return snapped_path, initial_yaw

# 物理车速获取 (供 PID 预瞄使用)
def get_physical_speed_kmh(vehicle):
    vel = vehicle.get_velocity()
    return 3.6 * math.hypot(vel.x, vel.y)


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
        RTB.set_static_weather(
            world, cloudiness=35.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
            sun_azimuth_angle=174.0, sun_altitude_angle=15.0, fog_density=23.0, fog_distance=0.75, fog_falloff=0.1,
            wetness=0.0, scattering_intensity=0.5, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已设置完毕。")

        # ==========================================
        # 2. 轨迹解析 + 多层海拔吸附修正
        # ==========================================
        # 告诉引擎：V3 在很高的桥上(大约30米)，请从高处(search_z=40.0)往下找路面！
        # 这样就绝对不可能吸附到地下的辅道了！
        traj_v1, yaw_v1 = process_and_snap_trajectory(carla_map, RAW_TRAJ_V1, interval=1.0, search_z=100.0)
        traj_ego, yaw_ego = process_and_snap_trajectory(carla_map, RAW_TRAJ_EGO, interval=1.0, search_z=100.0)
        traj_v3, yaw_v3 = process_and_snap_trajectory(carla_map, RAW_TRAJ_V3, interval=1.0, search_z=100.0)
        traj_v4, yaw_v4 = process_and_snap_trajectory(carla_map, RAW_TRAJ_V4, interval=1.0, search_z=100.0)

        # 此时的轨迹线不仅画在路面上，而且肯定是在上层主路！
        RTB.draw_preset_trajectory(world, traj_v1, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_v3, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_v4, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 山区车辆精准生成
        # ==========================================
        # 【注意】：因为 traj_v3[0].z 已经被上面的函数完美吸附到了高架桥面，
        # 所以这里的 z_offset 只需要 0.5 毫米防轮胎穿模就行了！千万不要再填 30 了！
        v1 = RTB.spawn_vehicle(world, 'vehicle.micro.microlino',
                               x=traj_v1[0].x, y=traj_v1[0].y, z=traj_v1[0].z, yaw=yaw_v1,
                               role_name="v1", z_offset=0.5)
        if v1: actor_list.append(v1)
        RTB.set_vehicle_initial_speed(v1, 0.0, yaw_v1)

        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                x=traj_ego[0].x, y=traj_ego[0].y, z=traj_ego[0].z, yaw=yaw_ego,
                                role_name="ego", z_offset=0.5)
        if ego: actor_list.append(ego)
        RTB.set_vehicle_initial_speed(ego, 30.0, yaw_ego)

        v3 = RTB.spawn_vehicle(world, 'vehicle.bmw.grandtourer',
                               x=traj_v3[0].x, y=traj_v3[0].y, z=traj_v3[0].z, yaw=yaw_v3,
                               role_name="v3", z_offset=0.5)
        if v3: actor_list.append(v3)
        RTB.set_vehicle_initial_speed(v3, 40.0, yaw_v3)

        v4 = RTB.spawn_vehicle(world, 'vehicle.micro.microlino',
                               x=traj_v4[0].x, y=traj_v4[0].y, z=traj_v4[0].z, yaw=yaw_v4,
                               role_name="v4", z_offset=0.5)
        if v4: actor_list.append(v4)
        RTB.set_vehicle_initial_speed(v4, 40.0, yaw_v4)

        # ==========================================
        # 4. PID控制器与灯光系统配置
        # ==========================================
        pid_lon_v1, pid_lat_v1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_v3, pid_lat_v3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_v4, pid_lat_v4 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        idx_v1, idx_ego, idx_v3, idx_v4 = 0, 0, 0, 0

        if ego:
            light_ego = RTB.VehicleLightManager(ego)
            light_ego.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)
        if v3:
            light_v3 = RTB.VehicleLightManager(v3)
            light_v3.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)
        if v4:
            light_v4 = RTB.VehicleLightManager(v4)
            light_v4.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)

        # ==========================================
        # 5. 长尾场景剧本状态机
        # ==========================================
        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        sm_v1.add_stage(trigger_type='time', trigger_val=10.0, target_speed=5.0, accel=5.0)

        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        sm_ego.add_stage(trigger_type='y_less', trigger_val=-54.0, target_speed=20.0, accel=25.0)
        sm_ego.add_stage(trigger_type='time', trigger_val=5.0, target_speed=50.0, accel=15.0)

        sm_v3 = RTB.MultiStageBehaviorMachine(initial_speed=40.0)
        sm_v4 = RTB.MultiStageBehaviorMachine(initial_speed=40.0)

        print("[仿真启动] 剧本已装载，主循环开始...")
        OFFROAD_TOLERANCE = 100.0

        # ==========================================
        # 6. 仿真主循环
        # ==========================================

        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt
            #
            # # ---------------- 视角跟随 ----------------
            # if ego and ego.is_alive:
            #     tf = ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))
            # ------------ V1 逻辑 ------------
            if v1 and v1.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v1, carla_map, threshold_dist=OFFROAD_TOLERANCE, auto_destroy=True):
                    target_spd_v1 = sm_v1.tick(v1.get_location(), sim_time, dt)
                    real_spd = get_physical_speed_kmh(v1)

                    wp_v1, idx_v1 = RTB.get_target_waypoint(
                        v1.get_location(), traj_v1, idx_v1, real_spd,
                        min_lookahead=2.0, lookahead_ratio=0.8, max_search_ahead=50
                    )
                    if wp_v1:
                        RTB.apply_pid_control(v1, pid_lon_v1, pid_lat_v1, target_spd_v1, wp_v1)
                    else:
                        v1.apply_control(carla.VehicleControl(brake=1.0))

            # ------------ EGO 逻辑 ------------
            if ego and ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, threshold_dist=OFFROAD_TOLERANCE, auto_destroy=True):
                    target_spd_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                    real_spd = get_physical_speed_kmh(ego)

                    wp_ego, idx_ego = RTB.get_target_waypoint(
                        ego.get_location(), traj_ego, idx_ego, real_spd,
                        min_lookahead=10.0, lookahead_ratio=0.8, max_search_ahead=50
                    )
                    if wp_ego:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd_ego, wp_ego)
                        RTB.draw_lookahead_point(world, ego.get_location(), wp_ego, life_time=0.1)
                    else:
                        ego.apply_control(carla.VehicleControl(brake=1.0))
                    light_ego.auto_update_from_control()

            # ------------ V3 逻辑 ------------
            if v3 and v3.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v3, carla_map, threshold_dist=OFFROAD_TOLERANCE, auto_destroy=True):
                    target_spd_v3 = sm_v3.tick(v3.get_location(), sim_time, dt)
                    real_spd = get_physical_speed_kmh(v3)

                    wp_v3, idx_v3 = RTB.get_target_waypoint(v3.get_location(), traj_v3, idx_v3, real_spd,
                        min_lookahead=8.0, lookahead_ratio=0.8, max_search_ahead=50)
                    if wp_v3:
                        RTB.apply_pid_control(v3, pid_lon_v3, pid_lat_v3, target_spd_v3, wp_v3)
                    else:
                        v3.apply_control(carla.VehicleControl(brake=1.0))
                    light_v3.auto_update_from_control()

            # ------------ V4 逻辑 ------------
            if v4 and v4.is_alive:
                if not RTB.check_vehicle_out_of_bounds(v4, carla_map, threshold_dist=OFFROAD_TOLERANCE, auto_destroy=True):
                    target_spd_v4 = sm_v4.tick(v4.get_location(), sim_time, dt)
                    real_spd = get_physical_speed_kmh(v4)

                    wp_v4, idx_v4 = RTB.get_target_waypoint(v4.get_location(), traj_v4, idx_v4, real_spd,
                        min_lookahead=8.0, lookahead_ratio=0.8, max_search_ahead=50)
                    if wp_v4:
                        RTB.apply_pid_control(v4, pid_lon_v4, pid_lat_v4, target_spd_v4, wp_v4)
                    else:
                        v4.apply_control(carla.VehicleControl(brake=1.0))
                    light_v4.auto_update_from_control()

            # ---------------- 硬件时钟补齐 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()