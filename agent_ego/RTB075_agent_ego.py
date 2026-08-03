# -*- coding: utf-8 -*-
"""
CARLA 0.9.15 三车长尾场景
- 使用 RoadTailBenchInitV9 标准化函数库
- 三辆车均按给定轨迹进行 PID 循迹
- 开启同步模式 dt=0.05s
- 按截图设置天气
- 车辆出界后自动销毁，但仿真继续运行
- Ego 在指定坐标附近减速到 25km/h，4 秒后恢复 90km/h
- slow 车在指定坐标附近加速到 90km/h，触发半径 2m
"""

import sys
import time
import math
import carla

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 2. 本地辅助函数
#    说明：你的原始轨迹是 (x, y, yaw)，
#    RTB.interpolate_trajectory 的第三列语义是 z，
#    所以这里先保留 yaw 用于生成初始朝向，
#    再把 PID 路径转成 (x, y, 0.0)，避免把 yaw 当成 z 绘制到地下。
# ==========================================
def parse_xy_yaw_table(table_text):
    """解析三列轨迹文本，返回 [(x, y, yaw), ...]。"""
    points = []
    for line in table_text.strip().splitlines():
        parts = line.strip().split()
        if len(parts) != 3:
            continue
        try:
            x, y, yaw = map(float, parts)
            points.append((x, y, yaw))
        except ValueError:
            continue
    return points

def build_pid_path_from_xy_yaw(raw_xy_yaw, interval=0.5, min_dist=0.5):
    """
    由 (x, y, yaw) 原始轨迹构建 PID 用稠密路径。
    返回：
        cleaned_xy_yaw: 清洗后的原始格式，用于取生成点 yaw
        dense_path:     [(x, y, 0.0), ...]，用于 get_target_waypoint / apply_pid_control
    """
    cleaned_xy_yaw = RTB.clean_trajectory(raw_xy_yaw, min_dist=min_dist)
    xy_zero_z = [(p[0], p[1], 0.0) for p in cleaned_xy_yaw]
    dense_path = RTB.interpolate_trajectory(xy_zero_z, interval=interval)
    return cleaned_xy_yaw, dense_path

def get_speed_kmh(vehicle):
    """读取车辆当前速度，单位 km/h。"""
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

# ==========================================
# 3. 三车原始轨迹，格式均为：x y yaw
# ==========================================
RAW_A2_TEXT = r"""
112.734 88.694 -74.28
114.219 83.878 -71.114
115.095 81.362 -71.353
115.84 78.975 -72.729
116.73 76.111 -71.609
118.356 71.28 -72.062
119.162 68.63 -72.838
120.581 64.032 -72.837
122.189 58.882 -72.627
123.627 54.289 -72.487
125.183 49.429 -71.988
126.263 46.108 -71.778
127.977 41.149 -70.771
129.795 35.607 -71.634
131.678 29.935 -71.634
133.374 24.813 -71.704
135.431 18.429 -72.414
136.979 13.719 -73.352
138.816 7.878 -71.652
140.893 1.503 -72.217
142.916 -5.045 -73.398
144.244 -9.671 -75.162
145.706 -15.165 -75.833
147.919 -23.777 -75.752
149.364 -29.577 -77.863
150.574 -35.73 -79.1
151.65 -41.348 -79.693
152.563 -46.519 -79.71
153.644 -52.245 -80.49
155.109 -61.165 -81.056
155.872 -66.21 -81.559
156.798 -72.561 -79.438
157.338 -77.189 -83.786
157.65 -81.404 -86.309
157.958 -86.206 -86.379
158.158 -92.467 -88.569
158.434 -99.022 -85.635
159.141 -104.371 -82.466
159.486 -108.437 -87.482
159.695 -113.683 -87.222
159.807 -120.243 -90.392
159.675 -128.847 -92.477
159.379 -135.695 -92.547
159.096 -142.398 -92.337
158.684 -151.284 -93.93
158.129 -158.701 -94.302
157.525 -165.529 -96.767
156.381 -174.498 -97.36
155.223 -183.465 -97.36
153.873 -192.404 -99.88
152.739 -198.867 -99.95
151.013 -207.592 -101.355
149.316 -216.472 -100.096
147.741 -225.375 -100.026
146.078 -234.26 -101.64
144.875 -240.116 -102.216
142.961 -248.952 -102.216
141.241 -257.533 -102.368
139.846 -263.945 -100.576
138.259 -271.359 -104.053
136.628 -277.565 -105.139
134.6 -284.414 -108.569
132.419 -290.912 -108.499
129.845 -298.354 -110.507
127.485 -304.789 -109.561
124.958 -311.78 -110.066
122.755 -317.648 -110.363
120.83 -321.738 -116.662
117.8 -327.558 -117.692
113.792 -335.171 -117.832
110.387 -341.619 -117.832
105.977 -349.507 -119.418
101.372 -357.285 -121.543
96.581 -364.953 -123.821
91.539 -372.457 -123.891
86.247 -379.604 -128.305
80.643 -386.698 -128.305
74.798 -393.592 -131.624
68.813 -400.367 -131.844
62.822 -406.936 -134.411
56.43 -413.333 -135.676
49.836 -419.517 -138.275
43.084 -425.531 -139.035
36.134 -431.296 -141.934
28.995 -436.839 -142.88
22.089 -442.662 -139.151
15.118 -448.417 -141.415
8.18 -453.983 -141.402
0.936 -459.39 -144.557
-6.342 -464.756 -142.968
-11.229 -468.45 -142.898
-11.229 -468.45 -142.898
-11.229 -468.45 -142.898
-11.229 -468.45 -142.898
-11.229 -468.45 -142.898
-11.229 -468.45 -142.898
"""

RAW_EGO_TEXT = r"""
106.771 117.891 -75.467
107.473 115.322 -74.031
108.005 113.568 -72.976
109.05 110.229 -71.934
109.672 108.328 -71.52
111.143 103.988 -71.04
112.199 100.914 -70.97
113.177 98.078 -70.97
114.071 95.478 -71.25
114.828 93.183 -72.221
115.745 90.414 -71.281
117.231 85.99 -71.894
118.143 83.132 -72.369
119.152 79.956 -71.197
120.072 77.276 -71.057
120.82 74.978 -72.026
122.157 70.855 -72.026
123.083 68.001 -72.026
124.444 63.8 -71.955
124.592 63.321 -72.594
125.54 60.477 -71.365
126.365 58.031 -71.719
126.734 56.837 -72.535
127.466 54.533 -72.371
128.395 51.768 -71.435
129.002 49.863 -71.324
129.948 47.103 -70.922
131.095 43.708 -72.002
132.582 39.022 -72.157
133.604 35.849 -72.157
134.833 32.042 -72.017
136.31 27.528 -71.8
137.398 24.201 -71.94
139.003 19.29 -71.703
140.062 16.13 -71.242
141.627 11.381 -71.835
142.266 9.398 -72.513
143.757 4.451 -72.556
144.666 1.505 -72.975
145.178 -0.168 -72.975
145.178 -0.168 -72.975
146.276 -3.754 -72.975
147.35 -7.26 -73.255
148.234 -10.388 -74.699
149.442 -15.24 -76.681
150.484 -19.015 -73.697
151.566 -22.952 -75.575
152.289 -25.863 -77.455
153.171 -30.105 -79.29
154.067 -34.94 -79.344
154.683 -38.469 -79.047
155.48 -42.303 -77.398
156.325 -46.381 -78.983
157.151 -50.635 -79.514
157.803 -54.243 -80.54
158.343 -57.532 -80.38
158.93 -61.236 -81.898
159.294 -63.793 -81.898
159.753 -67.431 -83.237
160.149 -70.774 -83.237
160.599 -74.581 -83.377
161.195 -79.713 -83.377
161.486 -82.95 -85.178
161.92 -88.098 -85.178
162.207 -91.587 -85.879
162.435 -94.996 -86.334
162.728 -100.154 -88.367
162.781 -102.486 -89.415
162.841 -107.567 -88.615
162.96 -111.566 -89.331
162.903 -116.232 -91.134
162.854 -120.982 -89.778
162.914 -126.065 -89.154
162.907 -131.231 -90.676
162.787 -136.396 -93.875
162.584 -141.474 -91.476
162.451 -146.64 -91.476
162.283 -151.637 -93.315
161.85 -156.786 -95.263
161.387 -161.933 -95.053
160.885 -167.076 -94.946
160.407 -172.138 -95.828
159.877 -177.277 -95.898
159.199 -182.314 -98.479
158.447 -187.342 -99.293
157.623 -192.442 -99.283
156.772 -197.452 -99.99
155.838 -202.533 -101.595
154.841 -207.603 -100.798
153.873 -212.68 -100.798
152.875 -217.748 -101.282
151.838 -222.724 -101.739
150.787 -227.782 -101.739
149.736 -232.84 -101.739
148.685 -237.898 -101.739
147.599 -242.951 -102.176
146.542 -247.923 -101.739
145.463 -252.976 -101.896
144.429 -258.039 -101.528
143.336 -263.089 -102.79
142.159 -268.12 -103.821
140.929 -273.053 -104.171
139.59 -278.043 -105.062
138.194 -283.017 -106.764
136.692 -287.96 -107.044
135.144 -292.801 -107.934
133.522 -297.704 -108.414
131.858 -302.594 -109.321
130.113 -307.456 -110.982
128.277 -312.197 -111.942
126.278 -316.962 -113.038
"""

RAW_SLOW_TEXT = r"""

227.443 -84.583 -158.17
225.808 -85.249 -157.535
224.703 -85.706 -157.535
223.768 -86.082 -158.385
222.711 -86.504 -158.142
222.066 -86.763 -158.142
220.308 -87.468 -158.142
218.964 -88.008 -157.589
217.021 -88.86 -155.744
215.999 -89.322 -155.394
214.009 -90.223 -155.998
211.954 -91.132 -156.574
211.897 -91.157 -156.139
209.426 -92.266 -155.44
208.194 -92.834 -155.289
206.32 -93.66 -157.138
204.477 -94.434 -157.278
203.498 -94.84 -157.488
202.404 -95.292 -157.628
200.223 -96.196 -157.088
198.006 -97.164 -155.697
196.081 -98.033 -155.697
194.269 -98.851 -155.837
192.283 -99.743 -156.022
190.128 -100.723 -154.839
188.67 -101.429 -152.444
186.204 -102.764 -150.064
184.242 -103.977 -146.877
182.333 -105.271 -141.898
180.762 -106.503 -140.533
179.432 -107.732 -136.674
177.869 -109.26 -131.016
177.213 -110.014 -125.807
175.872 -111.82 -126.822
174.635 -113.47 -129.013
173.189 -115.274 -128.172
172.13 -116.667 -126.057
170.849 -118.441 -125.777
169.216 -120.73 -124.41
168.069 -122.511 -122.727
166.588 -124.818 -122.29
165.17 -127.382 -113.906
164.353 -129.676 -106.402
163.874 -131.999 -98.467
163.349 -135.078 -98.283
162.974 -137.799 -92.572
162.832 -140.73 -94.745
162.537 -143.963 -94.839
162.249 -147.451 -94.494
161.985 -151.313 -93.068
161.774 -155.182 -93.172
161.488 -159.756 -95.74
160.914 -164.889 -96.943
160.44 -170.033 -94.88
159.983 -175.179 -96.055
159.374 -180.14 -97.044
158.732 -185.266 -97.34
157.989 -190.379 -98.555
157.087 -195.462 -101.933
156.52 -198.172 -100.747
155.905 -201.405 -100.817
155.197 -205.214 -100.083
154.477 -209.021 -100.69
153.77 -212.768 -100.69
153.051 -216.575 -100.691
152.344 -220.321 -100.691
151.618 -224.127 -101.531
150.856 -227.863 -101.531
150.094 -231.598 -101.531
149.344 -235.272 -101.531
148.601 -238.948 -100.757
147.877 -242.755 -100.757
147.154 -246.564 -100.757
146.43 -250.308 -101.317
145.582 -254.089 -104.022
144.643 -257.849 -104.022
143.704 -261.609 -104.022
142.765 -265.369 -104.022
141.826 -269.129 -104.022
140.887 -272.89 -104.022
139.948 -276.65 -104.022
138.994 -280.406 -104.302
137.993 -284.149 -105.116
136.968 -287.822 -105.705
135.894 -291.545 -107.052
134.755 -295.118 -108.708
133.496 -298.716 -110.686
132.122 -302.34 -110.966
130.736 -305.958 -110.966
129.371 -309.518 -110.966
127.985 -313.137 -110.966
126.598 -316.755 -110.966
125.212 -320.373 -110.966
123.825 -323.992 -110.966
122.198 -327.506 -117.183
120.456 -330.897 -117.183
118.685 -334.202 -118.98
116.808 -337.59 -118.98
114.932 -340.98 -118.35
113.099 -344.394 -117.93
111.317 -347.756 -117.93
109.453 -351.148 -119.941
107.486 -354.485 -122.274
105.454 -357.71 -121.895
103.462 -360.885 -122.175
101.35 -364.133 -124.306
99.237 -367.38 -122.409
97.129 -370.631 -123.179
95.038 -373.819 -124.051
92.832 -377.004 -125.362
90.581 -380.157 -126.17
88.293 -383.28 -126.373
85.995 -386.399 -126.373
83.529 -389.387 -130.165
80.913 -392.244 -132.506
78.296 -395.099 -132.506
75.757 -398.025 -130.746
73.228 -400.96 -130.746
70.7 -403.895 -130.746
68.171 -406.83 -131.516
65.573 -409.619 -133.886
62.89 -412.407 -134.763
60.112 -415.102 -136.01
57.283 -417.748 -137.809
55.749 -419.127 -138.083
55.749 -419.127 -138.083
55.749 -419.127 -138.083
55.749 -419.127 -138.083
"""

def main():
    actor_list = []
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = None

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

        # 按截图设置天气：Base Preset = ClearNoon
        RTB.set_static_weather(
            world,
            cloudiness=0.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=0.0,
            sun_azimuth_angle=120.0,
            sun_altitude_angle=10.0,
            fog_density=10.0,
            fog_distance=0.75,
            fog_falloff=0.10,
            wetness=0.0,
            scattering_intensity=5.5,
            mie_scattering_scale=0.21,
            rayleigh_scattering_scale=0.07,
            dust_storm=0.0
        )
        print("[场景配置] 同步模式已开启；天气已按截图设置为 ClearNoon + 轻雾/低太阳高度。")

        # ==========================================
        # 2. 轨迹数据硬编码、清洗与稠密化
        # ==========================================
        raw_a2 = parse_xy_yaw_table(RAW_A2_TEXT)
        raw_ego = parse_xy_yaw_table(RAW_EGO_TEXT)
        raw_slow = parse_xy_yaw_table(RAW_SLOW_TEXT)

        clean_a2, traj_a2 = build_pid_path_from_xy_yaw(raw_a2, interval=0.5, min_dist=0.5)
        clean_ego, traj_ego = build_pid_path_from_xy_yaw(raw_ego, interval=0.5, min_dist=0.5)
        clean_slow, traj_slow = build_pid_path_from_xy_yaw(raw_slow, interval=0.5, min_dist=0.5)

        print(f"[轨迹清洗] Audi A2: 原始 {len(raw_a2)} 点 -> 清洗 {len(clean_a2)} 点 -> 稠密 {len(traj_a2)} 点")
        print(f"[轨迹清洗] Ego Audi TT: 原始 {len(raw_ego)} 点 -> 清洗 {len(clean_ego)} 点 -> 稠密 {len(traj_ego)} 点")
        print(f"[轨迹清洗] Slow Impala: 原始 {len(raw_slow)} 点 -> 清洗 {len(clean_slow)} 点 -> 稠密 {len(traj_slow)} 点")

        # ==========================================
        # 3. 车辆安全生成
        # ==========================================
        car_a2 = RTB.spawn_vehicle(
            world,
            "vehicle.audi.a2",
            clean_a2[0][0],
            clean_a2[0][1],
            yaw=clean_a2[0][2],
            role_name="audi_a2",
            z_offset=0.5,
        )

        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        slow = RTB.spawn_vehicle(
            world,
            "vehicle.chevrolet.impala",
            clean_slow[0][0],
            clean_slow[0][1],
            yaw=clean_slow[0][2],
            role_name="slow",
            z_offset=0.5,
        )

        for v in [car_a2, ego, slow]:
            if v:
                actor_list.append(v)

        if not car_a2 or not ego or not slow:
            raise RuntimeError("至少有一辆车生成失败，请检查出生点是否被占用或是否在可驾驶道路附近。")

        # ==========================================
        # 4. 初速度注入
        # ==========================================
        RTB.set_vehicle_initial_speed(car_a2, 90.0, yaw_deg=clean_a2[0][2])
        RTB.set_vehicle_initial_speed(slow, 35.0, yaw_deg=clean_slow[0][2])

        # ==========================================
        # 5. 车辆 PID 控制器挂载
        #    每辆车必须拥有自己独立且专属的 PID 控制器
        # ==========================================

        # 可选：开启基础行车灯，轻雾下更方便观察车辆
        light_managers = {}
        for v in [car_a2, ego, slow]:
            lm = RTB.VehicleLightManager(v)
            lm.set_static_lights(low_beam=True, high_beam=False)
            light_managers[v.id] = lm

        # ==========================================
        # 6. 坐标触发速度事件配置
        #    触发半径按你的要求设置为 2m
        # ==========================================
        SPEED_TRIGGER_RADIUS = 2.0
        speed_event_state = {
            # Ego：经过指定点后减速到 65km/h，4 秒后恢复原速度 90km/h
            "ego_slowdown_point": (149.442, -15.24),
            "ego_original_speed": 90.0,
            "ego_slowdown_speed": 65.0,
            "ego_resume_delay_s": 6.0,
            "ego_slowdown_triggered": False,
            "ego_resume_triggered": False,
            "ego_resume_time": None,

            # slow：经过指定点后加速到 90km/h，并保持该目标速度
            "slow_speedup_point": (162.463, -141.630),
            "slow_speedup_speed": 90.0,
            "slow_speedup_triggered": False,
        }

        print("[仿真启动] 三车 PID 循迹场景开始运行。按 Ctrl+C 结束。")

        # ==========================================
        # 7. 仿真主循环
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # 遍历 actor_list 副本，允许在循环中移除出界车辆
            for actor in actor_list[:]:
                if not actor or not actor.is_alive:
                    actor_list.remove(actor)
                    light_managers.pop(actor.id, None)
                    continue

                # ---------------- 出界守护：出界车辆自动销毁，仿真继续 ----------------
                is_out = RTB.check_vehicle_out_of_bounds(
                    actor,
                    carla_map,
                    threshold_dist=8.0,
                    auto_destroy=True,
                )
                if is_out:
                    print(f"[出界清理] 车辆 {actor.id} 已出界并被销毁，场景继续运行。")
                    if actor in actor_list:
                        actor_list.remove(actor)
                    light_managers.pop(actor.id, None)
                    continue

                if actor.id not in pids:
                    continue

                ctrl_data = pids[actor.id]
                if ctrl_data["finished"]:
                    RTB.force_vehicle_stop(actor)
                    continue

                path = ctrl_data["path"]
                target_speed = ctrl_data["target_speed"]
                last_idx = ctrl_data["last_idx"]

                # ---------------- 获取当前速度与预瞄点 ----------------
                curr_speed = get_speed_kmh(actor)
                target_wp, cur_idx = RTB.get_target_waypoint(
                    actor.get_location(),
                    path,
                    last_idx,
                    curr_speed,
                    min_lookahead=5.0,
                    lookahead_ratio=0.4,
                    max_search_ahead=35,
                    fallback_dist=30.0,
                )

                ctrl_data["last_idx"] = cur_idx

                # ---------------- 轨迹终点处理：到达终点后停车，不销毁 ----------------
                final_x, final_y = path[-1][0], path[-1][1]
                loc = actor.get_location()
                dist_to_final = math.hypot(loc.x - final_x, loc.y - final_y)

                if cur_idx >= len(path) - 3 and dist_to_final < 3.0:
                    ctrl_data["finished"] = True
                    RTB.force_vehicle_stop(actor)
                    print(f"[轨迹完成] {ctrl_data['role']} 已到达轨迹终点并驻车。")
                    continue

                # ---------------- 坐标触发速度事件 ----------------
                # 注意：这里直接修改当前车辆 PID 目标速度 ctrl_data["target_speed"]。
                # Ego 减速事件触发后，会记录恢复时间；达到恢复时间后自动恢复到原速度。
                role = ctrl_data["role"]
                if role == "ego":
                    ego_px, ego_py = speed_event_state["ego_slowdown_point"]
                    dist_to_ego_trigger = math.hypot(loc.x - ego_px, loc.y - ego_py)

                    if (not speed_event_state["ego_slowdown_triggered"] and
                            dist_to_ego_trigger <= SPEED_TRIGGER_RADIUS):
                        ctrl_data["target_speed"] = speed_event_state["ego_slowdown_speed"]
                        speed_event_state["ego_slowdown_triggered"] = True
                        speed_event_state["ego_resume_time"] = sim_time + speed_event_state["ego_resume_delay_s"]
                        print(
                            f"[速度触发] Ego 距离触发点 {dist_to_ego_trigger:.2f}m，"
                            f"目标速度降为 {ctrl_data['target_speed']:.1f} km/h；"
                            f"将在 {speed_event_state['ego_resume_delay_s']:.1f}s 后恢复。"
                        )

                    if (speed_event_state["ego_slowdown_triggered"] and
                            not speed_event_state["ego_resume_triggered"] and
                            speed_event_state["ego_resume_time"] is not None and
                            sim_time >= speed_event_state["ego_resume_time"]):
                        ctrl_data["target_speed"] = speed_event_state["ego_original_speed"]
                        speed_event_state["ego_resume_triggered"] = True
                        print(f"[速度恢复] Ego 4 秒减速阶段结束，目标速度恢复为 {ctrl_data['target_speed']:.1f} km/h。")

                elif role == "slow":
                    slow_px, slow_py = speed_event_state["slow_speedup_point"]
                    dist_to_slow_trigger = math.hypot(loc.x - slow_px, loc.y - slow_py)

                    if (not speed_event_state["slow_speedup_triggered"] and
                            dist_to_slow_trigger <= SPEED_TRIGGER_RADIUS):
                        ctrl_data["target_speed"] = speed_event_state["slow_speedup_speed"]
                        speed_event_state["slow_speedup_triggered"] = True
                        print(
                            f"[速度触发] slow 车距离触发点 {dist_to_slow_trigger:.2f}m，"
                            f"目标速度提升为 {ctrl_data['target_speed']:.1f} km/h。"
                        )

                # 将最新目标速度写回 PID 控制输入
                target_speed = ctrl_data["target_speed"]

                # ---------------- PID 控制执行 ----------------
                if target_wp is not None:
                    RTB.apply_pid_control(
                        actor,
                        ctrl_data["lon"],
                        ctrl_data["lat"],
                        target_speed,
                        target_wp,
                    )

                # ---------------- 车辆灯光自动联动 ----------------
                if actor.id in light_managers:
                    light_managers[actor.id].auto_update_from_control()

                if ctrl_data["role"] == "ego" and target_wp is not None:
                    pass

            # 所有车辆都被销毁或结束后，仿真仍可继续；这里不主动 break
            if not actor_list:
                print("[提示] 当前已无存活车辆，仿真仍保持运行。按 Ctrl+C 结束。")

            # ---------------- 硬件时钟补齐：尽量维持 1X 真实时间 ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")

    except Exception as e:
        print(f"\n[场景异常] {e}")
        raise

    finally:
        # 恢复异步模式并一键清理场景实体
        if world is not None:
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == "__main__":
    main()
