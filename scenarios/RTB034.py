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
# 轨迹数据硬编码
# ==========================================
TRAJ_STR_1 = """Location_x	Location_y	Rotation_yaw
28.535	73.828	-122.947
25.081	67.893	-116.566
22.051	60.903	-109.886
19.641	53.667	-108.341
16.591	46.698	-119.857
12.733	40.121	-120.64
9.312	33.319	-110.947
7.61	26.033	-99.438
6.826	18.453	-93.221
6.45	10.839	-91.584
6.274	3.216	-90.952
6.155	-4.408	-90.742
6.087	-12.033	-90.462
6.154	-19.655	-86.197
6.941	-27.237	-82.745
7.909	-34.8	-82.675
8.797	-42.246	-85.316
8.998	-49.867	-90.555
8.852	-57.49	-91.328
8.676	-65.114	-91.328
8.604	-72.737	-89.128
8.892	-80.355	-86.423
9.746	-87.931	-82.317
10.765	-95.487	-82.317
11.755	-103.048	-83.531
12.304	-110.648	-89.412
12.352	-118.273	-90.252
12.286	-125.775	-90.605
12.104	-133.399	-91.471
11.908	-141.021	-91.471
11.765	-148.645	-90.771
11.679	-156.27	-90.491
11.688	-163.77	-89.149
11.849	-171.393	-88.659
11.947	-175.391	-88.589"""

TRAJ_STR_2 = """Location_x	Location_y	Rotation_yaw
43.936	105.267	-130.984
41.439	102.306	-128.223
39.231	99.2	-124.333
37.103	96.037	-123.28
35.057	92.82	-120.641
33.231	89.474	-117.85
31.434	86.112	-118.698
29.575	82.784	-119.261
28.018	80.003	-119.261
27.285	78.694	-119.261
25.425	75.367	-118.757
23.657	71.99	-115.949
22.073	68.522	-114.026
20.631	64.993	-111.652
19.242	61.442	-111.089
17.887	57.811	-109.455
16.625	54.213	-109.315
15.364	50.615	-109.315
14.102	47.016	-109.315
12.841	43.416	-109.315
11.579	39.818	-109.315
10.41	36.188	-105.239
10.059	34.73	-102.987
9.729	33.266	-102.568
8.943	29.536	-101.435
8.218	25.794	-100.235
7.615	22.03	-98.01
7.192	18.241	-95.166
6.912	14.44	-93.885
6.713	10.571	-91.963
6.598	6.823	-91.403
6.525	2.949	-90.703
6.496	-0.863	-90.003
6.525	-4.675	-88.722
6.626	-8.486	-88.162
6.749	-12.296	-88.162
6.871	-16.106	-88.092
7.149	-19.909	-83.91
7.677	-23.684	-80.249
8.327	-27.441	-80.179
8.917	-31.144	-82.465
9.331	-34.996	-85.983
9.488	-38.804	-89.77
9.433	-42.616	-91.626
9.315	-46.428	-91.416
9.264	-48.615	-91.346
9.175	-52.364	-91.346
9.086	-56.177	-91.346
8.915	-63.466	-91.346
8.6	-77.046	-90.991
8.516	-91.024	-90.278
8.514	-105.004	-89.928
8.532	-118.984	-89.928
8.55	-132.968	-89.928
8.552	-146.718	-90.278
8.466	-160.928	-90.348
8.396	-172.428	-90.348
8.385	-174.324	-90.348"""

TRAJ_STR_EGO = """Location_x	Location_y	Rotation_yaw
36.749	101.435	-128.778
34.813	98.983	-126.138
32.599	95.879	-124.339
30.567	92.654	-120.964
28.638	89.365	-119.413
26.812	86.019	-117.79
25.075	82.696	-117.367
23.34	79.301	-116.737
21.614	75.832	-115.523
20.004	72.376	-114.034
19.598	71.463	-113.894
18.677	69.341	-112.823
17.202	65.826	-112.752
15.81	62.345	-110.906
14.507	58.762	-108.93
13.35	55.13	-107.007
12.286	51.534	-106.025
11.275	47.923	-104.377
10.408	44.21	-101.098
9.711	40.462	-99.877
9.331	38.054	-98.721
8.923	35.397	-98.721
8.452	31.677	-96.277
8.048	27.884	-95.994
7.656	24.028	-95.069
7.377	20.226	-93.589
7.155	16.42	-93.306
6.949	12.612	-92.606
6.826	8.802	-91.263
6.753	4.99	-90.843
6.698	1.178	-90.633
6.691	-2.634	-89.931
6.697	-6.447	-89.791
6.715	-10.259	-89.721
6.735	-14.072	-89.581
6.763	-17.884	-89.581
6.791	-21.696	-89.581
6.819	-25.51	-89.581
6.847	-29.322	-89.581
6.875	-33.197	-89.581
6.903	-37.009	-89.581
6.912	-38.259	-89.581
6.939	-42.009	-89.721
6.902	-45.884	-90.851
6.828	-50.883	-90.851
6.699	-59.57	-90.851
6.513	-69.735	-91.061
6.452	-79.734	-90.208
6.381	-90.067	-90.557
6.24	-100.232	-90.837
6.1	-110.398	-90.907
5.939	-120.564	-90.907
5.778	-130.729	-90.907
"""

TRAJ_STR_4 = """Location_x	Location_y	Rotation_yaw
-0.299	-175.99	84.053
-0.247	-175.263	86.259
-0.074	-166.368	89.808
0.103	-157.328	88.242
0.387	-148.437	88.172
0.592	-139.544	89.167
0.722	-130.649	89.167
0.841	-121.753	89.237
0.911	-112.857	90.01
0.87	-103.962	90.36
0.798	-95.211	90.5
0.72	-86.316	90.5
0.67	-77.274	89.717
0.776	-68.377	88.877
1.015	-59.482	88.387
1.266	-50.59	88.387
1.483	-41.697	89.302
1.554	-32.802	89.582
1.637	-23.906	89.302
1.747	-14.865	89.302
1.856	-5.97	89.302
1.964	2.939	89.302
2.185	11.831	88.08
2.577	20.864	87.308
3.133	29.742	85.249
4.132	38.432	80.29
6.06	47.264	76.474
8.38	55.85	73.018
11.17	64.141	68.466
14.575	72.36	65.731
18.332	80.423	64.181
22.399	88.498	60.557
26.869	96.188	57.408
31.892	103.528	53.284
37.403	110.51	51.294
41.871	116.086	51.294"""


def get_parsed_traj(traj_str):
    """辅助函数：过滤表头，解析轨迹，稠密化至0.5m，并获取初始生成偏航角"""
    lines = traj_str.strip().split('\n')
    # 过滤掉表头
    data_lines = [line for line in lines if not line.startswith('Location_x')]
    clean_text = '\n'.join(data_lines)

    # 清洗重复点，容差0.5米
    clean_traj = RTB.parse_string_trajectory(clean_text, min_dist=0.5)
    # 稠密化到 0.5 米
    dense_traj = RTB.interpolate_trajectory(clean_traj, interval=0.5)

    # 提取第一行的偏航角用于初始生成
    initial_yaw = float(data_lines[0].split()[-1])
    return dense_traj, initial_yaw




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (5.778,-130.729)
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

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 依据要求截图精准配置天气参数
        RTB.set_static_weather(world,
                               cloudiness=20.0,
                               precipitation=20.0,
                               precipitation_deposits=20.0,
                               wind_intensity=10.0,
                               sun_azimuth_angle=-1.0,
                               sun_altitude_angle=-90.0,
                               fog_density=40.0,
                               fog_distance=10.0,
                               fog_falloff=1.0,
                               wetness=30.0,
                               scattering_intensity=1.0,
                               mie_scattering_scale=0.03,
                               rayleigh_scattering_scale=0.0331,
                               dust_storm=0.0
                               )
        print("[场景配置] 天气系统已设置")

        # ==========================================
        # 2. 轨迹数据清洗与绘制
        # ==========================================
        traj1, yaw1 = get_parsed_traj(TRAJ_STR_1)
        traj2, yaw2 = get_parsed_traj(TRAJ_STR_2)
        traj_ego, yaw_ego = get_parsed_traj(TRAJ_STR_EGO)
        traj4, yaw4 = get_parsed_traj(TRAJ_STR_4)

        # 绘制所有车辆的预设路径 (NPC为灰色，EGO为蓝色凸显)
        RTB.draw_preset_trajectory(world, traj1, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj2, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(0, 150, 255))
        RTB.draw_preset_trajectory(world, traj4, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 车辆生成与初始状态注入
        # ==========================================
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj1[0][0], y=traj1[0][1], yaw=yaw1,
                               role_name='npc1')
        v2 = RTB.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=traj2[0][0], y=traj2[0][1], yaw=yaw2,
                               role_name='npc2')
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj_ego[0][0], y=traj_ego[0][1], yaw=yaw_ego,
                                role_name='ego')
        v4 = RTB.spawn_vehicle(world, 'vehicle.tesla.model3', x=traj4[0][0], y=traj4[0][1], yaw=yaw4, color='0,0,0',
                               role_name='npc4')

        # 安全纳入列表
        for v in [v1, v2, ego, v4]:
            if v: actor_list.append(v)

        # 瞬间注入物理初速度
        RTB.set_vehicle_initial_speed(v1, target_speed_kmh=60.0, yaw_deg=yaw1)
        RTB.set_vehicle_initial_speed(v2, target_speed_kmh=70.0, yaw_deg=yaw2)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=80.0, yaw_deg=yaw_ego)
        RTB.set_vehicle_initial_speed(v4, target_speed_kmh=130.0, yaw_deg=yaw4)

        # 🚀【修复核心 1】: 强制刷新一帧世界！
        # 让车辆模型与材质在引擎中彻底初始化完毕，防止后续的静态灯光指令被底层吞噬。
        world.tick()

        # ==========================================
        # 4. 灯光系统分配
        # ==========================================
        # 车1：行车灯 + 近光灯 + 双闪爆闪
        lm1 = RTB.VehicleLightManager(v1)
        lm1.set_static_lights(low_beam=True, high_beam=False)
        lm1.start_flashing(mode='hazard')

        # 车2：行车灯 + 远光灯
        lm2 = RTB.VehicleLightManager(v2)
        lm2.set_static_lights(low_beam=False, high_beam=True)

        # EGO：行车灯 + 远光灯
        lm_ego = RTB.VehicleLightManager(ego)
        lm_ego.set_static_lights(low_beam=False, high_beam=True)

        # 车4：行车灯 + 远光灯
        lm4 = RTB.VehicleLightManager(v4)
        lm4.set_static_lights(low_beam=False, high_beam=True)

        # ==========================================
        # 5. PID控制器配置 (专属独立控制)
        # ==========================================
        pid_lon1, pid_lat1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon2, pid_lat2 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon4, pid_lat4 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # 【车1 剧本】
        sm1 = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm1.add_stage(trigger_type='y_less', target_speed=50.0, trigger_val=53.0, accel=20.0)
        sm1.add_stage(trigger_type='y_less', target_speed=60.0, trigger_val=-34.0, accel=20.0)
        sm1.add_stage(trigger_type='time', target_speed=80.0, trigger_val=3.0, accel=10.0)

        # 【车2 剧本】
        sm2 = RTB.MultiStageBehaviorMachine(initial_speed=70.0)
        sm2.add_stage(trigger_type='y_less', target_speed=50.0, trigger_val=80.0, accel=20.0)
        sm2.add_stage(trigger_type='y_less', target_speed=20.0, trigger_val=35.0, accel=20.0)
        sm2.add_stage(trigger_type='y_less', target_speed=60.0, trigger_val=-48.0, accel=20.0)
        sm2.add_stage(trigger_type='time', target_speed=90.0, trigger_val=5.0, accel=10.0)

        # 【EGO 剧本】
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        sm_ego.add_stage(trigger_type='y_less', target_speed=50.0, trigger_val=70.0, accel=25.0)
        sm_ego.add_stage(trigger_type='y_less', target_speed=30.0, trigger_val=30.0, accel=20.0)
        sm_ego.add_stage(trigger_type='y_less', target_speed=60.0, trigger_val=-30.0, accel=20.0)
        sm_ego.add_stage(trigger_type='time', target_speed=90.0, trigger_val=4.0, accel=15.0)

        # 【车4 剧本】
        sm4 = RTB.MultiStageBehaviorMachine(initial_speed=130.0)  # 维持 130km/h 巡航，无需其他阶段

        # 循环索引初始化
        idx1 = idx2 = idx_ego = idx4 = 0
        sim_time = 0.0

        print("\n[仿真开启] 长尾场景开始运行...")

        # ==========================================
        # 7. 仿真主循环
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- V1 控制逻辑 ----------------
            if v1 and v1.is_alive:
                lm1.tick(sim_time)  # 维持双闪特效
                # 🚀【修复核心 2】: 加上刹车灯联动，在减速时亮起红色刹车灯，增强长尾真实感！
                lm1.auto_update_from_control()
                sp1 = sm1.tick(v1.get_location(), sim_time, dt)
                wp1, idx1 = RTB.get_target_waypoint(v1.get_location(), traj1, idx1, sp1)
                if wp1:
                    RTB.apply_pid_control(v1, pid_lon1, pid_lat1, sp1, wp1)
                RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True)

            # ---------------- V2 控制逻辑 ----------------
            if v2 and v2.is_alive:
                sp2 = sm2.tick(v2.get_location(), sim_time, dt)
                wp2, idx2 = RTB.get_target_waypoint(v2.get_location(), traj2, idx2, sp2)
                if wp2:
                    RTB.apply_pid_control(v2, pid_lon2, pid_lat2, sp2, wp2)
                RTB.check_vehicle_out_of_bounds(v2, carla_map, auto_destroy=True)

            # ---------------- Ego 控制逻辑 ----------------
            if ego and ego.is_alive:
                sp_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, sp_ego)
                if wp_ego:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, sp_ego, wp_ego)
                    # 🚀 绘制EGO动态预瞄牵引线
                    RTB.draw_lookahead_point(world, ego.get_location(), wp_ego, color=carla.Color(0, 255, 0))
                RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)

            # ---------------- V4 控制逻辑 ----------------
            if v4 and v4.is_alive:
                sp4 = sm4.tick(v4.get_location(), sim_time, dt)
                wp4, idx4 = RTB.get_target_waypoint(v4.get_location(), traj4, idx4, sp4)
                if wp4:
                    RTB.apply_pid_control(v4, pid_lon4, pid_lat4, sp4, wp4)
                RTB.check_vehicle_out_of_bounds(v4, carla_map, auto_destroy=True)

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