# -*- coding: utf-8 -*-
import sys
import carla
import time
import math
import random

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 2. 轨迹数据硬编码
# ==========================================
RAW_TRAJ_V1 = """
Location_x	Location_y	Rotation_yaw
39.251	-42.61	50.538
40.716	-40.522	58.701
41.978	-38.259	64.692
42.899	-35.856	71.125
43.686	-33.437	74.468
44.243	-31.001	77.643
44.685	-28.533	82.346
44.883	-26.006	89.506
44.71	-23.495	98.406
44.342	-21.005	98.406
43.966	-18.464	98.863
43.398	-16.031	105.14
42.688	-13.524	106.894
41.845	-11.106	111.609
40.768	-8.797	117.335
39.576	-6.504	117.485
38.388	-4.201	116.603
37.273	-1.882	115.395
36.222	0.367	114.735
35.184	2.64	114.521
34.114	4.956	115.469
32.952	7.369	115.755
29.746	14.012	116.071
25.554	20.446	125.498
21.27	26.841	123.204
17.302	32.994	122.516
15.154	36.363	122.516
13.149	39.639	120.964
11.296	42.999	117.417
9.735	46.434	111.026
8.652	50.125	104.273
7.778	53.805	100.76
7.336	57.629	93.537
7.272	61.36	87.943
7.477	65.191	84.49
7.986	69.051	80.103
8.953	72.786	69.703
10.317	76.222	66.673
12.034	79.573	59.625
14.512	83.8	59.625
17.969	89.172	55.671
21.754	94.363	53.297
25.523	99.546	54.199
29.17	104.621	54.761
32.89	109.884	54.479
36.674	114.926	49.68
41.126	119.465	43.269
46.139	123.525	34.223
51.526	127.088	33.369
56.745	130.527	33.579
61.833	134.473	49.638
62.879	140.38	115.466
62.834	140.474	120.571
"""

RAW_TRAJ_EGO = """
Location_x	Location_y	Rotation_yaw
5.174	-64.187	28.96
7.393	-62.948	29.452
9.617	-61.718	28.808
11.808	-60.513	28.808
14.072	-59.268	28.808
16.262	-58.063	28.738
18.454	-56.861	28.738
19.66	-56.2	28.738
21.085	-55.419	28.738
23.316	-54.195	28.738
25.583	-52.954	28.388
27.782	-51.766	28.388
29.968	-50.553	31.459
32.056	-49.179	36.075
34.047	-47.534	42.076
35.8	-45.767	47.68
37.404	-43.826	53.238
38.831	-41.713	57.472
40.194	-39.577	57.472
41.429	-37.371	67.275
42.365	-35.011	69.48
43.147	-32.594	76.672
43.78	-29.335	79.188
44.359	-24.39	90.295
43.526	-19.322	101.309
42.426	-14.291	104.105
40.972	-9.399	108.338
39.04	-4.707	113.74
36.948	-0.015	114.95
34.612	4.579	118.344
32.134	9.114	119.194
29.646	13.373	120.689
26.99	17.842	121.056
24.349	22.227	121.056
21.698	26.456	122.948
18.814	30.69	125.603
15.866	34.808	125.603
12.972	38.885	123.635
10.617	43.402	111.6
8.957	48.246	104.936
8.024	53.186	98.545
7.411	58.182	94.222
7.357	63.348	86.346
8.067	68.427	78.151
9.473	73.183	68.115
11.454	77.84	64.599
13.975	82.326	58.435
14.16	82.626	58.435
"""

RAW_TRAJ_V3 = """
Location_x	Location_y	Rotation_yaw
12.57	50.634	-68.493
13.698	47.726	-67
15.298	44.301	-64.673
16.984	40.89	-61.469
18.978	37.679	-54.829
21.227	34.586	-53.429
23.287	31.776	-54.995
25.654	28.38	-55.359
27.787	25.185	-57.647
29.818	21.88	-59
31.761	18.646	-59
33.747	15.34	-59
34.468	14.286	-58.105
34.468	14.286	-58.105
34.553	14.15	-58.035
34.823	13.716	-58.034
35.321	12.919	-58.034
35.994	11.834	-59.006
36.623	10.752	-60.159
37.245	9.635	-61.27
37.853	8.498	-61.979
38.45	7.38	-61.952
39.044	6.259	-62.256
39.636	5.133	-62.256
40.238	3.984	-62.396
41.736	1.118	-62.645
43.221	-2.324	-70.772
44.086	-6.045	-84.791
43.991	-9.883	-101.031
42.972	-13.469	-100.899
43.314	-17.239	-84.043
43.845	-20.956	-62.273
46.249	-23.828	-61.939
47.166	-27.491	-85.743
47.161	-31.413	-94.342
46.556	-35.155	-104.187
45.351	-38.851	-109.823
43.803	-42.308	-118.35
41.882	-45.451	-124.088
39.542	-48.59	-129.357
36.976	-51.236	-139.587
33.961	-53.598	-143.69
30.806	-55.614	-149.67
27.469	-57.502	-150.594
24.091	-59.38	-151.316
20.801	-61.179	-151.456
17.476	-62.977	-151.451
14.095	-64.817	-151.451
10.818	-66.6	-151.451
5.907	-69.272	-151.451
0.363	-72.288	-151.451
-5.31	-75.374	-151.451
-10.726	-78.321	-151.451
-12.025	-79.028	-151.451
-12.427	-79.247	-151.451
-12.427	-79.247	-151.451
-12.427	-79.247	-151.451

"""

RAW_TRAJ_PED = """
Location_x	Location_y	Rotation_yaw
32.489	-4.092	1.703
32.489	-4.092	1.703
32.489	-4.092	1.703
32.489	-4.092	3.263
36.471	-3.863	-0.543
37.214	-3.927	-18.488
40.901	-5.597	-36.949
43.159	-7.654	-48.742
45.754	-11.11	-58.63
49.646	-17.752	-58.012
55.554	-22.082	-13.176
56.797	-22.344	-2.954
56.797	-22.344	-2.954
56.797	-22.344	-2.954

"""


def clean_and_strip_yaw(data_str):
    """【画龙修复利器】: 精准剥离第三列的 Yaw 数据，防止其被误认为Z轴(海拔)导致车辆腾空"""
    lines = data_str.strip().split('\n')
    raw_pts = []
    yaw_start = 0.0
    got_yaw = False

    for line in lines:
        if 'Location_x' in line or not line.strip(): continue
        parts = line.split()
        if len(parts) >= 3:
            raw_pts.append((float(parts[0]), float(parts[1])))
            if not got_yaw:
                yaw_start = float(parts[2])
                got_yaw = True

    cleaned = RTB.clean_trajectory(raw_pts, min_dist=0.5)
    return cleaned, yaw_start




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (13.975, 82.326)
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

        # 按照用户截图严格配置天气参数
        RTB.set_static_weather(
            world,
            cloudiness=5.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=59.0,
            sun_altitude_angle=27.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按截图完美设置。")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化
        # ==========================================
        clean_1, yaw_1 = clean_and_strip_yaw(RAW_TRAJ_V1)
        clean_ego, yaw_ego = clean_and_strip_yaw(RAW_TRAJ_EGO)
        clean_3, yaw_3 = clean_and_strip_yaw(RAW_TRAJ_V3)
        clean_ped, yaw_ped = clean_and_strip_yaw(RAW_TRAJ_PED)

        # 强行稠密化，确保PID寻点像轨道一样平滑
        traj_1 = RTB.interpolate_trajectory(clean_1, interval=0.5)
        traj_ego = RTB.interpolate_trajectory(clean_ego, interval=0.5)
        traj_3 = RTB.interpolate_trajectory(clean_3, interval=0.5)
        traj_ped = RTB.interpolate_trajectory(clean_ped, interval=0.5)

        # 绘制所有实体的完整寻路轨迹
        RTB.draw_preset_trajectory(world, traj_1, color=carla.Color(255, 0, 0))  # V1：红线
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(0, 255, 0))  # EGO：绿线
        RTB.draw_preset_trajectory(world, traj_3, color=carla.Color(0, 0, 255))  # V3：蓝线
        RTB.draw_preset_trajectory(world, traj_ped, color=carla.Color(255, 255, 255))  # 行人：白线

        # ==========================================
        # 3. 车辆与行人实体安全生成
        # ==========================================
        print("\n--- 正在生成场景实体 ---")
        # 车辆 1: Chevrolet Impala, 60km/h
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=clean_1[0][0], y=clean_1[0][1], yaw=yaw_1,
                               role_name="v1")
        RTB.set_vehicle_initial_speed(v1, 60.0, yaw_deg=yaw_1)
        actor_list.append(v1)

        # EGO 车辆: Audi TT, 60km/h
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=clean_ego[0][0], y=clean_ego[0][1], yaw=yaw_ego,
                                role_name="ego")
        RTB.set_vehicle_initial_speed(ego, 60.0, yaw_deg=yaw_ego)
        actor_list.append(ego)

        # 车辆 3: Lincoln MKZ 2020, 60km/h
        v3 = RTB.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=clean_3[0][0], y=clean_3[0][1], yaw=yaw_3,
                               role_name="v3")
        RTB.set_vehicle_initial_speed(v3, 60.0, yaw_deg=yaw_3)
        actor_list.append(v3)

        # 行人生成
        bp_ped = random.choice(bp_lib.filter('walker.pedestrian.*'))
        ped_tf = carla.Transform(carla.Location(x=clean_ped[0][0], y=clean_ped[0][1], z=1.0),
                                 carla.Rotation(yaw=yaw_ped))
        ped = world.try_spawn_actor(bp_ped, ped_tf)
        if ped:
            print("[RoadTailBench] ✅ 成功生成 行人实体。")
            actor_list.append(ped)

        # ==========================================
        # 4. 车辆PID与行人控制器挂载
        # ==========================================
        # 🚨 极其关键：必须为每辆车独立实例化专属 PID，绝不共用！
        pid_lon_1, pid_lat_1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_3, pid_lat_3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        # 行人控制器挂载 (严格循迹模式)
        ped_ctrl = RTB.PedestrianController(ped, mode='trajectory', target_list=traj_ped)

        idx1, idx_ego, idx3 = 0, 0, 0

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        light_1 = RTB.VehicleLightManager(v1)
        light_1.turn_on(carla.VehicleLightState.Position)  # V1: 行车灯

        light_ego = RTB.VehicleLightManager(ego)
        light_ego.set_static_lights(low_beam=True, high_beam=False)  # EGO: 行车灯+近光灯

        light_3 = RTB.VehicleLightManager(v3)
        light_3.turn_on(carla.VehicleLightState.Position)  # V3: 行车灯

        # ==========================================
        # 6. 剧本状态机编排
        # ==========================================
        # 【V1 剧本】: 初始60 -> 等3s变30 -> 等3s变70
        sm_1 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_1.add_stage('time', target_speed=30.0, trigger_val=3.0, accel=15.0)
        sm_1.add_stage('time', target_speed=70.0, trigger_val=3.0, accel=15.0)

        # 【EGO 剧本】: 初始60 -> X=35时急刹车到10 -> 等3s恢复60
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_ego.add_stage('x_greater', target_speed=10.0, trigger_val=25.0, accel=30.0)  # 刹车力度设大，模拟急刹
        sm_ego.add_stage('time', target_speed=60.0, trigger_val=3.0, accel=15.0)

        # 【V3 剧本】: 初始60 -> X=40时减到20 -> 等1.5s变回60
        sm_3 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_3.add_stage('x_greater', target_speed=20.0, trigger_val=40.0, accel=20.0)
        sm_3.add_stage('time', target_speed=60.0, trigger_val=1.5, accel=25.0)

        # 【行人 剧本】: 初始静止(0)
        sm_ped = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        sm_ped.add_stage('time', target_speed=3.5, trigger_val=0.2, accel=100.0)  # 行人加速度设极大，实现瞬间变走/跑
        sm_ped.add_stage('time', target_speed=6.5, trigger_val=0.2, accel=100.0)

        # ==========================================
        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        print("\n[RoadTailBench] 🚀 预热完毕，长尾仿真场景开始运行...")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ----------------【 车辆 1 控制与守护 】----------------
            if v1 and v1.is_alive:
                if RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True):
                    pass
                else:
                    target_spd_1 = sm_1.tick(v1.get_location(), sim_time, dt)
                    wp_1, idx1 = RTB.get_target_waypoint(v1.get_location(), traj_1, idx1, speed_kmh=target_spd_1)
                    if wp_1:
                        RTB.apply_pid_control(v1, pid_lon_1, pid_lat_1, target_spd_1, wp_1)
                    light_1.auto_update_from_control()

            # ----------------【 EGO 核心控制与守护 】----------------
            if ego and ego.is_alive:
                if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    pass
                else:
                    target_spd_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                    wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego,
                                                              speed_kmh=target_spd_ego)
                    if wp_ego:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_spd_ego, wp_ego)
                        # 绘制 Ego 当前追踪的预瞄点 (绿色)
                        RTB.draw_lookahead_point(world, ego.get_location(), wp_ego, color=carla.Color(0, 255, 0))
                    light_ego.auto_update_from_control()

            # ----------------【 车辆 3 控制与守护 】----------------
            if v3 and v3.is_alive:
                if RTB.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=True):
                    pass
                else:
                    target_spd_3 = sm_3.tick(v3.get_location(), sim_time, dt)
                    wp_3, idx3 = RTB.get_target_waypoint(v3.get_location(), traj_3, idx3, speed_kmh=target_spd_3)
                    if wp_3:
                        RTB.apply_pid_control(v3, pid_lon_3, pid_lat_3, target_spd_3, wp_3)
                    light_3.auto_update_from_control()

            # ----------------【 行人 控制 】----------------
            if ped and ped.is_alive:
                # 行人的状态机速度单位是 m/s，直接输入即可
                ped_target_speed = sm_ped.tick(ped.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=ped_target_speed)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[运行异常] {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()