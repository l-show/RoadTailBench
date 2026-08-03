import sys
import carla
import time
import math
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
TRAJ_STR_1 = """
-47.325	-45.555	10.386
-47.325	-45.555	10.386
-47.325	-45.555	10.386
-46.136	-45.337	10.386
-44.888	-45.098	10.952
-43.621	-44.849	11.162
-42.354	-44.597	11.372
-41.128	-44.351	11.372
-39.801	-44.084	11.372
-36.226	-43.365	11.372
-32.427	-42.601	11.372
-28.627	-41.846	10.95
-24.883	-41.122	10.95
-21.283	-40.425	10.95
-19.954	-40.168	10.95
-17.663	-39.725	10.95
-15.127	-39.234	10.95
-13.04	-38.83	10.95
-11.772	-38.585	11.236
-10.548	-38.331	12.149
-9.294	-38.023	15.453
-8.087	-37.628	20.996
-6.902	-37.114	25.012
-5.743	-36.544	28.14
-4.652	-35.892	33.18
-3.62	-35.117	40.547
-2.704	-34.208	51.119
-1.944	-33.165	57.162
-1.364	-32.036	67.662
-0.899	-30.832	69.867
-0.477	-29.633	70.725
-0.103	-28.398	78.161
0.079	-27.14	83.703
0.203	-25.855	85.191
0.305	-24.567	87.072
0.328	-23.296	89.731
0.336	-22.004	89.381
0.353	-20.462	89.381
0.399	-16.212	89.381
0.478	-11.046	88.818
0.601	-5.964	88.468
0.722	-0.799	89.098
0.779	4.201	89.938
0.758	9.367	90.788
0.69	14.45	90.718
0.632	19.617	90.648
0.574	24.7	90.648
0.533	29.866	90.158
0.562	35.032	89.24
0.661	40.031	88.68
0.76	45.114	89.03
0.821	50.28	89.52
0.864	55.363	89.52
0.897	60.447	89.8
0.909	65.445	90.01
0.909	70.444	90.01
0.908	70.777	90.01
0.908	70.777	90.01
0.908	70.777	90.01
"""

TRAJ_STR_EGO = """
4.728	55.663	-95.561
4.728	55.663	-95.561
4.728	55.663	-95.561
4.32	51.442	-94.916
4.012	46.294	-91.847
3.903	41.447	-91.06
3.831	37.578	-91.06
3.833	33.768	-89.224
3.879	29.893	-89.364
3.91	27.164	-89.364
3.927	25.602	-89.364
3.956	23.101	-89.224
3.987	20.559	-89.434
4.018	17.977	-88.799
4.096	15.401	-88.167
4.177	12.866	-88.167
4.246	10.303	-89.221
4.269	7.72	-89.711
4.272	5.179	-90.061
4.258	2.595	-90.341
4.243	0.011	-90.341
4.229	-2.28	-90.341
4.229	-2.28	-90.341
4.229	-2.28	-90.341
4.229	-2.28	-90.341
4.229	-2.28	-90.341
4.217	-4.322	-90.341
4.2	-6.823	-90.411
4.182	-9.407	-90.411
4.163	-11.99	-90.411
4.148	-14.532	-90.271
4.144	-17.073	-89.781
4.161	-19.66	-89.431
4.187	-22.243	-89.431
4.206	-24.203	-89.711
4.206	-24.203	-89.711
4.206	-24.203	-89.711
4.21	-25.328	-89.781
4.041	-27.862	-99.639
3.171	-30.286	-115.593
1.876	-32.472	-122.974
0.435	-34.615	-124.972
-1.116	-36.627	-131.357
-2.974	-38.351	-143.716
-5.103	-39.81	-147.103
-7.251	-41.081	-152.342
-9.604	-42.139	-159.152
-12.029	-42.887	-166.863
-14.551	-43.446	-168.727
-17.021	-43.823	-171.966
-19.578	-44.183	-171.549
-21.308	-44.441	-171.479
-21.308	-44.441	-171.479
-23.242	-44.74	-170.2
-25.779	-45.23	-168.844
-28.558	-45.778	-168.844
-34.485	-46.947	-168.844
-40.822	-48.197	-168.844
-47.158	-49.446	-168.844
-53.494	-50.695	-168.844
-59.73	-51.915	-169.344
-66.077	-53.109	-169.344
"""

TRAJ_STR_3 = """
-79.395	-50.97	7.738
-79.395	-50.97	7.738
-79.188	-50.942	7.668
-76.669	-50.603	7.738
-74.11	-50.249	7.878
-69.776	-49.65	7.878
-64.658	-48.949	7.033
-59.505	-48.664	0.452
-54.345	-48.803	-3.595
-49.27	-49.081	-1.815
-44.274	-49.001	4.423
-39.145	-48.379	8.348
-34.13	-47.552	10.42
-29.061	-46.553	11.908
-24.168	-45.521	11.908
-19.112	-44.455	11.908
-14.138	-43.407	11.908
-9.15	-42.075	19.832
-4.604	-39.827	34.483
-0.984	-36.192	58.205
1.352	-31.677	64.971
2.924	-26.771	77.194
3.862	-21.87	90.833
2.94	-16.887	104.948
1.595	-11.899	105.093
0.573	-6.929	95.716
0.213	-1.778	92.79
0.131	3.386	89.786
0.202	8.468	88.011
0.39	13.631	87.801
0.559	18.795	88.431
0.693	23.877	88.501
0.828	29.045	88.501
0.904	34.211	90.046
0.868	39.377	90.261
0.846	44.46	90.121
0.866	49.627	89.631
0.906	54.793	89.491
0.952	59.877	89.491
0.998	65.043	89.491
1.043	70.21	89.491
1.088	75.21	89.491
1.144	80.293	89.351
1.203	85.46	89.351
1.261	90.626	89.351
1.319	95.709	89.351
1.376	100.792	89.351
1.397	102.626	89.351
1.397	102.626	89.351
1.397	102.626	89.351
"""

def force_spawn_vehicle(world, bp_name, x, y, yaw, z_offset=1.5, role_name="background"):
    """
    【专为无锚点地图定制的强制生成函数】
    功能：跳过 CARLA 地图的 get_waypoint 拓扑检测，强行利用绝对坐标进行物理生成。
    原理：由于轨迹只有 XY 没有 Z，将其在半空（默认 Z=1.5米）生成，让物理引擎的重力使其自然落地，防止卡进地底爆炸。
    """
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(bp_name)

    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', role_name)

    # 强制在设定的 Z 高度生成
    spawn_transform = carla.Transform(
        carla.Location(x=x, y=y, z=z_offset),
        carla.Rotation(yaw=yaw)
    )

    actor = world.try_spawn_actor(bp, spawn_transform)
    if actor:
        print(f"✅ 强制无锚点生成成功 [{bp_name.split('.')[-1]}] | 坐标:({x:.1f}, {y:.1f})")
    else:
        print(f"❌ 强制生成失败! 位置({x:.1f}, {y:.1f}) 可能发生严重物理干涉。")

    return actor

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-66.077	,-53.109)
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
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按照截图数据配置静态天气参数
        RTB.set_static_weather(
            world,
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_azimuth_angle=124.0, sun_altitude_angle=15.0,
            fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照截图参数成功设置。")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (插值到 0.5m)
        # ==========================================
        # 提取原始点
        raw_t1 = RTB.parse_string_trajectory(TRAJ_STR_1, min_dist=0.1)
        raw_tego = RTB.parse_string_trajectory(TRAJ_STR_EGO, min_dist=0.1)
        raw_t3 = RTB.parse_string_trajectory(TRAJ_STR_3, min_dist=0.1)

        # 进行密集插值以保证 PID 控制器丝滑预瞄
        dense_t1 = RTB.interpolate_trajectory(raw_t1, interval=0.5)
        dense_tego = RTB.interpolate_trajectory(raw_tego, interval=0.5)
        dense_t3 = RTB.interpolate_trajectory(raw_t3, interval=0.5)

        # ==========================================
        # 3. 车辆实体强制生成 (调用自定义函数绕过地图锚点检测)
        # ==========================================
        # 第一辆车 (NPC 1)
        v1 = force_spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                 x=raw_t1[0][0], y=raw_t1[0][1], yaw=raw_t1[0][2], role_name='npc1')
        if v1: actor_list.append(v1)

        # 第二辆车 (EGO)
        ego = force_spawn_vehicle(world, 'vehicle.audi.tt',
                                  x=raw_tego[0][0], y=raw_tego[0][1], yaw=raw_tego[0][2], role_name='ego')
        if ego: actor_list.append(ego)

        # 第三辆车 (NPC 2)
        v3 = force_spawn_vehicle(world, 'vehicle.citroen.c3',
                                 x=raw_t3[0][0], y=raw_t3[0][1], yaw=raw_t3[0][2], role_name='npc2')
        if v3: actor_list.append(v3)

        # ==========================================
        # 4. 车辆 PID 控制器独立挂载
        # ==========================================
        pid_lon_1 = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_1 = RTB.PIDLateralController(preset='default_car', dt=dt)

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_ego = RTB.PIDLateralController(preset='default_car', dt=dt)

        pid_lon_3 = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_3 = RTB.PIDLateralController(preset='default_car', dt=dt)

        # ==========================================
        # 5. 车辆灯光管理器
        # ==========================================
        if ego:
            ego_lights = RTB.VehicleLightManager(ego)
            ego_lights.set_static_lights(low_beam=True, high_beam=False)

        # ==========================================
        # 6. EGO 剧本状态机编排
        # ==========================================
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # 阶段1: 坐标越往南走 y 越小。所以触发条件是 y_less
        ego_sm.add_stage(trigger_type='y_less', trigger_val=20.0, target_speed=15.0, accel=25.0)
        # 阶段2: 减速至 10km/h
        ego_sm.add_stage(trigger_type='y_less', trigger_val=-10.0, target_speed=10.0, accel=15.0)
        # 阶段3: 过2秒后恢复至 30km/h
        ego_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=30.0, accel=15.0)
        # 阶段4: 在 y=-44 恢复至 60km/h
        ego_sm.add_stage(trigger_type='y_less', trigger_val=-44.0, target_speed=60.0, accel=20.0)

        # ==========================================
        # 7. 预热与初始状态注入
        # 注意：这里稍微Tick一下世界，让悬空的车辆落地稳定后再注入速度，防止初速度带着下落趋势乱窜
        # ==========================================
        for _ in range(10):
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break

        if v1: RTB.set_vehicle_initial_speed(v1, 20.0, yaw_deg=raw_t1[0][2])
        if ego: RTB.set_vehicle_initial_speed(ego, 60.0, yaw_deg=raw_tego[0][2])
        if v3: RTB.set_vehicle_initial_speed(v3, 65.0, yaw_deg=raw_t3[0][2])

        # 车辆轨迹跟踪最近点索引游标
        idx1, idx_ego, idx3 = 0, 0, 0

        print("[场景运行] 仿真系统正式拉起，按 Ctrl+C 停止运行。")
        # ==========================================
        # 8. 仿真主循环 (帧率同步与环境守护)
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # 【重要修改】：由于地图无锚点，原版的 RTB.check_vehicle_out_of_bounds() 一定会误判出界。
            # 这里已将其彻底移除，完全靠纯几何 PID 循迹保证车辆不乱跑！

            # ---------------- 车辆 1 (恒速 20km/h) ----------------
            if v1 and v1.is_alive:
                wp1, idx1 = RTB.get_target_waypoint(v1.get_location(), dense_t1, idx1, speed_kmh=20.0)
                if wp1:
                    RTB.apply_pid_control(v1, pid_lon_1, pid_lat_1, target_speed_kmh=20.0, target_wp=wp1)

            # ---------------- 车辆 EGO (状态机控制) ----------------
            if ego and ego.is_alive:
                # 状态机 Tick 更新当前需达到的目标速度
                target_speed_ego = ego_sm.tick(ego.get_location(), sim_time, dt)

                # 获取前方预瞄点
                vel = ego.get_velocity()
                ego_real_speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), dense_tego, idx_ego,
                                                          speed_kmh=max(5.0, ego_real_speed))

                if wp_ego:
                    # 执行 PID
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_speed_kmh=target_speed_ego,
                                          target_wp=wp_ego)

            # ---------------- 车辆 3 (恒速 65km/h) ----------------
            if v3 and v3.is_alive:
                wp3, idx3 = RTB.get_target_waypoint(v3.get_location(), dense_t3, idx3, speed_kmh=65.0)
                if wp3:
                    RTB.apply_pid_control(v3, pid_lon_3, pid_lat_3, target_speed_kmh=65.0, target_wp=wp3)

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