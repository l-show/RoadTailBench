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
# 轨迹数据硬编码区域 (去除了表头，方便解析)
# ==========================================
RAW_TRAJ_V1 = """
0.662	-2.863	86.541
0.662	-2.863	86.541
0.662	-2.863	86.401
0.662	-2.863	86.121
0.73	-1.866	86.121
1.219	5.737	87.176
1.443	13.357	89.162
1.546	20.982	89.442
1.566	28.606	90.141
1.498	36.105	90.704
1.349	43.855	91.194
1.195	51.477	90.914
1.149	59.102	90.004
1.149	66.601	90.004
1.148	74.226	90.004
1.148	81.851	90.004
1.147	89.602	90.004
1.175	97.226	89.444
1.249	104.851	89.444
1.287	108.726	89.444
"""

RAW_TRAJ_EGO = """
-63.613	-48.935	9.507
-61.441	-48.553	10.072
-58.984	-48.107	10.352
-56.485	-47.65	10.352
-53.986	-47.194	10.352
-51.488	-46.733	10.633
-49.032	-46.262	11.336
-46.541	-45.76	11.616
-44.051	-45.249	11.546
-41.561	-44.74	11.546
-39.071	-44.232	11.546
-36.58	-43.725	11.476
-34.089	-43.22	11.406
-31.596	-42.726	11.126
-29.101	-42.235	11.126
-26.607	-41.744	11.126
-24.113	-41.254	11.126
-21.66	-40.771	11.126
-19.163	-40.297	9.479
-16.643	-39.97	4.608
-14.105	-39.847	-0.122
-11.569	-39.999	-6.061
-11.031	-40.057	-6.061
-10.326	-40.131	-6.061
-7.796	-40.37	-1.354
-5.281	-40.082	17.625
-3.029	-38.925	34.965
-1.115	-37.26	44.035
0.396	-35.31	70.042
1.048	-32.859	81.144
1.334	-30.334	83.968
1.257	-27.8	96.58
0.879	-25.287	99.035
0.54	-22.769	95.161
0.388	-20.232	91.716
0.338	-17.691	90.801
0.318	-15.131	90.171
0.317	-14.923	90.171
0.34	-14.548	86.371
0.495	-12.012	86.939
0.612	-9.431	87.569
0.718	-6.933	87.569
0.819	-4.394	87.849
0.895	-1.853	88.761
0.95	0.688	88.761
1.007	3.354	88.761
1.14	9.498	88.761
1.35	19.215	88.761
1.496	29.38	89.687
1.524	39.55	89.967
1.53	49.55	89.967
1.536	59.716	89.967
1.563	69.716	89.547
"""

RAW_TRAJ_V3 = """
5.261	36.066	-88.155
5.348	33.738	-87.515
5.443	31.201	-88.145
5.507	28.661	-89.057
5.518	26.12	-89.902
5.524	23.058	-89.902
5.53	19.245	-89.902
5.536	15.495	-89.902
5.543	11.682	-89.902
5.55	7.869	-89.902
5.557	4.056	-89.972
5.555	0.243	-90.042
5.552	-3.569	-90.042
5.549	-7.319	-90.042
5.546	-11.131	-90.112
5.528	-14.881	-90.394
5.464	-18.695	-91.387
5.317	-22.505	-93.669
4.931	-26.234	-98.578
4.22	-29.979	-104.327
2.974	-33.576	-114.181
1.029	-36.763	-130.095
-1.557	-39.548	-143.111
-4.952	-41.219	-161.82
-8.575	-42.406	-162.47
-12.289	-43.244	-171.884
-16.001	-43.774	-171.884
-19.769	-44.35	-169.647
-23.505	-45.106	-168.146
-27.664	-45.979	-168.146
-34.841	-47.486	-168.146
-42.195	-49.003	-168.426
-49.675	-50.477	-169.555
-57.07	-51.733	-170.967
-64.601	-52.93	-170.967
-72.129	-54.139	-170.319
-79.614	-55.588	-167.9
-85.746	-56.903	-167.9
-87.05	-57.182	-167.9
"""

RAW_TRAJ_PED = """
-3.615	-3.633	159.297
-3.74	-3.587	159.647
-4.214	-3.405	158.522
-4.684	-3.211	157.457
-5.151	-3.011	156.407
-5.612	-2.796	153.253
-6.023	-2.505	129.347
-6.317	-2.095	105.412
-6.402	-1.598	85.702
-6.261	-1.124	62.279
-5.86	-0.847	17.38
-5.354	-0.863	-8.068
-4.886	-1.072	-25.51
-4.43	-1.276	-23.992
-3.958	-1.486	-23.922
-3.492	-1.691	-23.641
-3.034	-1.891	-23.641
-2.569	-2.095	-23.641
-2.546	-2.105	-23.641
"""

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (1.563	,69.716)
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
        # 1. 环境初始化：帧率同步与精确天气复刻
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照你提供的天气截图数值进行赋值
        RTB.set_static_weather(world,
                               cloudiness=0.0, precipitation=0.0, precipitation_deposits=0.0,
                               wind_intensity=10.0, sun_azimuth_angle=200.0, sun_altitude_angle=5.0,
                               fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
                               scattering_intensity=2.0, mie_scattering_scale=0.03,
                               rayleigh_scattering_scale=0.0331, dust_storm=0.0
                               )
        print("[场景配置] 同步模式开启，精细天气系统已设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析与清洗 (去重与稀疏化)
        # ==========================================
        traj_v1 = RTB.parse_string_trajectory(RAW_TRAJ_V1, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)
        traj_v3 = RTB.parse_string_trajectory(RAW_TRAJ_V3, min_dist=0.5)
        traj_ped = RTB.parse_string_trajectory(RAW_TRAJ_PED, min_dist=0.2)  # 行人步幅较小，阈值调低

        # ==========================================
        # 3. 实体生成与控制器绑定
        # ==========================================

        # --- 车辆1 (Chevrolet Impala) ---
        v1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                               x=traj_v1[0][0], y=traj_v1[0][1], yaw=traj_v1[0][2], role_name='npc_1')
        actor_list.append(v1)
        pid_v1_lon = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_v1_lat = RTB.PIDLateralController(preset='default_car', dt=dt)
        v1_lights = RTB.VehicleLightManager(v1)

        # --- 车辆2 (Ego: Audi TT) ---
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2], role_name='ego')
        actor_list.append(ego)
        pid_ego_lon = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_ego_lat = RTB.PIDLateralController(preset='default_car', dt=dt)
        ego_lights = RTB.VehicleLightManager(ego)

        # --- 车辆3 (Citroen C3) ---
        v3 = RTB.spawn_vehicle(world, 'vehicle.citroen.c3',
                               x=traj_v3[0][0], y=traj_v3[0][1], yaw=traj_v3[0][2], role_name='npc_2')
        actor_list.append(v3)
        pid_v3_lon = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_v3_lat = RTB.PIDLateralController(preset='default_car', dt=dt)

        # --- 行人生成 ---
        bp_ped = bp_lib.filter('walker.pedestrian.*')[0]
        ped_spawn_loc = carla.Location(x=traj_ped[0][0], y=traj_ped[0][1], z=0.5)
        walker = world.try_spawn_actor(bp_ped, carla.Transform(ped_spawn_loc, carla.Rotation(yaw=traj_ped[0][2])))
        if walker:
            actor_list.append(walker)
            # 行人控制中枢，使用严格轨迹模式
            ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped, default_speed=3)
            print("[实体生成] 行人生成成功。")

        # ==========================================
        # 4. 剧本状态机编排与初始速度注入
        # ==========================================

        # V1剧本: 初始0km/h，等待5秒，然后加速到60km/h
        RTB.set_vehicle_initial_speed(v1, 0.0)
        sm_v1 = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        sm_v1.add_stage('time', trigger_val=5.0, target_speed=60.0, accel=20.0)
        v1_lights.start_flashing(mode='hazard')  # 初始开启双闪

        # EGO剧本: 初始50，x=-11减速到20，x=0减速到0，等2s到30，等2s到55
        RTB.set_vehicle_initial_speed(ego, 50.0)
        ego_lights.set_static_lights(low_beam=True)  # 开启行车灯
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=50.0)
        sm_ego.add_stage('x_greater', trigger_val=0.0, target_speed=20.0, accel=20.0)
        sm_ego.add_stage('y_less', trigger_val=-13.0, target_speed=0.0, accel=25.0)
        sm_ego.add_stage('time', trigger_val=2.0, target_speed=30.0, accel=15.0)
        sm_ego.add_stage('time', trigger_val=2.0, target_speed=55.0, accel=15.0)

        # V3剧本: 全程50km/h
        RTB.set_vehicle_initial_speed(v3, 50.0)
        sm_v3 = RTB.MultiStageBehaviorMachine(initial_speed=50.0)

        # 寻迹指针索引初始化
        idx_v1, idx_ego, idx_v3 = 0, 0, 0

        # 让引擎缓冲一下，应用初始速度
        for _ in range(5): world.tick()

        # ==========================================
        # 5. 仿真主循环
        # ==========================================
        print("[主循环] 仿真正式开始...")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 车辆出界守护清理 ----------------
            RTB.check_vehicle_out_of_bounds(v1, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(v3, carla_map, auto_destroy=True)

            # ---------------- 车辆 1 (Impala) 逻辑 ----------------
            if v1 and v1.is_alive:
                # 状态机获取当前平滑速度
                spd_v1 = sm_v1.tick(v1.get_location(), sim_time, dt)

                # 预瞄与控制
                wp_v1, idx_v1 = RTB.get_target_waypoint(v1.get_location(), traj_v1, idx_v1, spd_v1)
                if wp_v1:
                    RTB.apply_pid_control(v1, pid_v1_lon, pid_v1_lat, spd_v1, wp_v1)
                else:
                    v1.apply_control(carla.VehicleControl(brake=1.0))  # 走到终点刹车

                # 灯光剧本联动：一旦进入第一个阶段(结束了前5秒的等待)，关闭双闪
                if sm_v1.current_idx > 0:
                    v1_lights.stop_flashing()
                else:
                    v1_lights.tick(sim_time)

            # ---------------- 车辆 2 (EGO TT) 逻辑 ----------------
            if ego and ego.is_alive:
                # 状态机获取当前平滑速度
                spd_ego = sm_ego.tick(ego.get_location(), sim_time, dt)

                # 预瞄与控制
                wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, spd_ego)
                if wp_ego:
                    RTB.apply_pid_control(ego, pid_ego_lon, pid_ego_lat, spd_ego, wp_ego)
                else:
                    ego.apply_control(carla.VehicleControl(brake=1.0))

                # 智能车灯管理：联动物理刹车板
                ego_lights.auto_update_from_control()

            # ---------------- 车辆 3 (C3) 逻辑 ----------------
            if v3 and v3.is_alive:
                spd_v3 = sm_v3.tick(v3.get_location(), sim_time, dt)
                wp_v3, idx_v3 = RTB.get_target_waypoint(v3.get_location(), traj_v3, idx_v3, spd_v3)
                if wp_v3:
                    RTB.apply_pid_control(v3, pid_v3_lon, pid_v3_lat, spd_v3, wp_v3)
                else:
                    v3.apply_control(carla.VehicleControl(brake=1.0))

            # ---------------- 行人控制逻辑 ----------------
            if walker and walker.is_alive:
                # 第5秒严格销毁要求
                if sim_time >= 5.0:
                    walker.destroy()
                    print(f"[事件触发] 仿真时间到达 {sim_time:.1f}s，行人已被强制清除。")
                else:
                    ped_ctrl.run_step(dt, sim_time)  # 否则按轨迹巡航行走

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
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()