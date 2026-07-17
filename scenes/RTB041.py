import sys
import carla
import time
import random

# 1. 动态引入标准化函数库路径 (请根据实际情况修改)
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码 (已去除表头，直接输入纯数据)
# ==========================================
RAW_TRAJ_VAN = """
2.117	-19.877	-92.549
2.117	-19.877	-92.549
2.117	-19.877	-92.549
2.117	-19.877	-90.453
2.081	-24.378	-90.453
2.216	-34.542	-87.802
2.651	-44.699	-87.519
3.099	-54.689	-86.951
3.891	-64.872	-84.651
4.85	-74.993	-84.438
5.9	-85.105	-82.574
7.464	-95.15	-79.976
9.262	-105.326	-79.976
11.376	-115.252	-67.899
15.745	-124.419	-71.117
18.649	-134.157	-76.09
20.76	-144.103	-78.355
22.834	-154.055	-78.213
25.201	-163.938	-74.02
28.095	-173.509	-72.871
31.161	-183.376	-72.591
34.487	-192.981	-69.746
38.128	-202.469	-68.167
41.946	-211.888	-67.815
45.942	-221.415	-66.33
50.097	-230.693	-65.267
54.462	-240.058	-64.982
58.769	-249.267	-64.912
62.168	-256.526	-64.912
62.168	-256.526	-64.912
62.168	-256.526	-64.912
"""

RAW_TRAJ_SEDAN = """
19.77	-169.788	106.501
19.77	-169.788	106.501
19.77	-169.788	106.501
19.77	-169.788	106.501
17.995	-163.795	106.501
14.475	-151.799	106.218
11.348	-139.271	102.211
8.715	-127.051	102.141
6.046	-114.413	101.296
3.952	-102.088	98.851
2.049	-89.523	97.082
0.673	-76.89	95.295
-0.243	-64.215	93.933
-1.069	-51.325	92.868
-1.545	-38.623	91.874
-1.895	-25.92	90.593
-1.823	-13.004	87.78
-1.257	-0.309	87.844
-0.85	12.392	88.264
-0.445	25.302	87.769
0.146	38.205	87.279
0.807	50.896	86.638
1.601	63.789	86.426
2.397	76.681	86.496
2.97	86.038	86.496
2.97	86.038	86.496
2.97	86.038	86.496
"""

RAW_TRAJ_EGO = """
6.47	68.542	-92.325
6.47	68.542	-92.325
6.445	67.961	-92.817
6.378	66.674	-92.96
6.297	65.41	-93.786
6.153	63.233	-93.786
5.901	59.439	-93.786
5.646	55.643	-93.996
5.369	51.844	-94.206
5.082	47.98	-93.623
4.842	44.175	-93.623
4.605	40.37	-93.553
4.368	36.565	-93.553
4.048	31.409	-93.553
3.682	25.065	-92.843
3.405	18.717	-92.698
3.106	12.37	-92.698
2.798	5.919	-92.838
2.483	-0.427	-92.838
2.178	-6.776	-92.412
2	-13.129	-90.567
2.026	-19.482	-89.074
2.118	-25.94	-89.354
2.142	-32.294	-89.991
2.185	-38.647	-88.626
2.368	-45.104	-87.923
2.617	-51.557	-87.643
2.954	-58.007	-86.713
3.442	-64.345	-85.142
4.011	-70.674	-83.78
4.732	-77.092	-83.567
5.44	-83.302	-82.929
6.28	-89.704	-82.497
6.797	-93.628	-82.497
7.468	-98.687	-81.784
8.489	-104.959	-80.156
8.507	-105.061	-80.156
9.793	-110.411	-68.563
12.491	-116.27	-61.55
13.234	-117.644	-71.226
13.234	-117.644	-71.226
13.234	-117.644	-71.226
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.296
13.234	-117.644	-71.863
14.006	-120.021	-72.003
15.95	-126.07	-72.712
17.712	-132.169	-76.096
18.962	-138.392	-82.911
19.447	-144.727	-86.308
19.862	-151.068	-85.446
20.611	-157.397	-80.588
20.628	-157.5	-79.943
20.628	-157.5	-79.731
20.628	-157.5	-79.378
20.839	-158.626	-79.378
22.066	-164.966	-78.017
23.632	-171.121	-73.406
25.452	-177.208	-73.266
27.282	-183.294	-73.266
29.317	-189.311	-68.789
31.647	-195.223	-68.437
33.987	-201.242	-69.794
36.154	-207.215	-70.079
38.405	-213.157	-68.722
40.925	-218.987	-63.915
43.744	-224.683	-63.566
46.549	-230.385	-64.126
48.412	-234.228	-64.126
48.412	-234.228	-64.126
48.412	-234.228	-64.126
"""

RAW_TRAJ_PED = """
43.273	-131.765	179.63
43.273	-131.765	179.63
43.273	-131.765	179.63
43.273	-131.765	179.982
43.273	-131.765	179.274
39.838	-131.695	178.707
33.965	-131.556	178.637
28.742	-131.419	178.139
27.304	-131.37	-177.561
27.304	-131.37	-169.733
24.938	-132.125	-160.176
19.133	-134.342	-158.738
13.249	-136.701	-157.324
8.183	-138.892	-153.144
4.84	-140.585	-153.372
-0.95	-143.441	-154.597
-2.455	-144.156	-154.597
-2.643	-144.245	-154.597
-3.302	-144.558	-154.597
-3.302	-144.558	-154.597
"""




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (48.412, -234.228)
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
        bp_lib = world.get_blueprint_library()
        dt = 0.05
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        # 开启严格的同步模式
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照要求截图设置静态天气参数
        RTB.set_static_weather(world,
                               cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
                               wind_intensity=10.0, sun_azimuth_angle=-1.0, sun_altitude_angle=15.0,
                               fog_density=2.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
                               scattering_intensity=1.0, mie_scattering_scale=0.03,
                               rayleigh_scattering_scale=0.0331, dust_storm=0.0
                               )
        print("[场景配置] 天气系统已按照指定参数设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析与清洗、轨迹可视化
        # ==========================================
        traj_van = RTB.parse_string_trajectory(RAW_TRAJ_VAN, min_dist=0.5)
        traj_sedan = RTB.parse_string_trajectory(RAW_TRAJ_SEDAN, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)
        traj_ped = RTB.parse_string_trajectory(RAW_TRAJ_PED, min_dist=0.2)

        # 绘制所有车辆的全局预设轨迹锚点（灰色）
        RTB.draw_preset_trajectory(world, traj_van, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_sedan, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(150, 150, 150))
        # 绘制行人轨迹（蓝色方便区分）
        RTB.draw_preset_trajectory(world, traj_ped, color=carla.Color(0, 0, 255))

        # ==========================================
        # 3. 实体生成与初始速度注入
        # ==========================================
        # [车辆1] 小货车 (使用工业级安全生成器生成，并应用物理偏航角)
        van = RTB.spawn_vehicle(world, 'vehicle.mercedes.sprinter', x=traj_van[0][0], y=traj_van[0][1],
                                yaw=traj_van[0][2], role_name='van')
        if van: actor_list.append(van)

        # [车辆2] 小轿车 Audi TT
        sedan = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj_sedan[0][0], y=traj_sedan[0][1],
                                  yaw=traj_sedan[0][2], role_name='sedan')
        if sedan: actor_list.append(sedan)

        # [车辆3] Ego Impala
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj_ego[0][0], y=traj_ego[0][1],
                                yaw=traj_ego[0][2], role_name='hero')
        if ego: actor_list.append(ego)

        # [行人]
        walker_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        walker_spawn_tf = carla.Transform(carla.Location(x=traj_ped[0][0], y=traj_ped[0][1], z=1.5),
                                          carla.Rotation(yaw=traj_ped[0][2]))
        walker = world.try_spawn_actor(walker_bp, walker_spawn_tf)
        if walker: actor_list.append(walker)

        # 确保全部生成后再统一推进世界一帧，让物理引擎就绪
        world.tick()

        # 瞬间注入初始物理速度 (无缝衔接PID，防原地打滑)
        RTB.set_vehicle_initial_speed(van, target_speed_kmh=70.0)
        RTB.set_vehicle_initial_speed(sedan, target_speed_kmh=40.0)
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=80.0)

        # ==========================================
        # 4. 控制器、灯光管理器配置
        # ==========================================
        # 为了突破 Carla 对重型车(Sprinter)加速度较弱的限制，采用卡车(truck)大扭矩预设 PID
        van_pid_lon = RTB.PIDLongitudinalController(preset='truck')
        van_pid_lat = RTB.PIDLateralController(preset='truck')

        sedan_pid_lon = RTB.PIDLongitudinalController(preset='default_car')
        sedan_pid_lat = RTB.PIDLateralController(preset='default_car')

        ego_pid_lon = RTB.PIDLongitudinalController(preset='default_car')
        ego_pid_lat = RTB.PIDLateralController(preset='default_car')

        # 灯光配置
        sedan_lights = RTB.VehicleLightManager(sedan)
        sedan_lights.set_static_lights(low_beam=True)  # 开启行车灯

        ego_lights = RTB.VehicleLightManager(ego)
        ego_lights.set_static_lights(low_beam=True)  # 开启行车灯

        # 行人控制器挂载 (严格循迹模式)
        ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped)

        # ==========================================
        # 5. 复杂剧本：多阶段状态机编排
        # ==========================================
        # [Ego状态机]: 初始 80km/h。
        # 注意: 根据轨迹坐标，穿越 -117 的是 Y 轴而不是 X 轴，因此触发器修正为 'y_less'。
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=80.0)
        ego_sm.add_stage(trigger_type='y_less', trigger_val=-100.0, target_speed=30.0, accel=40.0)  # 减速
        ego_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=60.0, accel=15.0)  # 等待2秒后恢复至60

        # [行人状态机]: 初始行走速度 1.5m/s。
        ped_sm = RTB.MultiStageBehaviorMachine(initial_speed=1.5)
        # 前2秒维持1.5m/s，2秒后触发奔跑，加速度设极大以模拟瞬间爆发
        ped_sm.add_stage(trigger_type='time', trigger_val=2.0, target_speed=4.5, accel=100.0)

        # 车辆寻路索引初始化
        idx_van, idx_sedan, idx_ego = 0, 0, 0

        # ==========================================
        # 6. 仿真主循环
        # ==========================================
        print("\n[仿真启动] 开始执行长尾场景主循环...")
        while True:
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # 出界拦截守护 (如判定出界将自动销毁实体)
            RTB.check_vehicle_out_of_bounds(van, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(sedan, carla_map, auto_destroy=True)
            RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)

            # ---------------- 小货车控制 ----------------
            if van and van.is_alive:
                van_wp, idx_van = RTB.get_target_waypoint(van.get_location(), traj_van, idx_van, speed_kmh=70.0)
                if van_wp:
                    RTB.apply_pid_control(van, van_pid_lon, van_pid_lat, target_speed_kmh=70.0, target_wp=van_wp)

            # ---------------- 小轿车控制 ----------------
            if sedan and sedan.is_alive:
                sedan_wp, idx_sedan = RTB.get_target_waypoint(sedan.get_location(), traj_sedan, idx_sedan,
                                                              speed_kmh=40.0)
                if sedan_wp:
                    RTB.apply_pid_control(sedan, sedan_pid_lon, sedan_pid_lat, target_speed_kmh=40.0,
                                          target_wp=sedan_wp)
                sedan_lights.auto_update_from_control()  # 动态刹车/转向灯联动

            # ---------------- Ego 控制 (结合状态机) ----------------
            if ego and ego.is_alive:
                # 从状态机获取当前帧的目标平滑速度
                current_ego_speed = ego_sm.tick(ego.get_location(), sim_time, dt)
                ego_wp, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego,
                                                          speed_kmh=current_ego_speed)

                if ego_wp:
                    RTB.apply_pid_control(ego, ego_pid_lon, ego_pid_lat, target_speed_kmh=current_ego_speed,
                                          target_wp=ego_wp)
                    # 动态绘制 Ego 正在追踪的预瞄点及牵引线 (亮绿色)
                    RTB.draw_lookahead_point(world, ego.get_location(), ego_wp, color=carla.Color(0, 255, 0))

                ego_lights.auto_update_from_control()

            # ---------------- 行人控制 (结合状态机) ----------------
            if walker and walker.is_alive:
                # 从状态机获取当前应该行走的速度 (1.5 突变为 4.5)
                current_ped_speed = ped_sm.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=current_ped_speed)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # ==========================================
        # 环境恢复与安全清扫
        # ==========================================
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()