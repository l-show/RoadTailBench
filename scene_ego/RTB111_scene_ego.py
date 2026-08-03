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
MOTO_TRAJ_STR = """
-104.253	-96.815	91.923
-104.253	-96.815	91.923
-104.253	-96.815	91.923
-104.253	-96.815	92.063
-104.253	-96.815	92.273
-104.501	-90.263	91.271
-104.432	-81.371	89.278
-104.467	-72.622	90.835
-104.61	-63.727	91.045
-104.814	-54.688	91.464
-105.027	-45.795	91.322
-105.256	-36.902	91.672
-105.512	-28.01	91.178
-105.577	-19.657	90.33
-105.564	-14.493	83.934
-104.238	-9.686	71.135
-102.096	-5.105	53.299
-98.292	-1.771	36.182
-94.116	1.116	29.673
-89.455	3.113	17.072
-84.52	4.313	8.449
-79.455	4.704	1.919
-74.372	4.784	-0.136
-69.373	4.745	-0.626
-64.206	4.707	0.011
-59.206	4.708	0.011
-50.894	4.709	0.011
-40.728	4.645	-0.991
-30.562	4.538	0.036
-20.395	4.579	0.246
-10.229	4.606	-0.244
-0.067	4.444	-1.722
10.088	4.03	-2.435
20.246	3.649	-1.665
30.411	3.493	0.313
40.408	3.573	0.453
50.572	3.599	0.443
60.737	3.543	-0.55
70.902	3.439	-0.76
81.068	3.282	-1.04
88.734	3.142	-1.04
88.734	3.142	-1.04
88.734	3.142	-1.04
88.734	3.142	-1.04
"""

EGO_TRAJ_STR = """
34.259	-0.853	176.204
34.259	-0.853	176.204
34.259	-0.853	176.204
27.635	-0.413	176.274
17.648	-0.233	-179.618
7.316	-0.269	179.816
-2.684	-0.209	179.536
-13.017	-0.13	179.676
-22.933	-0.084	179.746
-30.434	-0.05	179.746
-35.517	-0.028	179.746
-40.601	0.004	178.661
-45.592	0.262	174.887
-50.647	0.784	173.741
-55.782	1.344	174.091
-60.761	1.787	175.712
-65.837	2.036	178.356
-70.921	2.128	179.767
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-74.253	2.081	-178.51
-78.751	2.082	178.925
-83.833	2.154	179.846
-88.912	2.019	-175.148
-93.836	0.886	-155.955
-98.086	-1.854	-135.632
-99.765	-3.918	-120.734
-99.765	-3.918	-120.734
-100.148	-4.563	-120.734
-102.259	-9.172	-107.144
-103.282	-14.146	-97.619
-103.593	-19.133	-91.144
-103.263	-24.197	-80.681
-102.211	-29.17	-77.495
-101.378	-34.18	-84.651
-101.169	-39.258	-89.006
-101.059	-44.34	-88.507
-100.887	-49.42	-87.303
-100.665	-54.416	-87.804
-100.482	-59.496	-87.944
-100.3	-64.576	-87.944
-100.178	-69.658	-90.075
-100.243	-74.74	-90.781
-100.344	-82.321	-90.501
-100.077	-91.211	-86.82
-99.592	-99.947	-86.82
-99.14	-108.831	-87.45
-98.905	-117.718	-89.29
"""

def prepare_trajectory(raw_str):
    """
    【内部轨迹预处理】
    功能：解析纯数值文本，防止yaw被当成高度Z错误插值，最后还原为贴地的 0.5m 间距密集锚点。
    """
    # 1. 字符串清洗并提取 (x, y, yaw)
    raw_tuples = RTB.parse_string_trajectory(raw_str, min_dist=0.1)
    initial_yaw = raw_tuples[0][2] if len(raw_tuples[0]) > 2 else 0.0

    # 2. 剥离 yaw，仅保留 XY 并传入 Z=0.0 给插值器，防止插值混乱
    xy_tuples = [(p[0], p[1], 0.0) for p in raw_tuples]

    # 3. 0.5米稠密化
    dense_tuples = RTB.interpolate_trajectory(xy_tuples, interval=0.5)

    # 4. 转化为符合 RTB 工具链的标准化 carla.Location 对象 (默认抬高 0.5 米防止埋地)
    path_locations = [carla.Location(x=p[0], y=p[1], z=0.5) for p in dense_tuples]

    return path_locations, initial_yaw

# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (-98.905, -117.718)
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

        # 严格按照需求截图定制的天气参数
        weather = RTB.build_weather(
            cloudiness=5.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=45.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已按照参数设置完毕。")

        # ==========================================
        # 2. 轨迹数据解析与绘制
        # ==========================================
        path_moto, yaw_moto = prepare_trajectory(MOTO_TRAJ_STR)
        path_ego, yaw_ego = prepare_trajectory(EGO_TRAJ_STR)

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        # 第一辆：雅马哈摩托车
        moto = RTB.spawn_vehicle(world, 'vehicle.yamaha.yzf', path_moto[0].x, path_moto[0].y, yaw=yaw_moto,
                                 role_name="moto")
        actor_list.append(moto)

        # 第二辆：Ego 小轿车
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', path_ego[0].x, path_ego[0].y, yaw=yaw_ego,
                                role_name="ego")
        actor_list.append(ego)

        # 赋予无视物理阻塞的瞬间初速度 (60km/h)
        RTB.set_vehicle_initial_speed(moto, 60.0)
        RTB.set_vehicle_initial_speed(ego, 60.0)

        # ==========================================
        # 4. 车辆PID控制器挂载与灯光设置
        # ==========================================
        # 摩托车控制器 (使用专属的两轮 motorcycle 预设，防止抽搐侧翻)
        pid_lon_moto = RTB.PIDLongitudinalController(preset='motorcycle')
        pid_lat_moto = RTB.PIDLateralController(preset='motorcycle')
        idx_moto = 0

        # Ego 控制器
        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        idx_ego = 0

        # Ego 灯光系统：按照要求开启 行车灯、近光灯
        light_ego = RTB.VehicleLightManager(ego)
        light_ego.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        # ==========================================
        # 5. 剧本状态机编排
        # ==========================================

        # 【Moto 剧本】：初始40，无其他变化要求，维持 40 km/h。
        sm_moto = RTB.MultiStageBehaviorMachine(initial_speed=40.0)

        # 【Ego 剧本】：初始45 -> x < -74 时减到15 -> 等待5s恢复45。
        # 逻辑：Ego 轨迹的 x 坐标从 0 一路减小到 -159，因此在 -74 处需用 x_less 触发。
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=45.0)
        sm_ego.add_stage('x_less', trigger_val=-55.0, target_speed=15.0, accel=25.0)
        sm_ego.add_stage('time', trigger_val=5.0, target_speed=45.0, accel=15.0)

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

            # ------------- 摩托车 控制 -------------
            if moto and moto.is_alive:
                if RTB.check_vehicle_out_of_bounds(moto, carla_map, auto_destroy=True):
                    moto = None  # 实体已被销毁，释放指针防止后续报错
                else:
                    target_spd = sm_moto.tick(moto.get_location(), sim_time, dt)
                    target_wp, idx_moto = RTB.get_target_waypoint(moto.get_location(), path_moto, idx_moto,
                                                                  speed_kmh=target_spd)
                    if target_wp:
                        RTB.apply_pid_control(moto, pid_lon_moto, pid_lat_moto, target_spd, target_wp)

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

                    # 动态更新 Ego 的刹车灯/转向灯，保留静态配置好的近光灯
                    light_ego.auto_update_from_control()

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