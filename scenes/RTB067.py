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
# 原始轨迹数据字典 (字符串格式)
# ==========================================
RAW_TRAJ_DATA = {
    "bus": """
        72.781	17.001	-178.242
        72.781	17.001	-178.242
        72.781	17.001	-178.242
        72.781	17.001	-178.242
        72.781	17.001	-178.242
        72.781	17.001	-178.311
        72.781	17.001	-178.311
        72.29	16.989	-178.944
        71.79	16.98	-178.944
        70.82	16.962	-178.944
        69.55	16.952	-179.873
        68.301	16.953	179.917
        67.03	16.955	179.917
        65.76	16.956	179.847
        64.49	16.964	179.427
        63.219	16.979	179.287
        61.954	16.995	179.287
        60.684	17.011	179.287
        59.434	17.026	179.287
        58.164	17.045	178.65
        56.894	17.079	178.44
        55.624	17.121	177.88
        54.356	17.19	174.995
        53.097	17.362	169.486
        51.862	17.655	163.835
        50.649	18.035	161.56
        49.47	18.452	160.135
        48.275	18.884	160.135
        47.08	19.316	160.135
        45.904	19.74	160.135
        44.708	20.171	160.485
        43.505	20.58	162.843
        42.274	20.894	167.447
        41.006	21.14	170.902
        39.749	21.326	172.522
        38.5	21.469	174.643
        37.232	21.551	177.305
        35.962	21.581	-179.685
        34.691	21.573	-179.615
        33.42	21.564	-179.475
        32.17	21.546	-179.125
        30.899	21.527	-179.125
        29.629	21.507	-179.125
        28.358	21.488	-179.125
        27.087	21.469	-179.125
        25.817	21.436	-177.617
        24.551	21.329	-172.095
        23.304	21.088	-166.011
        22.112	20.715	-158.324
        20.951	20.199	-154.272
        19.839	19.586	-147.536
        18.807	18.845	-140.942
        17.864	17.995	-134.931
        17.017	17.049	-128.306
        16.301	16.001	-118.666
        15.786	14.864	-107.631
        15.424	13.646	-103.496
        15.328	12.381	-90.731
        15.341	11.131	-88.527
        15.373	9.861	-88.527
        15.387	8.59	-90.291
        15.377	7.32	-90.501
        15.366	6.028	-90.501
        15.334	2.403	-90.501
        15.273	-4.578	-90.501
        15.206	-12.202	-90.501
        15.111	-19.826	-90.781
        14.992	-27.45	-90.994
        14.822	-35.074	-91.486
        14.624	-42.696	-91.486
        14.449	-50.319	-90.565
        14.396	-57.82	-90.355
        14.346	-65.445	-90.425
        14.247	-73.069	-91.125
        14.107	-80.693	-90.915
        13.984	-88.317	-90.985
        13.794	-95.939	-91.618
        13.579	-103.559	-91.618
        13.367	-111.058	-91.618
        13.173	-118.68	-91.058
        13.055	-126.304	-90.567
        12.978	-133.93	-90.638
        12.881	-141.554	-90.847
        12.76	-149.177	-90.917
        12.623	-156.796	-91.197
        12.475	-164.418	-91.057
        12.323	-172.041	-91.197
        12.166	-179.54	-91.197
        12.036	-187.164	-90.917
        11.914	-194.788	-90.917
        11.792	-202.411	-90.917
        11.696	-208.411	-90.917
        11.696	-208.411	-90.917
        11.696	-208.411	-90.917
        11.696	-208.411	-90.917
    """,
    "ego": """
        179.942	21.664	176.432
        179.942	21.664	176.432
        179.942	21.664	176.502
        178.028	21.768	176.992
        175.49	21.888	177.415
        172.952	22.005	177.135
        170.455	22.13	177.135
        167.916	22.257	177.135
        165.377	22.384	177.135
        162.836	22.468	178.784
        160.295	22.515	179.134
        157.754	22.548	179.274
        155.212	22.58	179.274
        152.671	22.612	179.274
        150.13	22.644	179.274
        146.505	22.69	179.274
        142.692	22.739	179.274
        138.88	22.811	178.641
        135.069	22.924	178.011
        131.259	23.06	177.871
        127.512	23.199	177.871
        123.701	23.341	177.871
        119.891	23.483	177.871
        116.081	23.624	177.871
        112.268	23.733	178.644
        108.518	23.802	178.996
        104.706	23.85	179.626
        100.894	23.862	-179.814
        97.144	23.837	-179.534
        93.331	23.806	-179.534
        89.518	23.796	179.973
        85.706	23.82	179.063
        81.942	23.906	178.353
        78.132	24.016	178.353
        74.321	24.126	178.353
        70.572	24.234	178.353
        66.761	24.35	178.213
        65.824	24.379	178.213
        64.325	24.426	178.213
        60.514	24.51	179.556
        56.701	24.53	179.698
        52.951	24.55	179.698
        49.139	24.566	179.768
        45.262	24.617	178.77
        41.451	24.707	178.56
        37.64	24.803	178.56
        33.827	24.892	178.77
        30.014	24.974	178.7
        26.234	25.401	166.141
        22.676	26.752	156.086
        19.283	28.47	145.06
        16.485	31.042	127.157
        14.412	34.24	119.881
        13.334	37.878	97.361
        12.918	41.668	95.919
        12.881	45.475	86.856
        12.971	49.286	89.691
        12.991	53.098	89.691
        13.02	56.911	89.551
        13.052	61.057	89.551
        13.121	69.807	89.551
        13.192	78.702	89.268
        13.352	87.485	88.915
        13.576	96.378	88.215
        13.883	105.258	88.005
        14.176	114.144	88.285
        14.39	123.036	89.055
        14.509	131.782	89.265
    """,
    "truck": """
        131.54	19.38	179.861
        131.54	19.38	179.861
        131.54	19.38	179.861
        131.457	19.38	179.861
        128.915	19.393	179.231
        126.374	19.441	178.811
        123.832	19.494	178.811
        121.291	19.547	178.811
        118.75	19.599	178.811
        116.167	19.653	178.811
        113.625	19.706	178.811
        111.084	19.758	178.811
        108.543	19.811	178.811
        106.001	19.864	178.811
        103.46	19.917	178.811
        100.918	19.969	178.811
        98.377	20.022	178.811
        95.794	20.076	178.811
        93.252	20.128	178.811
        90.711	20.181	178.811
        88.17	20.234	178.811
        85.628	20.287	178.811
        83.087	20.339	178.811
        82.379	20.354	178.811
        82.379	20.354	178.811
        80.005	20.403	178.671
        77.516	20.616	169.909
        75.036	21.181	164.863
        72.553	21.894	163.591
        70.115	22.612	163.591
        67.714	23.307	164.503
        65.242	23.896	169.642
        62.734	24.303	172.515
        60.206	24.57	175.301
        57.671	24.755	176.656
        55.131	24.829	179.425
        52.59	24.835	-179.805
        50.048	24.82	-179.525
        47.465	24.794	-179.385
        44.923	24.767	-179.455
        42.382	24.757	-179.875
        39.882	24.755	179.845
        37.298	24.773	179.355
        34.757	24.813	179.005
        32.174	24.858	179.005
        29.633	24.902	179.005
        27.091	24.947	179.005
        24.55	24.991	179.005
        21.967	24.981	-177.477
        19.432	24.805	-174.618
        16.911	24.487	-171.581
        14.356	24.109	-171.581
        11.881	23.754	-172.211
        9.363	23.41	-172.351
        6.798	23.1	-174.335
        4.267	22.859	-175.245
        1.729	22.705	-177.744
        -0.813	22.665	-179.547
        -3.354	22.651	-179.967
        -7.563	22.648	-179.967
        -13.918	22.645	-179.967
        -20.376	22.641	-179.967
        -26.73	22.649	179.54
        -33.084	22.751	178.698
        -39.436	22.895	178.698
        -45.774	23.039	178.698
        -52.127	23.182	178.838
        -58.482	23.311	178.838
        -64.838	23.44	178.838
        -71.191	23.569	178.838
        -77.544	23.678	179.26
        -83.898	23.741	179.54
        -90.356	23.793	179.54
        -96.606	23.846	179.4
        -102.96	23.912	179.4
        -109.314	23.983	179.33
        -115.667	24.049	179.47
        -122.021	24.108	179.47
        -128.374	24.193	178.98
        -134.726	24.306	178.98
        -134.726	24.306	178.98
        -134.726	24.306	178.98
    """,
    "car": """
        19.133	149.177	-89.758
        19.133	149.177	-89.758
        19.133	149.177	-89.758
        19.113	136.887	-90.979
        18.825	124.182	-91.539
        18.462	111.273	-91.822
        18.021	98.574	-92.102
        17.555	85.871	-92.102
        17.144	73.168	-91.332
        16.901	60.671	-90.701
        16.774	47.964	-90.561
        16.647	35.048	-90.561
        16.522	22.321	-90.561
        16.264	9.616	-91.686
        15.826	-3.085	-92.109
        15.422	-15.994	-91.334
        15.215	-28.493	-90.494
        15.121	-41.201	-90.424
        15.016	-53.908	-90.774
        14.713	-66.804	-91.476
        14.386	-79.506	-91.476
        14.059	-92.204	-91.476
        13.727	-105.107	-91.476
        13.426	-117.599	-91.336
        13.106	-130.51	-91.616
        12.735	-143.207	-91.686
        12.377	-155.908	-91.405
        12.117	-168.607	-90.775
        11.975	-181.318	-90.565
        11.823	-194.022	-90.705
        11.705	-203.603	-90.635
        11.705	-203.603	-90.635
        11.705	-203.603	-90.635
        11.705	-203.603	-90.635
    """
}


def setup_agent(world, bp_name, role_name, raw_str, init_speed, pid_preset, z_offset=0.5):
    """
    辅助函数：整合解析轨迹、稠密化、实体生成、速度注入、PID与状态机初始化
    """
    # 1. 轨迹解析与初筛去重 (使用自带的 parse_string_trajectory)
    parsed_traj = RTB.parse_string_trajectory(raw_str, min_dist=0.1)
    if not parsed_traj:
        return None

    # 保存原始的初始航向角，用于完美朝向生成
    spawn_yaw = parsed_traj[0][2]

    # 2. 轨迹稠密化插值 (要求稠密化到 0.5m)
    dense_traj = RTB.interpolate_trajectory(parsed_traj, interval=0.5)

    # 3. 实体生成
    vehicle = RTB.spawn_vehicle(
        world, bp_name,
        x=parsed_traj[0][0], y=parsed_traj[0][1], yaw=spawn_yaw,
        role_name=role_name, z_offset=z_offset
    )
    if not vehicle:
        return None

    # 4. 初速度注入
    RTB.set_vehicle_initial_speed(vehicle, init_speed, yaw_deg=spawn_yaw)

    # 5. PID控制器挂载 (每辆车需要独立的实例)
    pid_lon = RTB.PIDLongitudinalController(preset=pid_preset)
    pid_lat = RTB.PIDLateralController(preset=pid_preset)

    # 6. 状态机初始化
    sm = RTB.MultiStageBehaviorMachine(initial_speed=init_speed)

    return {
        "vehicle": vehicle,
        "traj": dense_traj,  # 稠密化后的追踪轨迹
        "idx": 0,  # 轨迹索引
        "sm": sm,  # 状态机
        "pid_lon": pid_lon,
        "pid_lat": pid_lat,
        "active": True
    }




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (14.509, 131.782)
_RTB_OPT_EGO_TYPE_ID = 'vehicle.mitsubishi.fusorosa'
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

        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照截图参数设置天气系统
        RTB.set_static_weather(world,
                               cloudiness=25.0,
                               precipitation=50.0,
                               precipitation_deposits=75.0,
                               wind_intensity=50.0,
                               sun_azimuth_angle=10.0,
                               sun_altitude_angle=0.0,
                               fog_density=2.0,
                               fog_distance=0.75,
                               fog_falloff=0.1,
                               wetness=100.0,
                               scattering_intensity=1.0,
                               mie_scattering_scale=0.01,
                               rayleigh_scattering_scale=0.01,
                               dust_storm=0.0
                               )
        print("[场景配置] 天气系统已按截图设置完毕")

        # ==========================================
        # 2. 实体生成与剧本编排
        # ==========================================
        agents = []

        # 第一辆：公交车 (Bus)
        # 剧本：静止1s -> 0加到20 -> 等待2s -> 加到55
        # 突破重量限制：使用 truck PID 预设
        agent_bus = setup_agent(world, 'vehicle.mitsubishi.fusorosa', 'bus', RAW_TRAJ_DATA['bus'], 0.0, 'truck',
                                z_offset=1.5)
        if agent_bus:
            # 阶段1：等待3秒后，加速到 20km/h
            agent_bus['sm'].add_stage('time', target_speed=20.0, trigger_val=3.0, accel=10.0)
            # 阶段2：等待2秒后，加速到 55km/h
            agent_bus['sm'].add_stage('time', target_speed=55.0, trigger_val=2.0, accel=15.0)
            agents.append(agent_bus)
            actor_list.append(agent_bus['vehicle'])

        # 第二辆：Ego小轿车 (Ego)
        # 剧本：初始75，x=100减速到50，x=75减速到25，过4s恢复70
        agent_ego = setup_agent(world, 'vehicle.audi.tt', 'ego', RAW_TRAJ_DATA['ego'], 75.0, 'default_car')
        if agent_ego:
            # 注意轨迹是从X=179往X=15行驶，所以条件为 x_less
            agent_ego['sm'].add_stage('x_less', target_speed=50.0, trigger_val=120.0, accel=25.0)
            agent_ego['sm'].add_stage('x_less', target_speed=20.0, trigger_val=90.0, accel=25.0)
            agent_ego['sm'].add_stage('x_less', target_speed=35.0, trigger_val=33.0, accel=25.0)
            agent_ego['sm'].add_stage('time', target_speed=70.0, trigger_val=2.0, accel=15.0)

            # 灯光管理器：开启行车灯
            ego_lights = RTB.VehicleLightManager(agent_ego['vehicle'])
            ego_lights.set_static_lights(low_beam=True)

            agents.append(agent_ego)
            actor_list.append(agent_ego['vehicle'])

        # 第三辆：中型货车 (Truck)
        # 剧本：初始60，x=82减速到25，过2s恢复60
        agent_truck = setup_agent(world, 'vehicle.carlamotors.carlacola', 'truck', RAW_TRAJ_DATA['truck'], 60.0,
                                  'truck', z_offset=1.5)
        if agent_truck:
            # 轨迹从 X=131 往 X=-134 行驶
            agent_truck['sm'].add_stage('x_less', target_speed=25.0, trigger_val=82.0, accel=20.0)
            agent_truck['sm'].add_stage('time', target_speed=60.0, trigger_val=2.0, accel=15.0)
            agents.append(agent_truck)
            actor_list.append(agent_truck['vehicle'])

        # 第四辆：小轿车 (Car)
        # 剧本：恒定30km/h
        agent_car = setup_agent(world, 'vehicle.chevrolet.impala', 'car', RAW_TRAJ_DATA['car'], 30.0, 'default_car')
        if agent_car:
            # 无需添加 stage，保持 initial_speed 即可
            agents.append(agent_car)
            actor_list.append(agent_car['vehicle'])

        # ==========================================
        # 3. 预绘制全量灰色预设轨迹
        # ==========================================
        for agent in agents:
            RTB.draw_preset_trajectory(world, agent['traj'], color=carla.Color(150, 150, 150))

        # ==========================================
        # 4. 仿真主循环
        # ==========================================
        sim_time = 0.0
        print("[主循环] 仿真开始...")

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # 执行每一辆车的控制逻辑
            for agent in agents:
                if not agent['active']:
                    continue

                v = agent['vehicle']

                # 安全守护：车辆出界即销毁 (不会中断主循环)
                if RTB.check_vehicle_out_of_bounds(v, carla_map, auto_destroy=True):
                    agent['active'] = False
                    continue

                v_loc = v.get_location()

                # 剧本状态机 Tick：获取当前帧要求的平滑速度
                target_speed_kmh = agent['sm'].tick(v_loc, sim_time, dt)

                # 动态获取前方预瞄点
                # 若需要让寻路反应更敏捷，可以微调 lookahead_ratio
                target_wp, agent['idx'] = RTB.get_target_waypoint(
                    v_loc, agent['traj'], agent['idx'], target_speed_kmh, lookahead_ratio=0.4
                )

                if target_wp:
                    # 将预瞄点与速度喂给专属 PID 执行器
                    RTB.apply_pid_control(v, agent['pid_lon'], agent['pid_lat'], target_speed_kmh, target_wp)

                    # 可视化 Ego 的动态预瞄点 (仅 Ego 绿色标识以便观察)
                    if v.attributes.get('role_name') == 'ego':
                        RTB.draw_lookahead_point(world, v_loc, target_wp, color=carla.Color(0, 255, 0))

            # 灯光系统刷新 (防止掉帧阻塞)
            if 'ego_lights' in locals():
                ego_lights.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()