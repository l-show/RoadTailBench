import sys
import carla
import time

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (91.702, -50.448)
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

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 严格按照截图数值设置天气参数
        weather = carla.WeatherParameters(
            cloudiness=30.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=10.0,
            sun_azimuth_angle=119.0,
            sun_altitude_angle=7.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=1.0,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0631,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 静态天气系统已按要求完全覆盖配置。")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 货车轨迹 (去除表头，直接放入字符串)
        raw_traj_truck = """
        -13.604	181.54	-89.926
        -13.604	181.54	-89.926
        -13.604	181.54	-89.926
        -13.604	181.54	-89.996
        -13.63	172.588	-90.172
        -13.173	163.595	-81.392
        -11.478	154.914	-80.009
        -10.783	146.026	-90.03
        -11.172	137.079	-93.737
        -11.561	128.056	-88.177
        -10.698	119.095	-81.825
        -9.256	110.363	-80.287
        -8.49	101.375	-91.769
        -9.005	92.429	-93.443
        -9.404	83.598	-89.833
        -8.881	74.61	-83.148
        -7.634	65.74	-82.9
        -6.962	56.853	-88.928
        -7.084	47.804	-91.994
        -7.359	39.184	-91.274
        -7.228	30.033	-86.882
        -6.646	21.272	-85.829
        -5.786	12.412	-83.749
        -4.806	3.463	-83.749
        -3.956	-5.414	-85.171
        -3.298	-14.296	-87.188
        -2.972	-23.226	-89.032
        -2.851	-32.119	-89.414
        -2.821	-41.054	-90.102
        -2.692	-50.034	-88.262
        -2.413	-58.87	-87.732
        -2.008	-67.914	-87.425
        -1.556	-77.97	-87.425
        -0.963	-91.159	-87.425
        -0.221	-107.665	-87.425
        0.627	-126.517	-87.425
        1.462	-145.561	-87.963
        1.997	-164.618	-88.641
        2.535	-183.672	-87.904
        3.562	-202.707	-85.607
        5.019	-221.712	-85.881
        6.095	-240.742	-87.036
        7.081	-259.774	-87.036
        7.872	-275.062	-87.036
        7.872	-275.062	-87.036
        7.872	-275.062	-87.036
        """

        # 轿车轨迹
        raw_traj_audi = """
        -6.68	-195.794	95.473
        -6.68	-195.794	95.473
        -6.68	-195.794	95.473
        -6.68	-195.794	96.388
        -7.828	-185.62	95.737
        -8.469	-174.236	92.596
        -8.876	-162.845	91.645
        -9.119	-151.393	90.976
        -9.292	-139.962	91.018
        -9.523	-128.211	91.268
        -9.922	-116.64	92.722
        -10.471	-105.05	92.512
        -10.81	-93.681	91.239
        -11.235	-82.066	93.356
        -11.9	-70.643	93.216
        -12.504	-59.266	92.866
        -12.795	-47.864	90.515
        -12.935	-36.324	91.194
        -13.202	-24.751	91.334
        -13.691	-13.323	93.261
        -14.345	-1.842	93.261
        -15.011	9.844	93.261
        -15.651	21.196	93.19
        -16.192	32.687	92.184
        -16.652	43.935	92.31
        -17.057	55.495	91.702
        -17.384	66.493	91.702
        -17.723	77.92	91.702
        -18.069	89.484	91.772
        -18.449	100.919	92.192
        -18.902	112.39	92.262
        -19.418	124.041	92.715
        -19.963	135.54	92.607
        -20.471	146.983	92.537
        -20.979	158.439	92.537
        -21.487	169.927	92.467
        -21.96	181.254	92.224
        -22.399	192.841	92.294
        -22.899	204.252	92.539
        -23.417	215.919	92.539
        -23.923	227.335	92.539
        -24.34	236.739	92.539
        -24.34	236.739	92.539
        """

        # Ego 轨迹
        raw_traj_ego = """
        -11.468	215.363	-91.092
        -11.468	215.363	-91.092
        -11.468	215.363	-91.092
        -11.468	215.363	-91.092
        -11.481	214.822	-91.302
        -11.528	212.332	-90.629
        -11.551	209.758	-90.209
        -11.507	207.252	-88.225
        -11.428	204.676	-88.225
        -11.318	201.133	-88.225
        -11.171	197.358	-87.063
        -10.975	193.53	-87.063
        -10.785	189.718	-87.588
        -10.634	185.894	-87.744
        -10.482	182.031	-87.744
        -10.255	176.289	-87.744
        -10.006	169.958	-87.744
        -9.778	163.604	-88.164
        -9.57	157.278	-88.024
        -9.352	150.962	-88.024
        -9.135	144.671	-88.024
        -8.916	138.322	-88.024
        -8.867	136.896	-88.024
        -8.867	136.896	-88.024
        -8.867	136.896	-88.024
        -8.867	136.896	-88.024
        -8.867	136.896	-88.024
        -8.831	135.94	-87.884
        -8.6	129.672	-87.884
        -8.364	123.291	-87.884
        -8.128	116.899	-87.884
        -7.894	110.567	-87.884
        -7.656	104.123	-87.884
        -7.423	97.827	-87.884
        -7.187	91.392	-88.024
        -6.968	85.048	-88.024
        -6.729	78.58	-87.649
        -6.456	72.274	-86.875
        -5.87	65.753	-82.496
        -4.867	59.476	-79.241
        -3.507	53.248	-75.44
        -1.627	47.288	-71.216
        0.546	41.236	-69.897
        2.754	35.241	-69.582
        5.069	29.334	-67.781
        7.569	23.444	-65.389
        10.386	17.77	-61.91
        13.658	12.28	-56.647
        17.262	7.115	-54.235
        21.116	1.889	-53.291
        24.803	-3.055	-53.291
        28.615	-8.168	-53.291
        32.467	-13.312	-53.051
        36.276	-18.325	-52.594
        40.262	-23.459	-52.139
        41.633	-25.223	-52.139
        42.055	-25.767	-52.139
        43.481	-27.601	-52.139
        45.014	-29.572	-52.139
        46.573	-31.607	-53.278
        48.048	-33.584	-53.278
        49.608	-35.66	-51.981
        51.179	-37.596	-50.105
        52.842	-39.516	-47.573
        54.615	-41.345	-44.061
        56.511	-43.048	-39.79
        58.454	-44.588	-36.476
        60.566	-46.017	-31.107
        62.77	-47.212	-25.307
        65.115	-48.196	-20.661
        67.503	-49.059	-18.81
        69.916	-49.825	-16.635
        72.344	-50.539	-15.622
        74.781	-51.162	-12.033
        77.276	-51.506	-3.59
        79.791	-51.507	2.794
        82.305	-51.31	5.48
        84.812	-51.069	5.48
        87.354	-50.833	5.059
        89.906	-50.607	5.059
        91.702	-50.448	5.059
        91.702	-50.448	5.059
        91.702	-50.448	5.059
        """

        # 调用标准化库清洗并解析为元组列表 (保留XY和Yaw)
        traj_truck = RTB.parse_string_trajectory(raw_traj_truck, min_dist=0.5)
        traj_audi = RTB.parse_string_trajectory(raw_traj_audi, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(raw_traj_ego, min_dist=0.5)

        # 🚀 在地图上画出三辆车的全部预设路线点
        RTB.draw_preset_trajectory(world, traj_truck, color=carla.Color(200, 0, 0))  # 货车: 红色
        RTB.draw_preset_trajectory(world, traj_audi, color=carla.Color(0, 0, 200))  # 奥迪: 蓝色
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(150, 150, 150))  # EGO: 灰色

        # ==========================================
        # 3. 车辆实体安全生成
        # ==========================================
        # 货车 (需要抬高 Z 轴，防止底盘穿模)
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck',
                                  x=traj_truck[0][0], y=traj_truck[0][1], yaw=traj_truck[0][2],
                                  z_offset=1.5, role_name="npc_truck")
        actor_list.append(truck)

        # 奥迪 TT
        audi = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                 x=traj_audi[0][0], y=traj_audi[0][1], yaw=traj_audi[0][2],
                                 z_offset=0.5, role_name="npc_audi")
        actor_list.append(audi)

        # Ego 轿车
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                z_offset=0.5, role_name="ego")
        actor_list.append(ego)

        # ==========================================
        # 4. 车辆 PID 挂载与剧本状态机编排
        # ==========================================
        # 货车比较重，使用卡车预设，并解除输出限制保证能加到 80km/h
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck', output_clip=(-1.0, 1.0))
        pid_lat_truck = RTB.PIDLateralController(preset='truck')

        pid_lon_audi = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_audi = RTB.PIDLateralController(preset='default_car')

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')

        # 【重点】Ego 剧本编排：初始60km/h，驶入 y=136 的点位后降速到 30km/h
        # 观察轨迹可知，Ego 是从 y=215 一直向负方向开，所以当 y 小于 136 时触发
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        ego_sm.add_stage(trigger_type='y_less', trigger_val=136.0, target_speed=30.0, accel=15.0)

        # 轨迹滑窗搜索索引记录器，提升 $O(1)$ 性能
        idx_truck, idx_audi, idx_ego = 0, 0, 0

        # ==========================================
        # 5. 车灯系统配置
        # ==========================================
        light_audi = RTB.VehicleLightManager(audi)
        light_audi.set_static_lights(low_beam=True)  # 开启行车灯

        light_ego = RTB.VehicleLightManager(ego)
        light_ego.set_static_lights(low_beam=True)  # 开启行车灯

        # ==========================================
        # 6. 初始物理状态注入 (一键防滑)
        # ==========================================
        # 锁死Z轴纯平面赋予初始速度，消除从0到80漫长的加速等待
        world.tick()  # 让物理引擎先接管Actor
        RTB.set_vehicle_initial_speed(truck, target_speed_kmh=80.0, yaw_deg=traj_truck[0][2])
        RTB.set_vehicle_initial_speed(audi, target_speed_kmh=110.0, yaw_deg=traj_audi[0][2])
        RTB.set_vehicle_initial_speed(ego, target_speed_kmh=60.0, yaw_deg=traj_ego[0][2])

        # ==========================================
        # 7. 仿真主循环（帧率同步与环境守护）
        # ==========================================
        print("[场景运行] 仿真开始...")
        sim_time = 0.0

        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 车辆1：货车控制 ----------------
            if truck and truck.is_alive:
                # 高鲁棒出界检测并自动销毁
                if not RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    # 获取预瞄点 (目标定速 80.0 km/h)
                    target_wp, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck,
                                                                   speed_kmh=80.0)
                    if target_wp:
                        RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, target_speed_kmh=80.0,
                                              target_wp=target_wp)

            # ---------------- 车辆2：奥迪控制 ----------------
            if audi and audi.is_alive:
                if not RTB.check_vehicle_out_of_bounds(audi, carla_map, auto_destroy=True):
                    target_wp, idx_audi = RTB.get_target_waypoint(audi.get_location(), traj_audi, idx_audi,
                                                                  speed_kmh=110.0)
                    if target_wp:
                        RTB.apply_pid_control(audi, pid_lon_audi, pid_lat_audi, target_speed_kmh=110.0,
                                              target_wp=target_wp)

            # ---------------- 车辆3：EGO 剧本控制 ----------------
            if ego and ego.is_alive:
                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):

                    # 1. 向状态机要本帧平滑处理过的目标速度（包含进入坐标 y<136 的触发减速逻辑）
                    current_target_speed = ego_sm.tick(ego.get_location(), sim_time, dt)

                    # 2. 根据该速度动态计算预瞄点
                    target_wp, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego,
                                                                 speed_kmh=current_target_speed)

                    # 3. 执行物理控制与特效绘制
                    if target_wp:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, target_speed_kmh=current_target_speed,
                                              target_wp=target_wp)
                        # 🚀 绘制 Ego 车辆的牵引线与预瞄点
                        RTB.draw_lookahead_point(world, ego.get_location(), target_wp)

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