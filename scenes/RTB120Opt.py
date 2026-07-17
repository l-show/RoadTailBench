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
# 轨迹数据硬编码区 (已去除表头，纯数值以适配解析器)
# ==========================================
RAW_TRAJ_TRUCK = """
-177.839	183.797	-91.181
-177.975	178.64	-91.601
-177.946	175.816	-88.731
-177.678	167.683	-86.222
-176.912	157.562	-83.766
-175.534	147.499	-81.68
-173.915	137.478	-79.559
-171.913	127.529	-77.765
-169.502	117.67	-73.733
-166.658	107.923	-73.663
-163.5	98.274	-70.028
-159.744	88.839	-67.123
-155.805	79.653	-66.265
-151.541	70.435	-63.385
-146.9	61.4	-62.463
-142.102	52.447	-61.601
-137.38	43.642	-62.38
-133.362	34.324	-70.466
-130.172	24.682	-71.627
-126.279	15.495	-61.191
-121.094	6.765	-56.81
-115.42	-1.658	-55.878
-109.637	-10.013	-55.017
-103.979	-18.442	-58.406
-98.816	-26.991	-59.984
-93.774	-35.813	-60.694
-88.779	-44.649	-60.344
-83.692	-53.436	-58.454
-78.202	-61.986	-57.017
-72.553	-70.43	-55.801
-66.605	-78.871	-53.662
-60.344	-86.867	-49.78
-53.714	-94.553	-48.789
-46.929	-102.096	-46.263
-39.816	-109.335	-45.4
-32.68	-116.548	-45.121
-25.486	-123.716	-44.771
-18.156	-130.748	-41.924
-10.406	-137.577	-40.426
-2.318	-144.011	-37.37
5.844	-150.062	-36.169
14.106	-155.958	-34.688
22.553	-161.573	-33.081
30.959	-166.952	-32.17
39.565	-172.337	-32.028
48.349	-177.752	-31.039
57.161	-182.802	-28.121
66.153	-187.53	-27.697
75.148	-192.252	-27.697
78.834	-194.187	-27.697
"""

RAW_TRAJ_AUDI = """
-38.146	-117.375	134.281
-43.185	-111.834	131.056
-49.953	-104.044	131.416
-56.861	-96.598	132.854
-63.657	-89.058	130.929
-69.917	-81.275	128.015
-75.786	-72.992	123.979
-81.555	-64.433	123.979
-87.098	-55.925	120.923
-92.147	-47.314	119.999
-97.213	-38.521	119.929
-102.344	-29.574	119.073
-107.779	-21.045	131.049
-113.465	-12.722	117.458
-118.476	-3.693	119.62
-123.844	4.927	122.051
-128.815	13.793	118.322
-133.611	22.756	118.112
-135.575	26.431	118.112
-140.394	35.375	118.747
-145.428	44.38	119.451
-150.385	53.237	118.599
-155.223	62.159	118.457
-159.778	71.227	115.869
-163.952	80.479	112.894
-167.734	89.903	110.979
-171.395	99.552	110.272
-174.494	109.052	107.356
-177.347	118.796	103.932
-179.737	128.666	103.152
-181.674	138.631	97.592
-182.923	148.71	97.017
-184.153	158.791	96.52
-185.083	168.902	93.513
-184.942	179.038	82.866
-182.886	188.97	74.613
-180.974	199.095	83.876
-179.921	209.199	84.057
-178.705	219.282	81.545
-176.858	229.262	77.758
-174.516	239.138	76.168
-171.775	248.908	72.018
-168.439	258.494	70.182
-164.905	268.009	67.874
-160.894	277.341	65.088
-156.581	286.546	64.876
-151.944	295.584	59.04
-146.479	304.142	56.535
-140.967	312.468	56.395
-135.239	321.048	55.967
-129.41	329.355	53.676
-124.962	335.376	53.464
"""

RAW_TRAJ_EGO = """
-174.57	179.195	-85.309
-174.43	177.288	-85.871
-174.071	172.229	-86.514
-173.79	167.166	-86.868
-173.401	162.106	-85.009
-172.952	156.965	-85.009
-172.51	151.907	-85.009
-172.009	146.856	-82.393
-171.301	141.83	-81.333
-170.361	136.759	-77.588
-169.282	131.882	-77.518
-168.159	126.931	-77.095
-166.987	121.99	-76.102
-165.726	117.07	-75.252
-164.412	112.164	-74.614
-163.018	107.28	-73.054
-161.412	102.374	-70.56
-159.649	97.61	-69.006
-157.778	92.887	-67.59
-155.754	88.226	-65.672
-153.599	83.627	-64.329
-151.429	79.133	-63.469
-149.162	74.593	-63.469
-146.891	70.057	-63.399
-144.62	65.522	-63.399
-142.323	60.978	-62.827
-141.145	58.683	-62.827
-140.879	58.165	-62.827
-139.086	53.434	-78.493
-138.261	48.424	-81.094
-137.148	43.478	-73.609
-135.747	38.597	-75.625
-134.551	33.749	-75.84
-132.935	28.939	-67.092
-130.78	24.342	-63.661
-130.522	23.819	-63.661
-129.93	22.625	-63.661
-127.653	18.085	-63.306
-125.334	13.568	-59.76
-122.51	9.354	-52.455
-119.391	5.455	-51.16
-116.191	1.482	-51.16
-112.984	-2.345	-48.22
-109.604	-6.128	-48.22
-106.288	-9.858	-49.296
-103.19	-13.872	-54.792
-100.316	-18.054	-57.38
-97.73	-22.422	-59.871
-95.175	-26.813	-59.731
-92.661	-31.13	-59.944
-90.108	-35.519	-59.731
-87.589	-39.835	-59.731
-85.029	-44.221	-59.661
-82.438	-48.588	-59.166
-79.835	-52.948	-59.166
-77.126	-57.242	-56.651
-74.377	-61.412	-56.581
-70.338	-67.526	-56.299
-65.812	-73.648	-52.509
-61.181	-79.537	-51.224
-56.339	-85.343	-49.441
-51.388	-91.128	-49.441
-46.517	-96.819	-49.441
-41.563	-102.6	-48.395
-36.328	-108.131	-46.013
-31.001	-113.572	-45.307
-25.611	-118.95	-44.245
-20.112	-124.215	-43.046
-14.438	-129.293	-40.26
-8.579	-134.165	-39.558
-2.705	-139.016	-39.416
3.185	-143.736	-37.999
9.229	-148.373	-37.149
15.258	-152.825	-35.073
21.593	-157.053	-32.289
28.095	-161.026	-30.939
34.628	-164.949	-31.009
41.264	-168.938	-31.009
47.686	-172.798	-31.009
54.22	-176.709	-30.87
60.8	-180.546	-29.588
66.909	-183.945	-28.739
"""




# === RoadTailBench Opt: ego endpoint cleanup guard ===
_RTB_OPT_EGO_GOAL_XY = (66.909, -183.945)
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

        # 1. 环境初始化：帧率同步与天气系统
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按照用户截图构建高度自定义天气环境 (极端风沙起雾环境)
        weather = carla.WeatherParameters(
            cloudiness=100.0,
            precipitation=0.0,
            precipitation_deposits=10.0,  # Puddles
            wind_intensity=100.0,
            sun_azimuth_angle=180.0,
            sun_altitude_angle=20.0,
            fog_density=60.0,
            fog_distance=0.0,
            fog_falloff=0.45,
            wetness=10.0,
            scattering_intensity=0.0,  # Scatter
            mie_scattering_scale=0.0,  # Mie
            rayleigh_scattering_scale=0.04,
            dust_storm=150.0  # Dust
        )
        world.set_weather(weather)
        print("[场景配置] 长尾极端天气系统已设置完毕。")

        # 2. 轨迹数据硬编码与清洗
        # 自动去重洗出干净锚点，距离阈值设为0.5m过滤原点重复抽搐点
        traj_truck = RTB.parse_string_trajectory(RAW_TRAJ_TRUCK, min_dist=0.5)
        traj_audi = RTB.parse_string_trajectory(RAW_TRAJ_AUDI, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)

        # 🚀 绘制出所有车辆的预设完整寻路轨迹线 (以不同颜色区分)
        RTB.draw_preset_trajectory(world, traj_truck, color=carla.Color(255, 0, 0), size=0.1)  # 货车: 红
        RTB.draw_preset_trajectory(world, traj_audi, color=carla.Color(0, 0, 255), size=0.1)  # 奥迪: 蓝
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(255, 255, 0), size=0.1)  # EGO: 黄

        # =========================================================================
        # 3. 🚨 长尾特征注入：结冰低摩擦力区域 (根据 Carla 文档 Issue #1562 要求配置)
        # =========================================================================
        # 官方 blueprint 中 extent 单位要求是【厘米】。范围至少5米，因此填 500.0。
        # 我们的 PID 控制器使用的是底层物理指令 (throttle/steer)，因此摩擦力突降会导致真实的打滑与失控！
        friction_extent_cm = (800.0, 800.0, 200.0)
        friction_loc = carla.Location(x=-121.224, y=12.816, z=26.688)

        # 调用标准库函数生成打滑区 (强制设 draw_debug=False，以防标准库用cm画出500米的错误巨型框)
        friction_trigger = RTB.spawn_friction_region(
            world, bp_lib,
            center_loc=friction_loc,
            friction=0.0,  # 0.0 表示极限冰面，毫无抓地力
            extent=friction_extent_cm,
            draw_debug=False
        )
        if friction_trigger:
            actor_list.append(friction_trigger)
            print("[场景配置] 🧊 成功生成 结冰极低摩擦力区域 (范围: 5米)。")

            # 手动可视化触发器区域 (使用正确的单位：【米】)
            # 可根据需要注释以下这行代码隐藏可视化框
            world.debug.draw_box(
                box=carla.BoundingBox(friction_loc, carla.Vector3D(8.0, 8.0, 2.0)),
                rotation=carla.Rotation(),
                thickness=0.2,
                color=carla.Color(0, 255, 255),  # 青蓝色代表结冰
                life_time=0.0  # 永久显示
            )
        # =========================================================================

        # 4. 车辆实体安全生成与物理突破限制
        # -- 车辆1：消防货车 --
        truck = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck',
                                  x=traj_truck[0][0], y=traj_truck[0][1], yaw=traj_truck[0][2],
                                  z_offset=1.5)  # 卡车底盘高，抬高防止爆胎卡地
        if truck:
            actor_list.append(truck)
            # 【核心突破】突破Carla物理限制，解决大型货车无法加速到70km/h的问题
            physics = truck.get_physics_control()
            physics.mass = 2500.0  # 骗过引擎，减低吨位
            physics.drag_coefficient = 0.1  # 消除风阻
            for gear in physics.forward_gears:
                gear.max_rpm = 7000.0  # 放宽转速红线
            truck.apply_physics_control(physics)
            RTB.set_vehicle_initial_speed(truck, 70.0, yaw_deg=traj_truck[0][2])

        # -- 车辆2：奥迪轿车 --
        audi = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                 x=traj_audi[0][0], y=traj_audi[0][1], yaw=traj_audi[0][2],
                                 z_offset=0.5)
        if audi:
            actor_list.append(audi)
            RTB.set_vehicle_initial_speed(audi, 70.0, yaw_deg=traj_audi[0][2])
            # 开启灯光
            audi_lights = RTB.VehicleLightManager(audi)
            audi_lights.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)

        # -- 车辆3：EGO轿车 --
        ego = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', role_name='ego',
                                x=traj_ego[0][0], y=traj_ego[0][1], yaw=traj_ego[0][2],
                                z_offset=0.5)
        if ego:
            actor_list.append(ego)
            RTB.set_vehicle_initial_speed(ego, 60.0, yaw_deg=traj_ego[0][2])
            # 开启灯光
            ego_lights = RTB.VehicleLightManager(ego)
            ego_lights.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.Fog)

        # 5. PID控制器与目标点缓存挂载
        pid_lon_truck = RTB.PIDLongitudinalController(preset='truck')  # 使用卡车大扭矩PID
        pid_lat_truck = RTB.PIDLateralController(preset='truck')
        idx_truck = 0

        pid_lon_audi = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_audi = RTB.PIDLateralController(preset='default_car')
        idx_audi = 0

        pid_lon_ego = RTB.PIDLongitudinalController(preset='default_car')
        pid_lat_ego = RTB.PIDLateralController(preset='default_car')
        idx_ego = 0

        # 6. 剧本状态机编排：控制EGO按要求变速
        # 从轨迹数据发现：车辆向东行驶，X坐标是从负向正递增 (-174 -> -141 -> 66)
        # 所以判定到达 "-141" 的条件应使用 x_greater
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # 阶段1：当驶过 x = -141 后，紧急减速到 20 km/h (减速力度设为25.0)
        ego_sm.add_stage(trigger_type='x_greater', trigger_val=-141.0, target_speed=20.0, accel=25.0)
        # 阶段2：进入阶段1后，持续维持等待 10.0 秒，然后恢复到 60 km/h
        ego_sm.add_stage(trigger_type='time', trigger_val=10.0, target_speed=60.0, accel=15.0)

        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()

        # 7. 仿真主循环（帧率同步与环境清理守护）
        print("[场景执行] 仿真开始，世界循环已启动...")
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            if _rtb_opt_goal_guard(locals(), client, world):
                break
            sim_time += dt

            # ---------------- 视角跟随 ----------------
            if ego and ego.is_alive:
                tf = ego.get_transform()
                spectator.set_transform(carla.Transform(
                    tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
                    carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
                ))

            # =============== EGO 控制与剧本联动 ===============
            if ego and ego.is_alive:
                # EGO出界检测拦截，出界即刻销毁
                if RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    ego = None
                    continue

                # 1. 状态机运算，获取当前帧 EGO 应该行驶的速度
                ego_target_spd = ego_sm.tick(ego.get_location(), sim_time, dt)

                # 2. 轨迹预瞄点捕获 (动态伸缩机制)
                target_wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, ego_target_spd)

                # 3. 🚀 画出 EGO 的当前实时预瞄点与牵引线
                RTB.draw_lookahead_point(world, ego.get_location(), target_wp_ego)

                # 4. 执行PID控制 (因为使用的是底层油门/刹车，进入冰面时摩擦力触发器会完美生效导致甩尾！)
                if target_wp_ego:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, ego_target_spd, target_wp_ego)

                # 5. 车灯物理随动同步
                ego_lights.auto_update_from_control()

            # =============== 消防货车 控制 ===============
            if truck and truck.is_alive:
                if RTB.check_vehicle_out_of_bounds(truck, carla_map, auto_destroy=True):
                    truck = None
                    continue

                target_wp_truck, idx_truck = RTB.get_target_waypoint(truck.get_location(), traj_truck, idx_truck, 70.0)
                if target_wp_truck:
                    RTB.apply_pid_control(truck, pid_lon_truck, pid_lat_truck, 70.0, target_wp_truck)

            # =============== 奥迪轿车 控制 ===============
            if audi and audi.is_alive:
                if RTB.check_vehicle_out_of_bounds(audi, carla_map, auto_destroy=True):
                    audi = None
                    continue

                target_wp_audi, idx_audi = RTB.get_target_waypoint(audi.get_location(), traj_audi, idx_audi, 70.0)
                if target_wp_audi:
                    RTB.apply_pid_control(audi, pid_lon_audi, pid_lat_audi, 70.0, target_wp_audi)
                audi_lights.auto_update_from_control()

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体防残留
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()