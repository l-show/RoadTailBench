import sys
import carla
import time
import math
import random
from pathlib import Path

# 1. 动态引入标准化函数库路径
LOCAL_LIBRARY_PATH = str(Path(__file__).resolve().parent)
if LOCAL_LIBRARY_PATH not in sys.path:
    sys.path.insert(0, LOCAL_LIBRARY_PATH)
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 外部轨迹数据集 (直接内嵌)
# ==========================================
EGO_TRAJ_STR = """
Location_x	Location_y	Rotation_yaw
3.04	117.364	-89.045
3.04	117.364	-89.045
3.04	117.364	-89.045
3.092	113.43	-89.255
3.153	102.012	-90.561
2.972	90.589	-90.844
2.82	79.34	-90.771
2.733	67.903	-89.852
2.825	56.282	-89.354
2.92	45.035	-89.921
2.917	33.599	-90.061
2.917	25.911	-89.991
2.917	25.911	-89.991
2.917	25.911	-89.991
2.917	25.911	-89.991
2.917	25.911	-89.991
2.917	25.911	-90.414
2.814	20.662	-91.128
2.593	9.414	-91.128
2.416	-2.209	-90.558
2.421	-13.834	-89.286
2.588	-25.271	-89.144
2.756	-36.524	-89.144
2.907	-47.773	-89.426
3.011	-59.397	-89.639
2.998	-71.022	-90.772
2.792	-82.646	-91.127
2.578	-93.519	-91.127
2.578	-93.519	-91.127
2.578	-93.519	-91.127
2.578	-93.519	-91.127
2.523	-96.331	-91.127
2.374	-107.767	-90.196
2.604	-119.014	-87.771
2.869	-130.26	-88.85
3.045	-141.698	-89.272
3.139	-153.325	-89.904
3.064	-164.574	-90.611
2.94	-176.199	-90.611
2.82	-187.45	-90.611
2.762	-192.887	-90.611
2.762	-192.887	-90.611
2.762	-192.887	-90.611
"""

NPC_TRAJ_STR = """
Location_x	Location_y	Rotation_yaw
-5.442	-169.083	87.22
-5.442	-169.083	87.22
-5.442	-169.083	87.22
-5.442	-169.083	87.22
-5.296	-166.089	87.22
-5.03	-154.853	89.798
-5.026	-143.607	90.148
-5.075	-131.982	90.57
-5.191	-120.734	90.427
-5.271	-109.112	90.357
-5.305	-97.864	90.074
-5.301	-86.239	89.654
-5.232	-74.802	89.654
-5.169	-63.553	89.867
-5.167	-51.928	90.007
-5.168	-40.303	90.007
-5.176	-28.673	90.287
-5.233	-17.232	90.287
-5.291	-5.604	90.287
-5.349	6.021	90.287
-5.416	17.645	90.357
-5.488	29.082	90.357
-5.56	40.706	90.357
-5.619	51.955	90.287
-5.713	63.58	90.5
-5.812	75.204	90.36
-5.809	86.646	89.724
-5.753	98.274	89.724
-5.697	109.9	89.724
-5.641	121.525	89.724
-5.586	132.962	89.724
-5.536	144.589	90.007
-5.621	156.214	90.499
-5.719	167.466	90.499
-5.817	178.716	90.499
-5.869	184.716	90.499
-5.869	184.716	90.499
-5.869	184.716	90.499
"""

PED_TRAJ_STR = """
Location_x	Location_y	Rotation_yaw
15.035	5.695	179.798
15.035	5.695	179.798
15.035	5.695	179.798
15.035	5.695	179.233
15.035	5.695	178.46
15.035	5.695	178.18
11.921	5.837	177.402
6.104	6.121	176.837
2.149	6.25	179.181
-0.558	6.285	179.041
-6.178	6.38	179.111
-11.803	6.421	-179.823
-13.886	6.415	-179.823
-18.051	6.452	178.805
-21.587	6.526	178.805
-21.587	6.526	178.805
-21.587	6.526	178.805
-21.587	6.526	178.805
"""


def process_traj_str(traj_str):
    """【轨迹预处理】剔除表头，并在第3列安全插入Z轴(0.0)，使Yaw平移至第4列以适配标准化库"""
    lines = traj_str.strip().split('\n')
    processed_lines = []
    for line in lines:
        if 'Location_x' in line: continue  # 过滤掉表头
        parts = line.split()
        if len(parts) >= 3:
            # 拼接格式: x, y, 0.0, yaw
            processed_lines.append(f"{parts[0]} {parts[1]} 0.0 {parts[2]}")
    return "\n".join(processed_lines)


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

        # 调用函数库一键设置长尾天气 (完全还原截图参数)
        weather = RTB.build_weather(
            cloudiness=30.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=20.0,
            sun_azimuth_angle=110.0,
            sun_altitude_angle=28.0,
            fog_density=2.0,
            fog_distance=0.0,
            fog_falloff=0.0,
            wetness=0.0,
            scattering_intensity=1.5,
            mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.05,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 长尾天气系统已设置完成")

        # ==========================================
        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        # 解析文本 -> 获得 (X, Y, Z, Yaw) 元组
        ego_raw = RTB.parse_string_trajectory(process_traj_str(EGO_TRAJ_STR), min_dist=0.5)
        npc_raw = RTB.parse_string_trajectory(process_traj_str(NPC_TRAJ_STR), min_dist=0.5)
        ped_raw = RTB.parse_string_trajectory(process_traj_str(PED_TRAJ_STR), min_dist=0.5)

        # 稀疏点稠密插值 (以适配 PID 和 行人步态控制)
        ego_dense = RTB.interpolate_trajectory(ego_raw, interval=1.0)
        npc_dense = RTB.interpolate_trajectory(npc_raw, interval=1.0)
        ped_dense = RTB.interpolate_trajectory(ped_raw, interval=0.5)

        # 【要求】绘制出所有车辆及行人的完整预设寻路轨迹(灰色静态展示)
        RTB.draw_preset_trajectory(world, ego_dense, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, npc_dense, color=carla.Color(150, 150, 150))
        RTB.draw_preset_trajectory(world, ped_dense, color=carla.Color(150, 150, 150))

        # ==========================================
        # 3. 车辆与行人实体生成
        # ==========================================
        # 1号车: Ego (奥迪TT) - 取轨迹起点的 X, Y, Yaw 自动修正高度防穿模
        ego_vehicle = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                        x=ego_raw[0][0], y=ego_raw[0][1], yaw=ego_raw[0][3], role_name='ego')
        if ego_vehicle: actor_list.append(ego_vehicle)

        # 2号车: NPC (雪佛兰)
        npc_vehicle = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala',
                                        x=npc_raw[0][0], y=npc_raw[0][1], yaw=npc_raw[0][3], role_name='npc')
        if npc_vehicle: actor_list.append(npc_vehicle)

        # 3号实体: 行人
        ped_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        # 动态获取地表绝对高度防止行人直接掉入地下
        ped_z = carla_map.get_waypoint(
            carla.Location(x=ped_raw[0][0], y=ped_raw[0][1], z=0.0)).transform.location.z + 1.0
        ped_transform = carla.Transform(carla.Location(x=ped_raw[0][0], y=ped_raw[0][1], z=ped_z),
                                        carla.Rotation(yaw=ped_raw[0][3]))
        pedestrian = world.try_spawn_actor(ped_bp, ped_transform)
        if pedestrian: actor_list.append(pedestrian)

        # ==========================================
        # 4. 车辆PID、控制流挂载与剧本状态机编排
        # ==========================================
        # 【Ego 剧本】 初始 60km/h，在 Y <= -25 减速至 10，然后过 3秒 加速至 100
        ego_pid_lon = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        ego_pid_lat = RTB.PIDLateralController(preset='default_car', dt=dt)
        ego_idx = 0
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        # 注意：这里 Y 值是从 117 走到 -190，不断变小，因此越线触发器应选择 'y_less'
        ego_sm.add_stage(trigger_type='y_less', trigger_val=-25.0, target_speed=10.0, accel=30.0)
        ego_sm.add_stage(trigger_type='time', trigger_val=3.0, target_speed=100.0, accel=20.0)

        # 【NPC 剧本】 恒定 70km/h
        npc_pid_lon = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        npc_pid_lat = RTB.PIDLateralController(preset='default_car', dt=dt)
        npc_idx = 0
        npc_sm = RTB.MultiStageBehaviorMachine(initial_speed=70.0)

        # 【行人 剧本】 初始走 1.5m/s，2秒后开始跑 3.5m/s
        ped_ctrl = RTB.PedestrianController(pedestrian, mode='trajectory', target_list=ped_dense)
        ped_sm = RTB.MultiStageBehaviorMachine(initial_speed=1.5)
        ped_sm.add_stage(trigger_type='time', trigger_val=5.0, target_speed=4.5, accel=100.0)

        # ==========================================
        # 5. 灯光管理器与初始动态注入
        # ==========================================
        if ego_vehicle:
            ego_lights = RTB.VehicleLightManager(ego_vehicle)
            # 开启行车灯(包括近光和日行灯)
            ego_lights.set_static_lights(low_beam=True)
            # 一键注入初始物理速度，化解原地打滑
            RTB.set_vehicle_initial_speed(ego_vehicle, target_speed_kmh=60.0, yaw_deg=ego_raw[0][3])

        if npc_vehicle:
            RTB.set_vehicle_initial_speed(npc_vehicle, target_speed_kmh=70.0, yaw_deg=npc_raw[0][3])

        # ==========================================
        # 6. 仿真主循环与环境守护
        # ==========================================
        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()

        print("[RoadTailBench] 🚀 剧本装载完毕，长尾仿真开始！")
        sim_time = 0.0

        while True:
            # 记录本帧开始时间
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 视角跟随 ----------------
            if ego_vehicle and ego_vehicle.is_alive:
                tf = ego_vehicle.get_transform()
                spectator.set_transform(carla.Transform(
                    tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
                    carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
                ))

            # 【要求: 车辆出界需要自动销毁拦截】
            if ego_vehicle and ego_vehicle.is_alive:
                RTB.check_vehicle_out_of_bounds(ego_vehicle, carla_map, auto_destroy=True)
            if npc_vehicle and npc_vehicle.is_alive:
                RTB.check_vehicle_out_of_bounds(npc_vehicle, carla_map, auto_destroy=True)

            # ---------------- Ego车控制逻辑 ----------------
            if ego_vehicle and ego_vehicle.is_alive:
                ego_loc = ego_vehicle.get_location()
                # 状态机平滑计算本帧应有速度
                ego_target_speed = ego_sm.tick(ego_loc, sim_time, dt)

                # 获取真实速度进行动态预瞄(速度越快预瞄越远)
                vel = ego_vehicle.get_velocity()
                ego_actual_speed_kmh = 3.6 * math.hypot(vel.x, vel.y)

                ego_target_wp, ego_idx = RTB.get_target_waypoint(ego_loc, ego_dense, ego_idx, ego_actual_speed_kmh)

                if ego_target_wp:
                    RTB.apply_pid_control(ego_vehicle, ego_pid_lon, ego_pid_lat, ego_target_speed, ego_target_wp)
                    # 【要求: 实时绘制 Ego 的动态牵引线与预瞄点】
                    RTB.draw_lookahead_point(world, ego_loc, ego_target_wp, color=carla.Color(0, 255, 0), life_time=0.1)

                # 自动关联刹车灯与转向灯
                ego_lights.auto_update_from_control()

            # ---------------- NPC车控制逻辑 ----------------
            if npc_vehicle and npc_vehicle.is_alive:
                npc_loc = npc_vehicle.get_location()
                npc_target_speed = npc_sm.tick(npc_loc, sim_time, dt)
                vel = npc_vehicle.get_velocity()
                npc_actual_speed_kmh = 3.6 * math.hypot(vel.x, vel.y)
                npc_target_wp, npc_idx = RTB.get_target_waypoint(npc_loc, npc_dense, npc_idx, npc_actual_speed_kmh)
                if npc_target_wp:
                    RTB.apply_pid_control(npc_vehicle, npc_pid_lon, npc_pid_lat, npc_target_speed, npc_target_wp)

            # ---------------- 行人控制逻辑 ----------------
            if pedestrian and pedestrian.is_alive:
                ped_loc = pedestrian.get_location()
                # 获取行人动态步态速度 (m/s)
                ped_target_speed = ped_sm.tick(ped_loc, sim_time, dt)
                # 传入控制器执行移动与防卡死跨栏
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=ped_target_speed)

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键安全清理场景内的所有残留实体
        if 'world' in locals():
            RTB.disable_synchronous_mode(world)
        if 'client' in locals() and actor_list:
            RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()
