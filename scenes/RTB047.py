# -*- coding: utf-8 -*-
import sys
import carla
import time
import math

# ==========================================
# 1. 动态引入标准化函数库路径
# ==========================================
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 2. 预设轨迹数据与自定义解析器
# ==========================================
TRAJ_CAR1_STR = """
Location_x	Location_y	Rotation_yaw
3.999	182.027	-93.438
3.898	180.407	-93.581
3.656	176.544	-93.581
3.449	172.739	-91.499
3.237	169	-98.712
2.427	165.214	-104.872
1.28	161.517	-107.59
0.284	157.906	-103.099
-0.214	154.138	-92.87
-0.32	150.393	-91.42
-0.491	143.485	-91.42
-0.68	135.744	-91.065
-0.643	127.997	-88.861
-0.454	120.501	-88.436
-0.275	112.76	-89.353
-0.219	105.153	-89.988
-0.252	97.448	-90.274
-0.295	89.98	-90.623
-0.45	82.498	-90.828
-0.562	74.76	-90.828
1.35	67.467	-55.309
3.511	60.269	-92.196
3.308	52.655	-91.019
3.125	45.206	-91.447
2.934	37.659	-91.447
2.824	30.212	-90.669
2.833	22.63	-89.601
2.83	15.032	-90.593
2.751	7.42	-90.593
2.723	4.674	-90.593
2.723	4.674	-90.593
2.723	4.674	-90.593
"""

TRAJ_EGO_STR = """
Location_x	Location_y	Rotation_yaw
-1.249	2.418	89.477
-1.249	2.418	89.477
-1.249	2.418	89.477
-1.244	3.02	89.477
-1.227	4.908	89.477
-1.182	11.33	89.83
-1.164	17.532	89.83
-1.143	23.937	89.62
-1.088	30.253	89.692
-1.054	36.57	89.692
-1.008	45.083	89.692
-0.979	54.078	89.905
-0.965	62.788	89.905
-0.943	71.806	89.692
-0.894	78.51	89.34
-0.872	80.405	89.339
-0.812	89.135	89.692
-0.762	97.799	89.622
-0.697	106.743	89.552
-0.652	115.571	89.765
-0.615	124.579	89.764
-0.589	133.613	89.904
-0.598	142.362	90.324
-0.653	151.094	90.394
-0.715	160.114	90.394
-0.775	168.843	90.394
-0.741	177.573	88.908
-0.449	186.304	87.637
-0.191	192.706	87.922
-0.191	192.706	87.922
-0.191	192.706	87.922
-0.191	192.706	87.922
"""

TRAJ_CAR3_STR = """
Location_x	Location_y	Rotation_yaw
3.559	147.886	-86.105
3.559	147.886	-86.105
3.559	147.886	-86.105
3.559	147.886	-86.458
3.82	140.165	-89.919
3.85	131.423	-89.634
3.762	122.678	-90.985
3.661	113.64	-90.274
3.616	104.767	-89.404
3.729	95.772	-89.264
3.809	86.922	-90.187
3.697	77.927	-91.323
3.526	69.192	-90.721
3.423	60.172	-90.651
3.325	51.484	-90.651
3.237	42.513	-90.296
3.243	33.682	-89.59
3.253	24.676	-90.08
3.221	15.952	-90.43
3.139	7.216	-90.713
2.923	-1.671	-91.778
2.882	-2.98	-91.778
2.882	-2.98	-91.778
"""


def parse_custom_trajectory(data_str):
    """
    【自研增强型解析器】
    防止提取了Yaw角被底层误当做Z轴海拔导致车辆飞天，这里精准剥离出 (X, Y) 坐标。
    并同步提取出起步的第一帧偏航角防画龙。
    """
    lines = data_str.strip().split('\n')
    raw_pts = []
    yaw_start = 0.0
    first_yaw_recorded = False

    for line in lines:
        if 'Location_x' in line or not line.strip():
            continue
        parts = line.split()
        if len(parts) >= 3:
            x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2])
            raw_pts.append((x, y))  # 仅保留XY交给底层处理
            if not first_yaw_recorded:
                yaw_start = yaw
                first_yaw_recorded = True

    # 调用 RTB 库的标准清洗器进行原点去重
    cleaned_traj = RTB.clean_trajectory(raw_pts, min_dist=0.5)
    return cleaned_traj, yaw_start


def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05
        sim_time = 0.0

        # ==========================================
        # 1. 环境初始化：帧率同步与天气系统配置
        # ==========================================
        # 强制开启 20FPS 帧率物理同步
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按照截图要求精准构建并挂载静态长尾天气
        weather = RTB.build_weather(
            cloudiness=0.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=0.0,
            sun_azimuth_angle=270.0,
            sun_altitude_angle=14.0,
            fog_density=2.0,
            fog_distance=0.75,
            fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=0.5,
            mie_scattering_scale=0.16,
            rayleigh_scattering_scale=0.03,
            dust_storm=0.0
        )
        world.set_weather(weather)
        print("[场景配置] 天气系统已按照指定截图设置完毕。")

        # ==========================================
        # 2. 轨迹数据清洗与解析提取
        # ==========================================
        traj1, yaw1 = parse_custom_trajectory(TRAJ_CAR1_STR)
        traj2, yaw2 = parse_custom_trajectory(TRAJ_EGO_STR)
        traj3, yaw3 = parse_custom_trajectory(TRAJ_CAR3_STR)

        # 全局可视化：画出这三辆车的完整预设轨迹点
        RTB.draw_preset_trajectory(world, traj1, color=carla.Color(255, 0, 0))  # 车1：红色
        RTB.draw_preset_trajectory(world, traj2, color=carla.Color(0, 255, 0))  # Ego：绿色
        RTB.draw_preset_trajectory(world, traj3, color=carla.Color(0, 0, 255))  # 车3：蓝色

        # ==========================================
        # 3. 车辆实体安全生成与初速度注入
        # ==========================================
        print("\n--- 正在生成车辆实体 ---")
        # 【第一辆车】：Chevrolet Impala，80km/h
        veh1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj1[0][0], y=traj1[0][1], yaw=yaw1,
                                 role_name="car1")
        RTB.set_vehicle_initial_speed(veh1, 80.0, yaw_deg=yaw1)
        actor_list.append(veh1)

        # 【第二辆车 (EGO)】：Audi TT，20km/h 起步
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=traj2[0][0], y=traj2[0][1], yaw=yaw2, role_name="ego")
        RTB.set_vehicle_initial_speed(ego, 20.0, yaw_deg=yaw2)
        actor_list.append(ego)

        # 【第三辆车】：Lincoln MKZ 2020，30km/h
        veh3 = RTB.spawn_vehicle(world, 'vehicle.lincoln.mkz_2020', x=traj3[0][0], y=traj3[0][1], yaw=yaw3,
                                 role_name="car3")
        RTB.set_vehicle_initial_speed(veh3, 30.0, yaw_deg=yaw3)
        actor_list.append(veh3)

        # ==========================================
        # 4. 车辆灯光管理器
        # ==========================================
        # 车1：开启行车灯、远光灯
        lights1 = RTB.VehicleLightManager(veh1)
        lights1.turn_on(carla.VehicleLightState.Position | carla.VehicleLightState.HighBeam)

        # Ego：开启行车灯、近光灯
        lights_ego = RTB.VehicleLightManager(ego)
        lights_ego.set_static_lights(low_beam=True, high_beam=False)

        # 车3：开启行车灯
        lights3 = RTB.VehicleLightManager(veh3)
        lights3.turn_on(carla.VehicleLightState.Position)

        # ==========================================
        # 5. PID控制器挂载与初始跟踪索引分配
        # ==========================================
        # 为每辆车独立分配纵向/横向 PID（统一采用默认私家车Preset预设）
        pid_lon1, pid_lat1 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon_ego, pid_lat_ego = RTB.PIDLongitudinalController(), RTB.PIDLateralController()
        pid_lon3, pid_lat3 = RTB.PIDLongitudinalController(), RTB.PIDLateralController()

        idx1, idx2, idx3 = 0, 0, 0
        speed_car1 = 80.0
        speed_car3 = 30.0

        # ==========================================
        # 6. EGO剧本状态机编排
        # ==========================================
        """
        需求逻辑编排：初始速度20km/h
        1. 立即触发(immediate)：慢慢加速到 40km/h
        2. 位置触发(y_greater)：因为y是递增的，当y>50时，执行刹车减速到 20km/h
        3. 时间触发(time)：在进入刹车阶段过5秒后，彻底恢复并加速到 60km/h
        """
        ego_sm = RTB.MultiStageBehaviorMachine(initial_speed=20.0)
        # 阶段1：缓慢加速
        ego_sm.add_stage(trigger_type='immediate', target_speed=40.0, accel=5.0)
        # 阶段2：y=50 的时候踩刹车减速
        ego_sm.add_stage(trigger_type='y_greater', trigger_val=50.0, target_speed=20.0, accel=15.0)
        # 阶段3：等待5秒后，全速恢复
        ego_sm.add_stage(trigger_type='time', trigger_val=5.0, target_speed=60.0, accel=10.0)

        # ==========================================
        # 7. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        print("\n[RoadTailBench] 🚀 预热完毕，长尾仿真场景开始运行...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- A. 物理出界监测守护机制 ----------------
            if veh1 and veh1.is_alive:
                RTB.check_vehicle_out_of_bounds(veh1, carla_map, auto_destroy=True)
            if ego and ego.is_alive:
                RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True)
            if veh3 and veh3.is_alive:
                RTB.check_vehicle_out_of_bounds(veh3, carla_map, auto_destroy=True)

            # ---------------- B. EGO 状态机时间轴更新 ----------------
            # 将Ego当前坐标喂给状态机进行逻辑判断，返回当前帧目标速度
            if ego and ego.is_alive:
                speed_ego = ego_sm.tick(ego.get_location(), sim_time, dt)

            # ---------------- C. PID循迹控制流下发 ----------------
            # 车1
            if veh1 and veh1.is_alive:
                vel = veh1.get_velocity()
                spd_kmh = 3.6 * math.hypot(vel.x, vel.y)
                # 获取预瞄点
                wp1, idx1 = RTB.get_target_waypoint(veh1.get_location(), traj1, idx1, spd_kmh)
                if wp1:
                    RTB.apply_pid_control(veh1, pid_lon1, pid_lat1, speed_car1, wp1)

            # 车2 (EGO)
            if ego and ego.is_alive:
                vel = ego.get_velocity()
                spd_kmh = 3.6 * math.hypot(vel.x, vel.y)
                wp2, idx2 = RTB.get_target_waypoint(ego.get_location(), traj2, idx2, spd_kmh)
                if wp2:
                    RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, speed_ego, wp2)
                    # 动态绘制Ego当前跟踪的预瞄点以及视觉牵引线（绿色）
                    RTB.draw_lookahead_point(world, ego.get_location(), wp2, color=carla.Color(0, 255, 0),
                                             life_time=0.1)

            # 车3
            if veh3 and veh3.is_alive:
                vel = veh3.get_velocity()
                spd_kmh = 3.6 * math.hypot(vel.x, vel.y)
                wp3, idx3 = RTB.get_target_waypoint(veh3.get_location(), traj3, idx3, spd_kmh)
                if wp3:
                    RTB.apply_pid_control(veh3, pid_lon3, pid_lat3, speed_car3, wp3)

            # ---------------- D. 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[运行异常] {e}")
    finally:
        # 恢复异步模式并一键清理场景残存实体资源
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收完毕。")


if __name__ == '__main__':
    main()