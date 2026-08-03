import sys
import carla
import time
import random

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据硬编码 (去除表头，保留纯数据)
# ==========================================
STR_NPC1 = """
-5.08 -153.54 89.273
-5.048 -151.037 89.273
-5.019 -148.495 89.693
-5.009 -145.953 89.765
-4.998 -143.412 89.765
-4.988 -140.911 89.765
-4.978 -138.411 89.765
-4.968 -135.869 89.765
-4.957 -133.369 89.765
-4.947 -130.868 89.765
-4.937 -128.326 90.121
-5.049 -125.83 94.837
-5.41 -123.315 102.011
-6.202 -120.949 112.411
-6.234 -120.872 112.411
-6.234 -120.872 112.411
-6.234 -120.872 112.411
-6.234 -120.872 112.411
-6.234 -120.872 112.411
-6.488 -120.255 112.411
-7.195 -117.869 95.339
-7.07 -115.419 81.869
-6.543 -112.857 76.682
-5.953 -110.364 76.682
-4.562 -104.488 76.682
-3.141 -98.189 78.393
-1.991 -92.047 80.034
-1.161 -85.752 86.431
-1.203 -79.505 92.707
-1.506 -73.263 92.987
-1.82 -67.021 92.707
-2.043 -60.671 91.298
-2.142 -54.422 90.388
-2.162 -48.068 90.108
-2.174 -41.818 90.108
-2.172 -35.568 89.828
-2.153 -29.214 89.828
-2.132 -22.86 89.828
-2.141 -16.402 90.528
-2.206 -10.153 90.668
-2.245 -3.799 89.968
-2.2 2.455 89.476
-2.112 12.019 89.476
-2.041 23.456 90.366
-2.172 34.887 90.646
-2.269 46.133 90.295
-2.303 57.57 90.085
-2.311 68.82 90.015
-2.315 80.258 90.015
-2.318 91.694 90.015
-2.321 103.131 90.015
-2.324 114.567 90.015
-2.326 123.712 90.015
-2.328 131.336 90.085
-3.236 138.886 102.798
-5.467 146.036 113.173
-9.089 152.734 123.378
-13.807 158.717 132.696
-19.313 163.983 140.47
-25.714 168.083 156.02
-33.048 170.058 173.084
-40.536 170.193 -177.784
-42.16 170.127 -177.292
-42.16 170.127 -177.292
"""

STR_NPC2 = """
-5.238 -276.510 91.183
-5.253 -274.821 90.527
-5.287 -269.726 89.827
-5.257 -264.790 89.617
-5.223 -259.738 89.617
-5.189 -254.665 89.617
-5.155 -249.550 89.617
-5.121 -244.497 89.617
-5.114 -239.452 90.037
-5.117 -234.366 90.037
-5.120 -229.278 90.037
-5.123 -224.306 90.037
-5.127 -219.185 90.037
-5.127 -214.184 89.967
-5.124 -209.140 89.967
-5.121 -204.112 89.967
-5.118 -199.053 89.967
-4.932 -193.997 85.631
-4.469 -188.987 83.665
-3.894 -184.018 83.385
-3.309 -178.969 83.385
-2.733 -174.001 83.385
-2.176 -168.948 84.667
-1.784 -163.880 86.363
-1.523 -158.885 87.343
-1.306 -153.889 87.553
-1.117 -148.785 88.538
-1.016 -143.773 89.028
-0.937 -138.667 89.238
-1.012 -133.666 91.431
-1.139 -128.584 91.431
-1.491 -123.516 97.493
-2.263 -118.493 101.789
-3.639 -113.691 110.563
-6.159 -109.300 127.411
-10.020 -106.054 149.813
-14.671 -104.060 167.100
-19.644 -103.816 -173.983
-24.642 -104.724 -165.883
-29.605 -105.794 -174.702
-34.601 -105.857 178.510
-39.681 -105.684 177.861
-44.763 -105.552 178.846
-50.366 -105.439 178.846
-59.135 -105.281 -179.847
-68.030 -105.409 179.990
-76.923 -105.247 177.629
-85.934 -104.813 177.209
-98.417 -104.209 177.419
-110.911 -103.856 179.331
-123.619 -103.693 179.121
-136.325 -103.448 178.841
-148.822 -103.145 178.421
-161.499 -102.803 178.491
-174.203 -102.482 178.561
-186.909 -102.209 179.191
-199.408 -102.033 179.191
-212.116 -102.058 -179.004
-212.949 -102.073 -179.004
-212.949 -102.073 -179.004
-212.949 -102.073 -179.004
"""

STR_EGO = """
-4.992 -238.838 89.94
-4.992 -238.838 89.94
-4.992 -238.838 89.94
-4.975 -235.94 89.66
-4.912 -225.765 89.572
-4.857 -215.577 89.782
-4.781 -205.347 89.502
-4.728 -195.336 89.852
-4.701 -185.169 89.852
-4.675 -175.002 89.852
-4.649 -164.98 89.852
-4.631 -157.98 89.852
-4.628 -156.834 89.852
-4.628 -156.834 89.852
-4.628 -156.834 89.852
-4.628 -156.834 89.852
-4.628 -156.834 89.852
-4.628 -156.834 89.852
-4.627 -156.568 89.852
-4.626 -155.984 89.852
-4.614 -151.505 89.852
-4.601 -146.505 89.852
-4.588 -141.422 89.852
-4.577 -137.297 89.852
-4.571 -134.755 89.852
-4.564 -132.213 89.852
-4.558 -129.712 89.852
-4.553 -127.17 90.422
-4.741 -124.597 97.936
-5.01 -123.079 101.386
-5.01 -123.079 101.386
-5.01 -123.079 101.386
-5.01 -123.079 101.386
-5.01 -123.079 101.386
-5.01 -123.079 101.386
-5.43 -120.996 101.386
-6.035 -118.528 106.119
-6.858 -116.125 113.232
-8.097 -113.91 124.35
-9.782 -111.953 132.784
-11.927 -110.661 164.402
-14.448 -110.409 -179.357
-16.985 -110.503 -177.352
-19.524 -110.606 -178.566
-22.065 -110.641 179.819
-24.606 -110.587 178.25
-27.147 -110.509 178.25
-29.75 -110.43 178.25
-36.559 -110.222 178.25
-44.18 -109.989 178.25
-51.8 -109.696 177.752
-59.419 -109.395 177.682
-67.038 -109.087 177.682
-74.535 -108.783 177.682
-82.155 -108.477 177.752
-89.774 -108.181 177.892
-97.395 -107.937 178.382
-105.018 -107.746 178.592
-112.641 -107.559 178.592
-120.265 -107.379 178.662
-127.888 -107.193 178.382
-135.637 -106.942 177.892
-143.257 -106.685 178.387
-150.759 -106.56 179.821
-158.385 -106.554 -179.969
-166.01 -106.544 179.751
-173.76 -106.506 179.329
"""

STR_PED = """
-13.061 -115.565 86.617
-13.061 -115.565 86.617
-13.061 -115.565 86.617
-13.061 -115.565 88.351
-13.023 -114.382 88.141
-12.953 -111.834 88.925
-12.912 -109.286 89.135
-12.909 -106.704 90.986
-12.968 -104.172 91.833
-13.113 -101.645 93.465
-13.268 -99.091 93.465
-13.421 -96.573 93.535
-13.58 -94 93.535
-13.736 -91.475 93.535
-13.893 -88.925 93.535
-14.052 -86.346 93.535
-14.209 -83.809 93.535
-14.366 -81.273 93.535
-14.511 -78.735 91.95
-14.595 -76.196 91.81
-14.672 -73.78 91.81
-14.672 -73.78 91.81
-14.672 -73.78 91.81
-14.672 -73.78 91.81
"""


def is_vehicle_near_trajectory_end(vehicle, trajectory, threshold=4.0):
    if not vehicle or not vehicle.is_alive or not trajectory:
        return False
    end_pt = trajectory[-1]
    end_x, end_y = (end_pt.x, end_pt.y) if hasattr(end_pt, 'x') else (end_pt[0], end_pt[1])
    loc = vehicle.get_location()
    return ((loc.x - end_x) ** 2 + (loc.y - end_y) ** 2) ** 0.5 <= threshold


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

        # 按照截图数据精确配置静态天气
        weather_kwargs = {
            'cloudiness': 5.0,
            'precipitation': 0.0,
            'precipitation_deposits': 0.0,
            'wind_intensity': 10.0,
            'sun_azimuth_angle': 275.0,
            'sun_altitude_angle': 3.0,
            'fog_density': 2.0,
            'fog_distance': 0.75,
            'fog_falloff': 0.1,
            'wetness': 0.0,
            'scattering_intensity': 1.0,
            'mie_scattering_scale': 0.03,
            'rayleigh_scattering_scale': 0.0331,
            'dust_storm': 0.0
        }
        RTB.set_static_weather(world, **weather_kwargs)
        print("[场景配置] 长尾天气系统已按要求设置完成")

        # ==========================================
        # 2. 轨迹数据清洗与稠密化 (间隔0.5米)
        # ==========================================
        # 解析纯文本轨迹（自动去重噪点，然后插值稠密化）
        raw_npc1 = RTB.parse_string_trajectory(STR_NPC1, min_dist=0.1)
        traj_npc1 = RTB.interpolate_trajectory(raw_npc1, interval=0.5)

        raw_npc2 = RTB.parse_string_trajectory(STR_NPC2, min_dist=0.1)
        traj_npc2 = RTB.interpolate_trajectory(raw_npc2, interval=0.5)

        raw_ego = RTB.parse_string_trajectory(STR_EGO, min_dist=0.1)
        traj_ego = RTB.interpolate_trajectory(raw_ego, interval=0.5)

        raw_ped = RTB.parse_string_trajectory(STR_PED, min_dist=0.1)
        traj_ped = RTB.interpolate_trajectory(raw_ped, interval=0.5)

        # 绘制所有实体的预设轨迹
        RTB.draw_preset_trajectory(world, traj_npc1, color=carla.Color(200, 0, 0))  # 红色 NPC1
        RTB.draw_preset_trajectory(world, traj_npc2, color=carla.Color(0, 0, 200))  # 蓝色 NPC2
        RTB.draw_preset_trajectory(world, traj_ego, color=carla.Color(0, 255, 0))  # 绿色 Ego
        RTB.draw_preset_trajectory(world, traj_ped, color=carla.Color(255, 255, 0))  # 黄色 行人

        # ==========================================
        # 3. 实体安全生成
        # ==========================================
        # 提取起点坐标(x, y, yaw)
        npc1 = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=raw_npc1[0][0], y=raw_npc1[0][1],
                                 yaw=raw_npc1[0][2], role_name='npc1')
        npc2 = RTB.spawn_vehicle(world, 'vehicle.tesla.model3', x=raw_npc2[0][0], y=raw_npc2[0][1], yaw=raw_npc2[0][2],
                                 role_name='npc2')
        ego = RTB.spawn_vehicle(world, 'vehicle.audi.tt', x=raw_ego[0][0], y=raw_ego[0][1], yaw=raw_ego[0][2],
                                role_name='ego')

        actor_list.extend([npc1, npc2, ego])

        # 生成行人
        walker_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))

        # 修改蓝图属性：关闭无敌状态（注意 Carla 的属性值通常是传字符串 'false'）
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')

        # 防止行人卡进地底，Z轴抬高一点
        walker_tf = carla.Transform(
            carla.Location(x=raw_ped[0][0], y=raw_ped[0][1], z=1.0),
            carla.Rotation(yaw=raw_ped[0][2])
        )
        walker = world.try_spawn_actor(walker_bp, walker_tf)
        if walker:
            actor_list.append(walker)
            print("[场景配置] 行人生成成功")

        # ==========================================
        # 4 & 6. 剧本状态机与控制器挂载
        # ==========================================
        # NPC1 状态机：初始30km/h，过 y=-120 减速停车，等1s后恢复 60km/h
        # 轨迹是从 -163 向正方向移动，因此触发条件是 y_greater
        sm_npc1 = RTB.MultiStageBehaviorMachine(initial_speed=30.0)
        sm_npc1.add_stage('y_greater', target_speed=0.0, trigger_val=-125.0, accel=35.0)
        sm_npc1.add_stage('time', target_speed=60.0, trigger_val=2, accel=35.0)

        # NPC2 状态机：初始65km/h，过 y=-160 减速到30km/h，等6s后恢复 60km/h
        sm_npc2 = RTB.MultiStageBehaviorMachine(initial_speed=65.0)
        sm_npc2.add_stage('y_greater', target_speed=30.0, trigger_val=-160.0, accel=15.0)
        sm_npc2.add_stage('time', target_speed=60.0, trigger_val=6.0, accel=15.0)

        # Ego 状态机：初始60km/h，过 y=-156 减速40km/h，到 y=-131 停车，等5s后恢复 60km/h
        sm_ego = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_ego.add_stage('y_greater', target_speed=40.0, trigger_val=-156.0, accel=15.0)
        sm_ego.add_stage('y_greater', target_speed=0.0, trigger_val=-131.0, accel=80.0)
        sm_ego.add_stage('time', target_speed=60.0, trigger_val=5.0, accel=15.0)

        # 行人状态机与控制器：静止6s，然后 5.5m/s (19.8km/h) 奔跑
        sm_ped = RTB.MultiStageBehaviorMachine(initial_speed=0.0)
        sm_ped.add_stage('time', target_speed=5.5 * 3.6, trigger_val=6.0, accel=100.0)  # 行人加速度设极大
        ped_ctrl = RTB.PedestrianController(walker, mode='trajectory', target_list=traj_ped)

        # 为每一辆车独立挂载 PID
        pid_lon_1 = RTB.PIDLongitudinalController()
        pid_lat_1 = RTB.PIDLateralController()

        pid_lon_2 = RTB.PIDLongitudinalController()
        pid_lat_2 = RTB.PIDLateralController()

        pid_lon_ego = RTB.PIDLongitudinalController()
        pid_lat_ego = RTB.PIDLateralController()

        # ==========================================
        # 5. 灯光管理器配置
        # ==========================================
        lm_npc2 = RTB.VehicleLightManager(npc2)
        lm_npc2.set_static_lights(low_beam=False, high_beam=False)  # NPC2仅开启行车灯

        lm_ego = RTB.VehicleLightManager(ego)
        lm_ego.set_static_lights(low_beam=True, high_beam=False)  # Ego开启行车灯与近光灯

        # ==========================================
        # 7. 预热与初始速度瞬间注入
        # ==========================================
        # 【此处已被修复】把关键字从 yaw 改成了标准库要求的 yaw_deg
        if npc1: RTB.set_vehicle_initial_speed(npc1, 30.0, yaw_deg=raw_npc1[0][2])
        if npc2: RTB.set_vehicle_initial_speed(npc2, 65.0, yaw_deg=raw_npc2[0][2])
        if ego:  RTB.set_vehicle_initial_speed(ego, 70.0, yaw_deg=raw_ego[0][2])

        # 寻路索引记录
        idx_npc1, idx_npc2, idx_ego = 0, 0, 0
        speed_npc1, speed_npc2, speed_ego = 30.0, 65.0, 70.0

        print("[仿真启动] 剧本场景开始执行...")

        # ==========================================
        # 8. 仿真主循环
        # ==========================================
        while True:
            # 记录本帧开始的时间，用于补齐时钟
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- 车辆1 (NPC1) 控制 ----------------
            if npc1 and npc1.is_alive:
                # 出界守护: true表示已出界并自动销毁
                if not RTB.check_vehicle_out_of_bounds(npc1, carla_map, auto_destroy=True):
                    speed_npc1 = sm_npc1.tick(npc1.get_location(), sim_time, dt)
                    target_wp_1, idx_npc1 = RTB.get_target_waypoint(npc1.get_location(), traj_npc1, idx_npc1,
                                                                    speed_npc1)
                    if target_wp_1:
                        RTB.apply_pid_control(npc1, pid_lon_1, pid_lat_1, speed_npc1, target_wp_1)

            # ---------------- 车辆2 (NPC2) 控制 ----------------
            if npc2 and npc2.is_alive:
                if not RTB.check_vehicle_out_of_bounds(npc2, carla_map, auto_destroy=True):
                    speed_npc2 = sm_npc2.tick(npc2.get_location(), sim_time, dt)
                    target_wp_2, idx_npc2 = RTB.get_target_waypoint(npc2.get_location(), traj_npc2, idx_npc2,
                                                                    speed_npc2)
                    if target_wp_2:
                        RTB.apply_pid_control(npc2, pid_lon_2, pid_lat_2, speed_npc2, target_wp_2)
                        lm_npc2.auto_update_from_control()  # 根据刹车转向自动控制车灯

            # ---------------- 车辆3 (Ego) 控制 ----------------
            if ego and ego.is_alive:
                if is_vehicle_near_trajectory_end(ego, traj_ego):
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    print("[场景结束] Ego 已到达轨迹终点附近，销毁所有 Actor 并结束运行。")
                    break

                if not RTB.check_vehicle_out_of_bounds(ego, carla_map, auto_destroy=True):
                    speed_ego = sm_ego.tick(ego.get_location(), sim_time, dt)
                    target_wp_ego, idx_ego = RTB.get_target_waypoint(ego.get_location(), traj_ego, idx_ego, speed_ego)
                    if target_wp_ego:
                        RTB.apply_pid_control(ego, pid_lon_ego, pid_lat_ego, speed_ego, target_wp_ego)
                        lm_ego.auto_update_from_control()

                        # 绘制Ego车辆当前的实时预瞄点与牵引线
                        RTB.draw_lookahead_point(world, ego.get_location(), target_wp_ego)

            # ---------------- 行人控制 ----------------
            if walker and walker.is_alive:
                # 状态机获取的是km/h，需要除以3.6转为m/s传入行人控制器
                ped_speed_kmh = sm_ped.tick(walker.get_location(), sim_time, dt)
                ped_ctrl.run_step(dt, sim_time, dynamic_speed=(ped_speed_kmh / 3.6))

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    except Exception as e:
        print(f"\n[错误异常] 仿真出现异常: {e}")
    finally:
        # 恢复异步模式并一键清理场景实体
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")


if __name__ == '__main__':
    main()