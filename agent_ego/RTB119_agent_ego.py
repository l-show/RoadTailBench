import sys
import carla
import time

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

# 全局导入标准化函数库
import RoadTailBenchInitV9 as RTB

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        dt = 0.05  # 20FPS

        # 1. 环境初始化：帧率同步与天气系统
        # ==========================================
        RTB.enable_synchronous_mode(world, dt=dt)

        # 按截图严格设置天气：暗夜（Sun Alt=-90）、积水雨天并带有微雾
        RTB.set_static_weather(
            world,
            cloudiness=5.0,
            precipitation=50.0,
            precipitation_deposits=50.0,
            wind_intensity=50.0,
            sun_azimuth_angle=-1.0,
            sun_altitude_angle=-90.0,
            fog_density=2.0,
            fog_distance=0.0,
            fog_falloff=2.0,
            wetness=50.0,
            scattering_intensity=0.0,
            mie_scattering_scale=0.0,
            rayleigh_scattering_scale=0.0,
            dust_storm=0.0
        )
        print("[场景配置] 天气系统已按照要求设置完毕。")

        # 2. 轨迹数据硬编码与清洗
        # ==========================================
        raw_traj_audi_str = """
        2.26	-297.12	88.858
        2.26	-297.12	88.858
        2.26	-297.12	88.858
        2.26	-297.12	88.858
        2.26	-297.12	89.85
        2.251	-288.684	90.245
        2.219	-278.678	90.035
        2.108	-268.497	92.337
        1.57	-258.44	92.907
        1.189	-248.236	90.52
        1.258	-238.269	89.188
        1.47	-228.243	88.628
        1.683	-217.959	89.692
        1.506	-207.812	91.328
        1.361	-197.87	90.502
        1.339	-187.76	90.012
        1.337	-177.543	90.012
        1.391	-167.679	89.196
        1.549	-157.262	89.126
        1.7	-147.35	89.126
        1.69	-137.318	91.201
        1.467	-127.315	91.621
        1.247	-117.002	89.902
        1.264	-106.819	89.902
        1.282	-96.71	89.902
        1.33	-86.476	89.622
        1.394	-76.649	89.622
        1.461	-66.507	89.622
        1.53	-56.136	89.692
        1.451	-45.876	90.553
        1.371	-36.086	90.073
        1.524	-25.859	88.651
        1.692	-15.686	89.743
        1.689	-5.558	90.261
        1.644	4.478	90.261
        1.519	14.576	91.262
        1.296	24.695	91.262
        1.087	34.921	91.052
        0.946	45.111	89.905
        0.963	55.193	89.905
        1.05	65.232	89.094
        1.212	75.474	89.094
        1.209	85.34	90.548
        1.209	85.34	90.548
        1.209	85.34	90.548
        """

        raw_traj_ego_str = """
        5.516	-55.515	-91.814
        5.516	-55.515	-91.814
        5.516	-55.515	-91.814
        5.516	-55.515	-91.814
        5.383	-59.024	-93.026
        5.183	-64.15	-90.656
        5.19	-69.257	-89.567
        5.23	-74.235	-89.497
        5.274	-79.269	-89.497
        5.318	-84.292	-89.567
        5.34	-89.498	-89.837
        5.354	-94.543	-89.837
        5.369	-99.643	-89.837
        5.383	-104.785	-89.837
        5.414	-109.761	-89.632
        5.448	-114.826	-89.237
        5.539	-119.96	-88.976
        5.625	-124.9	-89.248
        5.608	-130.116	-90.721
        5.525	-135.119	-91.071
        5.416	-140.209	-91.281
        5.315	-145.222	-90.861
        5.254	-150.431	-90.388
        5.234	-155.445	-89.968
        5.242	-160.396	-89.898
        5.251	-165.517	-89.898
        5.26	-170.665	-89.898
        5.269	-175.673	-89.898
        5.278	-180.779	-89.898
        5.208	-185.891	-94.181
        4.567	-190.915	-97.577
        3.952	-196.048	-96.738
        3.357	-201.082	-96.738
        2.763	-206.112	-96.738
        2.264	-211.096	-92.483
        2.326	-216.153	-86.721
        2.877	-221.228	-79.824
        3.785	-226.18	-79.6
        4.413	-229.611	-79.67
        4.413	-229.611	-79.67
        4.842	-231.967	-79.74
        5.491	-236.993	-86.775
        5.61	-242.116	-90.504
        5.498	-247.113	-90.625
        5.453	-252.175	-90.763
        5.379	-257.328	-90.833
        5.305	-262.395	-90.833
        5.232	-267.429	-90.833
        """

        # 文本一键解析与坐标密集清洗
        traj_audi = RTB.parse_string_trajectory(raw_traj_audi_str, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(raw_traj_ego_str, min_dist=0.5)

        # 3. 车辆实体安全生成
        # ==========================================
        audi = RTB.spawn_vehicle(world, 'vehicle.audi.tt',
                                 x=traj_audi[0][0], y=traj_audi[0][1], yaw=traj_audi[0][2],
                                 role_name="audi")
        if audi: actor_list.append(audi)

        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        # 4. 车辆 PID 控制器挂载
        # ==========================================
        pid_lon_audi = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_audi = RTB.PIDLateralController(preset='default_car', dt=dt)

        # 5. 车辆灯光管理器
        # ==========================================
        light_mgr_audi = None
        if audi:
            light_mgr_audi = RTB.VehicleLightManager(audi)
            light_mgr_audi.set_static_lights(low_beam=True, high_beam=True)  # 开启行车与远光灯

        light_mgr_ego = None
        if ego:
            pass

        # 6. 剧本状态机编排
        # ==========================================
        audi_sm = RTB.MultiStageBehaviorMachine(initial_speed=60.0)  # 第一辆车全图维持 60km/h

        # Ego车需求：在 y = -180 时减速到 30km/h。(由于是从 y=-55 往 y=-297 跑，即满足 y < -180)

        # 7. 预热与初始状态注入
        # ==========================================
        if audi: RTB.set_vehicle_initial_speed(audi, target_speed_kmh=60.0, yaw_deg=traj_audi[0][2])

        idx_audi = 0
        idx_ego = 0
        sim_time = 0.0

        print("[RoadTailBench] ✅ 车辆装载与物理预热完毕，仿真开始！")

        # 8. 仿真主循环（帧率同步与环境清理守护）
        # ==========================================
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # ---------------- Audi 控制逻辑 ----------------
            if audi and audi.is_alive:
                # 环境守护：如果偏离路线到达终点且出界，则直接销毁退出物理运算
                if not RTB.check_vehicle_out_of_bounds(audi, carla_map, auto_destroy=True):
                    # 车灯动态联动（遇弯打灯、减速亮刹车灯）
                    light_mgr_audi.auto_update_from_control()

                    # 状态机请求目标速度
                    target_spd_audi = audi_sm.tick(audi.get_location(), sim_time, dt)

                    # 动态预瞄寻路
                    wp_audi, idx_audi = RTB.get_target_waypoint(audi.get_location(), traj_audi, idx_audi,
                                                                target_spd_audi)
                    if wp_audi:
                        RTB.apply_pid_control(audi, pid_lon_audi, pid_lat_audi, target_spd_audi, wp_audi)

            # ---------------- Ego 控制逻辑 ----------------

            # 当两辆车都走完轨迹出界被销毁后，可主动跳出循环结束场景
            if (not audi or not audi.is_alive) and (not ego or not ego.is_alive):
                print("[场景结束] 两车均已完成剧本并驶出轨迹。")
                break

            # ---------------- 硬件时钟补齐 (强制 1X 真实时间流逝) ----------------
            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        # 恢复异步模式并一键清理场景实体，防止下一次启动时报错
        RTB.disable_synchronous_mode(world)
        RTB.cleanup_actors(client, actor_list)
        print("[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()