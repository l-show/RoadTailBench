# -*- coding: utf-8 -*-
import sys
import carla
import time
import math
import random  # 新增：用于生成自然翻滚的随机数

# 1. 动态引入标准化函数库路径
LIBRARY_PATH = r"G:\RoadTailCode\标准化函数库"
if LIBRARY_PATH not in sys.path:
    sys.path.append(LIBRARY_PATH)

import RoadTailBenchInitV9 as RTB

# ==========================================
# 轨迹数据
# ==========================================
RAW_TRAJ_TRUCK1 = """
153.573	-40.147	-78.304
154.86	-46.369	-78.304
156.048	-52.505	-80.542
157.097	-58.877	-80.751
158.009	-65.06	-82.455
158.843	-71.465	-82.667
159.55	-77.677	-84.546
160.138	-83.899	-84.686
160.632	-90.127	-86.368
161.038	-96.463	-86.228
160.933	-102.779	-105.849
158.074	-108.434	-117.68
156.591	-114.475	-99.678
156.282	-120.775	-82.089
156.917	-127.095	-86.654
157.268	-133.549	-87.571
157.442	-139.895	-88.785
157.419	-146.346	-91.512
157.155	-152.586	-92.591
156.826	-159.032	-93.156
156.465	-165.269	-94.081
155.92	-171.576	-95.573
155.248	-177.789	-97.361
154.362	-183.975	-98.229
153.425	-190.365	-98.512
152.39	-196.739	-100.295
151.255	-202.99	-100.295
150.133	-209.138	-100.575
148.953	-215.382	-100.718
147.755	-221.624	-100.931
146.512	-227.961	-101.213
145.271	-234.087	-101.639
143.968	-240.411	-101.639
142.629	-246.729	-102.86
141.238	-252.823	-102.86
139.82	-259.017	-102.93
138.351	-265.305	-103.786
136.856	-271.369	-103.786
135.335	-277.637	-103.577
133.821	-283.905	-103.577
132.3	-290.175	-104.002
130.677	-296.317	-105.496
128.976	-302.331	-106.349
127.103	-308.509	-107.052
125.2	-314.678	-107.91
123.246	-320.722	-107.91
121.325	-326.667	-107.91
119.34	-332.812	-107.91
119.276	-333.01	-107.91
"""

RAW_TRAJ_TRUCK2 = """
159.893	-52.673	-80.748
161.474	-62.71	-81.598
162.876	-72.606	-82.025
164.05	-82.71	-84.099
164.997	-92.999	-85.737
165.705	-103.141	-86.587
165.955	-113.47	-90.554
165.7	-120.632	-93.152
165.858	-130.246	-74.176
170.612	-138.167	-48.361
"""

RAW_TRAJ_EGO = """
135.405	13.571	-74.435
136.656	9.080	-74.435
137.946	4.598	-73.665
139.266	0.124	-73.525
140.728	-4.918	-74.015
142.006	-9.406	-74.155
143.280	-13.894	-74.155
144.554	-18.383	-74.155
145.827	-22.873	-74.225
147.058	-27.373	-74.924
148.243	-31.887	-76.324
149.451	-36.996	-77.794
150.362	-41.572	-79.824
151.186	-46.164	-79.824
151.907	-50.182	-79.824
152.526	-53.626	-79.824
153.114	-56.907	-79.824
153.700	-60.188	-80.174
154.264	-63.472	-80.383
154.792	-66.763	-81.293
155.286	-70.059	-81.783
155.754	-73.359	-81.993
156.231	-76.657	-81.363
156.747	-79.950	-80.593
157.299	-83.236	-80.383
157.856	-86.522	-80.383
158.419	-89.808	-80.243
159.207	-94.322	-80.033
159.207	-94.322	-80.033
159.207	-94.322	-80.033
159.207	-94.322	-80.033
159.207	-94.322	-80.033
159.736	-97.190	-78.703
160.396	-100.457	-78.423
161.068	-103.722	-78.353
161.711	-106.992	-79.583
162.250	-110.281	-81.333
162.704	-113.582	-86.163
162.753	-116.912	-93.022
162.445	-120.230	-96.452
162.071	-123.543	-96.452
161.576	-126.837	-100.301
160.958	-130.113	-101.981
160.242	-133.368	-102.611
159.521	-136.622	-102.121
158.873	-139.892	-99.882
158.353	-143.183	-96.382
158.048	-146.501	-94.128
157.834	-149.826	-93.358
157.639	-153.151	-93.358
157.413	-156.892	-93.848
157.183	-160.216	-94.128
156.925	-163.537	-94.828
156.638	-166.856	-94.968
156.273	-170.167	-97.068
156.068	-171.821	-97.068
155.914	-173.061	-97.068
155.501	-176.366	-97.348
155.066	-179.667	-97.628
154.624	-182.968	-97.628
154.182	-186.269	-97.628
153.722	-189.568	-98.188
153.247	-192.865	-98.188
152.770	-196.161	-98.678
152.238	-199.449	-100.078
151.655	-202.729	-100.078
151.063	-206.007	-100.778
150.431	-209.279	-101.268
149.775	-212.545	-101.408
149.116	-215.811	-101.408
148.457	-219.077	-101.408
147.798	-222.344	-101.408
147.135	-225.609	-101.478
146.468	-228.874	-101.688
145.708	-232.545	-101.688
145.033	-235.809	-101.688
144.358	-239.072	-101.688
144.190	-239.888	-101.688
144.190	-239.888	-101.688
"""

RAW_TRAJ_IMPALA = """
99.519	126.908	-72.669
100.442	123.966	-72.526
102.003	119.217	-71.379
103.644	114.32	-71.522
105.617	108.414	-71.522
107.855	101.717	-71.522
110.187	94.32	-73.036
112.486	86.919	-72.608
114.728	79.762	-72.608
117.387	71.273	-72.608
120.003	62.923	-72.608
122.662	54.434	-72.608
125.365	45.806	-72.608
128.124	37.195	-72.177
130.803	28.864	-72.177
133.526	20.395	-72.177
136.199	12.064	-72.247
138.686	3.375	-76.777
140.543	-5.325	-79.445
142.054	-14.091	-80.376
143.571	-22.857	-79.74
145.313	-31.431	-78.017
147.16	-40.133	-78.017
148.92	-48.704	-79.152
150.465	-57.612	-81.293
151.622	-66.252	-82.59
152.762	-75.076	-83.015
153.742	-84.063	-84.658
154.441	-92.93	-86.507
154.909	-101.959	-87.575
155.083	-110.997	-89.877
155.117	-120.038	-89.737
155.178	-128.786	-89.527
155.194	-137.825	-91.171
154.859	-146.565	-94.38
154.182	-155.218	-94.593
153.395	-164.223	-95.373
152.439	-173.212	-96.299
151.407	-181.902	-97.728
150.151	-190.56	-99.517
148.522	-199.451	-100.588
146.898	-208.047	-101.013
145.193	-216.628	-101.653
143.426	-225.196	-101.653
141.628	-233.911	-101.653
139.86	-242.483	-101.653
138.093	-251.05	-101.653
136.317	-259.617	-101.866
134.398	-268.152	-103.564
132.199	-276.922	-104.347
130.03	-285.4	-104.347
127.815	-294.016	-104.77
125.513	-302.609	-105.265
123.069	-311.01	-107.265
120.238	-319.289	-109.571
117.318	-327.566	-109.428
114.431	-335.827	-109.216
111.485	-344.376	-108.933
108.632	-352.648	-109.143
105.667	-361.19	-109.143
102.796	-369.46	-109.143
"""

def get_local_transform_data(base_tf, target_loc, target_yaw):
    yaw = math.radians(-base_tf.rotation.yaw)
    dx = target_loc.x - base_tf.location.x
    dy = target_loc.y - base_tf.location.y
    dz = target_loc.z - base_tf.location.z

    local_x = dx * math.cos(yaw) - dy * math.sin(yaw)
    local_y = dx * math.sin(yaw) + dy * math.cos(yaw)
    local_z = dz
    rel_yaw = target_yaw - base_tf.rotation.yaw
    return carla.Location(local_x, local_y, local_z), rel_yaw

def tune_high_speed_truck_physics(vehicle, label):
    if not vehicle:
        return
    try:
        physics = vehicle.get_physics_control()
        physics.mass = 3200.0
        physics.drag_coefficient = 0.05
        physics.max_rpm = 9000.0
        physics.torque_curve = [
            carla.Vector2D(x=0.0, y=4500.0),
            carla.Vector2D(x=1500.0, y=7000.0),
            carla.Vector2D(x=4500.0, y=9000.0),
            carla.Vector2D(x=9000.0, y=9000.0),
        ]
        if hasattr(physics, 'moi'):
            physics.moi = 1.0
        if hasattr(physics, 'damping_rate_full_throttle'):
            physics.damping_rate_full_throttle = 0.05
        if hasattr(physics, 'damping_rate_zero_throttle_clutch_engaged'):
            physics.damping_rate_zero_throttle_clutch_engaged = 0.05
        if hasattr(physics, 'damping_rate_zero_throttle_clutch_disengaged'):
            physics.damping_rate_zero_throttle_clutch_disengaged = 0.05
        if hasattr(physics, 'use_gear_autobox'):
            physics.use_gear_autobox = True
        if hasattr(physics, 'gear_switch_time'):
            physics.gear_switch_time = 0.05
        if hasattr(physics, 'clutch_strength'):
            physics.clutch_strength = 30.0
        if hasattr(physics, 'final_ratio'):
            physics.final_ratio = 2.4
        for gear in getattr(physics, 'forward_gears', []):
            if hasattr(gear, 'up_ratio'):
                gear.up_ratio = 0.95
            if hasattr(gear, 'down_ratio'):
                gear.down_ratio = 0.35
        for wheel in getattr(physics, 'wheels', []):
            if hasattr(wheel, 'tire_friction'):
                wheel.tire_friction = max(wheel.tire_friction, 4.0)
            if hasattr(wheel, 'max_steer_angle'):
                wheel.max_steer_angle = max(wheel.max_steer_angle, 35.0)
        vehicle.apply_physics_control(physics)
        print(f"[RTB117] Applied high-speed truck physics to {label}.")
    except Exception as exc:
        print(f"[RTB117] Failed to tune truck physics for {label}: {exc}")

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()
        dt = 0.05

        RTB.enable_synchronous_mode(world, dt=dt)
        RTB.set_static_weather(
            world, cloudiness=100.0, precipitation=100.0, precipitation_deposits=90.0,
            wind_intensity=100.0, sun_azimuth_angle=-1.0, sun_altitude_angle=45.0,
            fog_density=8.0, fog_distance=0.0, fog_falloff=0.5, wetness=0.0,
            scattering_intensity=1.0, mie_scattering_scale=0.03,
            rayleigh_scattering_scale=0.0331, dust_storm=0.0
        )

        traj_hgv = RTB.parse_string_trajectory(RAW_TRAJ_TRUCK1, min_dist=0.5)
        traj_firetruck = RTB.parse_string_trajectory(RAW_TRAJ_TRUCK2, min_dist=0.5)
        traj_ego = RTB.parse_string_trajectory(RAW_TRAJ_EGO, min_dist=0.5)
        traj_impala = RTB.parse_string_trajectory(RAW_TRAJ_IMPALA, min_dist=0.5)

        truck1_bp = 'vehicle.carlamotors.european_hgv'
        truck1 = RTB.spawn_vehicle(world, truck1_bp, x=traj_hgv[0][0], y=traj_hgv[0][1], yaw=traj_hgv[0][2],
                                   z_offset=0.2)
        actor_list.append(truck1)

        tune_high_speed_truck_physics(truck1, 'truck1')
        RTB.set_vehicle_initial_speed(truck1, target_speed_kmh=100.0, yaw_deg=traj_hgv[0][2])

        base_box_coords_m = [
            carla.Location(x=158.642646, y=-50.189965, z=1.3387532),
            carla.Location(x=160.375917, y=-49.741772, z=1.34702805),
            carla.Location(x=159.515068, y=-50.0458102, z=1.34702927),
            carla.Location(x=159.004023, y=-51.3481202, z=1.33873245),
            carla.Location(x=160.059335, y=-51.172705, z=1.34702988),
            carla.Location(x=159.775263, y=-52.688481, z=1.34702988)
        ]

        all_box_coords_m = []
        for i in range(6):
            loc = base_box_coords_m[i]
            all_box_coords_m.append(carla.Location(x=loc.x, y=loc.y, z=loc.z))
            # 【细节优化】：初始生成时，上层箱子间距额外增加 2cm，防止极其轻微的边缘摩擦
            all_box_coords_m.append(carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.66156677))

        bp_box = bp_lib.find('static.prop.mesh')
        bp_box.set_attribute('mesh_path', '/Game/Carla/Static/Dynamic/Trash/SM_Box01.SM_Box01')
        bp_box.set_attribute('mass', '5.0')
        bp_box.set_attribute('scale', '0.8')

        boxes = []
        box_rel_data = []

        virtual_truck2_tf = carla.Transform(
            carla.Location(x=159.893, y=-52.673, z=0.2),
            carla.Rotation(yaw=-80.748)
        )
        for abs_loc in all_box_coords_m:
            rel_loc, rel_yaw = get_local_transform_data(virtual_truck2_tf, abs_loc, 0.0)
            box_rel_data.append((rel_loc, rel_yaw))

        truck_tf = truck1.get_transform()
        for rel_loc, rel_yaw in box_rel_data:
            abs_loc = carla.Location(x=rel_loc.x, y=rel_loc.y, z=rel_loc.z)
            truck_tf.transform(abs_loc)
            abs_yaw = truck_tf.rotation.yaw + rel_yaw

            spawn_tf = carla.Transform(abs_loc, carla.Rotation(yaw=abs_yaw))
            box = world.try_spawn_actor(bp_box, spawn_tf)
            if box:
                box.set_simulate_physics(False)
                box.set_collisions(False)
                boxes.append(box)
                actor_list.append(box)

        print(f"[初始化] 成功生成 {len(boxes)} 个货物箱。")
        boxes_dropped = False

        truck2 = RTB.spawn_vehicle(world, 'vehicle.carlamotors.firetruck', x=traj_firetruck[0][0],
                                   y=traj_firetruck[0][1], yaw=traj_firetruck[0][2])
        actor_list.append(truck2)
        tune_high_speed_truck_physics(truck2, 'truck2')
        RTB.set_vehicle_initial_speed(truck2, target_speed_kmh=60.0, yaw_deg=traj_firetruck[0][2])

        ego = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed

        impala = RTB.spawn_vehicle(world, 'vehicle.chevrolet.impala', x=traj_impala[0][0], y=traj_impala[0][1],
                                   yaw=traj_impala[0][2])
        actor_list.append(impala)
        RTB.set_vehicle_initial_speed(impala, target_speed_kmh=100.0, yaw_deg=traj_impala[0][2])

        pid_lon_t1 = RTB.PIDLongitudinalController(preset='truck', dt=dt)
        pid_lat_t1 = RTB.PIDLateralController(preset='truck', dt=dt)
        idx_t1 = 0

        pid_lon_t2 = RTB.PIDLongitudinalController(preset='truck', dt=dt)
        pid_lat_t2 = RTB.PIDLateralController(preset='truck', dt=dt)
        idx_t2 = 0

        idx_ego = 0

        pid_lon_im = RTB.PIDLongitudinalController(preset='default_car', dt=dt)
        pid_lat_im = RTB.PIDLateralController(preset='default_car', dt=dt)
        idx_im = 0

        sm_truck1 = RTB.MultiStageBehaviorMachine(initial_speed=100.0)
        sm_truck1.add_stage(trigger_type='y_less', target_speed=30.0, trigger_val=-105.0, accel=30.0)
        sm_truck1.add_stage(trigger_type='time', target_speed=60.0, trigger_val=5.0, accel=10.0)

        sm_truck2 = RTB.MultiStageBehaviorMachine(initial_speed=60.0)
        sm_truck2.add_stage(trigger_type='time', target_speed=0.0, trigger_val=10.0, accel=15.0)

        sm_impala = RTB.MultiStageBehaviorMachine(initial_speed=100.0)
        sm_impala.add_stage(trigger_type='time', target_speed=60.0, trigger_val=8.0, accel=20.0)

        # 将视角绑定到 Ego 车后方以便观察
        spectator = world.get_spectator()

        sim_time = 0.0
        print("\n🚀 仿真正式开始...")

        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # # ---------------- 视角跟随 ----------------
            # if ego and ego.is_alive:
            #     tf = ego.get_transform()
            #     spectator.set_transform(carla.Transform(
            #         tf.location + carla.Location(z=3.0) - tf.get_forward_vector() * 6.0,
            #         carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)
            #     ))

            # ---- 长尾特效控制流：货物散落物理检测 ----
            if truck1 and truck1.is_alive:

                if boxes and boxes[0].is_alive and not boxes_dropped:
                    t_loc = truck1.get_location()
                    b_loc = boxes[0].get_location()
                    dist = t_loc.distance(b_loc)
                    print(
                        f"\r[雷达] HGV:({t_loc.x:.1f}, {t_loc.y:.1f}) | 箱子1:({b_loc.x:.1f}, {b_loc.y:.1f}) | 相对间距: {dist:.2f} m ",
                        end="")

                if not boxes_dropped:
                    if truck1.get_location().y < -105.0:
                        boxes_dropped = True
                        print("\n🚨 [触发] 急刹！执行防爆炸自由落体物理释放！")
                        current_vel = truck1.get_velocity()

                        new_boxes = []
                        for i, box in enumerate(boxes):
                            if box.is_alive:
                                box_tf = box.get_transform()
                                box.destroy()

                                # 🛡️【核心防爆与防掀车机制】🛡️
                                # 1. 抬升Z轴：拉开箱子与卡车底盘的距离，防止重叠导致的无限弹力掀车
                                box_tf.location.z += 0.1

                                # 2. 差分扩张：上层箱子再抬高一点，防止箱子互相挤压爆炸
                                if i % 2 != 0:
                                    box_tf.location.z += 0.5

                                    # 3. 随机微偏：打破矩阵排列的绝对刚性，防止侧面平行摩擦爆炸
                                box_tf.location.x += random.uniform(-0.02, 0.02)
                                box_tf.location.y += random.uniform(-0.02, 0.02)

                                new_box = world.try_spawn_actor(bp_box, box_tf)
                                if new_box:
                                    new_box.set_simulate_physics(True)
                                    new_box.set_collisions(True)

                                    # 4. 速度衰减与抛射：
                                    # 卡车是在 Y 轴负方向开。摩擦力损耗下，不完全继承100%车速，使用 80%
                                    # Z轴给一个微弱向上的抛射力(1.0)，让它像脱离车厢一样优美抛出
                                    drop_vel = carla.Vector3D(
                                        x=current_vel.x * 0.8,
                                        y=current_vel.y * 0.8,
                                        z=current_vel.z + 1.0
                                    )
                                    new_box.set_target_velocity(drop_vel)

                                    # 5. 翻滚效果：给一个随机角速度，让箱子自然翻滚落地，而不是生硬滑行
                                    tumble_vel = carla.Vector3D(
                                        x=random.uniform(-1.5, 1.5),
                                        y=random.uniform(-1.5, 1.5),
                                        z=random.uniform(-0.5, 0.5)
                                    )
                                    new_box.set_target_angular_velocity(tumble_vel)

                                    new_boxes.append(new_box)
                                    actor_list.append(new_box)
                        boxes = new_boxes
                    else:
                        truck_tf = truck1.get_transform()
                        for i, box in enumerate(boxes):
                            if box.is_alive:
                                rel_loc, rel_yaw = box_rel_data[i]
                                abs_loc = carla.Location(x=rel_loc.x, y=rel_loc.y, z=rel_loc.z)
                                truck_tf.transform(abs_loc)
                                box.set_transform(
                                    carla.Transform(abs_loc, carla.Rotation(yaw=truck_tf.rotation.yaw + rel_yaw)))

            # ---- 车辆PID运动控制流 ----
            if truck1 and truck1.is_alive:
                loc = truck1.get_location()
                target_speed = sm_truck1.tick(loc, sim_time, dt)
                wp, idx_t1 = RTB.get_target_waypoint(loc, traj_hgv, idx_t1, target_speed)
                if wp: RTB.apply_pid_control(truck1, pid_lon_t1, pid_lat_t1, target_speed, wp)

            if truck2 and truck2.is_alive:
                loc = truck2.get_location()
                target_speed = sm_truck2.tick(loc, sim_time, dt)
                wp, idx_t2 = RTB.get_target_waypoint(loc, traj_firetruck, idx_t2, target_speed)
                if wp: RTB.apply_pid_control(truck2, pid_lon_t2, pid_lat_t2, target_speed, wp)

            if impala and impala.is_alive:
                loc = impala.get_location()
                target_speed = sm_impala.tick(loc, sim_time, dt)
                wp, idx_im = RTB.get_target_waypoint(loc, traj_impala, idx_im, target_speed)
                if wp: RTB.apply_pid_control(impala, pid_lon_im, pid_lat_im, target_speed, wp)

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("\n[场景中断] 用户手动中断了仿真。")
    finally:
        try:
            RTB.disable_synchronous_mode(world)
            _rtb_opt_cleanup_scene(locals(), client, world)
        except Exception as e:
            pass
        print("\n[场景结束] 资源已安全回收。")

if __name__ == '__main__':
    main()
