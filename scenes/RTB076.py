import carla
import time
import math
import numpy as np

# ==========================================
# 1. Sprinter truck trajectory
# ==========================================
RAW_Sprinter_TRAJECTORY = [
    (3.677, 107.183, -93.454), (3.677, 107.183, -93.454), (3.677, 107.183, -93.454),
    (3.677, 107.183, -93.314), (3.23, 99.573, -92.894), (3.163, 97.581, -90.375),
    (3.163, 97.581, -90.445), (3.163, 97.581, -89.885), (3.163, 97.581, -89.885),
    (3.295, 89.837, -87.504), (3.45, 86.441, -87.714), (3.641, 80.945, -88.764),
    (3.647, 78.395, -91.494), (3.512, 72.397, -89.814), (3.476, 71.098, -91.563),
    (3.323, 67.802, -93.173), (3.256, 65.704, -91.283), (3.192, 62.855, -91.283),
    (3.239, 55.106, -89.813), (3.163, 53.509, -93.523), (3.076, 51.511, -91.423),
    (3.037, 49.962, -91.423), (3.000, 48.262, -87.643), (3.016, 45.863, -90.512),
    (2.986, 43.863, -90.862), (2.971, 43.063, -92.612), (2.922, 41.665, -102.2),
    (2.294, 40.414, -121.798), (1.552, 39.347, -124.808), (0.181, 35.654, -101.011),
    (0.359, 33.513, -83.442), (0.793, 30.242, -81.762), (1.537, 24.393, -90.721),
    (1.549, 20.094, -87.85), (1.691, 15.997, -88.34), (1.866, 9.95, -88.34),
    (1.91, 5.751, -90.23), (1.898, 1.602, -89.81), (1.871, -4.748, -90.86),
    (1.784, -11.747, -90.509), (1.739, -17.347, -90.37), (1.682, -23.196, -90.579),
    (1.626, -28.746, -90.579), (1.562, -34.995, -90.579), (1.51, -41.044, -90.23),
    (1.512, -45.844, -89.39), (1.593, -51.743, -88.69), (1.734, -57.891, -88.76),
    (1.826, -64.14, -89.249), (1.853, -69.439, -90.509), (1.796, -73.937, -90.859),
    (1.765, -78.436, -89.109), (1.846, -83.335, -88.479), (1.978, -89.783, -89.669),
    (1.912, -94.432, -91.138), (1.834, -98.78, -90.648), (1.828, -104.18, -89.528),
    (1.932, -109.929, -88.548), (2.021, -115.277, -89.738), (2.038, -118.977, -89.668),
    (2.1, -125.672, -89.528), (2.133, -131.42, -89.668), (2.288, -143.068, -89.598),
    (2.296, -153.366, -89.878), (2.356, -160.615, -89.528), (2.476, -172.064, -89.317),
    (2.517, -179.764, -89.947), (2.528, -191.414, -89.947), (2.538, -203.014, -89.947),
    (2.541, -205.964, -89.947), (2.546, -210.914, -89.947), (2.546, -210.914, -89.947)
]

Sprinter_TRAJECTORY = []
for p in RAW_Sprinter_TRAJECTORY:
    if not Sprinter_TRAJECTORY or p != Sprinter_TRAJECTORY[-1]:
        Sprinter_TRAJECTORY.append(p)

# ==========================================
# 2. HGV trajectory
# ==========================================
RAW_HGV_TRAJECTORY = [
    (-2.186, -49.028, 88.272), (-2.186, -49.028, 88.622), (-2.186, -49.028, 89.042),
    (-2.186, -49.028, 89.252), (-2.186, -49.028, 89.252), (-2.134, -44.628, 89.392),
    (-2.130, -44.328, 89.532), (-2.124, -43.528, 89.532), (-2.065, -36.328, 89.602),
    (-2.065, -36.328, 88.131), (-1.925, -32.581, 87.991), (-1.804, -29.034, 89.251),
    (-1.823, -25.084, 91.071), (-1.823, -25.084, 88.901), (-1.780, -22.834, 88.901),
    (-1.714, -19.435, 88.761), (-1.597, -16.638, 87.571), (-1.368, -12.494, 86.801),
    (-1.187, -9.249, 86.801), (-1.187, -9.249, 86.801), (-1.187, -9.249, 87.710),
    (-1.125, -7.102, 87.850), (-1.018, -5.064, 87.010), (-0.878, -2.381, 87.010),
    (-0.878, -2.381, 86.870), (-0.805, -1.039, 86.730), (-0.699, 0.800, 86.660),
    (-0.522, 4.685, 93.380), (-1.008, 6.625, 104.509), (-1.443, 8.006, 138.385),
    (-2.949, 9.238, 141.885), (-2.949, 9.238, 158.053), (-2.949, 9.238, 150.704),
    (-4.337, 10.026, 150.424), (-7.953, 11.451, 165.962), (-11.073, 12.128, 170.090),
    (-11.713, 12.236, -178.991), (-15.235, 11.833, -161.982), (-20.004, 10.172, -160.022),
    (-24.062, 8.910, -163.171), (-29.250, 7.918, -175.208), (-35.391, 7.619, -177.867),
    (-45.332, 7.248, -177.657), (-54.447, 6.552, -173.457), (-61.029, 5.631, -173.036),
    (-71.595, 4.837, -178.775), (-82.142, 4.660, -178.495), (-92.781, 4.180, -177.235),
    (-102.569, 3.707, -177.235), (-112.807, 3.213, -177.235), (-115.953, 3.061, -177.235),
    (-115.953, 3.061, -177.235)
]

HGV_TRAJECTORY = []
for p in RAW_HGV_TRAJECTORY:
    if not HGV_TRAJECTORY or p != HGV_TRAJECTORY[-1]:
        HGV_TRAJECTORY.append(p)

# ==========================================
# 3. Ego Lincoln MKZ trajectory
# ==========================================
RAW_EGO_TRAJECTORY =[
    (-84.976, 7.800, 3.135), (-83.147, 7.900, 3.135), (-80.612, 8.081, 4.467), (-78.121, 8.278, 4.467),
    (-75.592, 8.453, 3.861), (-73.063, 8.624, 3.861), (-70.575, 8.791, 3.861), (-68.085, 8.959, 3.861),
    (-65.552, 9.142, 4.225), (-63.057, 9.300, 3.498), (-60.561, 9.453, 3.498), (-57.983, 9.621, 4.104),
    (-55.449, 9.818, 4.588), (-52.918, 10.037, 5.315), (-50.430, 10.268, 5.315), (-47.902, 10.501, 4.653),
    (-45.367, 10.693, 3.804), (-42.831, 10.861, 3.804), (-40.334, 10.999, 2.713), (-37.838, 11.141, 3.902),
    (-35.305, 11.313, 3.902), (-32.813, 11.491, 4.508), (-30.280, 11.693, 4.630), (-27.789, 11.895, 4.630),
    (-25.255, 12.096, 4.145), (-22.845, 12.270, 4.145), (-22.330, 12.308, 4.145), (-21.832, 12.344, 4.145),
    (-21.317, 12.381, 4.145), (-20.818, 12.417, 4.145), (-20.320, 12.453, 4.145), (-19.813, 12.490, 4.145),
    (-19.314, 12.526, 4.145), (-18.816, 12.561, 3.071), (-18.308, 12.569, -1.936), (-17.808, 12.543, -3.277),
    (-17.309, 12.513, -5.185), (-16.804, 12.456, -7.055), (-16.301, 12.387, -8.165), (-15.357, 12.252, -8.165),
    (-14.098, 12.077, -5.994), (-12.850, 12.019, 0.355), (-11.926, 12.025, 0.355), (-11.417, 12.028, 0.355),
    (-10.851, 12.031, 0.355), (-9.600, 12.032, -1.704), (-8.354, 11.945, -6.026), (-7.121, 11.744, -12.319),
    (-5.897, 11.402, -18.726), (-4.759, 10.891, -29.332), (-3.710, 10.213, -36.586), (-2.754, 9.378, -44.997),
    (-1.927, 8.441, -49.243), (-1.098, 7.478, -50.194), (-0.356, 6.447, -58.823), (0.240, 5.326, -64.849),
    (0.724, 4.151, -69.785), (1.137, 2.950, -74.282), (1.449, 1.740, -75.793), (1.707, 0.517, -80.312),
    (1.898, -0.740, -83.385), (2.017, -1.984, -84.823), (2.192, -3.913, -84.823), (2.414, -7.717, -89.681),
    (2.435, -11.531, -89.681), (2.456, -15.343, -89.681), (2.477, -19.094, -89.681), (2.498, -22.906, -89.681),
    (2.519, -26.655, -89.681), (2.552, -30.467, -88.576), (2.645, -34.214, -88.576), (2.740, -38.024, -88.315),
    (2.850, -41.771, -88.315), (2.971, -45.581, -87.531), (3.152, -49.389, -88.218), (3.169, -53.139, -90.843),
    (3.113, -56.951, -90.843), (3.051, -60.762, -91.104), (3.004, -63.199, -91.104), (3.004, -63.199, -91.104),
    (3.004, -63.199, -91.104), (3.004, -63.199, -91.104)
]

EGO_TRAJECTORY = []
for p in RAW_EGO_TRAJECTORY:
    if not EGO_TRAJECTORY or p != EGO_TRAJECTORY[-1]:
        EGO_TRAJECTORY.append(p)

# ==========================================
# PID controllers
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


class PIDLateralController2:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d, self._dt = K_P, K_I, K_D, dt
        self._error_buffer = []

    def run_step(self, waypoint_loc, vehicle_transform):
        wp_x = waypoint_loc.x if isinstance(waypoint_loc, carla.Location) else waypoint_loc[0]
        wp_y = waypoint_loc.y if isinstance(waypoint_loc, carla.Location) else waypoint_loc[1]

        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([wp_x - v_begin.x, wp_y - v_begin.y, 0.0])

        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1: return 0.0

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0: _dot *= -1.0

        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30: self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)


# ==========================================
# Trajectory / waypoint helpers
# ==========================================
def get_target_from_trajectory(vehicle_loc, trajectory, lookahead_dist=10.0):
    """Find a lookahead target on fixed (x, y, yaw) trajectory points."""
    min_dist, closest_idx = float('inf'), 0
    for i, p in enumerate(trajectory):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist, closest_idx = dist, i

    target_idx = closest_idx
    current_dist = 0.0
    for i in range(closest_idx, len(trajectory) - 1):
        p1, p2 = trajectory[i], trajectory[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_idx = i + 1
        if current_dist >= lookahead_dist:
            break

    if target_idx >= len(trajectory) - 1 and current_dist < lookahead_dist:
        return None
    return trajectory[target_idx]


def update_speed_towards(current_speed, target_speed, accel_kmh_per_s, dt):
    step = max(0.0, accel_kmh_per_s) * dt
    if current_speed < target_speed:
        return min(current_speed + step, target_speed)
    if current_speed > target_speed:
        return max(current_speed - step, target_speed)
    return current_speed


def distance_to_trajectory_end(actor, trajectory):
    if not actor or not actor.is_alive or not trajectory:
        return float('inf')
    loc = actor.get_location()
    end_x, end_y = trajectory[-1][0], trajectory[-1][1]
    return math.sqrt((loc.x - end_x) ** 2 + (loc.y - end_y) ** 2)


def is_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=8.0):
    if not vehicle or not vehicle.is_alive:
        return False
    try:
        loc = vehicle.get_location()
        waypoint = carla_map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
        if waypoint is None:
            return True
        return loc.distance(waypoint.transform.location) > threshold_dist
    except Exception:
        return False


def destroy_scene_actors(actor_list):
    for actor in list(actor_list):
        try:
            if actor and actor.is_alive:
                actor.destroy()
        except Exception:
            pass


def get_next_waypoint_by_angle(current_wp, vehicle_transform, distance=5.0, action='straight'):
    """Pick the next CARLA waypoint by turn direction."""
    next_wps = current_wp.next(distance)
    if not next_wps:
        return None
    if len(next_wps) == 1:
        return next_wps[0]

    v_forward = vehicle_transform.get_forward_vector()
    best_wp = next_wps[0]

    if action == 'straight':
        min_angle = float('inf')
        for wp in next_wps:
            wp_forward = wp.transform.get_forward_vector()
            angle = math.degrees(math.acos(np.clip(
                v_forward.x * wp_forward.x + v_forward.y * wp_forward.y, -1.0, 1.0)))
            if angle < min_angle:
                min_angle = angle
                best_wp = wp

    elif action == 'left':
        min_cross_z = float('inf')
        for wp in next_wps:
            wp_forward = wp.transform.get_forward_vector()
            cross_z = v_forward.x * wp_forward.y - v_forward.y * wp_forward.x
            if cross_z < min_cross_z:
                min_cross_z = cross_z
                best_wp = wp
        return best_wp

    return best_wp


# ==========================================
# Main
# ==========================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    actor_list = []

    try:
        # Weather
        weather = carla.WeatherParameters(
            cloudiness=40.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=100.0, sun_azimuth_angle=140.0, sun_altitude_angle=60.0,
            fog_density=0.0, fog_distance=0.75, fog_falloff=0.1, wetness=0.0,
            scattering_intensity=6.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.1, dust_storm=0.0
        )
        world.set_weather(weather)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        dt = 0.05

        active_pid_vehicles = []

        # ================= 1. Sprinter truck =================
        bp_Sprinter = bp_lib.find('vehicle.mercedes.sprinter')
        # Spawn at trajectory start to avoid initial teleport jitter.
        start_x_s, start_y_s, start_yaw_s = Sprinter_TRAJECTORY[0]
        trans_Sprinter = carla.Transform(carla.Location(x=start_x_s, y=start_y_s, z=1.5), carla.Rotation(yaw=start_yaw_s))
        Sprinter = world.try_spawn_actor(bp_Sprinter, trans_Sprinter)
        if Sprinter:
            actor_list.append(Sprinter)
            active_pid_vehicles.append({
                'id': 'Sprinter', 'actor': Sprinter,
                'lon_pid': PIDLongitudinalController(), 'lat_pid': PIDLateralController2(),
                'target_speed': 60.0, 'mode': 'trajectory',
                'trajectory': Sprinter_TRAJECTORY
            })
            print("Sprinter spawned successfully (PID trajectory control, 60km/h)")

        # ================= 2. HGV tractor =================
        bp_hgv = bp_lib.find('vehicle.carlamotors.european_hgv')
        # Spawn at trajectory start.
        start_x_h, start_y_h, start_yaw_h = HGV_TRAJECTORY[0]
        trans_hgv = carla.Transform(carla.Location(x=start_x_h, y=start_y_h, z=1.5), carla.Rotation(yaw=start_yaw_h))
        hgv = world.try_spawn_actor(bp_hgv, trans_hgv)
        if hgv:
            actor_list.append(hgv)
            active_pid_vehicles.append({
                'id': 'HGV', 'actor': hgv,
                'lon_pid': PIDLongitudinalController(), 'lat_pid': PIDLateralController2(),
                'target_speed': 100.0, 'mode': 'trajectory',
                'trajectory': HGV_TRAJECTORY
            })
            print("HGV spawned successfully (PID trajectory control, 100km/h)")

        # ================= 3. Ego Lincoln MKZ =================
        bp_lincoln = bp_lib.find('vehicle.lincoln.mkz_2017')
        bp_lincoln.set_attribute('color', '192,192,192')
        if bp_lincoln.has_attribute('role_name'):
            bp_lincoln.set_attribute('role_name', 'ego')
        start_x_e, start_y_e, start_yaw_e = EGO_TRAJECTORY[0]
        trans_lincoln = carla.Transform(
            carla.Location(x=start_x_e, y=start_y_e, z=1.5),
            carla.Rotation(yaw=start_yaw_e)
        )
        ego = world.try_spawn_actor(bp_lincoln, trans_lincoln)

        if ego:
            actor_list.append(ego)
            ego.set_simulate_physics(True)
            active_pid_vehicles.append({
                'id': 'Ego', 'actor': ego, 'is_ego': True,
                'lon_pid': PIDLongitudinalController(), 'lat_pid': PIDLateralController2(),
                'target_speed': 60.0, 'desired_speed': 60.0, 'command_speed': 60.0,
                'accel_kmh_per_s': 15.0, 'mode': 'trajectory',
                'trajectory': EGO_TRAJECTORY,
                'stage': 'cruise', 'stop_start_time': None,
            })
            print("Lincoln MKZ Ego spawned successfully (PID trajectory control, 60km/h)")

        print("\nScenario initialized; simulation running...")

        # Simulation loop
        sim_time = 0.0
        while True:
            start_time = time.time()
            world.tick()
            sim_time += dt

            # Iterate backwards so finished actors can be removed safely.
            for v_data in reversed(active_pid_vehicles):
                vehicle = v_data['actor']
                if not vehicle.is_alive:
                    active_pid_vehicles.remove(v_data)
                    continue

                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                if is_vehicle_out_of_bounds(vehicle, carla_map, threshold_dist=8.0):
                    print(f"[{v_data['id']}] vehicle out of bounds; destroying actor.")
                    vehicle.destroy()
                    active_pid_vehicles.remove(v_data)
                    if vehicle in actor_list:
                        actor_list.remove(vehicle)
                    if v_data.get('is_ego'):
                        destroy_scene_actors(actor_list)
                        return
                    continue

                target_loc = None

                if v_data['mode'] == 'trajectory':
                    target_point = get_target_from_trajectory(tf.location, v_data['trajectory'], lookahead_dist=12.0)
                    if target_point is None:
                        if v_data.get('is_ego') and distance_to_trajectory_end(vehicle, v_data['trajectory']) > 5.0:
                            target_point = v_data['trajectory'][-1]
                            target_loc = target_point
                        else:
                            print(f"[{v_data['id']}] reached trajectory endpoint.")
                            if v_data.get('is_ego'):
                                destroy_scene_actors(actor_list)
                                return
                            vehicle.destroy()
                            active_pid_vehicles.remove(v_data)
                            if vehicle in actor_list:
                                actor_list.remove(vehicle)
                            continue
                    else:
                        target_loc = target_point

                elif v_data['mode'] == 'waypoint':
                    current_wp = carla_map.get_waypoint(tf.location)
                    target_wp = get_next_waypoint_by_angle(current_wp, tf, distance=12.0, action='straight')
                    if target_wp is None:
                        print(f"[{v_data['id']}] reached map end; destroying actor.")
                        vehicle.destroy()
                        active_pid_vehicles.remove(v_data)
                        if vehicle in actor_list:
                            actor_list.remove(vehicle)
                        continue
                    target_loc = target_wp.transform.location

                # PID command update
                if v_data.get('is_ego'):
                    loc = tf.location
                    stage = v_data['stage']
                    if stage == 'cruise' and loc.x >= -45.0:
                        v_data['stage'] = 'slow_30'
                        v_data['desired_speed'] = 30.0
                        v_data['accel_kmh_per_s'] = 12.0
                        print("[Ego] x=-45 trigger: slowing to 30km/h.")
                    elif stage == 'slow_30' and loc.x >= -25.0:
                        v_data['stage'] = 'brake_to_stop'
                        v_data['desired_speed'] = 0.0
                        v_data['accel_kmh_per_s'] = 35.0
                        print("[Ego] x=-25 trigger: braking to 0km/h.")
                    elif stage == 'brake_to_stop' and speed <= 1.0:
                        v_data['stage'] = 'waiting'
                        v_data['stop_start_time'] = sim_time
                        v_data['desired_speed'] = 0.0
                        print("[Ego] stopped; waiting 6s.")
                    elif (stage == 'waiting' and v_data['stop_start_time'] is not None and
                          sim_time - v_data['stop_start_time'] >= 6.0):
                        v_data['stage'] = 'resume'
                        v_data['desired_speed'] = 60.0
                        v_data['accel_kmh_per_s'] = 15.0
                        print("[Ego] wait complete: resuming 60km/h.")

                    v_data['command_speed'] = update_speed_towards(
                        v_data['command_speed'],
                        v_data['desired_speed'],
                        v_data['accel_kmh_per_s'],
                        dt
                    )
                    v_data['target_speed'] = v_data['command_speed']

                if target_loc is not None:
                    throttle_out = v_data['lon_pid'].run_step(v_data['target_speed'], speed)
                    steer_out = v_data['lat_pid'].run_step(target_loc, tf)

                    control = carla.VehicleControl()
                    control.steer = steer_out
                    if throttle_out >= 0.0:
                        control.throttle = throttle_out
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = abs(throttle_out)

                    vehicle.apply_control(control)

            # Keep roughly real-time at 20 Hz.
            compute_time = time.time() - start_time
            if compute_time < 0.05:
                time.sleep(0.05 - compute_time)

    except KeyboardInterrupt:
        print("\nUser interrupted simulation.")
    except Exception as e:
        print(f"\nScenario error: {e}")
    finally:
        print("\nCleaning scene and restoring Carla settings...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        destroy_scene_actors(actor_list)
        print("Cleanup complete.")


if __name__ == '__main__':
    main()
