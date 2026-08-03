import carla
import time
import math
import numpy as np
import random

# ==========================================
# 1. Trajectory data
# ==========================================
RAW_MOTO_DATA = [(-17.142, -3.011, -21.719), (-16.935, -3.094, -21.929), (-16.46, -3.289, -22.635),
    (-15.985, -3.487, -22.635), (-15.954, -3.5, -22.635), (-15.64, -3.631, -22.635),
    (-15.179, -3.823, -22.634), (-14.702, -4.021, -22.281), (-14.223, -4.216, -22.071),
    (-13.752, -4.406, -21.861), (-13.272, -4.599, -21.861), (-12.793, -4.791, -21.791),
    (-12.32, -4.979, -21.648), (-11.848, -5.166, -21.578), (-11.367, -5.356, -21.508),
    (-10.902, -5.538, -21.298), (-10.42, -5.724, -21.016), (-9.945, -5.906, -20.805),
    (-9.463, -6.088, -20.735), (-8.988, -6.267, -20.383), (-8.511, -6.442, -19.815),
    (-8.042, -6.611, -19.885), (-7.557, -6.79, -20.803), (-7.09, -6.967, -20.803),
    (-6.624, -7.147, -21.72), (-6.165, -7.338, -23.216), (-5.694, -7.549, -25.551),
    (-5.229, -7.775, -26.755), (-4.787, -8.009, -28.902), (-4.35, -8.251, -28.762),
    (-3.893, -8.493, -27.344), (-3.438, -8.72, -25.041), (-2.984, -8.93, -24.546),
    (-2.51, -9.135, -20.758), (-2.02, -9.298, -16.861), (-1.541, -9.44, -16.367),
    (-1.045, -9.584, -16.157), (-0.558, -9.732, -18.961), (-0.078, -9.921, -22.542),
    (0.394, -10.132, -26.419), (0.829, -10.393, -34.416), (1.239, -10.707, -40.919),
    (1.618, -11.058, -44.726), (1.963, -11.442, -51.379), (2.268, -11.848, -55.377),
    (2.546, -12.284, -58.469), (2.808, -12.729, -60.466), (3.043, -13.17, -64.505),
    (3.254, -13.622, -65.712), (3.455, -14.098, -68.505), (3.632, -14.566, -69.647),
    (3.806, -15.035, -69.647), (3.904, -15.3, -69.647), (4.026, -15.672, -73.265),
    (4.158, -16.154, -75.417), (4.286, -16.654, -75.697), (4.41, -17.138, -75.697),
    (4.532, -17.623, -76.048), (4.651, -18.108, -76.33), (4.768, -18.612, -77.112),
    (4.876, -19.1, -78.723), (4.973, -19.59, -79.369), (5.062, -20.082, -79.796),
    (5.152, -20.583, -79.796), (5.242, -21.083, -80.808), (5.312, -21.578, -82.215),
    (5.379, -22.09, -82.779), (5.432, -22.604, -84.427), (5.482, -23.11, -84.427),
    (5.53, -23.608, -84.427), (5.58, -24.122, -84.427), (5.629, -24.636, -84.847),
    (5.673, -25.143, -85.202), (5.715, -25.658, -85.692), (5.746, -26.157, -86.816),
    (5.771, -26.656, -87.378), (5.785, -27.164, -89.728), (5.794, -28.787, -89.658),
    (5.63, -36.396, -91.286), (5.527, -43.891, -90.924), (5.446, -51.394, -90.149),
    (5.47, -59.094, -89.589), (5.493, -66.841, -89.732), (5.494, -74.339, -91.008),
    (5.035, -82.071, -96.225), (4.131, -89.64, -96.872), (3.433, -97.104, -92.782),
    (3.531, -104.847, -85.773), (5.225, -115.034, -89.368), (5.252, -117.031, -89.228), (5.337, -125.026, -89.578), (5.396, -133.025, -89.578),
    (5.484, -142.023, -89.368), (5.565, -150.021, -89.438), (5.635, -158.018, -89.578), (5.682, -166.016, -89.857),
    (5.687, -174.015, -89.997), (5.669, -182.015, -90.207), (5.635, -190.014, -90.277), (5.592, -199.014, -90.277),
    (5.553, -207.014, -90.277), (5.514, -215.014, -90.277), (5.495, -219.014, -90.277), (5.495, -219.014, -90.277),
    (5.495, -219.014, -90.277), (5.495, -219.014, -90.277)
]

RAW_JEEP_DATA = [(6.222, 46.641, -89.447), (6.231, 45.371, -89.587), (6.236, 44.12, -90.082),
    (6.235, 42.87, -90.082), (6.233, 41.578, -90.082), (6.231, 40.308, -90.082),
    (6.229, 39.037, -90.082), (6.227, 37.787, -90.082), (6.226, 36.516, -90.082),
    (6.224, 35.224, -90.082), (6.222, 34.058, -90.082), (6.161, 33.081, -93.594),
    (6.076, 31.792, -93.876), (5.988, 30.503, -93.876), (5.916, 29.235, -92.006),
    (5.879, 27.944, -90.786), (5.891, 26.694, -88.57), (5.922, 25.445, -88.57),
    (5.958, 24.195, -88.288), (5.997, 22.904, -88.288), (6.034, 21.655, -88.358),
    (6.067, 20.364, -88.568), (6.097, 19.093, -88.708), (6.125, 17.843, -88.708),
    (6.154, 16.551, -88.708), (6.183, 15.301, -88.708), (6.211, 14.031, -88.708),
    (6.234, 12.781, -88.99), (6.257, 11.49, -88.99), (6.278, 10.24, -89.06),
    (6.291, 8.988, -89.918), (6.291, 7.697, -90.563), (6.279, 6.447, -90.563),
    (6.267, 5.176, -90.563), (6.255, 3.885, -90.493), (6.243, 2.593, -90.493),
    (6.237, 1.322, -90.21), (6.232, 0.03, -90.21), (6.227, -1.262, -90.21),
    (6.219, -2.553, -90.7), (6.2, -3.824, -90.91), (6.179, -5.115, -90.91),
    (6.159, -6.386, -90.91), (6.139, -7.657, -90.91), (6.119, -8.906, -90.91),
    (6.098, -10.156, -91.262), (6.063, -11.446, -91.755), (6.024, -12.695, -91.755),
    (5.985, -13.985, -91.755), (5.958, -14.859, -91.755), (5.911, -16.087, -91.747),
    (5.876, -17.378, -91.397), (5.862, -18.503, -90.542), (5.858, -18.878, -90.542),
    (5.832, -26.626, -90.049), (5.818, -34.375, -90.769), (5.771, -42.125, -90.064),
    (5.764, -49.749, -89.994), (5.764, -57.499, -89.994), (5.775, -65.248, -89.854),
    (5.81, -73.001, -89.714), (5.856, -80.501, -89.644), (5.904, -88.126, -89.644),
    (5.93, -95.876, -90.206), (5.88, -103.501, -90.556), (5.816, -110.893, -90.486),
    (5.751, -118.643, -90.486), (5.694, -126.143, -90.346), (5.681, -133.643, -89.994),
    (5.681, -141.268, -89.994), (5.682, -148.893, -89.994), (5.683, -156.394, -89.994),
    (5.684, -163.895, -89.994), (5.666, -171.395, -90.206), (5.64, -178.645, -90.206)
]

RAW_EGO_DATA = [(2.474, 35.603, -89.087), (2.474, 35.603, -89.087), (2.474, 35.603, -89.087), (2.474, 35.603, -89.087),
    (2.474, 35.603, -89.350), (2.515, 31.947, -89.482), (2.561, 26.872, -89.350), (2.641, 21.709, -89.087),
    (2.699, 18.126, -89.087), (2.707, 15.585, -90.612), (2.669, 13.044, -90.875), (2.627, 10.546, -91.534),
    (2.560, 8.051, -91.534), (2.492, 5.512, -91.534), (2.425, 3.012, -91.534), (2.390, 0.513, -89.968),
    (2.417, -2.028, -87.042), (2.746, -4.504, -79.120), (3.279, -6.945, -78.705), (3.477, -9.472, -93.124),
    (3.218, -12.041, -96.815), (2.992, -14.573, -93.970), (2.819, -17.067, -93.970), (2.658, -19.561, -93.307),
    (2.511, -22.099, -93.307), (2.362, -24.678, -93.307), (2.234, -27.174, -92.407), (2.129, -29.674, -92.267),
    (2.062, -32.215, -91.094), (2.048, -34.715, -90.040), (2.056, -37.257, -89.642), (2.086, -39.756, -88.846),
    (2.136, -42.256, -88.846), (2.226, -46.693, -88.846), (2.323, -51.692, -89.182), (2.313, -56.775, -90.835),
    (2.241, -61.774, -90.835), (2.168, -66.773, -90.835), (2.118, -71.856, -90.305), (2.091, -76.856, -90.305),
    (2.064, -81.939, -90.305), (2.037, -86.980, -90.305), (2.010, -92.147, -90.305), (1.983, -97.150, -90.305),
    (1.956, -102.234, -90.305), (1.929, -107.317, -90.305), (1.903, -112.317, -90.305), (1.903, -117.400, -89.111),
    (1.980, -122.400, -89.111), (2.059, -127.482, -89.111), (2.136, -132.482, -89.244), (2.160, -137.565, -90.040),
    (2.138, -142.565, -90.703), (2.062, -147.648, -91.233), (1.958, -152.645, -91.101), (1.897, -157.728, -89.774),
    (1.936, -162.733, -89.774), (1.954, -167.732, -89.907), (1.962, -172.732, -89.907), (1.953, -177.814, -90.172),
    (1.929, -182.897, -90.305), (1.902, -187.896, -90.305), (1.875, -192.979, -90.305), (1.870, -197.979, -89.774),
    (1.890, -202.978, -89.774), (1.910, -208.061, -89.774), (1.918, -210.145, -89.774), (1.918, -210.145, -89.774),
    (1.918, -210.145, -89.774), (1.918, -210.145, -89.774)
]

PEDESTRIAN_LOCATIONS = [
    carla.Location(x=9.287, y=1.477, z=1.0),
    carla.Location(x=10.566, y=-4.222, z=1.0),
    carla.Location(x=8.597, y=-10.224, z=1.0),
]

def clean_trajectory(raw_data):
    path = []
    if raw_data:
        path.append((raw_data[0][0], raw_data[0][1], 0.5, raw_data[0][2]))
        for i in range(1, len(raw_data)):
            if raw_data[i] != raw_data[i - 1]:
                path.append((raw_data[i][0], raw_data[i][1], 0.5, raw_data[i][2]))
    return path

def clamp(v, a, b):
    return max(a, min(b, v))

def get_transform(x, y, z, pitch=0.0, yaw=0.0, roll=0.0):
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
    )

MOTO_PATH = clean_trajectory(RAW_MOTO_DATA)
JEEP_PATH = clean_trajectory(RAW_JEEP_DATA)
EGO_PATH = clean_trajectory(RAW_EGO_DATA)

# ==========================================
# 2. PID trajectory controllers
# ==========================================
class PIDLongitudinalController:
    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.0, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

class PIDLateralController:
    def __init__(self, K_P=1.95, K_I=0.05, K_D=0.2, dt=0.05):
        self._k_p, self._k_i, self._k_d = K_P, K_I, K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_forward = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_forward.x, v_forward.y, 0.0])
        w_vec = np.array([waypoint[0] - v_begin.x, waypoint[1] - v_begin.y, 0.0])

        norm_w = np.linalg.norm(w_vec)
        if norm_w < 0.1:
            return 0.0

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / norm_w, -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._error_buffer.append(_dot)
        if len(self._error_buffer) >= 30:
            self._error_buffer.pop(0)
        _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >= 2 else 0.0
        _ie = sum(self._error_buffer) * self._dt
        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

def get_target_waypoint(vehicle_loc, path_points, lookahead_dist=4.0):
    min_dist = float('inf')
    closest_index = 0
    for i, p in enumerate(path_points):
        dist = math.sqrt((p[0] - vehicle_loc.x) ** 2 + (p[1] - vehicle_loc.y) ** 2)
        if dist < min_dist:
            min_dist, closest_index = dist, i

    target_index, current_dist = closest_index, 0.0
    for i in range(closest_index, len(path_points) - 1):
        p1, p2 = path_points[i], path_points[i + 1]
        d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        current_dist += d
        target_index = i + 1
        if current_dist > lookahead_dist:
            break
    return path_points[target_index]

def apply_pid_control(vehicle, pid_lon, pid_lat, target_speed, target_wp):
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    throttle_output = pid_lon.run_step(target_speed, speed)
    steer_output = pid_lat.run_step(target_wp, tf)

    control = carla.VehicleControl()
    control.steer = steer_output
    if throttle_output >= 0.0:
        control.throttle = throttle_output
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = abs(throttle_output)
    vehicle.apply_control(control)

def check_and_handle_out_of_bounds(vehicle, carla_map, name="Vehicle", threshold=4.0):
    if vehicle is None or not vehicle.is_alive:
        return True

    loc = vehicle.get_location()
    wp_exact = carla_map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving)
    wp_nearest = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)

    is_out = wp_exact is None
    if wp_nearest is None:
        is_out = True
    elif wp_nearest.transform.location.distance(loc) > threshold:
        is_out = True

    if is_out:
        print(f"[{name}] out of drivable road projection, destroyed.")
        vehicle.destroy()
        return True
    return False

# ==========================================
# 3. Main program
# ==========================================

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    weather = carla.WeatherParameters(
        cloudiness=10.0, precipitation=0.0, precipitation_deposits=0.0, wind_intensity=10.0,
        sun_azimuth_angle=85.0, sun_altitude_angle=14.0, fog_density=4.0, fog_distance=5.0,
        fog_falloff=0.0, wetness=0.0, scattering_intensity=0.5, mie_scattering_scale=0.21,
        rayleigh_scattering_scale=0.07, dust_storm=0.0,
    )
    world.set_weather(weather)
    bp_lib = world.get_blueprint_library()

    dt = 0.05
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        settings.max_substeps = 10
        world.apply_settings(settings)

        actor_list = []
        pids = {
            'moto': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'jeep': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
            'ego': {'lon': PIDLongitudinalController(dt=dt), 'lat': PIDLateralController(dt=dt)},
        }

        walker_bps = bp_lib.filter('walker.pedestrian.*')
        bp_walker = random.choice(walker_bps)

        initial_ped_loc = random.choice(PEDESTRIAN_LOCATIONS)
        trans_walker = get_transform(x=initial_ped_loc.x, y=initial_ped_loc.y, z=initial_ped_loc.z, yaw=0.0)
        walker = world.try_spawn_actor(bp_walker, trans_walker)
        if walker:
            actor_list.append(walker)
            print("Pedestrian spawned.")

        bp_moto = bp_lib.find('vehicle.harley-davidson.low_rider')
        if bp_moto.has_attribute('color'):
            bp_moto.set_attribute('color', '0,255,0')
        trans_moto = carla.Transform(carla.Location(x=MOTO_PATH[0][0], y=MOTO_PATH[0][1], z=0.5),
                                     carla.Rotation(yaw=MOTO_PATH[0][3]))
        moto = world.try_spawn_actor(bp_moto, trans_moto)
        if moto:
            actor_list.append(moto)

        bp_jeep = bp_lib.find('vehicle.jeep.wrangler_rubicon')
        if bp_jeep.has_attribute('color'):
            bp_jeep.set_attribute('color', '0,0,0')
        trans_jeep = carla.Transform(carla.Location(x=JEEP_PATH[0][0], y=JEEP_PATH[0][1], z=0.5),
                                     carla.Rotation(yaw=JEEP_PATH[0][3]))
        jeep = world.try_spawn_actor(bp_jeep, trans_jeep)
        if jeep:
            actor_list.append(jeep)

        bp_ego = bp_lib.find('vehicle.citroen.c3')
        if bp_ego.has_attribute('color'):
            bp_ego.set_attribute('color', '255,255,0')
        if bp_ego.has_attribute('role_name'):
            pass

        loc_ego = carla.Location(x=EGO_PATH[0][0], y=EGO_PATH[0][1], z=0.5)
        wp_ego = carla_map.get_waypoint(loc_ego, project_to_road=True, lane_type=carla.LaneType.Driving)
        trans_ego = carla.Transform(wp_ego.transform.location + carla.Location(z=0.5),
                                    carla.Rotation(yaw=EGO_PATH[0][3]))
        ego_vehicle = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed
        if ego_vehicle:
            print("Ego configured: role_name=ego, target speed 60 km/h, PID trajectory control.")

        print("Scenario loaded, waiting for physics to settle...")
        for _ in range(20):
            world.tick()
        print("Simulation started.")

        PED_SPEED_MPS = 5.0 / 3.6
        PED_ARRIVAL_DIST = 0.8
        current_ped_target = random.choice([loc for loc in PEDESTRIAN_LOCATIONS if loc != initial_ped_loc])
        ped_last_pos = walker.get_location() if walker else None
        ped_last_progress_time = time.time()

        moto_target_speed = 20.0
        moto_accel_triggered = False

        jeep_state = 'NORMAL'
        jeep_target_speed = 80.0
        jeep_stop_timestamp = 0.0
        ego_target_speed = 60.0

        while True:
            start_time = time.time()
            world.tick()
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            if walker:
                ped_loc = walker.get_location()
                dx = current_ped_target.x - ped_loc.x
                dy = current_ped_target.y - ped_loc.y
                dist = math.hypot(dx, dy)

                if dist <= PED_ARRIVAL_DIST:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = time.time()
                    continue

                ctrl = carla.WalkerControl()
                if dist > 1e-6:
                    nx, ny = dx / dist, dy / dist
                    ctrl.direction = carla.Vector3D(nx, ny, 0.0)
                    slow_radius = 2.0
                    if dist < slow_radius:
                        desired_speed = PED_SPEED_MPS * (dist / slow_radius)
                        ctrl.speed = clamp(desired_speed, 0.25, PED_SPEED_MPS)
                    else:
                        ctrl.speed = PED_SPEED_MPS
                else:
                    ctrl.speed = 0.0
                    ctrl.direction = carla.Vector3D(0.0, 0.0, 0.0)

                walker.apply_control(ctrl)

                current_time = time.time()
                if math.hypot(ped_loc.x - ped_last_pos.x, ped_loc.y - ped_last_pos.y) > 0.05:
                    ped_last_progress_time = current_time
                    ped_last_pos = ped_loc

                if current_time - ped_last_progress_time > 8.0:
                    available_targets = [loc for loc in PEDESTRIAN_LOCATIONS if loc != current_ped_target]
                    current_ped_target = random.choice(available_targets)
                    ped_last_progress_time = current_time

            if moto and moto.is_alive:
                loc = moto.get_location()
                if not moto_accel_triggered and loc.x >= 0.0:
                    moto_accel_triggered = True
                    print(f"[{sim_time:.1f}] Moto passed x=0, accelerating.")
                if moto_accel_triggered and moto_target_speed < 120.0:
                    moto_target_speed = min(120.0, moto_target_speed + 2.5)
                dist_to_end = math.sqrt((loc.x - MOTO_PATH[-1][0]) ** 2 + (loc.y - MOTO_PATH[-1][1]) ** 2)
                if dist_to_end < 2.0:
                    moto.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                else:
                    target_wp = get_target_waypoint(loc, MOTO_PATH, lookahead_dist=6.0)
                    apply_pid_control(moto, pids['moto']['lon'], pids['moto']['lat'], moto_target_speed, target_wp)

            if jeep and jeep.is_alive:
                if check_and_handle_out_of_bounds(jeep, carla_map, "Jeep"):
                    jeep = None
                else:
                    loc = jeep.get_location()
                    if jeep_state == 'NORMAL' and loc.y <= 35.0:
                        jeep_state = 'DECELERATING'
                        print(f"[{sim_time:.1f}] Jeep starts decelerating.")
                    elif jeep_state == 'DECELERATING':
                        jeep_target_speed = max(10.0, jeep_target_speed - 1.0)
                        if loc.y <= -15.0:
                            jeep_state = 'STOPPED'
                            jeep_stop_timestamp = sim_time
                            print(f"[{sim_time:.1f}] Jeep stopped.")
                    elif jeep_state == 'STOPPED':
                        jeep.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                        if sim_time - jeep_stop_timestamp >= 3.0:
                            jeep_state = 'RESUMING'
                            print(f"[{sim_time:.1f}] Jeep resumes.")
                    elif jeep_state == 'RESUMING':
                        jeep_target_speed = min(80.0, jeep_target_speed + 0.5)

                    if jeep_state != 'STOPPED':
                        target_wp = get_target_waypoint(loc, JEEP_PATH, lookahead_dist=5.0)
                        apply_pid_control(jeep, pids['jeep']['lon'], pids['jeep']['lat'], jeep_target_speed, target_wp)

            compute_time = time.time() - start_time
            if compute_time < dt:
                time.sleep(dt - compute_time)

    except KeyboardInterrupt:
        print("Simulation stopped.")
    finally:
        print("Cleaning up and restoring async settings...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if actor_list:
            actors_to_destroy = [a for a in actor_list if a is not None and a.is_alive]
            client.apply_batch([carla.command.DestroyActor(a) for a in actors_to_destroy])
        print("Cleanup done.")

if __name__ == '__main__':
    main()
