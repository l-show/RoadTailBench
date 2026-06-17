from .geometry import norm2, vec2, vec3


def ego(frame):
    return frame.get("ego", {})


def location_xy(entity):
    return vec2(entity.get("location", [0.0, 0.0, 0.0]))


def velocity_xy(entity):
    return vec2(entity.get("velocity", [0.0, 0.0, 0.0]))


def acceleration_xy(entity):
    return vec2(entity.get("acceleration", [0.0, 0.0, 0.0]))


def speed_mps(entity):
    if "speed_mps" in entity:
        return float(entity["speed_mps"])
    return norm2(velocity_xy(entity))


def yaw_deg(entity):
    return vec3(entity.get("rotation", [0.0, 0.0, 0.0]))[2]


def control(entity):
    return entity.get("control", {})
