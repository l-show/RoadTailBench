import math


def control_to_dict(control):
    if control is None:
        return {}
    return {
        "throttle": float(getattr(control, "throttle", 0.0)),
        "steer": float(getattr(control, "steer", 0.0)),
        "brake": float(getattr(control, "brake", 0.0)),
        "hand_brake": bool(getattr(control, "hand_brake", False)),
        "reverse": bool(getattr(control, "reverse", False)),
    }


def vector_to_list(v):
    return [float(v.x), float(v.y), float(v.z)]


def rotation_to_list(r):
    return [float(r.roll), float(r.pitch), float(r.yaw)]


def actor_to_record(actor):
    transform = actor.get_transform()
    vel = actor.get_velocity()
    acc = actor.get_acceleration()
    speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
    record = {
        "id": int(actor.id),
        "type_id": actor.type_id,
        "role_name": actor.attributes.get("role_name", ""),
        "location": vector_to_list(transform.location),
        "rotation": rotation_to_list(transform.rotation),
        "velocity": vector_to_list(vel),
        "acceleration": vector_to_list(acc),
        "speed_mps": float(speed),
    }
    try:
        record["control"] = control_to_dict(actor.get_control())
    except RuntimeError:
        pass
    return record


def metadata_location(data):
    if not data:
        return None
    loc = data.get("location", data) if isinstance(data, dict) else data
    try:
        if isinstance(loc, dict):
            return (float(loc.get("x", 0.0)), float(loc.get("y", 0.0)), float(loc.get("z", 0.5)))
        return (float(loc[0]), float(loc[1]), float(loc[2] if len(loc) > 2 else 0.5))
    except (TypeError, ValueError, IndexError):
        return None


def dict_to_transform(carla, data):
    loc = data.get("location", data) if isinstance(data, dict) else data
    rot = data.get("rotation", {}) if isinstance(data, dict) else {}
    if isinstance(loc, dict):
        x, y, z = loc.get("x", 0.0), loc.get("y", 0.0), loc.get("z", 0.5)
    else:
        x, y, z = loc[0], loc[1], loc[2] if len(loc) > 2 else 0.5
    return carla.Transform(
        carla.Location(x=float(x), y=float(y), z=float(z)),
        carla.Rotation(
            pitch=float(rot.get("pitch", 0.0)),
            yaw=float(rot.get("yaw", 0.0)),
            roll=float(rot.get("roll", 0.0)),
        ),
    )
