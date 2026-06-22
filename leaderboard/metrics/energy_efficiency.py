from .base import BaseMetric, MetricResult
from ..core.extractors import ego, speed_mps
from ..core.geometry import clamp, distance2


def _location(entity):
    loc = entity.get("location") or [0.0, 0.0, 0.0]
    vals = list(loc) + [0.0, 0.0, 0.0]
    return float(vals[0]), float(vals[1]), float(vals[2])


class EnergyEfficiencyMetric(BaseMetric):
    name = "energy_efficiency"

    def compute(self, frames, config, context=None):
        if len(frames) < 2:
            return MetricResult.make(self.name, 1.0, {"reason": "too_few_frames"})
        mass = float(config.get("ego_mass_kg", 1600.0))
        drag_cd = float(config.get("ego_drag_coefficient", 0.29))
        frontal_area = float(config.get("ego_frontal_area_m2", 2.2))
        rolling = float(config.get("ego_rolling_resistance", 0.012))
        efficiency = float(config.get("ego_drivetrain_efficiency", 0.85))
        regen_efficiency = float(config.get("ego_regen_efficiency", 0.45))
        air_density = float(config.get("air_density_kg_m3", 1.225))
        baseline_kwh_per_100km = float(config.get("baseline_energy_kwh_per_100km", 18.0))

        traction_energy_j = 0.0
        regen_energy_j = 0.0
        distance_m = 0.0
        prev = ego(frames[0])
        prev_t = float(frames[0].get("time", 0.0))
        prev_v = speed_mps(prev)
        prev_loc = _location(prev)
        for frame in frames[1:]:
            cur = ego(frame)
            t = float(frame.get("time", prev_t))
            dt = max(t - prev_t, 1e-3)
            v = speed_mps(cur)
            avg_v = 0.5 * (prev_v + v)
            accel = (v - prev_v) / dt
            loc = _location(cur)
            ds_xy = distance2(prev_loc[:2], loc[:2])
            ds = ds_xy if ds_xy > 1e-3 else max(avg_v * dt, 1e-3)
            dz = loc[2] - prev_loc[2]
            grade_sin = clamp(dz / ds, -0.25, 0.25)
            force_roll = mass * 9.81 * rolling
            force_drag = 0.5 * air_density * drag_cd * frontal_area * avg_v * avg_v
            force_grade = mass * 9.81 * grade_sin
            force_accel = mass * accel
            wheel_power = (force_roll + force_drag + force_grade + force_accel) * avg_v
            if wheel_power >= 0.0:
                traction_energy_j += wheel_power * dt / max(efficiency, 0.1)
            else:
                regen_energy_j += -wheel_power * dt * max(0.0, min(regen_efficiency, 1.0))
            distance_m += ds
            prev_t, prev_v, prev_loc = t, v, loc

        net_energy_j = max(0.0, traction_energy_j - regen_energy_j)
        energy_kwh = net_energy_j / 3_600_000.0
        traction_kwh = traction_energy_j / 3_600_000.0
        regen_kwh = regen_energy_j / 3_600_000.0
        per_100 = energy_kwh / max(distance_m / 100_000.0, 1e-6)
        if per_100 <= baseline_kwh_per_100km:
            score = 1.0
        else:
            score = clamp(baseline_kwh_per_100km / max(per_100, 0.1))
        return MetricResult.make(self.name, score, {
            "mode": "longitudinal_dynamics_with_regen",
            "score_mode": "baseline_ratio",
            "estimated_energy_kwh": energy_kwh,
            "traction_energy_kwh": traction_kwh,
            "regenerated_energy_kwh": regen_kwh,
            "distance_m": distance_m,
            "energy_per_100km_kwh": per_100,
            "baseline_energy_kwh_per_100km": baseline_kwh_per_100km,
            "ego_mass_kg": mass,
            "ego_drag_coefficient": drag_cd,
            "ego_frontal_area_m2": frontal_area,
            "ego_rolling_resistance": rolling,
            "ego_drivetrain_efficiency": efficiency,
            "ego_regen_efficiency": regen_efficiency,
        })
