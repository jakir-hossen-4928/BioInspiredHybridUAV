"""Run an isolated Webots drop, eight-step crawl and static force checks."""

import csv
import gzip
import json
import math
import os
from pathlib import Path
from statistics import mean
import sys


FEET = ("FL", "FR", "RL", "RR")
GAIT = ("FL", "RR", "FR", "RL") * 2
WALK_START = 2.0
STEP_TIME = 1.2
PERIOD_MS = 4
END_TIME = 17
MASS_KG = 3.9
PAD_MASS_KG = 0.025
GRAVITY = 9.81
SUPPORTED_WEIGHT = (MASS_KG - len(FEET) * PAD_MASS_KG) * GRAVITY
HORIZONTAL_LOADS = ((13.0, 14.8, 6.0, 0.0), (15.0, 16.8, 0.0, 6.0))
FIELDS = ("x_n", "y_n", "z_n", "world_x_n", "world_y_n", "world_z_n", "clearance_m")


def measure(samples):
    """Apply the same acceptance checks to a live run or a saved CSV."""
    if not all(math.isfinite(value) for row in samples for key, value in row.items()
               if key != "swing"):
        raise ValueError("All measurements must be finite")

    def window(start, stop):
        return [row for row in samples if start <= row["time_s"] < stop]

    def support(row):
        return sum(row[f"{foot}_world_z_n"] for foot in FEET)

    def magnitude(row, foot):
        return math.hypot(*(row[f"{foot}_{axis}_n"] for axis in "xyz"))

    airborne = window(0.02, 0.08)
    touchdown = window(0.08, 0.40)
    standing = window(1.0, 1.9)
    walking = window(WALK_START, WALK_START + STEP_TIME * len(GAIT))
    settled = window(12.1, 12.9)
    period_s = PERIOD_MS / 1000
    expected_samples = round(END_TIME / period_s)
    checks = {}

    def check(name, actual, lower, upper):
        checks[name] = {"value": actual, "min": lower, "max": upper,
                        "pass": lower <= actual and (upper is None or actual <= upper)}

    check("samples", len(samples), expected_samples, expected_samples)
    check("max_sample_period_error_s",
          max(abs(b["time_s"] - a["time_s"] - period_s) for a, b in zip(samples, samples[1:])),
          0, 1e-8)
    check("airborne_max_foot_force_n",
          max(magnitude(row, foot) for row in airborne for foot in FEET), 0, 0.1)
    check("airborne_min_pad_clearance_m",
          min(row[f"{foot}_clearance_m"] for row in airborne for foot in FEET), 0.020, 0.080)
    contact = next(index for index, row in enumerate(touchdown)
                   if min(row[f"{foot}_clearance_m"] for foot in FEET) <= 0)
    first_contact = touchdown[contact]
    last_impact_sample = touchdown[-1]
    measured_impulse = period_s * sum(
        support(row) - SUPPORTED_WEIGHT for row in touchdown[contact + 1:])
    # During the symmetric drop, chassis vertical speed approximates the supported mass speed.
    expected_impulse = (SUPPORTED_WEIGHT / GRAVITY) * (
        last_impact_sample["body_vz_m_s"] - first_contact["body_vz_m_s"])
    peak = max(touchdown, key=support)
    check("touchdown_peak_total_n", support(peak), SUPPORTED_WEIGHT * 1.1, None)
    check("touchdown_impulse_n_s", measured_impulse, expected_impulse - 0.05,
          expected_impulse + 0.05)
    check("standing_total_mean_n", mean(map(support, standing)),
          SUPPORTED_WEIGHT * 0.95, SUPPORTED_WEIGHT * 1.05)
    check("standing_max_horizontal_resultant_n",
          max(math.hypot(*(sum(row[f"{foot}_world_{axis}_n"] for foot in FEET)
                           for axis in "xy")) for row in standing), 0, 0.5)
    check("walking_forward_displacement_m",
          settled[-1]["body_x_m"] - standing[-1]["body_x_m"], 0.08, 0.25)
    check("walking_min_body_height_m", min(row["body_z_m"] for row in walking), 0.24, 0.33)
    check("walking_max_body_tilt_deg", max(row["body_tilt_deg"] for row in walking), 0, 15)
    check("final_standing_total_mean_n", mean(map(support, settled)),
          SUPPORTED_WEIGHT * 0.95, SUPPORTED_WEIGHT * 1.05)
    for foot in FEET:
        check(f"{foot}_standing_mean_n", mean(row[f"{foot}_world_z_n"] for row in standing),
              SUPPORTED_WEIGHT / 4 * 0.90, SUPPORTED_WEIGHT / 4 * 1.10)
        check(f"{foot}_touchdown_peak_n", max(row[f"{foot}_world_z_n"] for row in touchdown),
              SUPPORTED_WEIGHT / 4, None)
    for index, foot in enumerate(GAIT):
        start = WALK_START + STEP_TIME * index
        lift = window(start + 0.32, start + 0.68)
        landed = window(start + 1.00, start + 1.16)
        name = f"step_{index + 1}_{foot}"
        check(f"{name}_min_clearance_m", min(row[f"{foot}_clearance_m"] for row in lift),
              0.025, 0.10)
        check(f"{name}_max_swing_force_n", max(magnitude(row, foot) for row in lift), 0, 0.3)
        check(f"{name}_mean_stance_support_n", mean(map(support, lift)),
              SUPPORTED_WEIGHT * 0.9, SUPPORTED_WEIGHT * 1.1)
        check(f"{name}_reloaded_mean_n", mean(row[f"{foot}_world_z_n"] for row in landed),
              2, SUPPORTED_WEIGHT)
    for load_axis, (start, stop, load_x, load_y) in zip("xy", HORIZONTAL_LOADS):
        loaded = window(start + 0.8, stop)
        for measurement_axis, load in zip("xy", (load_x, load_y)):
            resultant = mean(sum(row[f"{foot}_world_{measurement_axis}_n"] for foot in FEET)
                             for row in loaded)
            check(f"{load_axis}_load_mean_{measurement_axis}_reaction_n", resultant,
                  -load - 0.3, -load + 0.3)
        check(f"{load_axis}_load_mean_vertical_support_n", mean(map(support, loaded)),
              SUPPORTED_WEIGHT * 0.95, SUPPORTED_WEIGHT * 1.05)
    return {"passed": all(item["pass"] for item in checks.values()),
            "rig_weight_n": MASS_KG * GRAVITY,
            "expected_sensor_support_n": SUPPORTED_WEIGHT,
            "first_pad_contact_s": first_contact["time_s"],
            "peak_touchdown_time_s": peak["time_s"],
            "touchdown_momentum_change_n_s": expected_impulse, "checks": checks}


def read_samples(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8", newline="") as stream:
        return [{key: value if key == "swing" else float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def run():
    from controller import Supervisor

    robot = Supervisor()
    period = int(robot.getBasicTimeStep())
    if period != PERIOD_MS:
        raise ValueError(f"Expected a {PERIOD_MS} ms physics step, got {period}")
    body = robot.getSelf()
    pads = {foot: robot.getFromDef(foot).getFromProtoDef("PAD") for foot in FEET}
    sensors = {foot: robot.getDevice(f"{foot} force") for foot in FEET}
    strides = {foot: robot.getDevice(f"{foot} stride") for foot in FEET}
    lifts = {foot: robot.getDevice(f"{foot} lift") for foot in FEET}
    for foot in FEET:
        sensors[foot].enable(period)
        strides[foot].setPosition(0)
        lifts[foot].setPosition(0)
    offsets = dict.fromkeys(FEET, 0.0)
    stage = -1
    step_ms = round(STEP_TIME * 1000)
    samples = []
    output = Path(os.environ.get("FOOT_FORCE_OUTPUT", Path.home() / "webots-foot-force"))
    output.mkdir(parents=True, exist_ok=True)

    while robot.step(period) != -1:
        now = round(robot.getTime(), 6)
        x, y, z = body.getPosition()
        tilt = math.degrees(math.acos(max(-1, min(1, body.getOrientation()[8]))))
        row = {"time_s": now, "swing": "", "body_x_m": x, "body_y_m": y,
               "body_z_m": z, "body_vz_m_s": body.getVelocity()[2], "body_tilt_deg": tilt}
        elapsed_ms = round((now - WALK_START) * 1000)
        index, offset_ms = divmod(elapsed_ms, step_ms)
        if 0 <= index < len(GAIT):
            phase = offset_ms / step_ms
            swing = GAIT[index]
            row["swing"] = swing
            if index != stage:
                if stage >= 0:
                    for foot in FEET:
                        offsets[foot] += 0.06 if foot == GAIT[stage] else -0.02
                    lifts[GAIT[stage]].setPosition(0)
                stage = index
            swing_progress = max(0, min(1, (phase - 0.18) / 0.40))
            swing_progress = swing_progress ** 2 * (3 - 2 * swing_progress)
            lift = max(0, min(1, phase / 0.23, (0.71 - phase) / 0.18))
            lifts[swing].setPosition(0.065 * lift)
            for foot in FEET:
                stride = 0.06 * swing_progress if foot == swing else -0.02 * phase
                strides[foot].setPosition(offsets[foot] + stride)
        for start, stop, load_x, load_y in HORIZONTAL_LOADS:
            if start <= now < stop:
                body.addForce([load_x, load_y, 0], False)
        for foot in FEET:
            local = sensors[foot].getValues()[:3]
            rotation = pads[foot].getOrientation()
            world = [sum(a * b for a, b in zip(rotation[start:start + 3], local))
                     for start in (0, 3, 6)]
            # Lowest point of the pad's oriented collision box, relative to the floor.
            clearance = pads[foot].getPosition()[2] - sum(
                abs(axis) * half for axis, half in zip(rotation[6:9], (0.10, 0.08, 0.015)))
            row.update({f"{foot}_{field}": value
                        for field, value in zip(FIELDS, [*local, *world, clearance])})
        samples.append(row)
        if now >= END_TIME:
            break

    else:
        return

    with (output / "foot_force.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    results = measure(samples)
    (output / "foot_force_summary.json").write_text(json.dumps(results, indent=2) + "\n",
                                                    encoding="utf-8")
    print(f"Foot-force checks {'PASSED' if results['passed'] else 'FAILED'}. Results: {output}",
          flush=True)
    robot.simulationQuit(0 if results["passed"] else 1)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        results = measure(read_samples(Path(sys.argv[1])))
        print(json.dumps(results, indent=2))
        sys.exit(0 if results["passed"] else 1)
    else:
        run()
