#!/usr/bin/env python3
"""Run a minimal screw-thread behavior trace for one mjlab.sdf.thread plugin.

The original and reimplemented libraries register the same MuJoCo plugin name,
so compare them by running this script in separate Python processes.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]


def quat_to_yaw(quat: np.ndarray) -> float:
    """Return z-axis yaw from a MuJoCo wxyz quaternion."""
    w, x, y, z = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def contact_metrics(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float | int]:
    nut_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "nut")
    bolt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "bolt")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "body")
    nut_bolt = 0
    nut_body = 0
    normal_force = 0.0
    for i in range(data.ncon):
        contact = data.contact[i]
        pair = {contact.geom1, contact.geom2}
        if pair == {nut_id, bolt_id}:
            nut_bolt += 1
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, wrench)
            normal_force += float(wrench[0])
        elif pair == {nut_id, body_id}:
            nut_body += 1
    return {
        "contacts": int(data.ncon),
        "nut_bolt_contacts": nut_bolt,
        "nut_body_contacts": nut_body,
        "nut_bolt_normal_force": normal_force,
    }


def sample_state(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    qpos = np.array(data.qpos[:7], dtype=float)
    return {
        "time": float(data.time),
        "cap_pos": qpos[:3].tolist(),
        "cap_quat": qpos[3:7].tolist(),
        "cap_yaw": quat_to_yaw(qpos[3:7]),
        "qvel": np.array(data.qvel[:6], dtype=float).tolist(),
        **contact_metrics(model, data),
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    plugin_path = Path(args.plugin).resolve()
    xml_path = (ROOT / args.xml).resolve()
    mujoco.mj_loadPluginLibrary(str(plugin_path))
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    if model.nu != 1:
        raise RuntimeError(f"expected one actuator in {xml_path}, got {model.nu}")

    trace = [sample_state(model, data)]
    for step in range(1, args.steps + 1):
        data.ctrl[0] = args.ctrl
        mujoco.mj_step(model, data)
        if step % args.sample_every == 0 or step == args.steps:
            trace.append(sample_state(model, data))

    start = trace[0]
    end = trace[-1]
    return {
        "plugin": str(plugin_path),
        "xml": str(xml_path),
        "mujoco_version": mujoco.__version__,
        "mj_version": int(mujoco.mj_version()),
        "steps": args.steps,
        "ctrl": args.ctrl,
        "sample_every": args.sample_every,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nplugin": int(model.nplugin),
        "start": start,
        "end": end,
        "delta_pos": (np.array(end["cap_pos"]) - np.array(start["cap_pos"])).tolist(),
        "delta_yaw": float(end["cap_yaw"] - start["cap_yaw"]),
        "max_contacts": max(int(row["contacts"]) for row in trace),
        "max_nut_bolt_contacts": max(int(row["nut_bolt_contacts"]) for row in trace),
        "max_nut_bolt_normal_force": max(float(row["nut_bolt_normal_force"]) for row in trace),
        "trace": trace if args.full_trace else [],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin", required=True, help="Path to the plugin library to load.")
    parser.add_argument("--xml", default="autobio/model/scene/screw_test.xml")
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument("--ctrl", type=float, default=1.0)
    parser.add_argument("--sample-every", type=int, default=100)
    parser.add_argument("--full-trace", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run(args), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
