#!/usr/bin/env python3
"""Compare the passive detent plugin on a minimal hinge model."""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np


XML = """
<mujoco model="detent test">
  <compiler angle="radian"/>
  <size nuser_jnt="1"/>
  <extension>
    <plugin plugin="mjlab.passive.detent">
      <instance name="detent"/>
    </plugin>
  </extension>
  <option timestep="0.002"/>
  <worldbody>
    <body name="knob">
      <joint name="knob" type="hinge" axis="0 0 1" range="0 4" stiffness="2.5"
             damping="0.3" armature="0.01" user="5"/>
      <plugin instance="detent"/>
      <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.01" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


def run(args: argparse.Namespace) -> dict[str, object]:
    mujoco.mj_loadPluginLibrary(str(Path(args.plugin).resolve()))
    with tempfile.NamedTemporaryFile("w", suffix=".xml") as xml:
        xml.write(XML)
        xml.flush()
        model = mujoco.MjModel.from_xml_path(xml.name)

    data = mujoco.MjData(model)
    rows = []
    for q in args.q:
        data.qpos[0] = q
        data.qvel[0] = args.qvel
        mujoco.mj_forward(model, data)
        rows.append({
            "qpos": q,
            "qvel": args.qvel,
            "qfrc_passive": float(data.qfrc_passive[0]),
            "qfrc_spring": float(data.qfrc_spring[0]),
            "qfrc_damper": float(data.qfrc_damper[0]),
        })

    return {
        "plugin": str(Path(args.plugin).resolve()),
        "mujoco_version": mujoco.__version__,
        "mj_version": int(mujoco.mj_version()),
        "nplugin": int(model.nplugin),
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin", required=True)
    parser.add_argument("--q", type=float, nargs="*", default=[-0.2, 0.1, 0.49, 0.51, 1.49, 2.51, 3.9, 4.2])
    parser.add_argument("--qvel", type=float, default=0.4)
    args = parser.parse_args()
    print(json.dumps(run(args), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
