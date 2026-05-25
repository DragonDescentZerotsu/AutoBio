# mjlab plugin compatibility reimplementation

This is a source reimplementation of selected MuJoCo plugins from AutoBio's bundled
`libmjlab.so.3.3.0` for newer MuJoCo versions. It is intended for compatibility testing against
the bundled binary, not yet as a fully validated physical replacement.

Currently implemented original plugin names:

- `mjlab.sdf.thread`
- `mjlab.passive.detent`

The original binary also exports `mjlab.sdf.grid`; the current repository does not reference that
plugin from any XML scene, so it has only been compile-tested with the original binary and is not
reimplemented here.

The eccentric mechanism described by the paper is represented in this repository with native MJCF
joints/equality constraints, for example the vortex mixer's `platform/pivot`, `platform/orbit`, and
`polycoef="0 -1"` equality. The quasi-static liquid model is Python-level code in `liquid.py` plus
render/evaluator helpers, not a compiled MuJoCo plugin in `libmjlab.so.3.3.0`.

## Thread

The implementation follows the bounded circular-helix SDF approximation described in
`AutoBio.pdf`, appendix A.2, with the axis/padding conventions inferred from the bundled binary.
It registers the original plugin name and accepts the same XML attributes:

- `pitch`: axial thread pitch per full revolution
- `radius`: helix radius
- `low`: lower bound in thread revolutions
- `high`: upper bound in thread revolutions
- `gauge`: thread tube radius, subtracted from the centerline distance

## Detent

`mjlab.passive.detent` adds passive snap-to-position behavior for limited one-DOF hinge/slide
joints. The joint range is divided into `user[0]` discrete steps. MuJoCo's native spring/damper
already contributes `-stiffness * qpos - damping * qvel`; the plugin contributes
`+stiffness * nearest_step_position`, so the total passive torque/force is equivalent to a spring
pulling toward the nearest detent.

Run the isolated detent regression in separate processes:

```bash
LD_LIBRARY_PATH=/data/tianang/miniconda/envs/autobio/lib/python3.11/site-packages/mujoco \
  /data/tianang/miniconda/envs/autobio/bin/python \
  autobio/plugins/mjlab_sdf_thread/compare_detent.py \
  --plugin autobio/libmjlab.so.3.3.0

LD_LIBRARY_PATH=/data/tianang/miniconda/envs/autobio/lib/python3.11/site-packages/mujoco \
  /data/tianang/miniconda/envs/autobio/bin/python \
  autobio/plugins/mjlab_sdf_thread/compare_detent.py \
  --plugin autobio/libmjlab_thread.so.3.3.0
```

In the MuJoCo 3.3.0 environment, the reimplementation matches the bundled binary exactly for the
tested `qfrc_passive`, `qfrc_spring`, and `qfrc_damper` rows.

## Build

Build for the current `aero_sim` MuJoCo wheel from the repository root:

```bash
./autobio/plugins/mjlab_sdf_thread/build.sh
```

The output is `autobio/libmjlab_thread.so.<mujoco-version>`.

`build.sh` accepts these optional overrides:

- `PYTHON_BIN`: Python interpreter whose installed `mujoco` wheel is used for headers/libs
- `GRADIENT_EPS`: runtime SDF gradient finite-difference step, default `1e-8`
- `GRADIENT_Z_SCALE`: experimental multiplier for the z gradient component, default `1.0`

## Validation

Run the isolated screw-thread regression in separate processes because the original and
reimplemented libraries both register `mjlab.sdf.thread`:

```bash
LD_LIBRARY_PATH=/data/tianang/miniconda/envs/autobio/lib/python3.11/site-packages/mujoco \
  /data/tianang/miniconda/envs/autobio/bin/python \
  autobio/plugins/mjlab_sdf_thread/compare_screw_test.py \
  --plugin autobio/libmjlab.so.3.3.0 --steps 2000 --ctrl 1

LD_LIBRARY_PATH=/data/tianang/miniconda/envs/autobio/lib/python3.11/site-packages/mujoco \
  /data/tianang/miniconda/envs/autobio/bin/python \
  autobio/plugins/mjlab_sdf_thread/compare_screw_test.py \
  --plugin autobio/libmjlab_thread.so.3.3.0 --steps 2000 --ctrl 1
```

Current validation:

- In the `autobio` MuJoCo 3.3.0 environment, this reimplementation produces the same generated
  SDF mesh vertex/face counts, bounds, mesh positions, and mesh quaternions as the bundled binary
  for `model/scene/screw_test.xml`.
- In the same MuJoCo 3.3.0 screw test, the current default `GRADIENT_EPS=1e-8` behavior closely
  matches the bundled binary:
  - `ctrl=1`, 2000 steps: original yaw `0.5075`, dz `0.000172`, max nut-bolt normal force
    `1.7502`; reimplementation yaw `0.5075`, dz `0.000172`, force `1.7500`.
  - `ctrl=-1`, 2000 steps: original yaw `-1.2978`, dz `-0.000980`, force `7.6780`;
    reimplementation yaw `-1.2981`, dz `-0.000971`, force `7.3911`.
- In the `aero_sim` MuJoCo 3.8.1 environment, `mani_pipette`, `screw_loosen`, and
  `screw_tighten` compile/reset with this plugin loaded manually; `mani_pipette` also steps.
- `mani_vortex_mixer` and `mani_thermal_cycler`, which use `mjlab.passive.detent`, compile/reset in
  the `aero_sim` MuJoCo 3.8.1 environment when this plugin is loaded manually.
- The Python liquid path was checked through `mani_pipette` in both original 3.3.0 and replacement
  3.8.1 environments; both produce valid liquid state with near-identical initial volume/height.
- MuJoCo 3.8.1 produces different contact dynamics from MuJoCo 3.3.0 even with this plugin, so
  exact behavior parity should be judged first in the `autobio` MuJoCo 3.3.0 environment.
