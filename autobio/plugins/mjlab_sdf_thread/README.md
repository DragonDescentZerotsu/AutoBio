# mjlab.sdf.thread reimplementation

This is a source reimplementation of the `mjlab.sdf.thread` MuJoCo SDF plugin for newer MuJoCo
versions. It is intended for compatibility testing against AutoBio's bundled `libmjlab.so.3.3.0`,
not yet as a fully validated physical replacement.

The implementation follows the bounded circular-helix SDF approximation described in
`AutoBio.pdf`, appendix A.2, with the axis/padding conventions inferred from the bundled binary.
It registers the original plugin name and accepts the same XML attributes:

- `pitch`: axial thread pitch per full revolution
- `radius`: helix radius
- `low`: lower bound in thread revolutions
- `high`: upper bound in thread revolutions
- `gauge`: thread tube radius, subtracted from the centerline distance

Build for the current `aero_sim` MuJoCo wheel from the repository root:

```bash
./autobio/plugins/mjlab_sdf_thread/build.sh
```

The output is `autobio/libmjlab_thread.so.<mujoco-version>`.

Current validation:

- In the `autobio` MuJoCo 3.3.0 environment, this reimplementation produces the same vertex and
  face counts as the bundled binary for the 50 ml screw tube SDF meshes.
- The remaining observed difference is a mirror in the generated z extent of the thread mesh.
- In the `aero_sim` MuJoCo 3.8.1 environment, the original `mani_pipette` task compiles, resets,
  and steps with this plugin loaded manually.
