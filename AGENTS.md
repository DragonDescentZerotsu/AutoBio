## Env instruction
Use the aero_sim conda env to run your code if mujoco is necessary. conda is at: /data/tianang/miniconda/condabin/conda

## MuJoCo / AutoBio notes
- `autobio/README.md` asks for a separate `autobio` conda env with Python 3.11 and `mujoco==3.3.0`.
- The bundled `autobio/libmjlab.so.3.3.0` depends on `libmujoco.so.3.3.0`; it is expected to work in that env.
- Do not binary-patch `libmjlab.so.3.3.0` for MuJoCo 3.8.x. It can load after patching but segfaults during model compilation because the plugin ABI/behavior is not compatible enough.
- For quick pipette visualization in the existing `aero_sim` env, use `AUTOBIO_SKIP_MJLAB=1` with `autobio/model/scene/mani_pipette_viewer.xml`. That viewer-only scene replaces the screw-threaded 50 ml tube with the non-plugin `centrifuge_50ml.gen.xml`; it is not equivalent for screw/unscrew physics.
- `mjlab.sdf.thread` implements helical thread SDFs configured by `pitch`, `radius`, `low`, `high`, and `gauge`. `AutoBio.pdf` gives the helix SDF approximation in appendix A.2, but the repo currently includes only the compiled plugin, not its source.
- A fresh `autobio` conda env was created at `/data/tianang/miniconda/envs/autobio` with Python 3.11 and README dependencies. In that env, the original bundled `libmjlab.so.3.3.0` loads and original `mani_pipette` compiles/resets/steps successfully when `LD_LIBRARY_PATH` includes the env's `site-packages/mujoco`.
- Experimental source for a MuJoCo 3.8.1 `mjlab.sdf.thread` reimplementation lives in `autobio/plugins/mjlab_sdf_thread/`. Build output is `autobio/libmjlab_thread.so.<version>` and is ignored by git. This reimplementation lets `mani_pipette` compile/reset/step in `aero_sim`. In a same-version MuJoCo 3.3.0 comparison it matches the original plugin's 50 ml screw tube SDF mesh vertex/face counts, but the generated thread mesh still shows a z-mirror difference, so screw/unscrew contact behavior still needs task-level validation.
