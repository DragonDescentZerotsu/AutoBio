#include <cmath>
#include <cstdint>
#include <vector>

#include <mujoco/mujoco.h>

namespace {

struct DetentJoint {
  int qpos_adr;
  int dof_adr;
  mjtNum range_low;
  mjtNum spacing;
  int steps;
  mjtNum stiffness;
};

struct Detent {
  std::vector<DetentJoint> joints;
};

Detent* GetDetent(const mjData* data, int instance) {
  return reinterpret_cast<Detent*>(
      static_cast<std::uintptr_t>(data->plugin_data[instance]));
}

bool IsOneDofJoint(int type) {
  return type == mjJNT_SLIDE || type == mjJNT_HINGE;
}

}  // namespace

#if mjVERSION_HEADER >= 3008000
#define MJLAB_DETENT_PLUGIN_INIT mjPLUGIN_LIB_INIT(mjlab_detent)
#else
#define MJLAB_DETENT_PLUGIN_INIT mjPLUGIN_LIB_INIT
#endif

MJLAB_DETENT_PLUGIN_INIT {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mjlab.passive.detent";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  plugin.nstate = +[](const mjModel*, int) {
    return 0;
  };

  plugin.init = +[](const mjModel* model, mjData* data, int instance) {
    Detent* detent = new Detent;

    for (int body = 0; body < model->nbody; ++body) {
      if (model->body_plugin[body] != instance) {
        continue;
      }
      if (model->body_jntnum[body] != 1) {
        mju_warning("Detent plugin only supports 1 DOF bodies");
        delete detent;
        return -1;
      }

      const int joint = model->body_jntadr[body];
      if (!IsOneDofJoint(model->jnt_type[joint])) {
        mju_warning("Detent plugin only supports 1 DOF bodies");
        delete detent;
        return -1;
      }
      if (!model->jnt_limited[joint]) {
        mju_warning("Detent plugin only supports limited joints");
        delete detent;
        return -1;
      }
      if (model->jnt_stiffness[joint] == 0) {
        mju_warning("Detent plugin requires non-zero stiffness");
        delete detent;
        return -1;
      }

      const int qpos_adr = model->jnt_qposadr[joint];
      if (model->qpos_spring[qpos_adr] != 0) {
        mju_warning("Detent plugin does not support non-zero spring reference");
        delete detent;
        return -1;
      }

      int steps = 0;
      if (model->nuser_jnt > 0) {
        steps = static_cast<int>(model->jnt_user[joint * model->nuser_jnt]);
      }
      if (steps <= 1) {
        mju_warning("Detent plugin requires at least 2 steps");
        delete detent;
        return -1;
      }

      DetentJoint info;
      info.qpos_adr = qpos_adr;
      info.dof_adr = model->jnt_dofadr[joint];
      info.range_low = model->jnt_range[2 * joint];
      info.spacing = (model->jnt_range[2 * joint + 1] - model->jnt_range[2 * joint]) /
                     static_cast<mjtNum>(steps - 1);
      info.steps = steps;
      info.stiffness = model->jnt_stiffness[joint];
      detent->joints.push_back(info);
    }

    data->plugin_data[instance] = reinterpret_cast<std::uintptr_t>(detent);
    return 0;
  };

  plugin.destroy = +[](mjData* data, int instance) {
    delete GetDetent(data, instance);
    data->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};

  plugin.compute = +[](const mjModel*, mjData* data, int instance, int capability_bit) {
    if (capability_bit != mjPLUGIN_PASSIVE) {
      return;
    }
    const Detent* detent = GetDetent(data, instance);
    for (const DetentJoint& joint : detent->joints) {
      int target_index = static_cast<int>(
          std::round((data->qpos[joint.qpos_adr] - joint.range_low) / joint.spacing));
      if (target_index < 0) {
        target_index = 0;
      } else if (target_index >= joint.steps) {
        target_index = joint.steps - 1;
      }
      const mjtNum target = joint.range_low + joint.spacing * target_index;
      data->qfrc_passive[joint.dof_adr] += joint.stiffness * target;
    }
  };

  mjp_registerPlugin(&plugin);
}
