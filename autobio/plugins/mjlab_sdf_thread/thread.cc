#include <cmath>
#include <cstdlib>
#include <cstdint>

#include <mujoco/mujoco.h>

namespace {

constexpr int kNumAttributes = 5;
constexpr const char* kAttributes[kNumAttributes] = {
    "pitch", "radius", "low", "high", "gauge",
};

constexpr mjtNum kDefaultPitch = 0.003;
constexpr mjtNum kDefaultRadius = 0.014;
constexpr mjtNum kDefaultLow = -1.0;
constexpr mjtNum kDefaultHigh = 1.2;
constexpr mjtNum kDefaultGauge = 0.0008;
#ifndef MJLAB_THREAD_GRADIENT_EPS
#define MJLAB_THREAD_GRADIENT_EPS 5e-5
#endif
constexpr mjtNum kGradientEps = MJLAB_THREAD_GRADIENT_EPS;

struct ThreadSdf {
  mjtNum pitch;
  mjtNum radius;
  mjtNum low;
  mjtNum high;
  mjtNum gauge;
};

void FillDefaults(mjtNum attribute[]) {
  attribute[0] = kDefaultPitch;
  attribute[1] = kDefaultRadius;
  attribute[2] = kDefaultLow;
  attribute[3] = kDefaultHigh;
  attribute[4] = kDefaultGauge;
}

mjtNum ParseOrDefault(const char* value, mjtNum fallback) {
  if (!value || !value[0]) {
    return fallback;
  }
  char* end = nullptr;
  mjtNum parsed = std::strtod(value, &end);
  return end == value ? fallback : parsed;
}

ThreadSdf FromAttributes(const mjtNum attribute[]) {
  ThreadSdf sdf;
  sdf.pitch = attribute[0];
  sdf.radius = attribute[1];
  sdf.low = attribute[2];
  sdf.high = attribute[3];
  sdf.gauge = attribute[4];
  if (sdf.pitch <= 0) {
    sdf.pitch = kDefaultPitch;
  }
  if (sdf.radius <= 0) {
    sdf.radius = kDefaultRadius;
  }
  if (sdf.gauge < 0) {
    sdf.gauge = 0;
  }
  if (sdf.high < sdf.low) {
    mjtNum temp = sdf.high;
    sdf.high = sdf.low;
    sdf.low = temp;
  }
  return sdf;
}

mjtNum DistanceToHelixEndpoint(
    const mjtNum point[3], const ThreadSdf& sdf, mjtNum turns, mjtNum closest[3]);

void HelixPoint(mjtNum helix_point[3], const ThreadSdf& sdf, mjtNum turns) {
  const mjtNum phase = 2.0 * mjPI * turns;
  helix_point[0] = sdf.radius * std::cos(phase);
  helix_point[1] = sdf.radius * std::sin(phase);
  helix_point[2] = sdf.pitch * turns;
}

mjtNum DistanceToHelixAt(
    const mjtNum point[3], const ThreadSdf& sdf, mjtNum turns, mjtNum closest[3]) {
  return DistanceToHelixEndpoint(point, sdf, turns, closest);
}

mjtNum DistanceToHelixEndpoint(
    const mjtNum point[3], const ThreadSdf& sdf, mjtNum turns, mjtNum closest[3]) {
  HelixPoint(closest, sdf, turns);
  const mjtNum dx = closest[0] - point[0];
  const mjtNum dy = closest[1] - point[1];
  const mjtNum dz = closest[2] - point[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void SelectClosestPoint(mjtNum closest[3], const mjtNum point[3], const ThreadSdf& sdf) {
  const mjtNum azimuth = std::atan2(point[1], point[0]);
  const mjtNum azimuth_turns = azimuth / (2.0 * mjPI);
  const mjtNum axial_turns = point[2] / sdf.pitch;
  const mjtNum nearest_turn = azimuth_turns + std::round(axial_turns - azimuth_turns);
  const mjtNum clamped_turn = mju_min(mju_max(nearest_turn, sdf.low), sdf.high);

  // Include endpoint candidates. This avoids poor behavior near bounded helix ends.
  mjtNum candidate[3];
  mjtNum distance = DistanceToHelixAt(point, sdf, clamped_turn, closest);

  mjtNum candidate_distance = DistanceToHelixEndpoint(point, sdf, sdf.low, candidate);
  if (candidate_distance < distance) {
    distance = candidate_distance;
    mju_copy3(closest, candidate);
  }

  candidate_distance = DistanceToHelixEndpoint(point, sdf, sdf.high, candidate);
  if (candidate_distance < distance) {
    mju_copy3(closest, candidate);
  }
}

mjtNum ThreadDistanceWithAttributes(const mjtNum point[3], const mjtNum attribute[]) {
  const ThreadSdf sdf = FromAttributes(attribute);
  mjtNum closest[3];
  SelectClosestPoint(closest, point, sdf);
  const mjtNum dx = closest[0] - point[0];
  const mjtNum dy = closest[1] - point[1];
  const mjtNum dz = closest[2] - point[2];
  const mjtNum distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  return distance - sdf.gauge;
}

void ThreadGradientWithAttributes(
    mjtNum gradient[3], const mjtNum point[3], const mjtNum attribute[]) {
  const mjtNum eps = kGradientEps;
  mjtNum p_plus[3] = {point[0], point[1], point[2]};
  mjtNum p_minus[3] = {point[0], point[1], point[2]};

  for (int i = 0; i < 3; ++i) {
    p_plus[i] = point[i] + eps;
    p_minus[i] = point[i] - eps;
    const mjtNum d_plus = ThreadDistanceWithAttributes(p_plus, attribute);
    const mjtNum d_minus = ThreadDistanceWithAttributes(p_minus, attribute);
    gradient[i] = (d_plus - d_minus) / (2.0 * eps);
    p_plus[i] = point[i];
    p_minus[i] = point[i];
  }

  const mjtNum norm = std::sqrt(
      gradient[0] * gradient[0] + gradient[1] * gradient[1] + gradient[2] * gradient[2]);
  if (norm > 1e-12) {
    gradient[0] /= norm;
    gradient[1] /= norm;
    gradient[2] /= norm;
  }
}

ThreadSdf* GetThread(const mjData* data, int instance) {
  return reinterpret_cast<ThreadSdf*>(
      static_cast<std::uintptr_t>(data->plugin_data[instance]));
}

}  // namespace

#if mjVERSION_HEADER >= 3008000
#define MJLAB_PLUGIN_INIT mjPLUGIN_LIB_INIT(mjlab_thread)
#else
#define MJLAB_PLUGIN_INIT mjPLUGIN_LIB_INIT
#endif

MJLAB_PLUGIN_INIT {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mjlab.sdf.thread";
  plugin.capabilityflags |= mjPLUGIN_SDF;
  plugin.nattribute = kNumAttributes;
  plugin.attributes = kAttributes;

  plugin.nstate = +[](const mjModel*, int) {
    return 0;
  };

  plugin.init = +[](const mjModel* model, mjData* data, int instance) {
    mjtNum attribute[kNumAttributes];
    FillDefaults(attribute);
    for (int i = 0; i < kNumAttributes; ++i) {
      attribute[i] = ParseOrDefault(
          mj_getPluginConfig(model, instance, kAttributes[i]), attribute[i]);
    }
    data->plugin_data[instance] = reinterpret_cast<std::uintptr_t>(
        new ThreadSdf(FromAttributes(attribute)));
    return 0;
  };

  plugin.destroy = +[](mjData* data, int instance) {
    delete GetThread(data, instance);
    data->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel*, mjData*, int, int) {};

  plugin.sdf_distance = +[](const mjtNum point[3], const mjData* data, int instance) {
    const ThreadSdf* sdf = GetThread(data, instance);
    const mjtNum attribute[kNumAttributes] = {
        sdf->pitch, sdf->radius, sdf->low, sdf->high, sdf->gauge,
    };
    return ThreadDistanceWithAttributes(point, attribute);
  };

  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
                            const mjData* data, int instance) {
    const ThreadSdf* sdf = GetThread(data, instance);
    const mjtNum attribute[kNumAttributes] = {
        sdf->pitch, sdf->radius, sdf->low, sdf->high, sdf->gauge,
    };
    ThreadGradientWithAttributes(gradient, point, attribute);
  };

  plugin.sdf_staticdistance = +[](const mjtNum point[3], const mjtNum* attribute) {
    return ThreadDistanceWithAttributes(point, attribute);
  };

  plugin.sdf_attribute = +[](mjtNum attribute[], const char*[], const char* value[]) {
    FillDefaults(attribute);
    for (int i = 0; i < kNumAttributes; ++i) {
      attribute[i] = ParseOrDefault(value[i], attribute[i]);
    }
  };

  plugin.sdf_aabb = +[](mjtNum aabb[6], const mjtNum* attribute) {
    const ThreadSdf sdf = FromAttributes(attribute);
    const mjtNum radial = sdf.radius + 2.0 * sdf.gauge;
    const mjtNum z_low = sdf.pitch * sdf.low;
    const mjtNum z_high = sdf.pitch * sdf.high;
    aabb[0] = 0;
    aabb[1] = 0;
    aabb[2] = 0.5 * (z_low + z_high);
    aabb[3] = radial;
    aabb[4] = radial;
    aabb[5] = 0.5 * (z_high - z_low) + 2.0 * sdf.gauge;
  };

  mjp_registerPlugin(&plugin);
}
