// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "spid.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <functional>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::spid
{
  namespace
  {
    constexpr char kAttrPGain[] = "kp";
    constexpr char kAttrIGain[] = "ki";
    constexpr char kAttrDGain[] = "kd";
    constexpr char kAttrIMax[] = "imax";
    constexpr char kAttrSlewMax[] = "slewmax";
    constexpr char kAttrFeedbackSensor[] = "feedback_sensor";
    constexpr char kAttrOutputMultiply[] = "output_multiply";

    void LogMessage(const std::string &message)
    {
      std::cout << "SPID LOG: " << message << std::endl;
    }

    std::optional<mjtNum> ReadOptionalDoubleAttr(const mjModel *m, int instance,
                                                 const char *attr)
    {
      const char *value = mj_getPluginConfig(m, instance, attr);
      if (value == nullptr || value[0] == '\0')
      {
        return std::nullopt;
      }
      return std::strtod(value, nullptr);
    }

    std::optional<const char *> ReadStringAttr(const mjModel *m, int instance,
                                               const char *attr)
    {
      const char *value = mj_getPluginConfig(m, instance, attr);
      if (value == nullptr || value[0] == '\0')
      {
        return std::nullopt;
      }
      return value;
    }

    bool HasSlew(const mjModel *m, int instance)
    {
      return ReadOptionalDoubleAttr(m, instance, kAttrSlewMax).has_value();
    }

    int ActDim(const mjModel *m, int instance, int actuator_id)
    {
      double i_gain = ReadOptionalDoubleAttr(m, instance, kAttrIGain).value_or(0);
      return (i_gain ? 1 : 0) + (HasSlew(m, instance) ? 1 : 0);
    }

  } // namespace

  SPidConfig SPidConfig::FromModel(const mjModel *m, int instance)
  {
    SPidConfig config;
    config.p_gain = ReadOptionalDoubleAttr(m, instance, kAttrPGain).value_or(0);
    config.i_gain = ReadOptionalDoubleAttr(m, instance, kAttrIGain).value_or(0);
    config.d_gain = ReadOptionalDoubleAttr(m, instance, kAttrDGain).value_or(0);
    config.multiply = ReadOptionalDoubleAttr(m, instance, kAttrOutputMultiply).value_or(1);

    const char *feedback_sensor_name = ReadStringAttr(m, instance, kAttrFeedbackSensor).value_or("");
    config.feedback_sensor_index = mj_name2id(m, mjOBJ_SENSOR, feedback_sensor_name);

    double i_clamp_max_force =
        ReadOptionalDoubleAttr(m, instance, kAttrIMax).value_or(0);
    config.i_max = i_clamp_max_force / config.i_gain;

    config.slew_max = ReadOptionalDoubleAttr(m, instance, kAttrSlewMax).value_or(0);

    return config;
  }

  std::unique_ptr<SPid> SPid::Create(const mjModel *m, int instance)
  {
    SPidConfig config = SPidConfig::FromModel(m, instance);

    if (config.i_max < 0)
    {
      mju_warning("negative imax");
      return nullptr;
    }

    if (config.slew_max)
    {
      mju_warning("slewmax must be non-negative");
      return nullptr;
    }

    std::vector<int> actuators;
    for (int i = 0; i < m->nu; i++)
    {
      if (m->actuator_plugin[i] == instance)
      {
        actuators.push_back(i);
      }
    }
    if (actuators.empty())
    {
      mju_warning("actuator not found for plugin instance %d", instance);
      return nullptr;
    }

    // Validate actnum values for all actuators:
    for (int actuator_id : actuators)
    {
      int actnum = m->actuator_actnum[actuator_id];
      int expected_actnum = ActDim(m, instance, actuator_id);
      int dyntype = m->actuator_dyntype[actuator_id];
      if (dyntype == mjDYN_FILTER || dyntype == mjDYN_FILTEREXACT ||
          dyntype == mjDYN_INTEGRATOR)
      {
        expected_actnum++;
      }
      if (actnum != expected_actnum)
      {
        mju_warning(
            "actuator %d has actdim %d, expected %d. Add actdim=\"%d\" to the "
            "actuator plugin element.",
            actuator_id, actnum, expected_actnum, expected_actnum);
        return nullptr;
      }
    }

    State state;
    return std::unique_ptr<SPid>(new SPid(config, actuators, state));
  }

  void SPid::Reset(mjtNum *plugin_state) {}

  void SPid::Compute(const mjModel *m, mjData *d, int instance)
  {
    mjtNum rad = d->sensordata[config_.feedback_sensor_index];
    double current_deg = int(rad * 180.0 / M_PI) % 360;

    for (int i = 0; i < actuators_.size(); i++)
    {
      int actuator_idx = actuators_[i];

      // 读取当前的目标控制值
      double target_deg = d->ctrl[actuator_idx];
      // 如果是0，则先设置成默认值
      if (abs(target_deg - 0) < 0.01)
      {
        return;
      }
      else
      {
        mjtNum output_force = mjtNum(Step(target_deg, current_deg, m->opt.timestep));
        d->actuator_force[actuator_idx] = output_force * config_.multiply;

        const char *actuator_name = m->names + m->name_actuatoradr[actuator_idx];
        std::cout << actuator_name << ", current degree: " << current_deg << ", target degree: " << target_deg << ", force: " << output_force << std::endl;
      }
    }
  }

  double SPid::Step(double target_value, double current_value, double dt)
  {
    double error = target_value - current_value;

    double kp = config_.p_gain;
    double ki = config_.i_gain;
    double kd = config_.d_gain;
    double imax = config_.i_max;
    double slewmax = config_.slew_max;

    double previous_target = state_.previous_target;
    bool previous_target_exists = state_.previous_target_exists;
    double previous_error = state_.previous_error;
    bool previous_error_exists = state_.previous_error_exists;
    double integral = state_.integral;

    // 比例项
    double p_term = kp * error;

    // 积分项
    integral += error * dt;
    if (imax > 0)
    {
      integral = std::clamp(integral, -imax, imax);
    }
    double i_term = ki * integral;

    // 微分项
    double d_term = 0.0;
    if (previous_error_exists)
    {
      d_term = kd * (error - previous_error) / dt;
    }

    // 计算PID控制输出
    double ctrl_value = p_term + i_term + d_term;

    // 施加变化率限制（slew rate limiting）
    if (previous_target_exists && slewmax > 0)
    {
      double max_change = slewmax * dt;
      double target_change = target_value - previous_target;
      target_value = previous_target + std::clamp(target_change, -max_change, max_change);
    }

    // 更新历史数据
    state_.previous_error = error;
    state_.previous_target = target_value;
    state_.previous_error_exists = true;
    state_.previous_target_exists = true;
    state_.integral = integral;

    return ctrl_value;
  }

  void SPid::RegisterPlugin()
  {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "mujoco.spid";

    plugin.capabilityflags |= mjPLUGIN_ACTUATOR;

    std::vector<const char *> attributes = {kAttrPGain, kAttrIGain, kAttrDGain,
                                            kAttrIMax, kAttrSlewMax, kAttrFeedbackSensor, kAttrOutputMultiply};
    plugin.nattribute = attributes.size();
    plugin.attributes = attributes.data();

    // stateless state
    plugin.nstate = +[](const mjModel *m, int instance)
    { return 0; };

    plugin.init = +[](const mjModel *m, mjData *d, int instance)
    {
      std::unique_ptr<SPid> pid = SPid::Create(m, instance);
      if (pid == nullptr)
      {
        return -1;
      }
      d->plugin_data[instance] = reinterpret_cast<uintptr_t>(pid.release());
      return 0;
    };

    plugin.destroy = +[](mjData *d, int instance)
    {
      delete reinterpret_cast<SPid *>(d->plugin_data[instance]);
      d->plugin_data[instance] = 0;
    };

    plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                       int instance)
    {
      auto *pid = reinterpret_cast<SPid *>(plugin_data);
      pid->Reset(plugin_state);
    };

    plugin.compute =
        +[](const mjModel *m, mjData *d, int instance, int capability_bit)
    {
      auto *pid = reinterpret_cast<SPid *>(d->plugin_data[instance]);
      pid->Compute(m, d, instance);
    };
    mjp_registerPlugin(&plugin);
  }

  SPid::SPid(SPidConfig config, std::vector<int> actuators, State state)
      : config_(std::move(config)), actuators_(std::move(actuators)), state_(std::move(state))
  {
  }

} // namespace mujoco::plugin::actuator
