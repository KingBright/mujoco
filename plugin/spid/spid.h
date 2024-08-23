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

#ifndef MUJOCO_PLUGIN_PID_H_
#define MUJOCO_PLUGIN_PID_H_

#include <memory>
#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

namespace mujoco::plugin::spid
{

  struct SPidConfig
  {
    double p_gain = 0.0;
    double i_gain = 0.0;
    double d_gain = 0.0;
    int feedback_sensor_index = -1;
    double multiply = 1.0;

    // NOTE: In the XML definition, the clamp values are specified as limits on
    // the *force* value, so are scaled by i_gain.
    double i_max;

    // Maximum speed at which the setpoint can change.
    double slew_max;

    // Reads plugin attributes to construct PID configuration.
    static SPidConfig FromModel(const mjModel *m, int instance);
  };

  struct State
  {
    double integral = 0.0;
    double previous_target = 0.0;
    double previous_error = 0.0;
    bool previous_target_exists = false;
    bool previous_error_exists = false;
  };

  // An actuator plugin which implements configurable PID control.
  class SPid
  {
  public:
    static std::unique_ptr<SPid> Create(const mjModel *m, int instance);

    void Compute(const mjModel *m, mjData *d, int instance);

    static void RegisterPlugin();

    void Reset(mjtNum *plugin_state);

  private:
    SPidConfig config_;
    std::vector<int> actuators_;
    State state_;

    SPid(SPidConfig config, std::vector<int> actuators, State state);

    double Step(double target_value, double current_value, double dt);
  };

}

#endif
