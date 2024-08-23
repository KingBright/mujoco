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

#ifndef MUJOCO_PLUGIN_ANGLE_MONITOR_H_
#define MUJOCO_PLUGIN_ANGLE_MONITOR_H_

#include <memory>
#include <optional>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

namespace mujoco::plugin::angle_monitor
{

  class AngleMonitor
  {
  public:
    AngleMonitor();

    void Init(const mjModel *m, int instance);

    static std::unique_ptr<AngleMonitor> Create(const mjModel *m, int instance);

    // Step function called at each simulation step
    void Step(const mjModel *m, const mjData *d, int instance);

    static void RegisterPlugin();

  private:
    // Sensor indices for chassis, boom, arm, and bucket
    int chassis_yaw_index;
    int boom_pitch_index;
    int arm_pitch_index;
    int bucket_pitch_index;
  };

}

#endif