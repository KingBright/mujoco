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

#include "angle_monitor.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::angle_monitor
{

  void LogMessage(const std::string &message)
  {
    // mju_warning("AngleMonitor:warning: %s", message.c_str());
    std::cout << message << std::endl;
  }

  void AngleMonitor::Init(const mjModel *m, int instance)
  {
    // Get sensor indices for chassis, boom, arm, and bucket
    LogMessage("AngleMonitor init...");
    chassis_yaw_index = mj_name2id(m, mjOBJ_SENSOR, "chassis_yaw_sensor");
    LogMessage("Sensor index for chassis yaw: " + std::to_string(chassis_yaw_index));
    boom_pitch_index = mj_name2id(m, mjOBJ_SENSOR, "boom_pitch_sensor");
    LogMessage("Sensor index for boom pitch: " + std::to_string(chassis_yaw_index));
    arm_pitch_index = mj_name2id(m, mjOBJ_SENSOR, "arm_pitch_sensor");
    LogMessage("Sensor index for arm pitch: " + std::to_string(chassis_yaw_index));
    bucket_pitch_index = mj_name2id(m, mjOBJ_SENSOR, "bucket_pitch_sensor");
    LogMessage("Sensor index for bucket pitch: " + std::to_string(chassis_yaw_index));
  }

  void AngleMonitor::Step(const mjModel *m, const mjData *d, int instance)
  {
    if (chassis_yaw_index >= 0 && boom_pitch_index >= 0 && arm_pitch_index >= 0 && bucket_pitch_index >= 0)
    {
      // Retrieve angle values from sensors
      mjtNum chassis_yaw_rad = d->sensordata[chassis_yaw_index];
      mjtNum boom_pitch_rad = d->sensordata[boom_pitch_index];
      mjtNum arm_pitch_rad = d->sensordata[arm_pitch_index];
      mjtNum bucket_pitch_rad = d->sensordata[bucket_pitch_index];

      // Convert radians to degrees
      mjtNum chassis_yaw_deg = int(chassis_yaw_rad * 180.0 / M_PI) % 360;
      mjtNum boom_pitch_deg = boom_pitch_rad * 180.0 / M_PI;
      mjtNum arm_pitch_deg = arm_pitch_rad * 180.0 / M_PI;
      mjtNum bucket_pitch_deg = bucket_pitch_rad * 180.0 / M_PI;

      // Output angles (for debugging)
      std::cout << "Chassis Yaw Angle (degrees): " << chassis_yaw_deg << std::endl;
      std::cout << "Boom Pitch Angle (degrees): " << boom_pitch_deg << std::endl;
      std::cout << "Arm Pitch Angle (degrees): " << arm_pitch_deg << std::endl;
      std::cout << "Bucket Pitch Angle (degrees): " << bucket_pitch_deg << std::endl;

      // You can also update a UI or send data to a visualization tool here
      // For example:
      // updateUI(chassis_yaw_deg, boom_pitch_deg, arm_pitch_deg, bucket_pitch_deg);
    }
    else
    {
      std::cerr << "Sensor indices not initialized correctly!" << std::endl;
    }
  }

  std::unique_ptr<AngleMonitor> AngleMonitor::Create(const mjModel *m, int instance)
  {
    AngleMonitor *monitor = new AngleMonitor();
    monitor->Init(m, instance);
    return std::unique_ptr<AngleMonitor>(monitor);
  }

  void AngleMonitor::RegisterPlugin()
  {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);

    plugin.name = "mujoco.angle_monitor";
    plugin.capabilityflags |= mjPLUGIN_SENSOR;

    // Input parameters
    const char *attributes[] = {"chassis_yaw_sensor", "boom_pitch_sensor", "arm_pitch_sensor", "bucket_pitch_sensor"};
    plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
    plugin.attributes = attributes;

    // stateless state
    plugin.nstate = +[](const mjModel *m, int instance)
    { return 0; };

    plugin.nsensordata = +[](const mjModel *m, int instance, int sensor_id)
    {
      return 0;
    };

    plugin.needstage = mjSTAGE_NONE;

    // Initialization callback
    plugin.init = +[](const mjModel *m, mjData *d, int instance)
    {
      std::unique_ptr<AngleMonitor> ptr = AngleMonitor::Create(m, instance);
      if (ptr == nullptr)
      {
        LogMessage("AngleMonitor: init failed");
        return -1;
      }
      d->plugin_data[instance] = reinterpret_cast<uintptr_t>(ptr.release());
      return 0;
    };

    // Destruction callback
    plugin.destroy = +[](mjData *d, int instance)
    {
      delete reinterpret_cast<AngleMonitor *>(d->plugin_data[instance]);
      d->plugin_data[instance] = 0;
    };

    // Compute callback
    plugin.compute =
        +[](const mjModel *m, mjData *d, int instance, int capability_bit)
    {
      auto *angle_monitor =
          reinterpret_cast<AngleMonitor *>(d->plugin_data[instance]);
      angle_monitor->Step(m, d, instance);
    };

    mjp_registerPlugin(&plugin);
  }

  AngleMonitor::AngleMonitor() : chassis_yaw_index(-1),
                                 boom_pitch_index(-1),
                                 arm_pitch_index(-1),
                                 bucket_pitch_index(-1)
  {
    LogMessage("Initiate for AngleMonitor...");
  }
}