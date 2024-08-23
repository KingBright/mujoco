# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(MUJOCO_SPID_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../..
                            ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

set(MUJOCO_SPID_SRCS
    spid.cc
    spid.h
    register.cc
)
# add_compile_options(-Wno-error=unused-private-field)
add_compile_options(-Wno-error=unused-function)
add_compile_options(-Wno-error)

add_library(spid SHARED)
target_sources(spid PRIVATE ${MUJOCO_SPID_SRCS})
target_include_directories(spid PRIVATE ${MUJOCO_SPID_INCLUDE})
target_link_libraries(spid PRIVATE mujoco)
target_compile_options(
  spid
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
  -Wno-error
)
target_link_options(
  spid
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
  -Wno-error
)
