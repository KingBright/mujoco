# Copyright 2021 DeepMind Technologies Limited
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

if(CMAKE_CXX_FLAGS MATCHES "-Werror")
    string(REPLACE "-Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

# 如果 CMAKE_C_FLAGS 中包含 -Werror，则将其移除
if(CMAKE_C_FLAGS MATCHES "-Werror")
    string(REPLACE "-Werror" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
endif()

set(MUJOCO_USER_SRCS
    user_api.cc
    user_api.h
    user_cache.cc
    user_cache.h
    user_composite.cc
    user_composite.h
    user_flexcomp.cc
    user_flexcomp.h
    user_init.c
    user_mesh.cc
    user_model.cc
    user_model.h
    user_objects.cc
    user_objects.h
    user_resource.cc
    user_resource.h
    user_util.cc
    user_util.h
    user_vfs.cc
    user_vfs.h
)

target_sources(mujoco PRIVATE ${MUJOCO_USER_SRCS})
add_compile_options(-Wno-deprecated-declarations)
target_compile_options(mujoco PRIVATE -Wno-error)
target_compile_definitions(mujoco PRIVATE MUJOCO_TINYOBJLOADER_IMPL)
