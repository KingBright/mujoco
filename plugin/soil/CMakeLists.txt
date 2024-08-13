set(MUJOCO_SOIL_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/register.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/soil.h
    ${CMAKE_CURRENT_SOURCE_DIR}/soil.cc
)
set(MUJOCO_SOIL_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/soil_dynamics_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wno-error=comment)
add_compile_options(-Wno-error=unused-variable)
add_compile_options(-Wno-error)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-lto")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-lto")

find_library(soil_simulator NAMES soil_simulator)

add_library(soil SHARED)
target_sources(soil PRIVATE ${MUJOCO_SOIL_SRCS})
target_include_directories(soil PRIVATE ${MUJOCO_SOIL_INCLUDE})

target_link_libraries(
  soil
  PRIVATE mujoco
          soil_simulator
)
target_compile_options(
  soil
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  soil
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)

add_subdirectory(soil_dynamics_cpp EXCLUDE_FROM_ALL)
