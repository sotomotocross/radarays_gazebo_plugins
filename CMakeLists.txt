cmake_minimum_required(VERSION 3.8)
project(radarays_gazebo_plugins)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

option(RADARAYS_GZSIM_PORT "Install the ROS 2 / Gazebo Harmonic migration path" ON)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake REQUIRED)

if(RADARAYS_GZSIM_PORT)
  find_package(rmagine_gazebo_plugins REQUIRED)
  find_package(radarays_ros REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-common5 REQUIRED)
  find_package(gz-math7 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-rendering8 REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rmagine 2.3
    COMPONENTS
      core
    OPTIONAL_COMPONENTS
      embree
      cuda
      optix
  )
  # radarays_ros/image_algorithms.h (perlin_noise, reused from
  # radarays_core) unconditionally includes opencv2/core.hpp for its other,
  # unused-here functions.
  find_package(OpenCV REQUIRED)

  include_directories(include ${OpenCV_INCLUDE_DIRS})

  add_library(radarays_embree_sensor_system SHARED
    src/gz/radarays_embree_sensor_system.cpp
  )

  target_include_directories(radarays_embree_sensor_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rmagine_gazebo_plugins_INCLUDE_DIRS}
  )

  target_link_libraries(radarays_embree_sensor_system
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-rendering8::gz-rendering8
    rmagine::core
    radarays_ros::radarays_core
    ${OpenCV_LIBS}
  )

  if(TARGET rmagine::embree)
    target_link_libraries(radarays_embree_sensor_system rmagine::embree)
  endif()

  ament_target_dependencies(radarays_embree_sensor_system
    rclcpp
    sensor_msgs
  )

  install(TARGETS radarays_embree_sensor_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  # GPU/OptiX radar path -- mirrors radarays_embree_sensor_system, physics
  # loop replaced by radarays_ros's GPU CUDA kernels (radarays_gpu target,
  # move_waves/signal_shader/fresnel_split + the plain-POD RadarMaterial).
  # Needs CUDA enabled in *this* CMake project too -- see radarays_ros's
  # CMakeLists.txt comment for why. See MIGRATION_HANDOFF.md, "radarays GPU
  # radar path", for the OptiX SDK/driver ABI notes this also inherits.
  if(TARGET rmagine::optix)
    if(CMAKE_CUDA_COMPILER)
      enable_language(CUDA)
    else()
      find_package(CUDAToolkit QUIET)
      if(CUDAToolkit_FOUND)
        enable_language(CUDA)
      endif()
    endif()
  endif()

  if(TARGET rmagine::optix AND CMAKE_CUDA_COMPILER AND TARGET radarays_ros::radarays_gpu_core)
    add_library(radarays_optix_sensor_system SHARED
      src/gz/radarays_optix_sensor_system.cpp
    )

    target_include_directories(radarays_optix_sensor_system PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${rmagine_gazebo_plugins_INCLUDE_DIRS}
    )

    target_link_libraries(radarays_optix_sensor_system
      gz-sim8::gz-sim8
      gz-plugin2::gz-plugin2
      gz-transport13::gz-transport13
      gz-common5::gz-common5
      gz-math7::gz-math7
      gz-msgs10::gz-msgs10
      gz-rendering8::gz-rendering8
      rmagine::core
      rmagine::optix
      radarays_ros::radarays_core
      radarays_ros::radarays_gpu_core
      ${OpenCV_LIBS}
    )

    ament_target_dependencies(radarays_optix_sensor_system
      rclcpp
      sensor_msgs
    )

    install(TARGETS radarays_optix_sensor_system
      LIBRARY DESTINATION lib/${PROJECT_NAME}
    )

    # Test registration below must be gated on this exact flag, not just
    # `TARGET rmagine::optix` -- an environment with OptiX but no CUDA
    # compiler or radarays_gpu_core would otherwise register a GPU
    # fixture test that references a .so that was never built, a
    # confusing test-registration failure rather than a real regression
    # (flagged in self-review).
    set(RADARAYS_OPTIX_SENSOR_SYSTEM_BUILT TRUE)
  elseif(TARGET rmagine::optix)
    message(WARNING "rmagine::optix found but radarays_ros::radarays_gpu or a CUDA compiler is missing -- radarays_optix_sensor_system will not be built.")
  endif()
endif()

install(
  DIRECTORY
    worlds
    urdf
    media
    testdata
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    README.md
    MIGRATION.md
    LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(RADARAYS_GZSIM_PORT)
  install(
    FILES
      launch/gz_static_radar_cpu.launch.py
      launch/gz_dynamic_radar_cpu.launch.py
      launch/gz_dynamic_radar_cpu_multi.launch.py
    DESTINATION share/${PROJECT_NAME}/launch
  )

  install(
    PROGRAMS
      scripts/radarays_fixture_harness.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME capture_radarays_fixture
  )

  install(
    PROGRAMS
      scripts/radarays_fixture_harness.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME compare_radarays_fixture
  )

  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
else()
  message(WARNING "RADARAYS_GZSIM_PORT is OFF. The legacy Gazebo Classic sources are preserved in the repository, but this package now builds only through the ROS 2 / Gazebo Harmonic path.")
endif()

if(BUILD_TESTING AND RADARAYS_GZSIM_PORT)
  # Wires the existing capture/compare fixture harness into `colcon test`
  # instead of only ever running by hand or via ci_build_and_test.sh --
  # see MIGRATION_HANDOFF.md, "Wired fixtures into colcon test". Each test
  # captures a fresh run of the world and compares it against the
  # committed fixture in testdata/radarays_harmonic/ in one shot.
  #
  # /usr/local/lib is where the libiomp5.so workaround symlink lives (see
  # "Known Environment Issues" in MIGRATION_HANDOFF.md) -- APPEND_LIBRARY_DIRS
  # puts it on LD_LIBRARY_PATH for the test process, matching what
  # ci_build_and_test.sh does at the shell level.
  find_package(ament_cmake_test REQUIRED)

  ament_add_test(radarays_fixture_static_cpu
    COMMAND bash -c "ros2 run radarays_gazebo_plugins capture_radarays_fixture static_cpu && ros2 run radarays_gazebo_plugins compare_radarays_fixture static_cpu"
    APPEND_LIBRARY_DIRS "/usr/local/lib"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )

  ament_add_test(radarays_fixture_dynamic_cpu
    COMMAND bash -c "ros2 run radarays_gazebo_plugins capture_radarays_fixture dynamic_cpu && ros2 run radarays_gazebo_plugins compare_radarays_fixture dynamic_cpu"
    APPEND_LIBRARY_DIRS "/usr/local/lib"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 180
  )

  ament_add_test(radarays_fixture_dynamic_multi_cpu
    COMMAND bash -c "ros2 run radarays_gazebo_plugins capture_radarays_fixture dynamic_multi_cpu && ros2 run radarays_gazebo_plugins compare_radarays_fixture dynamic_multi_cpu"
    APPEND_LIBRARY_DIRS "/usr/local/lib"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 180
  )

  # GPU/OptiX regression coverage -- gz_egomotion_radar_gpu.sdf (the
  # moving-SENSOR scenario, Tier1 #1) already existed on disk but was
  # never wired into colcon test at all, meaning this package's entire
  # GPU/OptiX path had zero automated coverage. Falsifiable motion checks
  # (center_value_span_min, center_column_peak_row_span_min) prove the
  # sensor's own motion is actually reflected in the output, not just
  # "some nonzero data" -- see README.md for the real numbers this was
  # verified against.
  if(RADARAYS_OPTIX_SENSOR_SYSTEM_BUILT)
    ament_add_test(radarays_fixture_egomotion_gpu
      COMMAND bash -c "ros2 run radarays_gazebo_plugins capture_radarays_fixture egomotion_gpu && ros2 run radarays_gazebo_plugins compare_radarays_fixture egomotion_gpu"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 180
    )
  endif()
endif()

ament_package()
