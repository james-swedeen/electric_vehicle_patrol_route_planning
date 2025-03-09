cmake_minimum_required(VERSION 3.8)
project(firefly_algorithm)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
## Debugging Flags
#add_compile_options(-g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)
#add_compile_options(-g3 -ggdb3 -Og -march=native -mtune=native -fno-inline -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract --param=max-vartrack-size=0) # SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0')
#add_link_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract)
#add_compile_options(-fsanitize=thread -DEIGEN_DONT_PARALLELIZE) # SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt')
#add_link_options(-fsanitize=thread)
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)
#add_compile_options(-O3 -march=native -mtune=native -fno-math-errno -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)
#add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
#add_compile_options(-O2 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
## Profiling Flags
#add_compile_options(-O3 -march=native -mtune=native -g3 -fno-inline -fno-omit-frame-pointer -DNDEBUG -DEIGEN_USE_LAPACKE)
#add_link_options(-Wl,--no-as-needed -lprofiler -Wl,--as-needed)

set(BUILD_FIREFLY_TUNER_SERVER 1)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(graph_surveillance_planning_msgs REQUIRED)
find_package(matplotlibcpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(LAPACK REQUIRED)
find_package(TBB REQUIRED)
set(TBB_USE_TBBMALLOC OFF)

###########
## Build ##
###########

## Firefly Parameter Tuner Server
if(${BUILD_FIREFLY_TUNER_SERVER})
  add_executable(firefly_tuner_server
    src/firefly_tuner_server.cpp
  )
  target_include_directories(firefly_tuner_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(firefly_tuner_server
    rclcpp
    graph_surveillance_planning_msgs
    OpenMP
    Eigen3
    TBB
    LAPACK
  )
  target_link_libraries(firefly_tuner_server
    OpenMP::OpenMP_CXX
    TBB::tbb
  )
endif()

## Firefly Benchmark
add_executable(firefly_benchmark
  src/firefly_benchmark.cpp
)
target_include_directories(firefly_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(firefly_benchmark
  rclcpp
  graph_surveillance_planning_msgs
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(firefly_benchmark
  OpenMP::OpenMP_CXX
  TBB::tbb
)

############
## Export ##
############

#ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  rclcpp
  rclpy
  graph_surveillance_planning_msgs
  matplotlibcpp
  Eigen3
  OpenMP
  TBB
  LAPACK
)

## Planner Test
add_executable(planner_test
  src/planner_test.cpp
)
target_include_directories(planner_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(planner_test
  rclcpp
  graph_surveillance_planning_msgs
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(planner_test
  OpenMP::OpenMP_CXX
  TBB::tbb
)

############
## Export ##
############

#ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  rclcpp
  rclpy
  graph_surveillance_planning_msgs
  matplotlibcpp
  Eigen3
  OpenMP
  TBB
  LAPACK
)

#############
## Install ##
#############

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  firefly_algorithm/firefly_tuner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

#install(
#  TARGETS
#  EXPORT ${PROJECT_NAME}Targets
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

install(
  TARGETS planner_test
          firefly_benchmark
  DESTINATION lib/${PROJECT_NAME}
)
if(${BUILD_FIREFLY_TUNER_SERVER})
  install(
    TARGETS firefly_tuner_server
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

#############
## Testing ##
#############

ament_package()
