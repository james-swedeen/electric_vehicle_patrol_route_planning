cmake_minimum_required(VERSION 3.8)
project(surveillance_planner)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
add_compile_options(-DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)

## Debugging Flags
#add_compile_options(-march=native -mtune=native -g3 -ggdb3 -O0 -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=undefined -fsanitize=address -fsanitize-recover=address -fsanitize=pointer-compare -fsanitize=pointer-subtract --param=max-vartrack-size=0) # SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0')
#add_link_options(-fsanitize=undefined -fsanitize=address -fsanitize-recover=address -fsanitize=pointer-compare -fsanitize=pointer-subtract)
#add_compile_options(-fsanitize=thread -DEIGEN_DONT_PARALLELIZE) # SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt')
#add_link_options(-fsanitize=thread)
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno)
#add_compile_options(-O2 -march=native -mtune=native -DNDEBUG -fno-math-errno)
## Profiling Flags
#add_compile_options(-O3 -march=native -mtune=native -fno-omit-frame-pointer -DNDEBUG -fno-math-errno -g3)
#add_link_options(-Wl,--no-as-needed  -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -lprofiler -Wl,--as-needed)

set(BUILD_PLANNER_TUNER_SERVER 1)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(street_graph REQUIRED)
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


## Planner Parameter Tuner Server
if(${BUILD_PLANNER_TUNER_SERVER})
  add_executable(planner_tuner_server
    src/planner_tuner_server.cpp
  )
  target_include_directories(planner_tuner_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(planner_tuner_server
    rclcpp
    graph_surveillance_planning_msgs
    street_graph
    OpenMP
    Eigen3
    TBB
    LAPACK
  )
  target_link_libraries(planner_tuner_server
    OpenMP::OpenMP_CXX
    TBB::tbb
  )
endif()

## Planner Test
add_executable(planner_test
  src/planner_test.cpp
)
target_include_directories(planner_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(planner_test
  rclcpp
  street_graph
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
  street_graph
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
  surveillance_planner/planner_tuner.py
  surveillance_planner/planner_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

#install(
#  TARGETS
#    ${PROJECT_NAME}_random_lib
#    ${PROJECT_NAME}_greedy_lib
#  EXPORT ${PROJECT_NAME}Targets
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS planner_test
  DESTINATION lib/${PROJECT_NAME}
)
if(${BUILD_PLANNER_TUNER_SERVER})
  install(
    TARGETS planner_tuner_server
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

#############
## Testing ##
#############

ament_package()
