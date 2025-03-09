cmake_minimum_required(VERSION 3.8)
project(street_graph)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
## Debugging Flags
#add_compile_options(-march=native -mtune=native -g3 -ggdb3 -O0 -fno-omit-frame-pointer)
#add_compile_options(-g3 -ggdb3 -O3 -march=native -mtune=native -fno-inline -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract --param=max-vartrack-size=0) # SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0')
#add_link_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract)
#add_compile_options(-fsanitize=thread -DEIGEN_DONT_PARALLELIZE) # SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt')
#add_link_options(-fsanitize=thread) # Might need prefix: setarch x86_64 -R # env TSAN_OPTIONS="report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt" gdb
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
#add_compile_options(-O2 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
## Profiling Flags
#add_compile_options(-O3 -march=native -mtune=native -g3 -fno-omit-frame-pointer -DNDEBUG -fno-math-errno)
#add_link_options(-Wl,--no-as-needed  -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -lprofiler -Wl,--as-needed)


##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(matplotlibcpp REQUIRED)

find_package(Python3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(LAPACK REQUIRED)
find_package(TBB REQUIRED)
set(TBB_USE_TBBMALLOC OFF)

###########
## Build ##
###########

## Graph Class
add_library(${PROJECT_NAME}_lib STATIC
  src/planning_graph.cpp
  src/graph.cpp
  src/nodes_edges.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_lib
  Eigen3
  TBB
  LAPACK
  OpenMP
)
target_link_libraries(${PROJECT_NAME}_lib
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Graph test node
add_executable(graph_test
  src/test_main.cpp
)
target_include_directories(graph_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(graph_test
  matplotlibcpp
  Eigen3
  TBB
  LAPACK
  OpenMP
)
target_link_libraries(graph_test
  ${PROJECT_NAME}_lib
  OpenMP::OpenMP_CXX
  TBB::tbb
  ${Python3_LIBRARIES}
)

############
## Export ##
############

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  Eigen3
  OpenMP
  TBB
  LAPACK
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  graph_test
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/graph_csv_gen.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############


#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
