cmake_minimum_required(VERSION 3.20)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 17)
project(mcts_gcs_planner C CXX )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # used to quiet some stuff during build
  add_compile_options(-Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare)

endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sa_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(swri_transform_util  REQUIRED)
find_package(drake REQUIRED)
# find_package(CGAL REQUIRED)
# find_package(GMP REQUIRED)

# set(msg_files
# "msg/Node.msg"
# "msg/WorldModel.msg"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}  
#   DEPENDENCIES 
#     builtin_interfaces 
#     std_msgs
#     geometry_msgs
#     nav_msgs
#     nav2_msgs
#  )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(gcs_planner 
  src/Util/Common.cpp 
  src/Util/GenerateSWM.cpp 
  src/Util/Astar.cpp 
  src/Util/MaverickPlanner.cpp
  src/MCTS/MCTS.cpp
  src/MCTS/MCTS_GCS.cpp
  src/MCTS/JobScheduler.cpp
  # src/GCS/mcts_gcs_trajectory_optimization.cpp
  # src/GCS/mcts_graph_of_convex_sets.cpp
  src/GCSPlanner.cpp 
  # src/MCTS_GCSPlanner.cpp
  src/main.cpp)

ament_target_dependencies(gcs_planner rclcpp std_msgs sa_msgs std_srvs geometry_msgs visualization_msgs nav_msgs nav2_msgs OpenCV drake swri_transform_util) #  CGAL

target_include_directories(gcs_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS 
  gcs_planner
  DESTINATION lib/${PROJECT_NAME})

# Install launch/config files.
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
