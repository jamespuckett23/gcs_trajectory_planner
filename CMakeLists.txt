cmake_minimum_required(VERSION 3.20)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 17)
project(sa_semantic_map C CXX )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # used to quiet some stuff during build
  add_compile_options(-Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare)

endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)





find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sa_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(swri_transform_util  REQUIRED)
# find_package(mapviz_msgs REQUIRED)
find_package(drake REQUIRED)
find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_libraries(${OpenCV_LIBS})

add_executable(sa_map src/main.cpp src/SemanticMap.cpp src/BoundaryMap.cpp src/CDT.cpp src/Planner.cpp src/UTM.cpp src/read_data.cpp)
ament_target_dependencies(sa_map rclcpp std_msgs sa_msgs std_srvs geometry_msgs visualization_msgs nav_msgs nav2_msgs OpenCV drake)

add_executable(trajectory_planner src/OptimizationPlannerNode.cpp src/TrajectoryPlanner.cpp src/GraphPlanner.cpp src/SemanticMap.cpp src/BoundaryMap.cpp src/CDT.cpp src/Planner.cpp src/UTM.cpp src/read_data.cpp src/GenerateSWM.cpp src/A*.cpp src/MaverickPlanner.cpp)
target_link_libraries(trajectory_planner ${GMP_LIBRARIES})
ament_target_dependencies(trajectory_planner rclcpp std_msgs sa_msgs std_srvs geometry_msgs visualization_msgs nav_msgs nav2_msgs OpenCV drake swri_transform_util CGAL)

target_include_directories(trajectory_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(TARGETS 
  trajectory_planner
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
# install(PROGRAMS
#   scripts/A_star.py
#   scripts/PublishPath.py
#   scripts/VisualizeSWM.py
#   scripts/GraphDisplayNode.py
#   scripts/EvaluateCostMap.py
#   scripts/MaverickNode.py
#   scripts/MaverickPlanner.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch/config files.
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)


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
