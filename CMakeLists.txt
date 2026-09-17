cmake_minimum_required(VERSION 3.8)
project(freedom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)

include_directories(
        SYSTEM
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
)


add_executable(freedom_node
        src/freedom.cpp
        src/depth_image.cpp
        src/map.cpp
        src/scanmap.cpp
        src/raycast.cpp
        src/mrmap.cpp
        src/freenode.cpp
        src/visualization.cpp
        src/node.cpp
)

ament_target_dependencies(freedom_node
        rclcpp
        geometry_msgs
        cv_bridge
        PCL
        OpenCV
        pcl_conversions
        sensor_msgs
        tf2_ros
        visualization_msgs
        std_msgs
        tf2_eigen
        tf2_eigen_kdl
)

add_executable(ground_truth_generate src/evaluate_tools/ground_truth_generate.cpp)
ament_target_dependencies(ground_truth_generate
        rclcpp
        geometry_msgs
        PCL
        pcl_conversions
        sensor_msgs
        std_msgs
        tf2_ros
)
target_link_libraries(ground_truth_generate ${PCL_LIBRARIES})

add_executable(static_map_evaluate src/evaluate_tools/static_map_evaluate.cpp)
ament_target_dependencies(static_map_evaluate
        rclcpp
        geometry_msgs
        PCL
)
target_link_libraries(static_map_evaluate ${PCL_LIBRARIES})

install(TARGETS
        freedom_node
        ground_truth_generate
        static_map_evaluate
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
        launch config generated_pcd
        DESTINATION share/${PROJECT_NAME}
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
