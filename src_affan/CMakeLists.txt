cmake_minimum_required(VERSION 3.8)
project(a2_description NONE)  # installs files only; no compiler needed

find_package(ament_cmake REQUIRED)

# A description package has nothing to compile: it only installs files so
# that launch files can find them with get_package_share_directory().
install(
  DIRECTORY urdf launch config worlds paths models
  DESTINATION share/${PROJECT_NAME}
)

# Helper nodes, runnable with `ros2 run a2_description <script>`.
install(
  PROGRAMS scripts/umbmark_driver.py scripts/path_publisher.py scripts/cone_driver.py
           scripts/odometry_node.py scripts/wall_follower.py
           scripts/spin_test.py scripts/cone_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Keep the requested executable name independent of the Python file extension.
install(
  PROGRAMS scripts/odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odometry_node
)
install(
  PROGRAMS scripts/wall_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wall_follower
)

ament_package()
