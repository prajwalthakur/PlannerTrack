cmake_minimum_required(VERSION 3.8)
project(scenarios)

find_package(ament_cmake REQUIRED)

# Each subdirectory is a self-contained scenario (launch/params/map). Install
# them wholesale, preserving structure, so a scenario's own launch file can
# locate its own params/map via get_package_share_directory('scenarios').
install(
  DIRECTORY ground_vehicle_racing industrial_yard intersection drone_racing
  DESTINATION share/${PROJECT_NAME}
)

# Also install the waypoints publisher script as a real `ros2 run`-able
# executable (the DIRECTORY install above only copies it under share/,
# which ros2 run doesn't look in).
install(PROGRAMS
  ground_vehicle_racing/scripts/global_waypoints_publisher.py
  intersection/scripts/reset_state.py
  intersection/scripts/generate_route_graph.py
  intersection/scripts/generate_lane_graph.py
  intersection/scripts/publish_lane_markers.py
  intersection/scripts/publish_route_graph_markers.py
  drone_racing/scripts/course_publisher.py
  drone_racing/scripts/camera_sensor_test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# intersection/ has its own global_waypoints_publisher.py (a different
# script -- ComputeRoute action client + trajectory generator, not the
# racing scenario's JSON-raceline reader) that happens to share a basename
# with the one above; RENAME on install avoids the two colliding at the
# same lib/scenarios/global_waypoints_publisher.py destination.
install(PROGRAMS
  intersection/scripts/global_waypoints_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME intersection_global_waypoints_publisher.py
)

ament_package()
