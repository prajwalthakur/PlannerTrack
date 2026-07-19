cmake_minimum_required(VERSION 3.8)
project(scenarios)

find_package(ament_cmake REQUIRED)

# Each subdirectory is a self-contained scenario (launch/params/map). Install
# them wholesale, preserving structure, so a scenario's own launch file can
# locate its own params/map via get_package_share_directory('scenarios').
install(
  DIRECTORY ground_vehicle_racing industrial_yard
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
