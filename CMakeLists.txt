cmake_minimum_required(VERSION 3.8)
project(my_tb3_world)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGrid.srv"
  "srv/UpdateTile.srv"
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  worlds
  srv
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  route_runner.py
  mux.py
  sensor_node.py
  plant_monitor.py
  grid_node.py
  navigation_node.py
  twin_core.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()