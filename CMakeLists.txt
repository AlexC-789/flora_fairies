cmake_minimum_required(VERSION 3.8)
project(my_tb3_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

