cmake_minimum_required(VERSION 3.10.2)
project(panda_gazebo_motion_planning_python_api)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

install(PROGRAMS
  scripts/motion_planning_python_api_tutorial.py
  scripts/motion_planning_python_api_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
	)

ament_package()
