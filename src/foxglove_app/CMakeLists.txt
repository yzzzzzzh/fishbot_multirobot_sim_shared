cmake_minimum_required(VERSION 3.8)
project(fishbot_foxglove)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch layouts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
