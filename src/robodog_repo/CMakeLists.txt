cmake_minimum_required(VERSION 3.5)
project(br2_robodog)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz worlds
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/walk.py scripts/walk_cmd_vel.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
