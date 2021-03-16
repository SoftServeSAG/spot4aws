cmake_minimum_required(VERSION 2.8.3)
project(rs_tests)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(
  DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})