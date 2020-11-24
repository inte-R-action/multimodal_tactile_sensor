cmake_minimum_required(VERSION 2.8.3)
project(icm_tactile_sensor_force)

add_executable(${PROJECT_NAME} src/icm_tactile_sensor_force.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

