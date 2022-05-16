cmake_minimum_required(VERSION 2.8.3)
project(icm_tactile_sensor_air)


add_executable(${PROJECT_NAME} src/icm_tactile_sensor_air.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} multimodal_tactile_sensor_generate_messages_cpp)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

