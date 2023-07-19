add_executable(moveKuka src/moveKuka.cpp)
target_link_libraries(moveKuka ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveKuka DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

