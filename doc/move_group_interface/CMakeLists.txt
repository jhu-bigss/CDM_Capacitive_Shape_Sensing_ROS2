add_executable(move_group_interface
    src/move_group_interface.cpp)
target_include_directories(move_group_interface
    PUBLIC include)
ament_target_dependencies(move_group_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS move_group_interface
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
