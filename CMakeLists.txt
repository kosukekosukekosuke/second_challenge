cmake_minimum_required(VERSION 3.8)
project(second_challenge)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/second_challenge.cpp
  src/second_challenge_node.cpp)

install(DIRECTORY
    DESTINATION share/${PROJECT_NAME}/
    )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
