cmake_minimum_required(VERSION 3.0.2)
project(eigen_example)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(eigen_example src/eigen_example.cpp)
add_dependencies(eigen_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eigen_example
        ${catkin_LIBRARIES}
        )