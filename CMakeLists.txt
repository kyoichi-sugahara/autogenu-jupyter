cmake_minimum_required(VERSION 3.14)
project(cgmres CXX)

# ## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include/)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_auto_package()
