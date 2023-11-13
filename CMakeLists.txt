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
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include/)
target_link_libraries(cgmres INTERFACE Eigen3::Eigen)
# set(CGMRES_LIB_HEADERS
#   include/cgmres/detail/continuation_gmres.hpp
#   include/cgmres/detail/continuation_gmres_condensing.hpp
#   include/cgmres/detail/control_input_bounds.hpp
#   include/cgmres/detail/control_input_bounds_shooting.hpp
#   include/cgmres/detail/macros.hpp
#   include/cgmres/detail/matrixfree_gmres.hpp
#   include/cgmres/detail/multiple_shooting_nlp.hpp
#   include/cgmres/detail/newton_gmres.hpp
#   include/cgmres/detail/single_shooting_nlp.hpp
#   include/cgmres/detail/zero_horizon_nlp.hpp
#   include/cgmres/horizon.hpp
#   include/cgmres/integrator.hpp
#   include/cgmres/logger.hpp
#   include/cgmres/multiple_shooting_cgmres_solver.hpp
#   include/cgmres/single_shooting_cgmres_solver.hpp
#   include/cgmres/solver_settings.hpp
#   include/cgmres/timer.hpp
#   include/cgmres/types.hpp
#   include/cgmres/zero_horizon_ocp_solver.hpp
# )
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   ${CGMRES_LIB_HEADERS}
# )

# target_include_directories(${PROJECT_NAME}
#   SYSTEM PUBLIC
#     "${EIGEN3_INCLUDE_DIR}"
# )

ament_auto_package()

# # the interface library
# add_library(${PROJECT_NAME} INTERFACE)
# add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
# target_include_directories(
#   ${PROJECT_NAME} INTERFACE
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
# )
# target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# # cmake configs
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#   ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
#   VERSION 0.1
#   COMPATIBILITY AnyNewerVersion
# )
# configure_package_config_file(
#   ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
#   ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
#   INSTALL_DESTINATION ${CGMRES_INSTALL_CMAKE_DIR}
#   NO_CHECK_REQUIRED_COMPONENTS_MACRO
#   NO_SET_AND_CHECK_MACRO
# )

# # install files
# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}-targets
# )
# install(
#   FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
#   ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
#   DESTINATION ${CGMRES_INSTALL_CMAKE_DIR}
# )
# install(
#   DIRECTORY ${PROJECT_SOURCE_DIR}/include/cgmres
#   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )

# # export
# export(EXPORT ${PROJECT_NAME}-targets
#   FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake
#   NAMESPACE cgmres::
# )
# install(EXPORT ${PROJECT_NAME}-targets
#   NAMESPACE cgmres::
#   DESTINATION ${CGMRES_INSTALL_CMAKE_DIR}
# )
