cmake_minimum_required(VERSION 3.14)

option(CROSSCOMPILE_ARM "Cross compiling for arm architecture" OFF)


if(CROSSCOMPILE_ARM)
    if(DEFINED ENV{LINUX_ARM_TOOLCHAIN_PATH})
        set (LINUX_ARM_TOOLCHAIN_PATH $ENV{LINUX_ARM_TOOLCHAIN_PATH})
    else()
        set (LINUX_ARM_TOOLCHAIN_PATH /usr/lib/ccache)
    endif()
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm_build.cmake)
endif()

project(GeometryTools VERSION 0.1.0)

add_compile_options(-Wall -Wextra -pedantic -Werror)
set (CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(GeometryTools STATIC src/Point.cpp src/Speed.cpp src/Trajectory.cpp)
target_link_libraries(GeometryTools PUBLIC Eigen3::Eigen)
target_include_directories(GeometryTools PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
        $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib)
)
set_target_properties(GeometryTools PROPERTIES PUBLIC_HEADER "include/GeometryTools/Point.h;include/GeometryTools/Speed.h;include/GeometryTools/Trajectory.h")
### Install

install(TARGETS GeometryTools
        EXPORT GeometryToolsTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
        PUBLIC_HEADER DESTINATION include/GeometryTools

)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    GeometryToolsConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT GeometryToolsTargets
        FILE GeometryToolsTargets.cmake
        NAMESPACE GeometryTools::
        DESTINATION lib/cmake/GeometryTools
)

configure_file(cmake/GeometryToolsConfig.cmake.in GeometryToolsConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/GeometryToolsConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/GeometryToolsConfigVersion.cmake"
        DESTINATION lib/cmake/GeometryTools
        )