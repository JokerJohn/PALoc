cmake_minimum_required(VERSION 3.0.2)
project(paloc)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PALOC_BUILD_FASTLIO "Build PALoc's bundled FAST-LIO frontend" OFF)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN -DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN -DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()

find_package(OpenMP QUIET)
if (OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif ()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        rosbag
        std_msgs
        std_srvs
        tf
        message_generation
        )

set(PALOC_OPEN3D_HINT_DIR "" CACHE PATH
        "Optional hint path to Open3D CMake config dir (directory containing Open3DConfig.cmake)")
set(_PALOC_OPEN3D_CATKIN_DIR "/home/xchu/xicp_ws/devel/.private/open3d_catkin/lib/cmake/Open3D")
set(_PALOC_VERIFIED_OPEN3D_EXAMPLE "${_PALOC_OPEN3D_CATKIN_DIR}")

set(_paloc_open3d_candidates "")

macro(_paloc_append_open3d_candidate _cand)
    if (NOT "${_cand}" STREQUAL "")
        list(APPEND _paloc_open3d_candidates "${_cand}")
    endif ()
endmacro()

if (DEFINED Open3D_DIR AND NOT "${Open3D_DIR}" STREQUAL "")
    _paloc_append_open3d_candidate("${Open3D_DIR}")
endif ()
if (DEFINED ENV{Open3D_DIR} AND NOT "$ENV{Open3D_DIR}" STREQUAL "")
    _paloc_append_open3d_candidate("$ENV{Open3D_DIR}")
endif ()
_paloc_append_open3d_candidate("${_PALOC_OPEN3D_CATKIN_DIR}")
if (NOT "${PALOC_OPEN3D_HINT_DIR}" STREQUAL "")
    _paloc_append_open3d_candidate("${PALOC_OPEN3D_HINT_DIR}")
endif ()
if (DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    _paloc_append_open3d_candidate("$ENV{CONDA_PREFIX}/lib/cmake/Open3D")
endif ()
if (DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    file(GLOB _paloc_open3d_anaconda_dirs LIST_DIRECTORIES true "$ENV{HOME}/anaconda3/envs/*/lib/cmake/Open3D")
    foreach (_cand IN LISTS _paloc_open3d_anaconda_dirs)
        _paloc_append_open3d_candidate("${_cand}")
    endforeach ()
    file(GLOB _paloc_open3d_miniconda_dirs LIST_DIRECTORIES true "$ENV{HOME}/miniconda3/envs/*/lib/cmake/Open3D")
    foreach (_cand IN LISTS _paloc_open3d_miniconda_dirs)
        _paloc_append_open3d_candidate("${_cand}")
    endforeach ()
endif ()
_paloc_append_open3d_candidate("/usr/local/lib/cmake/Open3D")
file(GLOB _paloc_open3d_usr_dirs LIST_DIRECTORIES true "/usr/lib*/cmake/Open3D")
foreach (_cand IN LISTS _paloc_open3d_usr_dirs)
    _paloc_append_open3d_candidate("${_cand}")
endforeach ()
_paloc_append_open3d_candidate("/usr/lib/x86_64-linux-gnu/cmake/Open3D")

set(_paloc_open3d_dir "")
foreach (_cand IN LISTS _paloc_open3d_candidates)
    if (EXISTS "${_cand}/Open3DConfig.cmake" OR EXISTS "${_cand}/open3d-config.cmake")
        set(_paloc_open3d_dir "${_cand}")
        break()
    endif ()
endforeach ()

if ("${_paloc_open3d_dir}" STREQUAL "")
    message(FATAL_ERROR
            "[paloc] Failed to locate Open3D package config.\n"
            "Set one of:\n"
            "  -DOpen3D_DIR=<dir containing Open3DConfig.cmake>\n"
            "  -DPALOC_OPEN3D_HINT_DIR=<fallback dir>\n"
            "Verified example on this machine:\n"
            "  ${_PALOC_VERIFIED_OPEN3D_EXAMPLE}")
endif ()

set(Open3D_DIR "${_paloc_open3d_dir}" CACHE PATH "Path to Open3DConfig.cmake" FORCE)
message(STATUS "[paloc] Using Open3D_DIR: ${Open3D_DIR}")

get_filename_component(_paloc_open3d_prefix "${Open3D_DIR}/../../.." ABSOLUTE)
set(_paloc_fmt_dir "${_paloc_open3d_prefix}/lib/cmake/fmt")
if (EXISTS "${_paloc_fmt_dir}/fmt-config.cmake")
    set(fmt_DIR "${_paloc_fmt_dir}" CACHE PATH "Path to fmt-config.cmake" FORCE)
endif ()

find_package(fmt QUIET)
find_package(Open3D REQUIRED CONFIG)
find_package(Boost REQUIRED COMPONENTS filesystem regex timer date_time program_options)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM 4.2.0 REQUIRED QUIET)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp rospy rosbag std_msgs std_srvs tf message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIRS}
)

set(PALOC_PGO_SOURCES
        src/paloc/data_saver.cpp
        src/paloc/paloc.cpp
        src/paloc/cloud_process.cpp
        include/base_type.cpp
        )

add_executable(pgo_localization ${PALOC_PGO_SOURCES})
target_link_libraries(pgo_localization
        ${catkin_LIBRARIES}
        ${Open3D_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        gtsam
        )
target_include_directories(pgo_localization PRIVATE ${PYTHON_INCLUDE_DIRS})

if (PALOC_BUILD_FASTLIO)
    add_executable(fastlio
            include/fast_lio/ikd-Tree/ikd_Tree.cpp
            src/fastlio/laserMapping.cpp
            src/fastlio/preprocess.cpp)
    target_link_libraries(fastlio
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${PYTHON_LIBRARIES}
            )
    target_include_directories(fastlio PRIVATE ${PYTHON_INCLUDE_DIRS})
endif ()

catkin_install_python(PROGRAMS
        scripts/convert_gt_tum_to_fs_frame.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/run_office_mid360_gt.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
