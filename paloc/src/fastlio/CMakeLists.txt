CMAKE_MINIMUM_REQUIRED(2.8)


#if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -fopenmp=libomp")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fopenmp=libomp")
#elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -fopenmp")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fopenmp")
#endif ()
#
## for fastlio
#add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
#find_package(PythonLibs REQUIRED)
#find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
#
#find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#        geometry_msgs
#        nav_msgs
#        sensor_msgs
#        roscpp
#        rospy
#        rosbag
#        std_msgs
#        tf
#        )
#
#
#AUX_SOURCE_DIRECTORY(. CURRENT_DIR_SRCS)
#ADD_LIBRARY(module_odomertry ${CURRENT_DIR_SRCS})
#TARGET_LINK_LIBRARIES(module_odomertry ${FASTLIO_LIBS})

