cmake_minimum_required(VERSION 2.8.3)
project(sensor_models)

# FIND LIBRARIES
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf visualization_msgs sensor_msgs kdl_parser std_msgs geometry_msgs genmsg moveit_ros_move_group wrap_object)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})


set(KALMAN_INCLUDE /home/guillaume/CppWorkSpace/kalman_filter/include)


# CREATE MESSAGES

add_service_files(FILES FingerIK_cmd.srv String_cmd.srv Parameter_cmd.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


# Declare catkin package
catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
    message_runtime
    urdf
    kdl_parser
  INCLUDE_DIRS include
)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-but-set-variable -Wno-reorder")        ## Optimize
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
endif()

find_library(LAPACK liblapack.so.3 PATHS /usr/lib/lapack/ )
find_library(BLAS   libblas.so.3 PATHS /usr/lib/ )

if(LAPACK)
  message("Found Lapack")
    message(STATUS "${Green} liblapack found ${ColourReset} "            )
    message(STATUS "    ${LAPACK}"                                       )
endif()

if(BLAS)
  message("Found Lapack")
    message(STATUS "${Green} libblas found ${ColourReset} "            )
    message(STATUS "    ${BLAS}"                                       )
endif()


find_package(Armadillo)

if(ARMADILLO_FOUND)
    message(STATUS "${Green} libarmadillo found ${ColourReset} "            )
    message(STATUS "    ${ARMADILLO_INCLUDE_DIRS}"                          )
    message(STATUS "    ${ARMADILLO_LIBRARIES}"                             )
elseif(ARMADILLO_FOUND)
    message(FATAL_ERROR "${Red} libarmadillo NOT FOUND! ${ColourReset}"     )
endif(ARMADILLO_FOUND)

#find_library(LIB_INTEL mkl_rt PATHS /usr/local/lib/intel64/ REQUIRED)

#if(LIB_INTEL)
#    message(STATUS "${Green} mkl_rt found ${ColourReset}"                   )
#    message(STATUS "    ${LIB_INTEL}"                                       )
#else()
#    message(FATAL_ERROR "${Red} mkl_rt NOT FOUND! ${ColourReset}"           )
#endif()

set(STATISTICS_LIB      /home/guillaume/CppWorkSpace/Statistics/lib/libstatistics.so)
set(STATISTICS_INCLUDE  /home/guillaume/CppWorkSpace/Statistics/include)


set(mrff_dir    /home/guillaume/CppWorkSpace/markvo_random_field_filter)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(${INCLUDE_DIR}
                    ${mrff_dir}/include
                    ${catkin_INCLUDE_DIRS}
                    ${ARMADILLO_INCLUDE_DIRS}
                    ${STATISTICS_INCLUDE}
                    ${KALMAN_INCLUDE}
)

set(HAND_MODEL_SRC ${SRC_DIR}/hand/hand_model.cpp
                   ${SRC_DIR}/hand/finger_filter.cpp
                   ${SRC_DIR}/hand/visualisation.cpp
                   ${SRC_DIR}/hand/marker_listener.cpp
                   ${SRC_DIR}/hand/hand_listener.cpp
                   ${SRC_DIR}/hand/hand_filter.cpp
                   ${SRC_DIR}/hand/hand_broadcaster.cpp
                   ${SRC_DIR}/hand/kinematics.cpp
)

set(HAND_MODEL_INC ${INCLUDE_DIR}/hand/hand_model.h
                   ${INCLUDE_DIR}/hand/finger_filter.h
                   ${INCLUDE_DIR}/hand/visualisation.h
                   ${INCLUDE_DIR}/hand/marker_listener.h
                   ${INCLUDE_DIR}/hand/kinematics.h
                   ${INCLUDE_DIR}/hand/hand_listener.h
                   ${INCLUDE_DIR}/hand/hand_filter.h
                   ${INCLUDE_DIR}/hand/hand_broadcaster.h
                   ${INCLUDE_DIR}/hand/types.h
                                                                )


add_library(hand_model ${HAND_MODEL_SRC} ${HAND_MODEL_INC}      )
target_link_libraries(hand_model ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} 
                     ${LAPACK} ${BLAS} ${STATISTICS_LIB} mlpack kdl_parser ${orocos_kdl_LIBRARIES})

add_library(point_model ${SRC_DIR}/point/point_model.cpp
                        ${INCLUDE_DIR}/point/point_model.h      )
target_link_libraries(point_model ${catkin_LIBRARIES} )

set(HEADER_FILES ${INCLUDE_DIR}/ros_communication.h)
set(SRC_FILES    ${SRC_DIR}/ros_communication.cpp
                 ${SRC_DIR}/run.cpp    )

add_executable(sensor_models ${SRC_FILES} ${HEADER_FILES}                       )
target_link_libraries(sensor_models ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} ${LAPACK} ${BLAS} ${STATISTICS_LIB} hand_model point_model  )


add_executable(hand_node  ${SRC_DIR}/hand/run.cpp                               )
target_link_libraries(hand_node ${catkin_LIBRARIES} hand_model                  )



