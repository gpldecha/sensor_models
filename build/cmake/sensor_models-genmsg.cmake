# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sensor_models: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sensor_models_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv" NAME_WE)
add_custom_target(_sensor_models_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_models" "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv" ""
)

get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv" NAME_WE)
add_custom_target(_sensor_models_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_models" "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv" ""
)

get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv" NAME_WE)
add_custom_target(_sensor_models_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sensor_models" "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models
)
_generate_srv_cpp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models
)
_generate_srv_cpp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models
)

### Generating Module File
_generate_module_cpp(sensor_models
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sensor_models_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sensor_models_generate_messages sensor_models_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_cpp _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_cpp _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_cpp _sensor_models_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_models_gencpp)
add_dependencies(sensor_models_gencpp sensor_models_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_models_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models
)
_generate_srv_lisp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models
)
_generate_srv_lisp(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models
)

### Generating Module File
_generate_module_lisp(sensor_models
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sensor_models_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sensor_models_generate_messages sensor_models_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_lisp _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_lisp _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_lisp _sensor_models_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_models_genlisp)
add_dependencies(sensor_models_genlisp sensor_models_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_models_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models
)
_generate_srv_py(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models
)
_generate_srv_py(sensor_models
  "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models
)

### Generating Module File
_generate_module_py(sensor_models
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sensor_models_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sensor_models_generate_messages sensor_models_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/Parameter_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_py _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/String_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_py _sensor_models_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/sensor_models/srv/FingerIK_cmd.srv" NAME_WE)
add_dependencies(sensor_models_generate_messages_py _sensor_models_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sensor_models_genpy)
add_dependencies(sensor_models_genpy sensor_models_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sensor_models_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sensor_models
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sensor_models_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(sensor_models_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sensor_models
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sensor_models_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(sensor_models_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sensor_models
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sensor_models_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(sensor_models_generate_messages_py geometry_msgs_generate_messages_py)
