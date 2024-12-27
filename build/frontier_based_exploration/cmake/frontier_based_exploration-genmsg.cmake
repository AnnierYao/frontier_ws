# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "frontier_based_exploration: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifrontier_based_exploration:/home/annie/frontier_ws/src/frontier_based_exploration/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(frontier_based_exploration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_custom_target(_frontier_based_exploration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "frontier_based_exploration" "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(frontier_based_exploration
  "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frontier_based_exploration
)

### Generating Services

### Generating Module File
_generate_module_cpp(frontier_based_exploration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frontier_based_exploration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(frontier_based_exploration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(frontier_based_exploration_generate_messages frontier_based_exploration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_dependencies(frontier_based_exploration_generate_messages_cpp _frontier_based_exploration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frontier_based_exploration_gencpp)
add_dependencies(frontier_based_exploration_gencpp frontier_based_exploration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frontier_based_exploration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(frontier_based_exploration
  "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frontier_based_exploration
)

### Generating Services

### Generating Module File
_generate_module_eus(frontier_based_exploration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frontier_based_exploration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(frontier_based_exploration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(frontier_based_exploration_generate_messages frontier_based_exploration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_dependencies(frontier_based_exploration_generate_messages_eus _frontier_based_exploration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frontier_based_exploration_geneus)
add_dependencies(frontier_based_exploration_geneus frontier_based_exploration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frontier_based_exploration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(frontier_based_exploration
  "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frontier_based_exploration
)

### Generating Services

### Generating Module File
_generate_module_lisp(frontier_based_exploration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frontier_based_exploration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(frontier_based_exploration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(frontier_based_exploration_generate_messages frontier_based_exploration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_dependencies(frontier_based_exploration_generate_messages_lisp _frontier_based_exploration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frontier_based_exploration_genlisp)
add_dependencies(frontier_based_exploration_genlisp frontier_based_exploration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frontier_based_exploration_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(frontier_based_exploration
  "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frontier_based_exploration
)

### Generating Services

### Generating Module File
_generate_module_nodejs(frontier_based_exploration
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frontier_based_exploration
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(frontier_based_exploration_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(frontier_based_exploration_generate_messages frontier_based_exploration_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_dependencies(frontier_based_exploration_generate_messages_nodejs _frontier_based_exploration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frontier_based_exploration_gennodejs)
add_dependencies(frontier_based_exploration_gennodejs frontier_based_exploration_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frontier_based_exploration_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(frontier_based_exploration
  "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frontier_based_exploration
)

### Generating Services

### Generating Module File
_generate_module_py(frontier_based_exploration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frontier_based_exploration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(frontier_based_exploration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(frontier_based_exploration_generate_messages frontier_based_exploration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/annie/frontier_ws/src/frontier_based_exploration/msg/PointArray.msg" NAME_WE)
add_dependencies(frontier_based_exploration_generate_messages_py _frontier_based_exploration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frontier_based_exploration_genpy)
add_dependencies(frontier_based_exploration_genpy frontier_based_exploration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frontier_based_exploration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frontier_based_exploration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frontier_based_exploration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(frontier_based_exploration_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(frontier_based_exploration_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frontier_based_exploration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frontier_based_exploration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(frontier_based_exploration_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(frontier_based_exploration_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frontier_based_exploration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frontier_based_exploration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(frontier_based_exploration_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(frontier_based_exploration_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frontier_based_exploration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frontier_based_exploration
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(frontier_based_exploration_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(frontier_based_exploration_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frontier_based_exploration)
  install(CODE "execute_process(COMMAND \"/home/annie/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frontier_based_exploration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frontier_based_exploration
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(frontier_based_exploration_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(frontier_based_exploration_generate_messages_py geometry_msgs_generate_messages_py)
endif()
