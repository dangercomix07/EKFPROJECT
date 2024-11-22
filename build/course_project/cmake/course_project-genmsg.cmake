# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "course_project: 2 messages, 0 services")

set(MSG_I_FLAGS "-Icourse_project:/home/ameya/EKFPROJECT/src/course_project/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(course_project_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_custom_target(_course_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "course_project" "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" ""
)

get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_custom_target(_course_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "course_project" "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" "course_project/Landmark"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/course_project
)
_generate_msg_cpp(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg"
  "${MSG_I_FLAGS}"
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/course_project
)

### Generating Services

### Generating Module File
_generate_module_cpp(course_project
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/course_project
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(course_project_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(course_project_generate_messages course_project_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_dependencies(course_project_generate_messages_cpp _course_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_dependencies(course_project_generate_messages_cpp _course_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(course_project_gencpp)
add_dependencies(course_project_gencpp course_project_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS course_project_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/course_project
)
_generate_msg_eus(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg"
  "${MSG_I_FLAGS}"
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/course_project
)

### Generating Services

### Generating Module File
_generate_module_eus(course_project
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/course_project
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(course_project_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(course_project_generate_messages course_project_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_dependencies(course_project_generate_messages_eus _course_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_dependencies(course_project_generate_messages_eus _course_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(course_project_geneus)
add_dependencies(course_project_geneus course_project_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS course_project_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/course_project
)
_generate_msg_lisp(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg"
  "${MSG_I_FLAGS}"
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/course_project
)

### Generating Services

### Generating Module File
_generate_module_lisp(course_project
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/course_project
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(course_project_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(course_project_generate_messages course_project_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_dependencies(course_project_generate_messages_lisp _course_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_dependencies(course_project_generate_messages_lisp _course_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(course_project_genlisp)
add_dependencies(course_project_genlisp course_project_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS course_project_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/course_project
)
_generate_msg_nodejs(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg"
  "${MSG_I_FLAGS}"
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/course_project
)

### Generating Services

### Generating Module File
_generate_module_nodejs(course_project
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/course_project
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(course_project_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(course_project_generate_messages course_project_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_dependencies(course_project_generate_messages_nodejs _course_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_dependencies(course_project_generate_messages_nodejs _course_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(course_project_gennodejs)
add_dependencies(course_project_gennodejs course_project_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS course_project_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project
)
_generate_msg_py(course_project
  "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg"
  "${MSG_I_FLAGS}"
  "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project
)

### Generating Services

### Generating Module File
_generate_module_py(course_project
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(course_project_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(course_project_generate_messages course_project_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Landmark.msg" NAME_WE)
add_dependencies(course_project_generate_messages_py _course_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ameya/EKFPROJECT/src/course_project/msg/Trilateration.msg" NAME_WE)
add_dependencies(course_project_generate_messages_py _course_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(course_project_genpy)
add_dependencies(course_project_genpy course_project_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS course_project_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/course_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/course_project
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(course_project_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/course_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/course_project
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(course_project_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/course_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/course_project
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(course_project_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/course_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/course_project
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(course_project_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/course_project
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(course_project_generate_messages_py std_msgs_generate_messages_py)
endif()
