# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "drone_decon: 6 messages, 1 services")

set(MSG_I_FLAGS "-Idrone_decon:/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Imavlink_lora:/home/andkgl/wspace/DronesForBlood/software/gcs/src/mavlink_lora/ros/mavlink_lora/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(drone_decon_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" "drone_decon/GPS"
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" ""
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" ""
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" ""
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" "drone_decon/GPS:drone_decon/UTMDrone"
)

get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_custom_target(_drone_decon_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_decon" "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" "drone_decon/GPS"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg;/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)
_generate_msg_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)

### Generating Services
_generate_srv_cpp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
)

### Generating Module File
_generate_module_cpp(drone_decon
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(drone_decon_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(drone_decon_generate_messages drone_decon_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_cpp _drone_decon_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_decon_gencpp)
add_dependencies(drone_decon_gencpp drone_decon_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_decon_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg;/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)
_generate_msg_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)

### Generating Services
_generate_srv_eus(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
)

### Generating Module File
_generate_module_eus(drone_decon
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(drone_decon_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(drone_decon_generate_messages drone_decon_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_eus _drone_decon_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_decon_geneus)
add_dependencies(drone_decon_geneus drone_decon_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_decon_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg;/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)
_generate_msg_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)

### Generating Services
_generate_srv_lisp(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
)

### Generating Module File
_generate_module_lisp(drone_decon
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(drone_decon_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(drone_decon_generate_messages drone_decon_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_lisp _drone_decon_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_decon_genlisp)
add_dependencies(drone_decon_genlisp drone_decon_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_decon_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg;/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)
_generate_msg_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)

### Generating Services
_generate_srv_nodejs(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
)

### Generating Module File
_generate_module_nodejs(drone_decon
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(drone_decon_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(drone_decon_generate_messages drone_decon_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_nodejs _drone_decon_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_decon_gennodejs)
add_dependencies(drone_decon_gennodejs drone_decon_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_decon_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg;/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)
_generate_msg_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg"
  "${MSG_I_FLAGS}"
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)

### Generating Services
_generate_srv_py(drone_decon
  "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
)

### Generating Module File
_generate_module_py(drone_decon
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(drone_decon_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(drone_decon_generate_messages drone_decon_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/heartbeatDecon.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/srv/takeOffAndLandCheck.srv" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RegisterDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/GPS.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/UTMDroneList.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/andkgl/wspace/DronesForBlood/software/gcs/src/drone_decon/msg/RedirectDrone.msg" NAME_WE)
add_dependencies(drone_decon_generate_messages_py _drone_decon_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_decon_genpy)
add_dependencies(drone_decon_genpy drone_decon_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_decon_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_decon
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(drone_decon_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET mavlink_lora_generate_messages_cpp)
  add_dependencies(drone_decon_generate_messages_cpp mavlink_lora_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_decon
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(drone_decon_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET mavlink_lora_generate_messages_eus)
  add_dependencies(drone_decon_generate_messages_eus mavlink_lora_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_decon
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(drone_decon_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET mavlink_lora_generate_messages_lisp)
  add_dependencies(drone_decon_generate_messages_lisp mavlink_lora_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_decon
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(drone_decon_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET mavlink_lora_generate_messages_nodejs)
  add_dependencies(drone_decon_generate_messages_nodejs mavlink_lora_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_decon
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(drone_decon_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET mavlink_lora_generate_messages_py)
  add_dependencies(drone_decon_generate_messages_py mavlink_lora_generate_messages_py)
endif()
