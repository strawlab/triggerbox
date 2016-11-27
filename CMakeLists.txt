cmake_minimum_required(VERSION 2.8)
project(triggerbox)

find_package(catkin REQUIRED message_generation)

add_message_files()
add_service_files()

catkin_python_setup()

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

catkin_install_python(PROGRAMS nodes/triggerbox_client nodes/triggerbox_host
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
