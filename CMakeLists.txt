cmake_minimum_required(VERSION 2.8.3)
project(triggerbox)


find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
    FILES
    AOutConfirm.msg
    AOutRaw.msg
    AOutVolts.msg
    TriggerClockMeasurement.msg
    TriggerClockModel.msg
)

add_service_files(
    FILES
    SetFramerate.srv
)

catkin_python_setup()

generate_messages()

catkin_package(CATKIN_DEPENDS message_generation message_runtime)

catkin_install_python(
    PROGRAMS
    nodes/triggerbox_client
    nodes/triggerbox_host
    scripts/set-framerate
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)



