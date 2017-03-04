cmake_minimum_required(VERSION 2.8.3)
project(communication)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation )

add_message_files(
        DIRECTORY
        msgs
        FILES
        MicroPhoneData.msg
        MicroPhoneControl.msg
        CameraControl.msg
        LEDControl.msg
        LEDPattern.msg
)

add_service_files(
		DIRECTORY
		srvs
		FILES
		Record.srv
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime message_generation)
