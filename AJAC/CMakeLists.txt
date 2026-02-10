cmake_minimum_required(VERSION 3.5)
project(USB2CAN_Linux)

set(CMAKE_CXX_STANDARD 14)

set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

include_directories(${PROJECT_SOURCE_DIR}/driver)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#find_package(Qt5 COMPONENTS
#        Core
#        Gui
#        Widgets
#        PrintSupport
#        REQUIRED)

set(CMAKE_CXX_STANDARD 17)
include_directories(${INC_DIR} usr_app/AJAC)
link_directories(${LINK_DIR})

add_executable(MOTOR_STEP_SWEEP usr_app/motor_step_sweep.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_STEP_SWEEP libusb-1.0.so pthread lcm)

add_executable(MOTOR_CTRL usr_app/motor_ctrl.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_CTRL libusb-1.0.so pthread lcm)

add_executable(MOTOR_TOOL usr_app/motor_tools.cpp driver/beast_usb2can.cpp)
target_link_libraries(MOTOR_TOOL libusb-1.0.so pthread lcm)

add_executable(AJAC usr_app/motor_ajac_ctrl.cpp driver/beast_usb2can.cpp usr_app/AJAC/AJAC.cpp)
target_link_libraries(AJAC libusb-1.0.so pthread lcm)

add_executable(LCM_READER usr_app/lcm_reader.cpp)
target_link_libraries(LCM_READER lcm)

#add_executable(PLOT usr_app/qt_plot.cpp
#        plot/mainwindow.cpp
#        plot/mainwindow.h
#        form/mainwindow.ui
#        plot/qcustomplot.h
#        plot/qcustomplot.cpp driver/beast_usb2can.cpp driver/beast_usb2can.h)
#target_link_libraries(PLOT
#        Qt5::Core
#        Qt5::Gui
#        Qt5::Widgets
#        Qt5::PrintSupport
#        libusb-1.0.so pthread lcm
#)
