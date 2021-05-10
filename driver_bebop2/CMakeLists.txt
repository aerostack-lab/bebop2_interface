cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME driverBebopROSModule)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake

add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)



set(DRIVER_BEBOP_SOURCE_DIR
	src/source)
	
set(DRIVER_BEBOP_INCLUDE_DIR
	src/include
	)

set(DRIVER_BEBOP_SOURCE_FILES
        src/source/droneOuts.cpp
        src/source/droneInps.cpp

	)
	
set(DRIVER_BEBOP_HEADER_FILES
        src/include/droneOuts.h
        src/include/droneInps.h

	)

find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs bebop_msgs sensor_msgs droneModuleROS droneMsgsROS lib_cvgutils image_transport)



catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs bebop_msgs sensor_msgs droneModuleROS droneMsgsROS lib_cvgutils image_transport
  )


include_directories(${DRIVER_BEBOP_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(driverBebopROSModule ${DRIVER_BEBOP_SOURCE_FILES} ${DRIVER_BEBOP_HEADER_FILES})
add_dependencies(driverBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(driverBebopROSModule ${catkin_LIBRARIES})


add_executable(batteryBebopROSModule src/source/droneBatteryNode.cpp)
add_dependencies(batteryBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(batteryBebopROSModule driverBebopROSModule)
target_link_libraries(batteryBebopROSModule ${catkin_LIBRARIES})

add_executable(rotationAnglesBebopROSModule src/source/droneRotationAnglesNode.cpp)
add_dependencies(rotationAnglesBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotationAnglesBebopROSModule driverBebopROSModule)
target_link_libraries(rotationAnglesBebopROSModule ${catkin_LIBRARIES})

add_executable(altitudeBebopROSModule src/source/droneAltitudeNode.cpp)
add_dependencies(altitudeBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(altitudeBebopROSModule driverBebopROSModule)
target_link_libraries(altitudeBebopROSModule ${catkin_LIBRARIES})

add_executable(frontCameraBebopROSModule src/source/droneFrontCameraNode.cpp)
add_dependencies(frontCameraBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontCameraBebopROSModule driverBebopROSModule)
target_link_libraries(frontCameraBebopROSModule ${catkin_LIBRARIES})

add_executable(bottomCameraBebopROSModule src/source/droneBottomCameraNode.cpp)
add_dependencies(bottomCameraBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(bottomCameraBebopROSModule driverBebopROSModule)
target_link_libraries(bottomCameraBebopROSModule ${catkin_LIBRARIES})


add_executable(commandBebopROSModule  src/source/droneCommandNode.cpp)
add_dependencies(commandBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(commandBebopROSModule driverBebopROSModule)
target_link_libraries(commandBebopROSModule ${catkin_LIBRARIES})

add_executable(statusBebopROSModule src/source/droneStatusNode.cpp)
add_dependencies(statusBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(statusBebopROSModule driverBebopROSModule)
target_link_libraries(statusBebopROSModule ${catkin_LIBRARIES})

add_executable(groundSpeedBebopROSModule src/source/droneGroundSpeedNode.cpp)
add_dependencies(groundSpeedBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(groundSpeedBebopROSModule driverBebopROSModule)
target_link_libraries(groundSpeedBebopROSModule ${catkin_LIBRARIES})

add_executable(gimbalBebopROSModule src/source/droneGimbalNode.cpp)
add_dependencies(gimbalBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(gimbalBebopROSModule driverBebopROSModule)
target_link_libraries(gimbalBebopROSModule ${catkin_LIBRARIES})

add_executable(wifiChannelBebopROSModule src/source/droneWifiChannelNode.cpp)
add_dependencies(wifiChannelBebopROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifiChannelBebopROSModule driverBebopROSModule)
target_link_libraries(wifiChannelBebopROSModule ${catkin_LIBRARIES})



add_executable(imuFreqIncreased src/source/imuFreqIncreased.cpp)
add_dependencies(imuFreqIncreased ${catkin_EXPORTED_TARGETS})
target_link_libraries(imuFreqIncreased ${catkin_LIBRARIES})

#add_executable(pressureBebopROSModule src/source/pressureNode.cpp)
#add_dependencies(pressureBebopROSModule ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pressureBebopROSModule driverBebopROSModule)
#target_link_libraries(pressureBebopROSModule ${catkin_LIBRARIES})


#add_executable(magnetometerBebopROSModule src/source/magnetometerNode.cpp)
#add_dependencies(magnetometerBebopROSModule ${catkin_EXPORTED_TARGETS})
#target_link_libraries(magnetometerBebopROSModule driverBebopROSModule)
#target_link_libraries(magnetometerBebopROSModule ${catkin_LIBRARIES})



#add_executable(temperatureBebopROSModule src/source/temperatureNode.cpp)
#add_dependencies(temperatureBebopROSModule ${catkin_EXPORTED_TARGETS})
#target_link_libraries(temperatureBebopROSModule driverBebopROSModule)
#target_link_libraries(temperatureBebopROSModule ${catkin_LIBRARIES})


