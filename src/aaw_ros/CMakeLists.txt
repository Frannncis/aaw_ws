cmake_minimum_required(VERSION 2.8.3)
project(aaw_ros)

add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)




#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS}${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
# 或者用下面这种方式使用Eigen库
# include_directories(
#   include ${catkin_INCLUDE_DIRS}
#   "/usr/include/eigen3"
# )

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package

#cs_add_executable(aaw_visualServo src/aaw_visualServo.cpp src/aawvertexesgainer.cpp src/aawibvs.cpp)
cs_add_executable(aaw_visualServo src/aaw_visualServo.cpp src/aawvertexesgainer2.cpp src/aawibvs.cpp)
cs_add_executable(aaw_moveRobotServer src/aaw_moveRobotServer.cpp src/aawtcpserver.cpp src/aawcoordtransform.cpp)
#cs_add_executable(aaw_getDesiredCoordsOnNP src/aaw_getDesiredCoordsOnNP.cpp src/aawvertexesgainer.cpp src/aawibvs.cpp)
cs_add_executable(aaw_getDesiredCoordsOnNP src/aaw_getDesiredCoordsOnNP.cpp src/aawvertexesgainer2.cpp src/aawibvs.cpp)
cs_add_executable(aaw_motorDriverServer_Up src/aaw_motorDriverServer_Up.cpp src/aawserver4motor.cpp)
cs_add_executable(aaw_motorDriverServer_Side src/aaw_motorDriverServer_Side.cpp src/aawserver4motor.cpp)

#cs_add_executable(harris_corners src/harris_corners.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)

target_link_libraries( aaw_visualServo ${OpenCV_LIBS})
target_link_libraries( aaw_moveRobotServer libjsoncpp.a)
target_link_libraries( aaw_getDesiredCoordsOnNP ${OpenCV_LIBS})

cs_install()
cs_export()