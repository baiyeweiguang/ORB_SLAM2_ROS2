cmake_minimum_required(VERSION 3.10)
project(orb_slam2_ros2)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Build ORBSLAM2
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(BOOST_SERIALIZATION boost_serialization)
include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/CMakeLists.txt)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam2/lib)
add_library(orb_slam2 SHARED
orb_slam2/src/System.cc
orb_slam2/src/Tracking.cc
orb_slam2/src/LocalMapping.cc
orb_slam2/src/LoopClosing.cc
orb_slam2/src/ORBextractor.cc
orb_slam2/src/ORBmatcher.cc
orb_slam2/src/FrameDrawer.cc
orb_slam2/src/Converter.cc
orb_slam2/src/MapPoint.cc
orb_slam2/src/KeyFrame.cc
orb_slam2/src/Map.cc
orb_slam2/src/Optimizer.cc
orb_slam2/src/PnPsolver.cc
orb_slam2/src/Frame.cc
orb_slam2/src/KeyFrameDatabase.cc
orb_slam2/src/Sim3Solver.cc
orb_slam2/src/Initializer.cc
)
target_include_directories(orb_slam2 PUBLIC 
${OpenCV_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/orb_slam2
${PROJECT_SOURCE_DIR}/orb_slam2/include
)
target_link_libraries(orb_slam2 
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
${BOOST_SERIALIZATION}
)
add_dependencies (orb_slam2 g2o DBoW2
)
install(
  TARGETS orb_slam2
  DESTINATION lib
)
install(
  FILES 
  ${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so 
  ${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
  DESTINATION lib
)


# Build ros2 node
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC 
include
${OpenCV_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}/orb_slam2
${PROJECT_SOURCE_DIR}/orb_slam2/include
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} orb_slam2)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ORB_SLAM2::MonoNode
  EXECUTABLE mono_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  )
