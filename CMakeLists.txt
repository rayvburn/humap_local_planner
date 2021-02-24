cmake_minimum_required(VERSION 2.8.3)
project(hubero_local_planner)

# fuzzylite library - external dependency
# path to the .so file
find_library(fuzzylite_LIBRARY fuzzylite $ENV{HOME}/libraries/fuzzylite/fuzzylite/release/bin)
# include fuzzylite's headers folder
include_directories($ENV{HOME}/libraries/fuzzylite/fuzzylite)

find_package(catkin REQUIRED
    COMPONENTS
        base_local_planner
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        nav_core
        nav_msgs
        pluginlib
        pcl_conversions
        roscpp
        tf
        hubero_common
)

set(IGN_MATH_VER 3)
find_package(ignition-math${IGN_MATH_VER} REQUIRED system)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${hubero_common_INCLUDE_DIRS}
    ${IGNITION-MATH_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/HuberoPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hubero_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
)

set(HEADERS_FUZZY
    include/${PROJECT_NAME}/fuzz/regions.h
    include/${PROJECT_NAME}/fuzz/processor.h
    include/${PROJECT_NAME}/fuzz/trapezoid_parted.h
    include/${PROJECT_NAME}/fuzz/trapezoid_loc_indep.h
    include/${PROJECT_NAME}/fuzz/trapezoid_loc_dep.h
    include/${PROJECT_NAME}/fuzz/social_behaviours_db.h
    include/${PROJECT_NAME}/fuzz/social_conductor.h
)
set(SOURCES_FUZZY
    src/fuzz/processor.cpp
    src/fuzz/trapezoid_parted.cpp
    src/fuzz/trapezoid_loc_indep.cpp
    src/fuzz/trapezoid_loc_dep.cpp
    src/fuzz/social_behaviours_db.cpp
    src/fuzz/social_conductor.cpp
)

set(HEADERS_INFLATION
    include/${PROJECT_NAME}/inflation/border.h
    include/${PROJECT_NAME}/inflation/box.h
    include/${PROJECT_NAME}/inflation/circle.h
    include/${PROJECT_NAME}/inflation/ellipse.h
)
set(SOURCES_INFLATION
    src/inflation/border.cpp
    src/inflation/box.cpp
    src/inflation/circle.cpp
    src/inflation/ellipse.cpp
)

set(HEADERS_SFM
    include/${PROJECT_NAME}/sfm/inflator.h
    include/${PROJECT_NAME}/sfm/social_force_model.h
    include/${PROJECT_NAME}/sfm/sfm_debug.h
)
set(SOURCES_SFM
    src/sfm/inflator.cpp
    src/sfm/social_force_model.cpp
    src/sfm/sfm_debug.cpp
)

set(HEADERS_VIS
    include/${PROJECT_NAME}/vis/arrow.h
    include/${PROJECT_NAME}/vis/grid.h
    include/${PROJECT_NAME}/vis/grid_force.h
    include/${PROJECT_NAME}/vis/heat_cell.h
    include/${PROJECT_NAME}/vis/heatmap.h
    include/${PROJECT_NAME}/vis/line_list.h
    include/${PROJECT_NAME}/vis/marker_base.h
    include/${PROJECT_NAME}/vis/text.h
)
set(SOURCES_VIS
    src/vis/arrow.cpp
    src/vis/grid.cpp
    src/vis/grid_force.cpp
    src/vis/heat_cell.cpp
    src/vis/heatmap.cpp
    src/vis/line_list.cpp
    src/vis/marker_base.cpp
    src/vis/text.cpp
)

## Custom libraries
### Fuzzy
add_library(fuzzy ${HEADERS_FUZZY} ${SOURCES_FUZZY})
target_link_libraries(fuzzy ${fuzzylite_LIBRARY})
set_target_properties(fuzzy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
### Inflation
add_library(inflation ${HEADERS_INFLATION} ${SOURCES_INFLATION})
set_target_properties(inflation PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
### SFM
add_library(sfm ${HEADERS_SFM} ${SOURCES_SFM})
set_target_properties(sfm PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
### Visualization
add_library(vis ${HEADERS_VIS} ${SOURCES_VIS})
set_target_properties(vis PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

## Library
add_library(hubero_local_planner src/hubero_planner.cpp src/hubero_planner_ros.cpp)
add_dependencies(hubero_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hubero_local_planner ${catkin_LIBRARIES} fuzzy inflation sfm vis)

## Install
install(TARGETS hubero_local_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
