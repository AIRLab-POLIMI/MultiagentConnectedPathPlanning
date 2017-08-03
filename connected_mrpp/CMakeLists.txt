cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp)
project(connected_mrpp)

catkin_package()

# ----------------------- INCLUDES ----------------------------

include_directories(include ${catkin_INCLUDE_DIRS})

# ----------------------- LIBRARIES ---------------------------

find_package(Eigen3 REQUIRED)
find_package(Boost 1.53 REQUIRED COMPONENTS graph)

include_directories(${EIGEN3_INCLUDE_DIR})



# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ------------------------ MESSAGES ----------------------------


# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE cmrpp_SOURCE src/*.cpp)
list(REMOVE_ITEM cmrpp_SOURCE "src/testGeneral.cpp")

add_library(cmrpp_planner ${cmrpp_SOURCE})
                        
target_link_libraries(cmrpp_planner ${catkin_LIBRARIES})     

add_executable(cmrpp_test "src/testGeneral.cpp") 
target_link_libraries(cmrpp_test cmrpp_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})       
                        
                        
# ------------------------ TESTS -------------------------------  
#add_subdirectory(test)                      

# ------------------------ INSTALLATION ------------------------

# Mark executables and/or libraries for installation
install(TARGETS cmrpp_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
# Install headers
install(DIRECTORY include
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

# Install plugin files
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


# Install launch files
install(DIRECTORY launch config maps rviz_config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


# ------------------------ DOCUMENTATION ------------------------
#add_subdirectory(doc EXCLUDE_FROM_ALL)




