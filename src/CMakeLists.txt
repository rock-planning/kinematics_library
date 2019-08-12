project(kinematics_library)

SET(TARGET_NAME ${PROJECT_NAME})
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)
pkg_check_modules(YAML REQUIRED yaml-cpp)

#store external library name
set (EXTERNAL_LIB ${kdl_parser_LIBRARIES})

# assign the available planner's path
set (KINEMATIC_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include)

SET (SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/abstract/AbstractKinematics.cpp	
    ${CMAKE_CURRENT_LIST_DIR}/HandleKinematicConfig.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/abstract/IkFastSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/abstract/KdlSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/abstract/KinematicsHelper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver/shimizu_method/SRSKinematicSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver/shimizu_method/SRSKinematicHelper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/KinematicsFactory.cpp )

# if trac_ik is found, add it to the kinematic factory
set(TRAC_IK_LIB_FOUND false)  #This variable need to be defined here, so that C++ complier knows about it.
pkg_check_modules(TRAC_IK trac_ik)

if(${TRAC_IK_FOUND})
    set(TRAC_IK_LIB_FOUND ${TRAC_IK_FOUND})
    message ("TRAC_IK found !")
    message("TRAC_IK dir ${TRAC_IK_INCLUDE_DIRS}")
    message(${TRAC_IK_LIBRARY_DIRS})
    message(${TRAC_IK_LIBRARIES})
    include_directories(${KINEMATIC_INCLUDE_PATH} ${TRAC_IK_INCLUDE_DIRS})
    link_directories(${TRAC_IK_LIBRARY_DIRS})
    LIST(APPEND EXTERNAL_LIB ${TRAC_IK_LIBRARIES}) 
    LIST(APPEND SRCS ${CMAKE_CURRENT_LIST_DIR}/abstract/TracIkSolver.cpp)
else(${TRAC_IK_FOUND})
    message ("TRAC_IK not found !")
endif()

add_definitions(-DTRAC_IK_LIB_FOUND=${TRAC_IK_LIB_FOUND})
message("trac_ik = ${TRAC_IK_LIB_FOUND}")
#message (${SRCS})

include_directories( ${KINEMATIC_INCLUDE_PATH} ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS})

add_library(kinematics_library SHARED ${SRCS})
target_link_libraries(kinematics_library ${EXTERNAL_LIB} ${CMAKE_DL_LIBS} ${YAML_LIBRARIES})

set(TARGET_NAME ${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/kinematics_library)
install(TARGETS kinematics_library DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 
