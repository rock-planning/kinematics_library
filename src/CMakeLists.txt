project(kinematics_library)

SET(TARGET_NAME ${PROJECT_NAME})
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)

pkg_check_modules(BASE_TYPES REQUIRED base-types)
pkg_search_module(BASE_LOGGING REQUIRED base-logging)
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

set(KINEMATIC_LIB_INCLUDES ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${BASE_TYPES_INCLUDE_DIRS} 
                           ${BASE_LOGGING_INCLUDE_DIRS} ${OROCOS_KDL_INCLUDE_DIRS} ${KDL_PARSER_INCLUDE_DIRS} ${URDF_INCLUDE_DIRS} 
                           ${URDFHEADER_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS})

set(KINEMATIC_LIB_LINK_DIRS ${BASE_TYPES_LIBRARY_DIRS} ${BASE_LOGGING_LIBRARY_DIRS} ${OROCOS_KDL_LIBRARY_DIRS}
                            ${KDL_PARSER_LIBRARY_DIRS} ${URDF_LIBRARY_DIRS} 
                            ${URDFHEADER_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS})

set(KINEMATIC_LIB_DEP_LIBS ${BASE_TYPES_LIBRARIES} ${BASE_LOGGING_LIBRARIES} ${OROCOS_KDL_LIBRARIES}
                           ${KDL_PARSER_LIBRARIES} ${URDF_LIBRARIES} ${URDFHEADER_LIBRARIES} ${YAMLCPP_LIBRARIES})

include_directories(${KINEMATIC_LIB_INCLUDES})

link_directories(${KINEMATIC_LIB_LINK_DIRS})

SET (SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/abstract/AbstractKinematics.cpp	
    ${CMAKE_CURRENT_LIST_DIR}/HandleKinematicConfig.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/abstract/IkFastSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/abstract/KdlSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/abstract/KinematicsHelper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver/shimizu_method/SRSKinematicSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver/shimizu_method/SRSKinematicHelper.cpp    
    ${CMAKE_CURRENT_LIST_DIR}/solver/asfour_method/IK7DoFSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/KinematicsFactory.cpp )

# if trac_ik is found, add it to the kinematic factory
set(TRAC_IK_LIB_FOUND false)  #This variable need to be defined here, so that C++ complier knows about it.
pkg_check_modules(TRAC_IK trac_ik)

if(${TRAC_IK_FOUND})
    set(TRAC_IK_LIB_FOUND ${TRAC_IK_FOUND})
    message ("TRAC_IK found !")
    include_directories(${KINEMATIC_LIB_INCLUDES} ${TRAC_IK_INCLUDE_DIRS})
    link_directories(${TRAC_IK_LIBRARY_DIRS})
    LIST(APPEND KINEMATIC_LIB_DEP_LIBS ${TRAC_IK_LIBRARIES}) 
    LIST(APPEND SRCS ${CMAKE_CURRENT_LIST_DIR}/abstract/TracIkSolver.cpp)
    list(APPEND PKGCONFIG_REQUIRES trac_ik)
else(${TRAC_IK_FOUND})
    message ("TRAC_IK not found !")
endif()

add_definitions(-DTRAC_IK_LIB_FOUND=${TRAC_IK_LIB_FOUND})
#message("trac_ik = ${TRAC_IK_LIB_FOUND}")

# if nlopt is found, add it to the kinematic factory
set(OPT_LIB_FOUND false)  #This variable need to be defined here, so that C++ complier knows about it.
pkg_check_modules(NLOPT nlopt)

if(${NLOPT_FOUND})
    set(OPT_LIB_FOUND ${NLOPT_FOUND})
    message ("NLopt found !")
    include_directories(${KINEMATIC_LIB_INCLUDES} ${NLOPT_INCLUDE_DIRS})
    link_directories(${NLOPT_LIBRARY_DIRS})
    LIST(APPEND KINEMATIC_LIB_DEP_LIBS ${NLOPT_LIBRARIES}) 
    LIST(APPEND SRCS ${CMAKE_CURRENT_LIST_DIR}/solver/optimization_method/ProblemFormulation.cpp)
    LIST(APPEND SRCS ${CMAKE_CURRENT_LIST_DIR}/solver/optimization_method/OptSolver.cpp)
    LIST(APPEND SRCS ${CMAKE_CURRENT_LIST_DIR}/solver/optimization_method/HybridIkSolver.cpp)
    list(APPEND PKGCONFIG_REQUIRES nlopt)
else(${NLOPT_FOUND})
    message ("NLOPT not found !")
    message ("-------------------------------------------------------------------------")
    message (" ")
    message ("Optimization-based kinematics libraries (OptIK & HybridIK) won't be build")
    message (" ")
    message ("-------------------------------------------------------------------------")
endif()

add_definitions(-DOPT_LIB_FOUND=${OPT_LIB_FOUND})
#message("nlopt = ${OPT_LIB_FOUND}")


add_library(kinematics_library SHARED ${SRCS})
target_link_libraries(kinematics_library ${KINEMATIC_LIB_DEP_LIBS} ${CMAKE_DL_LIBS})

set(TARGET_NAME ${PROJECT_NAME})
set(PROJECT_DESCRIPTION "A framework for handling serial robot kinematics")
# now we add all the required libraries in DEPS_PKGCONFIG macro
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES orocos_kdl)
list(APPEND PKGCONFIG_REQUIRES kdl_parser)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
list(APPEND PKGCONFIG_REQUIRES urdfdom_headers)
list(APPEND PKGCONFIG_REQUIRES yaml-cpp)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/kinematics_library DESTINATION include)
install(TARGETS kinematics_library DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 
