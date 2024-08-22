cmake_minimum_required(VERSION 3.16)
project(kinematics_library
    VERSION 0.1
    DESCRIPTION "A framework for handling serial robot kinematics"
)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(DEPENDENCIES
    orocos_kdl
    kdl_parser
    urdfdom
    urdfdom_headers
    yaml-cpp
    base-logging
    base-types
)

set (SRCS 
    ${PROJECT_SOURCE_DIR}/src/abstract/AbstractKinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/HandleKinematicConfig.cpp 
    ${PROJECT_SOURCE_DIR}/src/abstract/IkFastSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/abstract/KdlSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/abstract/KinematicsHelper.cpp
    ${PROJECT_SOURCE_DIR}/src/solver/shimizu_method/SRSKinematicSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/solver/shimizu_method/SRSKinematicHelper.cpp    
    ${PROJECT_SOURCE_DIR}/src/solver/asfour_method/IK7DoFSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/KinematicsFactory.cpp )

# if trac_ik is found, add it to the kinematic factory
pkg_check_modules(TRAC_IK trac_ik)
if(${TRAC_IK_FOUND})
    message ("TRAC_IK found !")
    add_definitions(-DTRAC_IK_LIB_FOUND=1)

    LIST(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/abstract/TracIkSolver.cpp)
    list(APPEND DEPENDENCIES trac_ik)
else(${TRAC_IK_FOUND})
    message ("TRAC_IK not found !")
endif()

# if nlopt is found, add it to the kinematic factory
pkg_check_modules(NLOPT nlopt)
if(${NLOPT_FOUND})
    message ("NLopt found !")
    add_definitions(-DOPT_LIB_FOUND=1)

    LIST(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/ProblemFormulation.cpp)
    LIST(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/OptSolver.cpp)
    LIST(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/HybridIkSolver.cpp)
    list(APPEND DEPENDENCIES nlopt)
else(${NLOPT_FOUND})
    message ("NLOPT not found !")
    message ("-------------------------------------------------------------------------")
    message (" ")
    message ("Optimization-based kinematics libraries (OptIK & HybridIK) won't be build")
    message (" ")
    message ("-------------------------------------------------------------------------")
endif()

pkg_check_modules(Dependencies REQUIRED IMPORTED_TARGET ${DEPENDENCIES})

# Build the library
add_library(${TARGET_NAME} SHARED ${SRCS})

target_link_libraries(${TARGET_NAME}
    PUBLIC
        PkgConfig::Dependencies
        Threads::Threads
        Eigen3::Eigen
        ${CMAKE_DL_LIBS}
)

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

list(JOIN DEPENDENCIES " " PKGCONFIG_REQUIRES)
configure_file("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/kinematics_library DESTINATION include)
install(TARGETS ${TARGET_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

# Build the test
add_executable(test_kinematics_library
    test/test_kinematics_library.cpp
)

target_link_libraries(test_kinematics_library
   ${TARGET_NAME}
)

