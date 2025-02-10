cmake_minimum_required(VERSION 3.10)
project(kinematics_library VERSION 0.1 DESCRIPTION "A framework for handling serial robot kinematics")

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(TARGET_NAME ${PROJECT_NAME})

set(NLOPT_DIR "/usr/lib/x86_64-linux-gnu/cmake/nlopt")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

# pkg_check for required dependencies
include(FindPkgConfig)
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(BASE_TYPES REQUIRED base_types)
pkg_check_modules(BASE_LOGGING REQUIRED base_logging)

message(STATUS "BASE_TYPES_LIBRARIES: ${BASE_TYPES_LIBRARIES}")
message(STATUS "BASE_TYPES_LIBRARY_DIRS: ${BASE_TYPES_LIBRARY_DIRS}")

# Include directories for dependencies
set(KINEMATIC_LIB_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OROCOS_KDL_INCLUDE_DIRS}
    ${KDL_PARSER_INCLUDE_DIRS}
    ${URDF_INCLUDE_DIRS}
    ${URDFHEADER_INCLUDE_DIRS}
    ${BASE_TYPES_INCLUDE_DIRS}
    ${BASE_LOGGING_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Set libraries to link with
set(KINEMATIC_DEP_LIBS
    Threads::Threads
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${OROCOS_KDL_LIBRARIES}
    ${KDL_PARSER_LIBRARIES}
    ${URDF_LIBRARIES}
    ${URDFHEADER_LIBRARIES}
    ${BASE_TYPES_LIBRARIES}
    ${BASE_LOGGING_LIBRARIES}
)

# Sources for the library
set(SRCS
    ${PROJECT_SOURCE_DIR}/src/abstract/AbstractKinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/HandleKinematicConfig.cpp
    ${PROJECT_SOURCE_DIR}/src/abstract/IkFastSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/abstract/KdlSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/abstract/KinematicsHelper.cpp
    ${PROJECT_SOURCE_DIR}/src/solver/shimizu_method/SRSKinematicSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/solver/shimizu_method/SRSKinematicHelper.cpp
    ${PROJECT_SOURCE_DIR}/src/solver/asfour_method/IK7DoFSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/KinematicsFactory.cpp
)

# Optional dependencies
pkg_check_modules(TRAC_IK trac_ik)
if(TRAC_IK_FOUND)
    message("TRAC_IK found!")
    add_definitions(-DTRAC_IK_LIB_FOUND=1)
    list(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/abstract/TracIkSolver.cpp)
    list(APPEND KINEMATIC_DEP_LIBS trac_ik)
else()
    message("TRAC_IK not found!")
endif()

pkg_check_modules(NLOPT nlopt)
if(NLOPT_FOUND)
    message("NLopt found!")
    add_definitions(-DOPT_LIB_FOUND=1)
    list(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/ProblemFormulation.cpp)
    list(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/OptSolver.cpp)
    list(APPEND SRCS ${PROJECT_SOURCE_DIR}/src/solver/optimization_method/HybridIkSolver.cpp)
    list(APPEND KINEMATIC_DEP_LIBS nlopt)
else()
    message("NLOPT not found!")
    message("Optimization-based kinematics libraries (OptIK & HybridIK) won't be built")
endif()

# Build the library
add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${KINEMATIC_DEP_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${KINEMATIC_LIB_INCLUDES})

# Package configuration
list(APPEND PKGCONFIG_REQUIRES base_types base_logging orocos_kdl kdl_parser urdfdom urdfdom_headers)
list(JOIN PKGCONFIG_REQUIRES " " PKGCONFIG_REQUIRES_STR)
configure_file("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.pc.in" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc" @ONLY)

# Install
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

# Test executable
add_executable(test_kuka test/test_kuka.cpp)
target_include_directories(test_kuka PRIVATE ${KINEMATIC_LIB_INCLUDES})
target_link_libraries(test_kuka PRIVATE ${PROJECT_NAME} ${KINEMATIC_DEP_LIBS})

add_executable(test_vispa test/test_vispa.cpp)
target_include_directories(test_vispa PRIVATE ${KINEMATIC_LIB_INCLUDES})
target_link_libraries(test_vispa PRIVATE ${PROJECT_NAME} ${KINEMATIC_DEP_LIBS})
