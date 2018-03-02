project(kinematics_library)


CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)


INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
#find_package(orocos_kdl REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)



#include_directories(${KINEMATICS_DIR_PATH}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} )

message(${CMAKE_SOURCE_DIR}/include)

add_library(kinematics_library SHARED
            ${CMAKE_CURRENT_LIST_DIR}/abstract/AbstractKinematics.cpp
            ${CMAKE_CURRENT_LIST_DIR}/abstract/IkFastSolver.cpp
            ${CMAKE_CURRENT_LIST_DIR}/abstract/KdlSolver.cpp
            ${CMAKE_CURRENT_LIST_DIR}/KinematicsHelper.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotKinematics.cpp

)

#target_link_libraries(kinematics_library ${kdl_parser_LIBRARY} ${orocos_kdl_LIBRARY})
target_link_libraries(kinematics_library ${kdl_parser_LIBRARY} )



install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/kinematics_library)
install(TARGETS kinematics_library DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 




