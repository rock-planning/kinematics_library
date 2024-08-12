INCLUDE(FindPkgConfig)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)
link_directories(${kdl_parser_LIBRARY_DIRS})

rock_executable(test_kinematics_library test_kinematics_library.cpp
   DEPS kinematics_library)

target_link_libraries(test_kinematics_library
${Boost_FILESYSTEM_LIBRARY}
${Boost_SYSTEM_LIBRARY}
)

#rock_testsuite(test_kinematics_library test_kinematics_library.cpp   
#   DEPS kinematics_library)
