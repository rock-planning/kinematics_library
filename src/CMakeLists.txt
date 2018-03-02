rock_library(kinematics_library
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(kinematics_library_bin Main.cpp
    DEPS kinematics_library)

