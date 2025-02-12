# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.16)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(unitree_kinematics)

# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(unitree_kinematics src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/kinematic_state.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(unitree_kinematics
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(unitree_kinematics PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(unitree_kinematics PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(frame_main src/frame_main.cpp)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
target_link_libraries(frame_main unitree_kinematics)

# install the include files by copying the whole include directory
install(DIRECTORY include/unitree_kinematics DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS frame_main unitree_kinematics
        EXPORT unitree_kinematics-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT unitree_kinematics-targets
        FILE unitree_kinematics-config.cmake
        NAMESPACE unitree_kinematics::
        DESTINATION lib/cmake/${PROJECT_NAME})

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(test_unitree_kinematics tests/test_geometry2d.cpp tests/test_se2d.cpp tests/test_svg.cpp tests/test_kinematic_state.cpp)
    target_link_libraries(test_unitree_kinematics Catch2::Catch2WithMain unitree_kinematics) # AnyOtherLibrariesAsNeeded)

    # register the test with CTest, telling it what executable to run
    add_test(NAME tests_for_unitree_kinematics COMMAND test_unitree_kinematics)
endif()
