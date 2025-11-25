cmake_minimum_required(VERSION 3.20)
project(ClionAutonomous C)

set(CMAKE_C_STANDARD 11)

# Tell CMake where Webots is installed
set(WEBOTS_HOME "E:/AutonomousRobot/Webots")
# Base directory for your controller
set(CONTROLLER_DIR "F:/Ai/Master/Autonomous_Robot/Maze1/controllers/rosbot")

# Webots C API include path
include_directories(
        "${WEBOTS_HOME}/include/controller/c"
        "${CONTROLLER_DIR}/HeaderFile"  # Add header file directory
)

# Webots controller library path
link_directories(
        "${WEBOTS_HOME}/lib/controller"
)

# All modular source files from CodeFile folder
set(SOURCES
        "${CONTROLLER_DIR}/rosbot.c"
        "${CONTROLLER_DIR}/astar.c"
        "${CONTROLLER_DIR}/grid.c"
        "${CONTROLLER_DIR}/costmap.c"
        "${CONTROLLER_DIR}/frontier.c"
        "${CONTROLLER_DIR}/robot.c"
        "${CONTROLLER_DIR}/pid.c"
        "${CONTROLLER_DIR}/display.c"
)

# All header files from HeaderFile folder (for IDE organization)
set(HEADERS
        "${CONTROLLER_DIR}/config.h"
        "${CONTROLLER_DIR}/astar.h"
        "${CONTROLLER_DIR}/grid.h"
        "${CONTROLLER_DIR}/costmap.h"
        "${CONTROLLER_DIR}/frontier.h"
        "${CONTROLLER_DIR}/robot.h"
        "${CONTROLLER_DIR}/pid.h"
        "${CONTROLLER_DIR}/display.h"
)

# Your controller executable with all source files
add_executable(rosbot ${SOURCES} ${HEADERS})

# Link Webots Controller library and math library (for Linux/Mac compatibility)
if(WIN32)
    target_link_libraries(rosbot Controller)
else()
    target_link_libraries(rosbot Controller m)
endif()

# Compiler flags for better code quality and optimization
if(MSVC)
    target_compile_options(rosbot PRIVATE /W4)
else()
    target_compile_options(rosbot PRIVATE -Wall -Wextra -O2)
endif()

# Set output directory (optional - organizes executable)
set_target_properties(rosbot PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Print configuration info for debugging
message(STATUS "==========================================")
message(STATUS "ROSBot Controller Configuration")
message(STATUS "==========================================")
message(STATUS "Webots Home: ${WEBOTS_HOME}")
message(STATUS "Controller Dir: ${CONTROLLER_DIR}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Source Files: ${SOURCES}")
message(STATUS "Header Files: ${HEADERS}")
message(STATUS "==========================================")