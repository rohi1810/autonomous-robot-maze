cmake_minimum_required(VERSION 3.20)
project(ClionAutonomous C)

set(CMAKE_C_STANDARD 11)

# Tell CMake where Webots is installed
set(WEBOTS_HOME "E:/AutonomousRobot/Webots")
# Base directory for your controller
set(CONTROLLER_DIR "F:/Ai/Master/Autonomous_Robot/Maze1/controllers/rosbot")

# Webots path
set(WEBOTS_HOME "E:/AutonomousRobot/Webots")

include_directories(
        "${WEBOTS_HOME}/include/controller/c"
        "${CMAKE_SOURCE_DIR}/src"  # Add src folder to include path
)

link_directories(
        "${WEBOTS_HOME}/lib/controller"
)

# Collect all source files from src directory
file(GLOB SOURCES "src/*.c")
file(GLOB HEADERS "src/*.h")

# Create executable
add_executable(rosbot ${SOURCES} ${HEADERS})

# Link Webots controller library
target_link_libraries(rosbot Controller)