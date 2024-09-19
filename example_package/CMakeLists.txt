cmake_minimum_required(VERSION 3.5)
project(example_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # For Python nodes
find_package(rclcpp REQUIRED) # For C++ nodes

# Define the directory where Python scripts (executables) are stored
set(SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# Function to find all Python scripts in the "scripts" directory
file(GLOB_RECURSE PYTHON_SCRIPTS RELATIVE ${SCRIPTS_DIR} ${SCRIPTS_DIR}/*.py)

# Iterate over each Python script and install them as executables
foreach(PYTHON_SCRIPT ${PYTHON_SCRIPTS})
  get_filename_component(SCRIPT_NAME ${PYTHON_SCRIPT} NAME_WE)
  install(PROGRAMS ${SCRIPTS_DIR}/${PYTHON_SCRIPT}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# Define the directory where C++ source files are stored
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Function to find all C++ source files in the "src" directory
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${SRC_DIR} ${SRC_DIR}/*.cpp)

# Iterate over each C++ source file and create executables
foreach(CPP_SOURCE ${CPP_SOURCES})
  get_filename_component(CPP_NAME ${CPP_SOURCE} NAME_WE)
  add_executable(${CPP_NAME} ${SRC_DIR}/${CPP_SOURCE})
  target_link_libraries(${CPP_NAME} rclcpp)  # Link against rclcpp
  install(TARGETS ${CPP_NAME}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# Install other resources (e.g., launch)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
