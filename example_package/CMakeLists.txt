cmake_minimum_required(VERSION 3.5)
project(example_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # For Python nodes
find_package(rclcpp REQUIRED) # For C++ nodes

# Auto-detect and install Python executables from both scripts/ and package directories
file(GLOB PYTHON_SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
file(GLOB PYTHON_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/*.py")

# Create a list of all Python files from both locations
set(ALL_PYTHON_FILES ${PYTHON_SCRIPTS_DIR} ${PYTHON_PACKAGE_DIR})

# Iterate over each Python file and install them as executables
foreach(python_file ${ALL_PYTHON_FILES})
    get_filename_component(script_name ${python_file} NAME_WE)
    install(PROGRAMS
        ${python_file}
        DESTINATION lib/${PROJECT_NAME}
        RENAME ${script_name}
    )
endforeach()

# If using Python package organization, install the package
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}")
    ament_python_install_package(${PROJECT_NAME})
endif()

# Define the directory where C++ source files are stored
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Function to find all C++ source files in the "src" directory
file(GLOB_RECURSE CPP_SOURCES RELATIVE ${SRC_DIR} ${SRC_DIR}/*.cpp)

# Iterate over each C++ source file and create executables
foreach(CPP_SOURCE ${CPP_SOURCES})
  get_filename_component(CPP_NAME ${CPP_SOURCE} NAME_WE)
  add_executable(${CPP_NAME} ${SRC_DIR}/${CPP_SOURCE})
  
  # Properly include and link rclcpp
  ament_target_dependencies(${CPP_NAME} rclcpp)
  
  install(TARGETS ${CPP_NAME}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# Install other resources (e.g., launch)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
