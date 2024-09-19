# Reusable CMakeLists.txt for ROS2 Python and C++ Packages

This repository provides a solution for the common challenges encountered when developing ROS2 packages: managing `CMakeLists.txt` and `setup.py`. If you’ve ever worked with building both Python and C++ executables, you know how tricky it can be to configure these files correctly.

## Why This Exists

We wanted to eliminate the repetitive work of manually setting up both `CMakeLists.txt` for C++ and `setup.py` for Python. The goal? Create a **single** `CMakeLists.txt` file that handles everything. Whether you’re working with Python nodes, C++ nodes, or additional resources like URDFs and launch files, this `CMakeLists.txt` can manage it all.

This approach allows you to focus on building your ROS2 applications without worrying about the build setup.

## Features

- Automatically detects and installs **Python executables** from the `scripts/` directory.
- Automatically builds and installs **C++ executables** from the `src/` directory.
- Handles all additional resources like **launch files**, **URDFs**, **meshes**, and **config files**.
- Reusable across multiple ROS2 packages with minimal changes.
- Example package included to demonstrate how to use the `CMakeLists.txt`.

## How to Use

### 1. Clone the Repository
Clone this repository and use the `CMakeLists.txt` for your ROS2 package:

```bash
git clone https://github.com/SAKErobotics/ros2_reusable_cmakelists.git
```

### 2. Copy the `CMakeLists.txt` into Your ROS2 Package

Place this `CMakeLists.txt` in the root of your ROS2 package. Ensure your package follows this structure:
```
your_package/
├── CMakeLists.txt       # The reusable CMakeLists.txt from this repository
├── package.xml          # Your package metadata
├── scripts/             # Python executables (nodes)
├── src/                 # C++ source files (nodes)
├── launch/              # Optional launch files
├── urdf/                # Optional URDF files
└── config/              # Optional config files
```

### 3. Customize the `package.xml`
Modify the `package.xml` in your ROS2 package to include the correct package metadata and dependencies for your nodes.

### 4. Build and Test
Build your package using `colcon`:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Run your nodes:

- For Python nodes: `ros2 run your_package your_python_node`
- For C++ nodes: `ros2 run your_package your_cpp_node`

### Example Package
An example ROS2 package is provided in the `example_package/` directory that demonstrates how to use the `CMakeLists.txt`. You can explore this example for guidance.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Contributing

Feel free to submit issues or pull requests if you find any bugs or have suggestions for improvements!
