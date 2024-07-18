# ASBR Integration

This repository provides a demo integration for the ASBR package, including URDF descriptions and visualizations for various robotic components. The integration includes support for Universal Robots, Robotiq grippers, and RealSense cameras.

## Repository Structure

```
ASBR_integration-/
├── asbr_package/
│ └── asbr_test_ws/
│ ├── src/
│ │ ├── asbr_description/
│ │ │ ├── config/
│ │ │ ├── launch/
│ │ │ ├── rviz/
│ │ │ └── urdf/
│ │ │ ├── CMakeLists.txt
│ │ │ └── package.xml
│ │ ├── realsense2_description/
│ │ │ ├── .github/
│ │ │ ├── launch/
│ │ │ ├── meshes/
│ │ │ ├── rviz/
│ │ │ └── urdf/
│ │ │ ├── CMakeLists.txt
│ │ │ └── package.xml
│ │ ├── robotiq_description/
│ │ │ ├── robotiq/
│ │ │ ├── robotiq_2f_85_gripper_visualization/
│ │ │ │ ├── launch/
│ │ │ │ ├── meshes/
│ │ │ │ └── urdf/
│ │ │ │ ├── robotiq_adapter.xacro
│ │ │ │ ├── robotiq_arg2f_85_model_macro.xacro
│ │ │ │ ├── robotiq_arg2f_85_model.xacro
│ │ │ │ ├── robotiq_arg2f_transmission.xacro
│ │ │ │ └── robotiq_arg2f.xacro
│ │ │ ├── CMakeLists.txt
│ │ │ └── package.xml
│ ├── build/
│ ├── install/
│ └── log/
├── .gitignore
├── CONTRIBUTING.md
├── Intel Copyright
├── LICENSE
├── NOTICE.md
├── README.md
└── security.md


```

## Cloning the Repository

To get started, clone the repository using the following command:

```bash
git clone https://github.com/Seyi-roboticist/ASBR_integration-.git
```

## Building the Package

Navigate to the `asbr_test_ws` workspace and build the selected packages using `colcon`:

```bash
cd asbr_package/asbr_test_ws
colcon build --packages-select asbr_description ur_description robotiq_2f_85_gripper_visualization realsense2_description
```

## Package Descriptions

- **asbr_description**: Contains URDF and description files specific to the ASBR demo.
- **ur_description**: Includes URDF and description files for Universal Robots.
- **robotiq_2f_85_gripper_visualization**: Provides URDF and visualization files for the Robotiq 2F-85 gripper.
- **realsense2_description**: Contains URDF and description files for Intel RealSense cameras.

## Launching the Demo

To visualize the integrated demo in RViz, you can create a launch file that includes the necessary components. Here is an example launch file:

```
ros2 launch asbr_description display_asbr_ur5e.launch.xml
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For questions or issues, please open an issue on GitHub or contact Seyi R. Afolayan directly.

---

This README file provides the necessary steps to set up and run the ASBR demo integration. For further details or updates, refer to the repository: [ASBR Integration](https://github.com/Seyi-roboticist/ASBR_integration-.git).
