# ASBR Integration

This repo provides a clean integration for the ASBR package, including URDF descriptions and visualizations for various robotic components. The integration includes support for Universal Robots, Robotiq grippers, and RealSense cameras. Please be aware this integration is still currently under development.

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

- **asbr_description**: Contains URDF and description files specific to the ASBR course.
- **ur_description**: Includes URDF and description files for Universal Robots.
- **robotiq_2f_85_gripper_visualization**: Provides URDF and visualization files for the Robotiq 2F-85 gripper.
- **realsense2_description**: Contains URDF and description files for Intel RealSense cameras.

## Launching the Demo

To visualize the integrated demo in RViz, use the xml launch file I created by running the below command:

```
ros2 launch asbr_description display_asbr_ur5e.launch.xml
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For questions or issues, please open an issue on GitHub or contact Seyi R. Afolayan directly.

---

This README file provides the necessary steps to set up and run the ASBR demo integration. For further details or updates, refer to the repository: [ASBR Integration](https://github.com/Seyi-roboticist/ASBR_integration-.git).
Further work will include gazebo, ROS2 control and much more. 
