# Littlebot RQT Plugin

The `littlebot_rqt_plugin` package provides a graphical interface for monitoring and interacting with the Littlebot differential-drive robot through ROS 2.

The plugin is implemented in C++ using the ROS 2 `rqt_gui_cpp` framework and Qt5. It provides a graphical interface for hardware connection management, robot status monitoring, and wheel velocity visualization.

## Features

- Serial device discovery and selection.
- Hardware connection and disconnection management.
- Wheel status monitoring.
- Real-time wheel velocity visualization.
- Velocity setpoint input through the graphical interface.
- ROS 2 publisher and subscriber interfaces for exchanging messages with other Littlebot components.
- Protocol and error message visualization.

Some functionalities are still under development. In particular, the velocity command path and plot data export require further implementation.

## Architecture

The plugin is organized into three main components:

| Component | Responsibility |
|---|---|
| `LittlebotRqtPlugin` | Integrates the graphical interface with the ROS 2 rqt framework and manages ROS 2 communication. |
| `LittlebotGui` | Implements the graphical interface, user interactions, status displays, and plots. |
| `LittlebotComm` | Handles communication with the robot hardware through the `littlebot_base` interfaces. |

The graphical interface is designed using Qt Designer (`ui/littlebot_gui.ui`).


## Building

The package is intended to be built inside a ROS 2 workspace using `colcon`.

First, source the ROS 2 environment:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

Install the package dependencies using `rosdep` from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the plugin:

```bash
colcon build --packages-select littlebot_rqt_plugin
```

Source the workspace:

```bash
source install/setup.bash
```

## Running

Launch the plugin using its ROS 2 launch file:

```bash
ros2 launch littlebot_rqt_plugin littlebot_rqt_plugin.launch.py
```

Alternatively, run it through the rqt framework:

```bash
ros2 run littlebot_rqt_plugin littlebot_rqt_plugin
```

## Contributing

Contributions are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting changes.

## License

This project is distributed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for details.

