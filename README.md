<!--
SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)

SPDX-License-Identifier: CC-BY-NC-ND-4.0
-->

[![experimental](http://badges.github.io/stability-badges/dist/experimental.svg)](http://github.com/badges/stability-badges)

> This work is ongoing, and I plan to add support for additional services in the future. Feel free to share any features you’d like to see included! You can reach me via sebastian dot schirmer at dlr dot de or open an issue.
# RTLola-Ros2-Monitor

ROS 2 adapter for CISPA's RTLola monitoring interpreter.

The adapter assumes a running ROS 2 workspace, i.e., `ros2 topic list` returns a list of topics that are currently executed. The adapter will then automatically generate RTLola input stream binding for all available topic data. If a topic called `RTLolaOut` is an available interace, i.e. `ros2 interface list`, it will also generate a binding to the respective RTLola outputs. 

For more RTLola information see [here](https://rtlola.cispa.de/).

A comprehensive RTLola tutorial can be found [here](https://rtlola.cispa.de/playground/tutorial).

## Table of Content
- [RTLola-Ros2-Monitor](#rtlola-ros2-monitor)
  - [Table of Content](#table-of-content)
  - [Getting Started](#getting-started)
  - [Contributors](#contributors)
  - [Contributing](#contributing)
  - [Changes](#changes)
  - [License](#license)



## Getting Started
- source ROS 2 
- `cd <path_repo>/test_ws`
- `colcon build`
- `source install/setup.bash`
- `ros2 run tester_publisher tester`
- `cd <path_repo>/monitor`
- fill out [ros2_config.toml](monitor/ros2_config.toml):
  - `generate_file = true` if new bindings shall be generated
  - `local_setup_script` should point to the ROS 2 overlay you want to generate bindings for
- `cargo run -- ../specs/test_spec.lola`


## Contributors
- Sebastian Schirmer
  
## Contributing

Please see [the contribution guidelines](CONTRIBUTING.md) for further information about how to contribute.

## Changes

Please see the [Changelog](CHANGELOG.md) for notable changes of the material.

## License

Please see the file [LICENSE.md](LICENSE.md) for further information about how the content is licensed.
