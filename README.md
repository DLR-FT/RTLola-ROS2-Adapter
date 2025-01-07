<!--
SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)

SPDX-License-Identifier: CC-BY-NC-ND-4.0
-->

[![experimental](http://badges.github.io/stability-badges/dist/experimental.svg)](http://github.com/badges/stability-badges)

> This work is ongoing, and I plan to add features in the future. Feel free to share any requests! You can reach me via sebastian dot schirmer at dlr dot de or open an issue.
# RTLola-Ros2-Monitor

ROS 2 adapter for CISPA's RTLola monitoring interpreter.

The adapter assumes a running ROS 2 workspace, i.e., `ros2 topic list` returns a list of topics that are currently executed. The adapter will then automatically generate RTLola input stream binding for all available topic data. If a topic called `RTLolaOutput` is an available interace, i.e., `RTLolaOutput` is returned by `ros2 interface list`, it will also generate a binding to the respective RTLola outputs. 

> [ros2_config.toml](monitor/ros2_config.toml) allows you to freeze the binding by setting `generate_file` to `false`.

> Members of a ros2 topic are prefixed by the package name, i.e., ``<package_name_lowercase>__<member>``. For instance, if the topic ``Adc`` has a member ``float32 a0``, then this member can be accessed by the RTLola specification using ``input adc__a0: Float32``.
 
For more RTLola information see [here](https://rtlola.cispa.de/).

A comprehensive RTLola tutorial can be found [here](https://rtlola.cispa.de/playground/tutorial).

> Note that `build.rs` runs only if there are changes detected by Rust. Hence, it wont update due to changes in the ROS workspace. If so, just change spacing in `build.rs`.  

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
