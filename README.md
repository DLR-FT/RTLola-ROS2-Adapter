<!--
SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)

SPDX-License-Identifier: CC-BY-NC-ND-4.0
-->

[![experimental](http://badges.github.io/stability-badges/dist/experimental.svg)](http://github.com/badges/stability-badges)

> This work is ongoing, and I plan to add support for additional services in the future. Feel free to share any features you’d like to see included! You can reach me via sebastian dot schirmer at dlr dot de or open an issue.
# RTLola-Ros2-Monitor

ROS 2 adapter for CISPA's RTLola monitoring interpreter.

For more RTLola information see [here](https://rtlola.cispa.de/).

A comprehensive RTLola tutorialcan be found [here](https://rtlola.cispa.de/playground/tutorial).

## Table of Content
- [RTLola-Ros2-Monitor](#rtlola-ros2-monitor)
  - [Table of Content](#table-of-content)
  - [Getting Started](#getting-started)
  - [Contributors](#contributors)
  - [Contributing](#contributing)
  - [Changes](#changes)
  - [License](#license)



## Getting Started
- Call `/install/setup.bash` of your target Ros2 workspace as required by the used [r2r crate](https://github.com/sequenceplanner/r2r)
- Build the Ros2 node using `cargo build`
- Run the Ros2 node `cargo run` with the respective parameters. For more information call `cargo run -- --help`
- Note that for publisher msgs there are two options: [add pub msg to target workspace or create a dedicated workspace](https://github.com/m-dahl/r2r_minimal_node/)`




## Contributors
- Sebastian Schirmer
  
## Contributing

Please see [the contribution guidelines](CONTRIBUTING.md) for further information about how to contribute.

## Changes

Please see the [Changelog](CHANGELOG.md) for notable changes of the material.

## License

Please see the file [LICENSE.md](LICENSE.md) for further information about how the content is licensed.
