<!--
SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)

SPDX-License-Identifier: CC-BY-NC-ND-4.0
-->

This is ongoing work. If features are missing please let me know, I extent it on-demand.

# RTLolaRos2Monitor

ROS 2 adapter for CISPA's RTLola monitoring interpreter.

For more RTLola information see [here](https://rtlola.cispa.de/).

A comprehensive RTLola tutorialcan be found [here](https://rtlola.cispa.de/playground/tutorial).

# Getting Started
- Call `/install/setup.bash` of your target Ros2 workspace as required by the used [r2r crate](https://github.com/sequenceplanner/r2r)
- Build the Ros2 node using `cargo build`
- Run the Ros2 node `cargo run` with the respective parameters. For more information call `cargo run -- --help`
- Note that for publisher msgs there are two options: [add pub msg to target workspace or create a dedicated workspace](https://github.com/m-dahl/r2r_minimal_node/)`


# Overview
- `/specs` contains all speciifcations
- `/monitor` represents the Ros2 monitor code written in Rust
