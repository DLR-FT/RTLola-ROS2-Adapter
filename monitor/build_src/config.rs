// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use serde::Deserialize;

#[allow(non_snake_case)]
#[derive(Debug, Deserialize)]
pub struct Config {
    pub GenerateFlag: GenerateFlag,
    pub LocalSetupScript: LocalSetupScript,
    pub QoS_Default: QoS,
    pub QoS_RTLolaOutput: QoS,
    pub QoS_RTLolaService: QoS,
    pub Blacklist: Blacklist,
    pub Destination: Destination,
}

#[derive(Debug, Deserialize)]
pub struct GenerateFlag {
    pub generate: bool,
}

#[derive(Debug, Deserialize)]
pub struct LocalSetupScript {
    pub local_setup_script: Vec<String>,
}

#[derive(Debug, Deserialize)]
pub struct QoS {
    pub history: String,
    pub depth: u32,
    pub reliability: String,
    pub durability: String,
    pub deadline_ms: u64,
    pub lifespan_ms: u64,
    pub liveliness: String,
    pub liveliness_lease_duration_ms: u64,
    pub avoid_ros_namespace_conventions: bool,
}

#[derive(Debug, Deserialize)]
pub struct Blacklist {
    pub topics: Vec<String>,
}

#[derive(Debug, Deserialize)]
pub struct Destination {
    pub path: String,
}

impl Config {
    pub fn new(config_str: &str) -> Config {
        let content = std::fs::read_to_string(config_str).unwrap();
        let config: Config = toml::from_str(&content).unwrap();
        config
    }

    /// Returns the location of the local setup script used for ros2
    pub fn get_local_setup_script(&self) -> String {
        let local_setup_script: String = if self.LocalSetupScript.local_setup_script.is_empty() {
            panic!(
                "You need to specify the location of your ./install/setup.bash that should be sourced!"
            );
        } else {
            self.LocalSetupScript.local_setup_script.join(" ")
        };
        local_setup_script
    }
}
