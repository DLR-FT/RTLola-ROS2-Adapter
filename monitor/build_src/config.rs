// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use serde::Deserialize;

#[derive(Deserialize, Debug)]
pub struct Config {
    pub generate_file: bool,
    local_setup_script: Vec<String>,
    pub blacklist: Vec<String>,
    pub dest_path_generation: String,
}

impl Config {
    pub fn new(config_str: &str) -> Config {
        let content = std::fs::read_to_string(config_str).unwrap();
        let config: Config = toml::from_str(&content).unwrap();
        config
    }

    /// Returns the location of the local setup script used for ros2
    pub fn get_local_setup_script(&self) -> String {
        let local_setup_script: String = if self.local_setup_script.is_empty() {
            panic!(
                "You need to specify the location of your ./install/setup.bash that should be sourced!"
            );
        } else {
            self.local_setup_script.join(" ")
        };
        local_setup_script
    }
}
