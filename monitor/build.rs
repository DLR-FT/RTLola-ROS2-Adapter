// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

mod build_src {
    pub mod config;
    pub mod ros2_reader;
    pub mod rust_file_generator;
    pub mod fn_generator {
        pub mod generate_inputs;
        pub mod generate_main;
        pub mod generate_outputs;
        pub mod generate_ros2_handler;
        pub mod generate_ros2_service;
        pub mod generate_rtloladata;
        pub mod generate_sink_error;
        pub mod generate_transformations;
    }
}

use build_src::config::Config;
use build_src::ros2_reader::Ros2Reader;
use build_src::rust_file_generator::RustFileGenerator;
use std::fs::{self};
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    // Ensure that build.rs is always executed
    let start = SystemTime::now();
    let since_the_epoch = start
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards");

    println!("cargo:rerun-if-changed=build.rs");
    println!(
        "cargo:rerun-if-changed=force_build_{}",
        since_the_epoch.as_secs()
    );
    // Start building
    let config = Config::new("ros2_config.toml");
    if config.generate_file {
        let ros2_reader: Ros2Reader = Ros2Reader::new(
            config.blacklist.clone(),
            config.get_local_setup_script().clone(),
        );
        // Overwrite folders
        fs::create_dir_all(format!("{}/", config.dest_path_generation)).unwrap();
        fs::create_dir_all(format!("{}/input/", config.dest_path_generation)).unwrap();
        fs::create_dir_all(format!("{}/output/", config.dest_path_generation)).unwrap();

        // Inputs
        let subscribable_topics: Vec<(String, String)> = ros2_reader.get_subscribable_topics();
        // Outputs
        let rtlolaout_topic = ros2_reader.get_rtlola_output_msg();
        let rtlolaout_service = ros2_reader.get_rtlola_service();
        // Generate rust code for inputs and outputs
        let mut rust_generator = RustFileGenerator::new(ros2_reader, config.dest_path_generation);
        rust_generator.generate_file_inputs(&subscribable_topics, &rtlolaout_service);
        rust_generator.generate_file_main(&subscribable_topics, rtlolaout_service.is_some());
        rust_generator.generate_file_ros2handler(&subscribable_topics, rtlolaout_service.is_some());
        rust_generator.generate_file_rtloladata(&subscribable_topics, &rtlolaout_service);
        rust_generator.generate_file_transformations();
        rust_generator.generate_file_rtlolaout_publisher(&rtlolaout_topic);
        rust_generator.generate_file_rtlolaout_service(&rtlolaout_service);
        rust_generator.generate_sink_error();
    };
}
