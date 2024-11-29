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
        pub mod generate_rtloladata;
        pub mod generate_sink_error;
    }
}

use build_src::config::Config;
use build_src::ros2_reader::Ros2Reader;
use build_src::rust_file_generator::RustFileGenerator;
use std::fs::{self};

fn main() {
    let config = Config::new("ros2_config.toml");
    if config.generate_file {
        let ros2_reader: Ros2Reader = Ros2Reader::new(
            config.blacklist.clone(),
            config.get_local_setup_script().clone(),
        );
        // Overwrite folders
        println!("{}", config.dest_path_generation);
        fs::create_dir_all(format!("{}/", config.dest_path_generation)).unwrap();
        fs::create_dir_all(format!("{}/input/", config.dest_path_generation)).unwrap();
        fs::create_dir_all(format!("{}/output/", config.dest_path_generation)).unwrap();

        // Inputs
        let subscribable_topics: Vec<(String, String)> = ros2_reader.get_subscribable_topics();
        // Outputs
        let rtlolaout_topic = ros2_reader.get_rtlola_output_msg();
        // let rtlolaout_service: Option<bool> = None; //ros2_reader.get_rtlola_output_srv();
        // Generate rust code for inputs and outputs
        let mut rust_generator = RustFileGenerator::new(ros2_reader, config.dest_path_generation);
        rust_generator.generate_file_inputs(&subscribable_topics);
        rust_generator.generate_file_main(&subscribable_topics);
        rust_generator.generate_file_ros2handler(&subscribable_topics);
        rust_generator.generate_file_rtloladata(&subscribable_topics);
        rust_generator.generate_file_rtlolaout_publisher(&rtlolaout_topic);
        rust_generator.generate_sink_error();
    };
    println!("end");
}
