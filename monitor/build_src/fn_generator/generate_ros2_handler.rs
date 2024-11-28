// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use crate::build_src::rust_file_generator::RustFileGenerator;
use std::fs::File;
use std::io::{Read, Write};

impl RustFileGenerator {
    pub fn generate_file_ros2handler(&self, topics: &Vec<(String, String)>) {
        // File that is generated
        let file_location = format!("{}/ros2_handler.rs", self.dest_path);
        let file = File::create(&file_location).unwrap();
        // Statements to be added depending on ros2 readings
        let mut rtlola_enum = String::new();
        let mut ros2_msgs = String::new();
        let mut subscribers = String::new();
        let mut select_statement = String::new();
        // Build statements
        for (topic, _) in topics {
            let parts: Vec<&str> = topic.split("/").collect();
            let topic_name = parts.last().unwrap();
            rtlola_enum.push_str(&format!(
                "{}::RTLola{topic_name},",
                topic_name.to_lowercase()
            ));
            ros2_msgs.push_str(&format!("{topic_name},"));
            subscribers.push_str(&format!(
                "let {}_subscriber = node
            .subscribe::<{topic_name}>(\"{topic}\", QosProfile::default())?
            .map(|d| RTLolaData::{topic_name}(RTLola{topic_name}::from(d)));",
                topic_name.to_lowercase()
            ));
            select_statement.push_str(&format!("{}_subscriber,", topic_name.to_lowercase()));
        }
        // Read the contents of the file into a String
        let mut file_content = String::new();
        let crate_root = std::env::current_dir().expect("Failed to get current directory");
        let absolute_path_to_input_template =
            crate_root.join("build_src/templates/t_ros2_handler.rs");
        File::open(absolute_path_to_input_template)
            .unwrap()
            .read_to_string(&mut file_content)
            .unwrap();
        // Replace each template parameter by the actual values
        file_content = file_content.replace("$RTLOLAENUM$", &rtlola_enum);
        file_content = file_content.replace("$ROS2MSGS$", &ros2_msgs);
        file_content = file_content.replace("$SUBSCRIBERS$", &subscribers);
        file_content = file_content.replace("$SELECTSTATEMENT$", &select_statement);
        // Write input source codeto file
        writeln!(&file, "{}", file_content).unwrap();
        // Format generated file
        RustFileGenerator::cargo_fmt_file(&file_location);
    }
}
