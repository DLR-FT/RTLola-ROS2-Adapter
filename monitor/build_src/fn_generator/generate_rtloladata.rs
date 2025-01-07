// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use crate::build_src::rust_file_generator::RustFileGenerator;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;

impl RustFileGenerator {
    pub fn generate_file_rtloladata(
        &self,
        topics: &Vec<(String, String)>,
        rtlolaout_service: &Option<(
            String,
            Vec<(String, String, i32)>,
            Vec<(String, String, i32)>,
        )>,
    ) {
        // File that is generated
        // Build path for each topic
        let file_location = Path::new(&format!("{}/input/", self.dest_path)).join("rtloladata.rs");
        let file = File::create(&file_location).unwrap();
        // Statements to be added depending on ros2 readings
        let mut members = String::new();
        let mut includes = String::new();
        // Build statements
        for (topic, _) in topics {
            let parts: Vec<&str> = topic.split('/').collect();
            let topic_name = parts.last().unwrap();
            members.push_str(&format!("{topic_name} (RTLola{topic_name}),"));
            includes.push_str(&format!(
                "{}::RTLola{},",
                topic_name.to_lowercase(),
                topic_name
            ));
        }
        let package_name = if let Some(service) = rtlolaout_service {
            members.push_str(&format!("RTLolaRequest (RTLolaServiceRequest),"));
            includes.push_str(&format!("rtlola_request::RTLolaServiceRequest,"));
            Some(service.0.clone())
        } else {
            None
        };
        // Read the contents of the file into a String
        let mut file_content = String::new();
        let crate_root = std::env::current_dir().expect("Failed to get current directory");
        let absolute_path_to_input_template =
            crate_root.join("build_src/templates/t_RTLolaData.rs");
        File::open(absolute_path_to_input_template)
            .unwrap()
            .read_to_string(&mut file_content)
            .unwrap();
        // Replace each template parameter by the actual values
        file_content = file_content.replace("$MEMBERS$", &members);
        if let Some(package_name) = package_name {
            file_content = file_content.replace("$PACKAGENAME$", &package_name);
        }
        file_content = file_content.replace("$INCLUDE$", &includes);
        // Write input source codeto file
        writeln!(&file, "{}", file_content).unwrap();
        // Format generated file
        RustFileGenerator::cargo_fmt_file(file_location.to_str().unwrap());
    }
}
