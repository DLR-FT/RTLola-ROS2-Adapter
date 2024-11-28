// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use crate::build_src::rust_file_generator::RustFileGenerator;
use std::{
    fs::File,
    io::{Read, Write},
};

impl RustFileGenerator {
    pub fn generate_file_main(&self, topics: &Vec<(String, String)>) {
        // File that is generated
        let file_location = format!("{}/main.rs", self.dest_path);
        let file = File::create(&file_location).unwrap();
        // Create file content
        let mut pub_mods_content = String::new();
        for (topic_name, _) in topics {
            let parts: Vec<&str> = topic_name.split("/").collect();
            let topic_name = parts.last().unwrap().to_lowercase();
            pub_mods_content.push_str(&format!("\t\tpub mod {topic_name};\n"));
        }
        // Read the contents of the file into a String
        let mut file_content = String::new();
        let crate_root = std::env::current_dir().expect("Failed to get current directory");
        let absolute_path_to_input_template = crate_root.join("build_src/templates/t_main.rs");
        File::open(absolute_path_to_input_template)
            .unwrap()
            .read_to_string(&mut file_content)
            .unwrap();
        // Replace each template parameter by the actual values
        file_content = file_content.replace("$INPUTMODS$", &pub_mods_content);
        // Write input source codeto file
        writeln!(&file, "{}", file_content).unwrap();
        // Format generated file
        RustFileGenerator::cargo_fmt_file(&file_location);
    }
}
