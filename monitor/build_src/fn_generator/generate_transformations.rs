// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use crate::build_src::rust_file_generator::RustFileGenerator;
use std::fs::File;
use std::io::{Read, Write};

impl RustFileGenerator {
    // Create value to typ transformer
    pub fn generate_file_transformations(&self) {
        let file_location = format!("{}/output/transformations.rs", self.dest_path);
        let file = File::create(&file_location).unwrap();
        let mut file_content = String::new();
        let crate_root = std::env::current_dir().expect("Failed to get current directory");
        let absolute_path_to_input_template =
            crate_root.join("build_src/templates/t_transformations.rs");
        File::open(absolute_path_to_input_template)
            .unwrap()
            .read_to_string(&mut file_content)
            .unwrap();
        // Write input source codeto file
        writeln!(&file, "{}", file_content).unwrap();
        // Format generated file
        RustFileGenerator::cargo_fmt_file(&file_location);
    }
}
