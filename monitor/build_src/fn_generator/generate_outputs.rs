// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use crate::build_src::rust_file_generator::RustFileGenerator;
use std::fs::File;
use std::io::{Read, Write};

impl RustFileGenerator {
    // Create rtlola publisher
    pub fn generate_file_rtlolaout_publisher(
        &self,
        rtlolaout_topic: &Option<(String, Vec<(String, String, i32)>)>,
    ) {
        match rtlolaout_topic {
            Some((_, members_type_and_name)) => {
                // File that is generated
                let file_location = format!("{}/output/rtlolaout_publisher.rs", self.dest_path);
                let file = File::create(&file_location).unwrap();
                // Statements to be added depending on ros2 readings
                let mut output_names = String::new();
                let mut default_values = String::new();
                let mut value_change = String::new();
                // Build statements
                for (i, (ty, name, arr_access)) in members_type_and_name.iter().enumerate() {
                    output_names.push_str(&format!("\"{name}\","));
                    match ty.as_str() {
                        "bool" => default_values.push_str(&format!("{name}: false,")),
                        "float32" | "float64" => default_values.push_str(&format!("{name}: 0.0,")),
                        "int8" | "uint8" | "int16" | "uint16" | "int32" | "uint32" | "int64"
                        | "uint64" => default_values.push_str(&format!("{name}: 0,")),
                        _ => panic!(
                            "Unsupported Ros2 Type in {}: {}[{}]",
                            name,
                            ty.as_str(),
                            arr_access,
                        ),
                    };
                    value_change.push_str(&format!("{i} => self.latest_pub.{name} = val,"))
                }
                // Read the contents of the file into a String
                let mut file_content = String::new();
                let crate_root = std::env::current_dir().expect("Failed to get current directory");
                let absolute_path_to_input_template =
                    crate_root.join("build_src/templates/t_rtlolaout_publisher.rs");
                File::open(absolute_path_to_input_template)
                    .unwrap()
                    .read_to_string(&mut file_content)
                    .unwrap();
                // Replace each template parameter by the actual values
                file_content = file_content.replace("$OUTPUTNAMES$", &output_names);
                file_content = file_content.replace("$DEFAULTVALUES$", &default_values);
                file_content = file_content.replace("$VALUECHANGE$", &value_change);
                // Write input source codeto file
                writeln!(&file, "{}", file_content).unwrap();
                // Format generated file
                RustFileGenerator::cargo_fmt_file(&file_location);
            }
            None => (),
        }
    }
}
