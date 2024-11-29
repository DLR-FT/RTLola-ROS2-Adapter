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
            Some((rtlola_package, members_type_and_name)) => {
                let rtlola_package = rtlola_package.replace('/', "::");
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
                        "bool" => {
                            default_values.push_str(&format!("{name}: false,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_bool(v)?,"));
                        }
                        "float32" => {
                            default_values.push_str(&format!("{name}: 0.0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_f32(v)?,"));
                        }
                        "float64" => {
                            default_values.push_str(&format!("{name}: 0.0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_f64(v)?,"));
                        }
                        "int8" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_i8(v)?,"));
                        }
                        "uint8" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_u8(v)?,"));
                        }
                        "int16" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_i16(v)?,"));
                        }
                        "uint16" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_u16(v)?,"));
                        }
                        "int32" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_i32(v)?,"));
                        }
                        "uint32" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_u32(v)?,"));
                        }
                        
                        "int64" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_i64(v)?,"));
                        }
                        
                        "uint64" => {
                            default_values.push_str(&format!("{name}: 0,"));
                            value_change.push_str(&format!("{i} => self.latest_pub.{name} = value_as_u64(v)?,"));
                        }
                        _ => {
                            panic!(
                                "Unsupported Ros2 Type in {}: {}[{}]",
                                name,
                                ty.as_str(),
                                arr_access,
                            )
                        }
                    } 
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
                file_content = file_content.replace("$RTLOLAPACKAGE$", rtlola_package.as_str());
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
