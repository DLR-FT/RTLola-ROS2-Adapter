// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use super::ros2_reader::Ros2Reader;
use core::panic;
use std::process::Command;

pub(crate) struct RustFileGenerator {
    pub ros2_reader: Ros2Reader,
    pub dest_path: String,
    pub generated_topics: Vec<(String, String, String)>,
}

impl RustFileGenerator {
    pub fn new(ros2_reader: Ros2Reader, dest_path: String) -> Self {
        Self {
            ros2_reader,
            dest_path,
            generated_topics: Vec::<(String, String, String)>::new(),
        }
    }

    pub fn cargo_fmt_file(file_path: &str) {
        Command::new("rustfmt")
            .args([file_path])
            .output()
            .expect("Failed to execute process");
    }

    pub fn transform_type_ros2rust(types: &String) -> &str {
        let ty_transformed = match types.as_str() {
            "bool" => "bool",
            "byte" => todo!(),
            "char" => todo!(),
            "float32" => "f32",
            "float64" => "f64",
            "int8" => "i8",
            "uint8" => "u8",
            "int16" => "i16",
            "uint16" => "u16",
            "int32" => "i32",
            "uint32" => "u32",
            "int64" => "i64",
            "uint64" => "u64",
            "string" => todo!(),
            _ => panic!("Unsupported Ros2 Type: {}", types),
        };
        ty_transformed
    }
}
