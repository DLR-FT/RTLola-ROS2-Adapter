// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use regex::Regex;

use crate::build_src::ros2_reader::Ros2Reader;
use crate::build_src::rust_file_generator::RustFileGenerator;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;

impl RustFileGenerator {
    pub fn generate_file_inputs(&mut self, topics: &Vec<(String, String)>) {
        // Reads each msg but ignore /RTLolaOut
        for (topic_name, msg_name) in topics {
            // builds bash call uses the workaround script since the local ros2 setup.bash is not used and hence no information using ros2 interface can be received
            let (name_datatype, name_package, members_type_and_name) =
                Ros2Reader::read_interface_msg(
                    &self.ros2_reader.get_location_setup_script().to_string(),
                    msg_name,
                );
            // Fills vector of topics that is also used in other file generation calls
            self.generated_topics.push((
                topic_name.clone(),
                name_package.to_string(),
                name_datatype.to_string(),
            ));
            // Build path for each topic
            let new_file_location = Path::new(&format!("{}/input/", self.dest_path))
                .join(format!("{}.rs", name_datatype.to_lowercase()));
            // Creates file
            let file = File::create(&new_file_location).unwrap();
            // Create content of file
            self.create_input_ros2_msg(
                file,
                name_package.to_string(),
                name_datatype.to_string(),
                members_type_and_name,
            );
            // Format generated file
            RustFileGenerator::cargo_fmt_file(new_file_location.to_str().unwrap());
        }
    }

    fn create_input_ros2_msg(
        &self,
        file: File,
        package_name: String,
        topic_name: String,
        members_type_and_name: Vec<(String, String, i32)>,
    ) {
        let topic_name_lowercase = topic_name.to_lowercase();
        // Statements to be added depending on ros2 readings
        let mut struct_data = String::new();
        let mut from_data = String::new();
        // Extracting Information
        for (ty, name, i) in &members_type_and_name {
            let rtlola_name = if name == "type" {
                "type_".to_string()
            } else {
                format!("{topic_name_lowercase}__{name}")
            };
            let ty_transformed = RustFileGenerator::transform_type_ros2rust(ty);
            let access: String = if *i < 0 {
                name.to_string()
            } else {
                let r = Regex::new(r"__\d").unwrap();
                let m = r.find(name).unwrap();
                let name = &name[0..m.start()];
                format!("{name}[{i}]")
            };
            struct_data.push_str(&format!("{rtlola_name}: {ty_transformed},\n"));
            from_data.push_str(&format!("{rtlola_name}: data.{access},\n"));
        }
        println!("{package_name} {topic_name} {struct_data}{from_data}");
        // Read the contents of the file into a String
        let mut file_content = String::new();
        let crate_root = std::env::current_dir().expect("Failed to get current directory");
        let absolute_path_to_input_template = crate_root.join("build_src/templates/t_input.rs");
        File::open(absolute_path_to_input_template)
            .unwrap()
            .read_to_string(&mut file_content)
            .unwrap();
        // Replace each template parameter by the actual values
        file_content = file_content.replace("$PACKAGENAME$", &package_name);
        file_content = file_content.replace("$TOPICNAME$", &topic_name);
        file_content = file_content.replace("$STRUCTDATA$", &struct_data);
        file_content = file_content.replace("$FROMDATA$", &from_data);
        // Write input source codeto file
        writeln!(&file, "{}", file_content).unwrap();
    }
}
