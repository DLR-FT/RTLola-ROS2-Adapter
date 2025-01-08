// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use regex::Regex;
use std::process::Command;

pub(crate) struct Ros2Reader {
    pub blacklist: Vec<String>,
    pub location_setup_script: String,
}

impl Ros2Reader {
    pub(crate) fn new(blacklist: Vec<String>, location_setup_script: String) -> Self {
        Self {
            blacklist,
            location_setup_script,
        }
    }

    //Checks what topics are currently running
    pub fn get_subscribable_topics(&self) -> Vec<(String, String)> {
        // Receive a list of topcs
        let res = Command::new("ros2")
            .args(["topic", "list", "-t"])
            .output()
            .expect("Failed to execute process");
        let tmp = String::from_utf8_lossy(&res.stdout).to_string();
        // Extract all relevant topics by filtering using blacklist
        let msgs: Vec<(String, String)> = tmp
            .trim()
            .split('\n')
            .collect::<Vec<&str>>()
            .into_iter()
            .map(|rcv| {
                let start = rcv.find('[').unwrap() + 1;
                let end = rcv.find(']').unwrap();
                let topic_name = rcv[0..start - 2].to_string();
                let msg_name = rcv[start..end].to_string();
                (topic_name, msg_name)
            })
            .filter(|(topic_name, msg_name)| {
                !self
                    .blacklist
                    .iter()
                    .any(|f| topic_name.contains(f) || msg_name.contains(f))
            })
            .filter(|(topic_name, msg_name)| {
                !(topic_name == "/RTLolaOutput" || msg_name.ends_with("RTLolaOutput"))
            })
            .collect();
        if msgs.is_empty() {
            panic!(
                "\n\n\n\nNo topics available! Please start the processes that publish the respective topics.\n\n\n\n"
            );
        }
        msgs
    }

    // Checks whether RTLolaOutput msg exists
    pub fn get_rtlola_output_msg(&self) -> Option<(String, Vec<(String, String, i32)>)> {
        let call = format!(
            "source workaround_ros2_local_setup.bash 1 {}",
            self.location_setup_script
        );
        let res = Command::new("bash")
            .arg("-c")
            .arg(&call)
            .output()
            .unwrap_or_else(|_| panic!("Error when calling: {}", call));
        let tmp = String::from_utf8_lossy(&res.stdout).to_string();
        let r = Regex::new(r"[a-zA-Z0-9_/]*/msg/RTLolaOutput").unwrap();
        let rtlolaout = match r.find(&tmp) {
            Some(idx) => {
                let msg_name = tmp[idx.start()..idx.end()].to_string();
                let (_, _, members_type_and_name) =
                    Self::read_interface_msg(&self.location_setup_script, &msg_name);
                Some((msg_name, members_type_and_name))
            }
            None => None,
        };
        rtlolaout
    }

    // Checks whether RTLolaOut srv exists
    pub fn get_rtlola_service(
        &self,
    ) -> Option<(
        String,
        Vec<(String, String, i32)>,
        Vec<(String, String, i32)>,
    )> {
        let call = format!(
            "source workaround_ros2_local_setup.bash 2 {}",
            self.location_setup_script
        );
        let res = Command::new("bash")
            .arg("-c")
            .arg(&call)
            .output()
            .unwrap_or_else(|_| panic!("Error when calling: {}", call));
        let tmp = String::from_utf8_lossy(&res.stdout).to_string();
        let r = Regex::new(r"[a-zA-Z0-9_/]*/srv/RTLolaService").unwrap();
        let rtlola_service = match r.find(&tmp) {
            Some(idx) => {
                let msg_name = tmp[idx.start()..idx.end()].to_string();
                let (_, package, request, response) =
                    Self::read_interface_srv(&self.location_setup_script, &msg_name);
                Some((package, request, response))
            }
            None => None,
        };
        rtlola_service
    }

    // fn split_message_content(content: String) {}

    pub fn read_interface_msg(
        local_setup_script: &String,
        msg_name: &String,
    ) -> (String, String, Vec<(String, String, i32)>) {
        let call = format!(
            "source workaround_ros2_local_setup.bash 0 {} {}",
            msg_name, local_setup_script
        );
        // actual bash script call
        let res = Command::new("bash")
            .arg("-c")
            .arg(&call)
            .output()
            .unwrap_or_else(|_| panic!("Error when calling: {}", call));
        let tmp: String = String::from_utf8_lossy(&res.stdout).to_string();
        //formating msg name , [ (type, name), (type, name), ...]
        let index = msg_name.rfind('/').unwrap() + 1;
        let name_datatype = &msg_name[index..];
        let index = msg_name.find('/').unwrap();
        let name_package = &msg_name[..index];
        let members_type_and_name = get_type_name(msg_name.to_string(), tmp);
        (
            name_datatype.to_string(),
            name_package.to_string(),
            members_type_and_name,
        )
    }

    pub fn read_interface_srv(
        local_setup_script: &String,
        msg_name: &String,
    ) -> (
        String,
        String,
        Vec<(String, String, i32)>,
        Vec<(String, String, i32)>,
    ) {
        let call = format!(
            "source workaround_ros2_local_setup.bash 0 {} {}",
            msg_name, local_setup_script
        );
        // actual bash script call
        let res = Command::new("bash")
            .arg("-c")
            .arg(&call)
            .output()
            .unwrap_or_else(|_| panic!("Error when calling: {}", call));
        let tmp = String::from_utf8_lossy(&res.stdout).to_string();
        //formating msg name , [ (type, name), (type, name), ...]
        let index = msg_name.rfind('/').unwrap() + 1;
        let name_datatype = &msg_name[index..];
        let index = msg_name.find('/').unwrap();
        let name_package = &msg_name[..index];
        let req_resp: Vec<&str> = tmp.split("---").collect();
        assert_eq!(req_resp.len(), 2);
        let request = get_type_name(msg_name.to_string(), req_resp[0].to_string());
        let response = get_type_name(msg_name.to_string(), req_resp[1].to_string());
        (
            name_datatype.to_string(),
            name_package.to_string(),
            request,
            response,
        )
    }

    pub(crate) fn get_location_setup_script(&self) -> &str {
        self.location_setup_script.as_ref()
    }
}

fn get_type_name(msg_name: String, tmp: String) -> Vec<(String, String, i32)> {
    let members_type_and_name: Vec<(String, String, i32)> = tmp
        .split('\n')
        .filter(|s| {
            let s = s.trim();
            let equal_before_hash =
                if let (Some(pos_equal), Some(pos_hash)) = (s.find('='), s.find('#')) {
                    pos_equal < pos_hash
                } else {
                    matches!((s.find('='), s.find('#')), (Some(_), None))
                };
            !(s.starts_with('#') || s.is_empty() || equal_before_hash)
        })
        .flat_map(|ss| {
            let ss = ss.trim();
            // Regex that is used to remove unnecessary \s
            let r = Regex::new(r"\s+").unwrap();
            let s = r.replace_all(ss, " ").to_string();
            // Regex that is used to split messages
            let r = Regex::new(r"\s").unwrap();
            let mut split_iter = r.split(s.trim());
            let ty = split_iter.next().unwrap().to_string();
            let r = Regex::new(r"\[\d+\]").unwrap();
            let (ty, arr_size) = if let Some(m) = r.find(&ty) {
                (
                    ty[0..m.start()].to_string(),
                    ty[m.start() + 1..m.end() - 1].parse::<usize>().unwrap(),
                )
            } else {
                (ty, 0_usize)
            };
            match ty.as_str() {
                "bool" | "float32" | "float64" | "int8" | "uint8" | "int16" | "uint16"
                | "int32" | "uint32" | "int64" | "uint64" => (),
                "byte" | "char" | "string" => todo!(),
                _ => panic!(
                    "Unsupported Ros2 Type in {}: {}[{}]\n(full line:  {}   )",
                    msg_name,
                    ty.as_str(),
                    arr_size,
                    ss
                ),
            };
            let name = split_iter.next().unwrap().to_string();
            if let Some(s) = split_iter.next() {
                if !s.starts_with('#') {
                    panic!("Expected type and name, but more were given: {}", ss);
                }
            }
            let mut ty_name_vec = Vec::<(String, String, i32)>::new();
            if arr_size == 0 {
                ty_name_vec.push((ty, name, -1));
            } else {
                for i in 0..arr_size {
                    ty_name_vec.push((ty.clone(), format!("{}__{}", name, i), i as i32));
                }
            }
            ty_name_vec
        })
        .collect();
    members_type_and_name
}
