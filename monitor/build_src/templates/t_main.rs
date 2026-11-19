// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

pub mod ros2_handler;
pub mod input {
    $INPUTMODS$
    pub mod rtloladata;
}
pub mod output {
    pub mod rtlolaout_publisher;
    $SERVICEAVAILABLE$
    pub mod sink_error;
    pub mod transformations;
}
use crate::{
    input::rtloladata::RTLolaData, ros2_handler::Ros2Handler,
};
use rtlola_interpreter::{input::AssociatedEventFactory, monitor::Incremental, rtlola_frontend::{self, ParserConfig}, ConfigBuilder};
use std::fs;

#[tokio::main]
async fn main() {
    let spec = fs::read_to_string("$SPECLOCATION$").unwrap();
    // Parse specification
    let parser_config = ParserConfig::for_string(spec.clone());
    let mir = rtlola_frontend::parse(&parser_config).expect("Failed to parse RTLola specification");
    let available_inputs: Vec<String> = mir.inputs.iter().map(|i| i.name.clone()).collect();
    println!("Available RTLola inputs: {:?}", available_inputs);
    // Create monitor
    let monitor_result = ConfigBuilder::new()
        .with_ir(mir)
        .online()
        .with_event_factory::<<RTLolaData as AssociatedEventFactory>::Factory>()
        .with_verdict::<Incremental>()
        .monitor();
    let monitor = match monitor_result {
        Ok(monitor) => monitor,
        Err(err) => {
            eprintln!("Failed to build RTLola monitor: {:?}", err);
            return;
        }
    };
    println!("Run monitor ...");
    // Runs monitor
    Ros2Handler::run(monitor).await.unwrap();
    println!("... finished monitoring.");
}
