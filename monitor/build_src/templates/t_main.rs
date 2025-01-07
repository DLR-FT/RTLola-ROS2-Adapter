// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

pub mod ros2_handler;
pub mod input {
    $INPUTMODS$
    pub mod rtloladata;
}
pub mod output {
    pub mod rtlolaout_publisher;
    pub mod rtlolaout_service;
    pub mod sink_error;
    pub mod transformations;
}
use crate::{
    input::rtloladata::RTLolaData, input::rtloladata::RTLolaDataFactory, ros2_handler::Ros2Handler,
};
use rtlola_interpreter::{input::AssociatedFactory, monitor::Incremental, ConfigBuilder, Monitor};
use std::{env, fs};

#[tokio::main]
async fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 2);
    // Reads specification
    let spec = fs::read_to_string(&args[1]).unwrap();
    // Creates monitor
    let monitor: Monitor<RTLolaDataFactory, rtlola_interpreter::config::OnlineMode> =
        ConfigBuilder::new()
            .spec_str(spec.as_str())
            .online()
            .with_event_factory::<<RTLolaData as AssociatedFactory>::Factory>()
            .with_verdict::<Incremental>()
            .monitor()
            .expect("Failed to build monitor");
    println!("Run monitor ...");
    // Runs monitor
    Ros2Handler::run(monitor).await.unwrap();
    println!("... finished monitoring.");
}
