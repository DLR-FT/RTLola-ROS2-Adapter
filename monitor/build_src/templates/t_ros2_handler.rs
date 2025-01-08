// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use std::error::Error;

use crate::{
    input::{$RTLOLAENUM$
        $SERVICEAVAILABLE$
        rtloladata::{RTLolaData, RTLolaDataFactory, RTLolaType}},
    output::{rtlolaout_publisher::Ros2Publisher, $SERVICEHANDLERUSE$},
};
use futures::{stream_select, StreamExt};
use r2r::{
    rtlola_testnode::msg::{$ROS2MSGS$},
    QosProfile,
};
use rtlola_interpreter::{
    monitor::{Change, Verdicts},
    Monitor,
};
use tokio::time::{timeout, Duration};

pub struct Ros2Handler {}

impl Ros2Handler {
    pub async fn run(
        mut monitor: Monitor<RTLolaDataFactory, rtlola_interpreter::config::OnlineMode>,
    ) -> Result<(), Box<dyn Error>> {
        // r2r context
        let ctx = r2r::Context::create().unwrap();
        // quality of service
        let qos = QosProfile::default();
        // creates ros2 node
        let mut node = r2r::Node::create(ctx, "rtlola", "").unwrap();
        // creates subscribers
        $SUBSCRIBERS$
        // create service
        $SERVICE$
        // creates one ros2 publisher for RTLolaOutput
        let mut publisher = Ros2Publisher::new(monitor.ir(), &mut node)?;
        // create service handler
        $SERVICEHANDLER$
        // uses futures to handle asynchonous access
        let mut all = stream_select!($SELECTSTATEMENT$);
        // specifies frequency of monitor: 100ms = 10Hz
        let _handle = tokio::task::spawn_blocking(move || loop {
            node.spin_once(std::time::Duration::from_millis(100));
        });
        println!("...running...");
        loop {
            // awaits data, i.e., data from subscribed topics
            let mut verdicts_vec = vec![];
            match timeout(Duration::from_millis(100), all.next()).await {
                Ok(data) => {
                    match data {
                        Some(ty) => {
                            match ty {
                                RTLolaType::Subscription(rtlola_data) => {
                                    // Event received
                                    let Verdicts {
                                        timed,
                                        event,
                                        ts: _,
                                    } = monitor
                                        .accept_event(rtlola_data, ())
                                        .map_err(|_| println!("Message could not be parsed"))
                                        .unwrap();
                                    let timed_v: Vec<Vec<(usize, Vec<Change>)>> =
                                        timed.into_iter().map(|(_, v)| v).collect();
                                    verdicts_vec.extend(timed_v);
                                    verdicts_vec.push(event);
                                }
                                $SERVICECASE$
                            }
                        }
                        None => {
                            // No event received
                            let timed = monitor.accept_time(());
                            let timed_v: Vec<Vec<(usize, Vec<Change>)>> =
                                timed.into_iter().map(|(_, v)| v).collect();
                            verdicts_vec.extend(timed_v);
                        }
                    }
                }
                Err(_) => {
                    // No event received
                    let timed = monitor.accept_time(());
                    let timed_v: Vec<Vec<(usize, Vec<Change>)>> =
                        timed.into_iter().map(|(_, v)| v).collect();
                    verdicts_vec.extend(timed_v);
                }
            };
            // publishes data
            publisher.publish(verdicts_vec).unwrap()
        }
    }
}