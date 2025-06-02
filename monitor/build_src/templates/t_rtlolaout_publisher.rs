// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::{$RTLOLAPACKAGE$, Node, Publisher, QosProfile, qos::{HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy}};
use rtlola_interpreter::{monitor::Change,rtlola_mir::RtLolaMir};
use tokio::time::Duration;
use super::sink_error::SinkErr;
use super::transformations::*;
pub struct Ros2Publisher {
    publisher: Publisher<RTLolaOutput>,
    streams: Vec<usize>,
    latest_pub: RTLolaOutput,
}

impl Ros2Publisher {
    pub fn new(ir: &RtLolaMir, node: &mut Node) -> Result<Self, SinkErr> {
        let publisher = node
            .create_publisher::<RTLolaOutput>("/RTLolaOutput", $QOS$)
            .map_err(SinkErr::R2rError)?;
        let mut streams = vec![];
        let look_for = [$OUTPUTNAMES$];
        for out_stream in look_for {
            let found_stream = ir
                .outputs
                .iter()
                .find_map(|o| {
                    if o.name == out_stream {
                        Some(o.reference.out_ix())
                    } else {
                        None
                    }
                })
                .ok_or(SinkErr::StreamNotFound(out_stream.to_string()))?;
            streams.push(found_stream);
        }
        Ok(Self {
            publisher,
            streams,
            latest_pub: RTLolaOutput {
                $DEFAULTVALUES$
            },
        })
    }

    fn get_match(&self, sr: usize) -> Option<usize> {
        let mut found = None;
        for (i, stream_sr) in self.streams.iter().enumerate() {
            if *stream_sr == sr {
                found = Some(i);
                break;
            }
        }
        found
    }

    pub fn publish(&mut self, verdicts: Vec<Vec<(usize, Vec<Change>)>>) -> Result<(), SinkErr> {
        // println!("Published: {:?}", verdicts);
        for verdict in verdicts {
            for (sr, change) in verdict {
                // Find matching stream in publish topic
                let v = value_for_changes(&change);
                match self.get_match(sr) {
                    Some(ros2_pos) => {
                        match ros2_pos {
                            $VALUECHANGE$
                            _ => unreachable!("should not end here"),
                        }
                    }
                    None => continue,
                }
            }
        }
        // Publish updated RTLolaOutput topic
        self.publisher
            .publish(&self.latest_pub)
            .map_err(SinkErr::R2rError)
    }
}