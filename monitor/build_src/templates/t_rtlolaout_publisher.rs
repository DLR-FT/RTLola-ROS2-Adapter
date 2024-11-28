// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::{rtlolaout::msg::RTLolaOut, Node, Publisher, QosProfile};
use rtlola_interpreter::{
    monitor::Change,
    rtlola_mir::{RtLolaMir, Type},
    Value,
};

use super::sink_error::SinkErr;

pub struct Ros2Publisher {
    publisher: Publisher<RTLolaOut>,
    streams: Vec<usize>,
    latest_pub: RTLolaOut,
}

impl Ros2Publisher {
    pub fn new(ir: &RtLolaMir, node: &mut Node) -> Result<Self, SinkErr> {
        let publisher = node
            .create_publisher::<RTLolaOut>("/RTLolaOut", QosProfile::default())
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
            latest_pub: RTLolaOut {
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
        println!("Received: {:?}", verdicts);
        for verdict in verdicts {
            for (sr, change) in verdict {
                // Find matching stream in publish topic
                match self.get_match(sr) {
                    Some(ros2_pos) => {
                        // Get value
                        let val = value_as_i64(value_for_changes(&change));
                        match val {
                            Ok(val) => match ros2_pos {
                                $VALUECHANGE$
                                _ => unreachable!("should not end here"),
                            },
                            Err(e) => return Err(e),
                        }
                    }
                    None => continue,
                }
            }
        }
        // Publish updated RTLolaOut topic
        self.publisher
            .publish(&self.latest_pub)
            .map_err(SinkErr::R2rError)
    }
}

fn value_for_changes(c: &[Change]) -> Option<&Value> {
    c.iter().find_map(|c| match c {
        Change::Close(_) | Change::Spawn(_) => None,
        Change::Value(_, v) => Some(v),
    })
}

fn value_as_i64(v: Option<&Value>) -> Result<i64, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt64),
            got: v.cloned(),
        }),
    }
}
