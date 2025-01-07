// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use super::sink_error::SinkErr;
use super::transformations::*;
use r2r::{$RTLOLAPACKAGE$::srv::RTLolaService::Response};
use rtlola_interpreter::{monitor::Change, rtlola_mir::RtLolaMir};
pub struct Ros2ServiceHandler {
    streams: Vec<usize>,
    latest_response: Response,
}

impl Ros2ServiceHandler {
    pub fn new(ir: &RtLolaMir) -> Result<Self, SinkErr> {
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
            streams,
            latest_response: Response {
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

    pub fn handle(
        &mut self,
        request: r2r::ServiceRequest<r2r::rtlola_testnode::srv::RTLolaService::Service>,
        verdicts: &Vec<Vec<(usize, Vec<Change>)>>,
    ) -> Result<(), SinkErr> {
        println!("Response: {:?}", verdicts);
        for verdict in verdicts {
            for (sr, change) in verdict {
                // Find matching stream in publish topic
                let v = value_for_changes(&change);
                match self.get_match(*sr) {
                    Some(ros2_pos) => match ros2_pos {
                        $VALUECHANGE$
                        _ => unreachable!("should not end here"),
                    },
                    None => continue,
                }
            }
        }
        //Response
        request
            .respond(self.latest_response.clone())
            .map_err(SinkErr::R2rError)
    }
}
