// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::{$RTLOLAPACKAGE$, Node, Publisher, QosProfile};
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
        println!("Published: {:?}", verdicts);
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

fn value_as_bool(v: Option<&Value>) -> Result<bool, SinkErr> {
    match v {
        Some(Value::Bool(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Bool,
            got: v.cloned(),
        }),
    }
}

fn value_as_i64(v: Option<&Value>) -> Result<i64, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int64),
            got: v.cloned(),
        }),
    }
}

fn value_as_i32(v: Option<&Value>) -> Result<i32, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i32),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int32),
            got: v.cloned(),
        }),
    }
}

fn value_as_i16(v: Option<&Value>) -> Result<i16, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i16),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int16),
            got: v.cloned(),
        }),
    }
}

fn value_as_i8(v: Option<&Value>) -> Result<i8, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i8),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int8),
            got: v.cloned(),
        }),
    }
}

fn value_as_u64(v: Option<&Value>) -> Result<u64, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt64),
            got: v.cloned(),
        }),
    }
}

fn value_as_u32(v: Option<&Value>) -> Result<u32, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u32),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt32),
            got: v.cloned(),
        }),
    }
}

fn value_as_u16(v: Option<&Value>) -> Result<u16, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u16),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt16),
            got: v.cloned(),
        }),
    }
}

fn value_as_u8(v: Option<&Value>) -> Result<u8, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u8),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt8),
            got: v.cloned(),
        }),
    }
}

fn value_as_f64(v: Option<&Value>) -> Result<f64, SinkErr> {
    match v {
        Some(Value::Float(s)) => {
            let r = s.into_inner();
            Ok(r)
        }
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Float(rtlola_interpreter::rtlola_mir::FloatTy::Float64),
            got: v.cloned(),
        }),
    }
}

fn value_as_f32(v: Option<&Value>) -> Result<f32, SinkErr> {
    match v {
        Some(Value::Float(s)) => {
            let r = s.as_f32().into_inner();
            Ok(r)
        }
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Float(rtlola_interpreter::rtlola_mir::FloatTy::Float32),
            got: v.cloned(),
        }),
    }
}