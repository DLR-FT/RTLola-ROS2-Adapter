// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use rtlola_interpreter::{rtlola_mir::Type, Value};

#[derive(Debug)]
pub enum SinkErr {
    InvalidPacing,
    R2rError(r2r::Error),
    StreamNotFound(String),
    InvalidNumberTriggers(usize),
    UnexpectedValue { expected: Type, got: Option<Value> },
}

impl std::fmt::Display for SinkErr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SinkErr::InvalidNumberTriggers(num) => {
                writeln!(f, "Can only handle one trigger, got {num}")
            }
            SinkErr::StreamNotFound(name) => writeln!(f, "Could not find {name} in specification"),
            SinkErr::UnexpectedValue { expected, got } => {
                writeln!(f, "Expected {expected} got {:?}", got)
            }
            SinkErr::R2rError(e) => writeln!(f, "Publisher: {e}"),
            SinkErr::InvalidPacing => writeln!(f, "Invalid Pacing"),
        }
    }
}

impl std::error::Error for SinkErr {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SinkErr::R2rError(e) => Some(e),
            SinkErr::InvalidPacing
            | SinkErr::StreamNotFound(_)
            | SinkErr::InvalidNumberTriggers(_)
            | SinkErr::UnexpectedValue { .. } => None,
        }
    }
}
