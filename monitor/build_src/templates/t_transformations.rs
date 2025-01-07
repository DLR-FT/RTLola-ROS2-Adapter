use super::sink_error::SinkErr;
use rtlola_interpreter::{monitor::Change, rtlola_mir::Type, Value};

pub fn value_for_changes(c: &[Change]) -> Option<&Value> {
    c.iter().find_map(|c| match c {
        Change::Close(_) | Change::Spawn(_) => None,
        Change::Value(_, v) => Some(v),
    })
}

pub fn value_as_bool(v: Option<&Value>) -> Result<bool, SinkErr> {
    match v {
        Some(Value::Bool(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Bool,
            got: v.cloned(),
        }),
    }
}

pub fn value_as_i64(v: Option<&Value>) -> Result<i64, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int64),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_i32(v: Option<&Value>) -> Result<i32, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i32),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int32),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_i16(v: Option<&Value>) -> Result<i16, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i16),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int16),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_i8(v: Option<&Value>) -> Result<i8, SinkErr> {
    match v {
        Some(Value::Signed(s)) => Ok(*s as i8),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::Int(rtlola_interpreter::rtlola_mir::IntTy::Int8),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_u64(v: Option<&Value>) -> Result<u64, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt64),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_u32(v: Option<&Value>) -> Result<u32, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u32),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt32),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_u16(v: Option<&Value>) -> Result<u16, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u16),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt16),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_u8(v: Option<&Value>) -> Result<u8, SinkErr> {
    match v {
        Some(Value::Unsigned(s)) => Ok(*s as u8),
        _ => Err(SinkErr::UnexpectedValue {
            expected: Type::UInt(rtlola_interpreter::rtlola_mir::UIntTy::UInt8),
            got: v.cloned(),
        }),
    }
}

pub fn value_as_f64(v: Option<&Value>) -> Result<f64, SinkErr> {
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

pub fn value_as_f32(v: Option<&Value>) -> Result<f32, SinkErr> {
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
