// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use super::{$INCLUDE$};
use rtlola_interpreter_macros::CompositFactory;

#[derive(CompositFactory, Debug)]
#[allow(dead_code)]
pub enum RTLolaData {
    $MEMBERS$
}
