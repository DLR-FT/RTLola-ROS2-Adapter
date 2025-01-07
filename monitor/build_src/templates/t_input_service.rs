// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::$PACKAGENAME$::srv::RTLolaService::Request;
use rtlola_interpreter_macros::ValueFactory;

#[derive(ValueFactory, Debug)]
#[allow(non_snake_case)]
pub struct RTLolaServiceRequest {
    $STRUCTDATA$
}

impl From<Request> for RTLolaServiceRequest {
    fn from(data: Request) -> RTLolaServiceRequest {
        RTLolaServiceRequest {
            $FROMDATA$
        }
    }
}
