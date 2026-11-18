// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::$PACKAGENAME$::msg::$TOPICTYPE$;
use rtlola_interpreter_macros::ValueFactory;

#[derive(ValueFactory, Debug)]
#[allow(non_snake_case)]
pub struct RTLola_$TOPICNAME$ {
    $STRUCTDATA$
}

impl From<$TOPICTYPE$> for RTLola_$TOPICNAME$ {
    fn from(data: $TOPICTYPE$) -> RTLola_$TOPICNAME$ {
        RTLola_$TOPICNAME$ {
            $FROMDATA$
        }
    }
}
