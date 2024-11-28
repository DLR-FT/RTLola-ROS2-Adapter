// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

use r2r::$PACKAGENAME$::msg::$TOPICNAME$;
use rtlola_interpreter_macros::ValueFactory;

#[derive(ValueFactory, Debug)]
#[allow(non_snake_case)]
pub struct RTLola$TOPICNAME$ {
    $STRUCTDATA$
}

impl From<$TOPICNAME$> for RTLola$TOPICNAME$ {
    fn from(data: $TOPICNAME$) -> RTLola$TOPICNAME$ {
        RTLola$TOPICNAME$ {
            $FROMDATA$
        }
    }
}
