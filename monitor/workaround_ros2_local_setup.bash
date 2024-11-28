# SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
# SPDX-License-Identifier: Apache-2.0

#!/bin/bash

if test $# -eq 0 || test $# -eq 1; then
    echo "At least two arguments are needed but $# were given!"
else
    if test $1 -eq 0; then
        if test $# -eq 2; then
            echo "To show a specific interface, at least three arguments were expected but $# were given. The second argument is the message."
        else
            for ARG in "${@:3}"; do
                source $ARG &>/dev/null
            done        
            ros2 interface show $2
        fi
    elif test $1 -eq 1; then
        for ARG in "${@:2}"; do
            source $ARG &>/dev/null
        done
        ros2 interface list -m
    else
        for ARG in "${@:2}"; do
            source $ARG &>/dev/null
        done
        ros2 interface list -s
    fi
fi
