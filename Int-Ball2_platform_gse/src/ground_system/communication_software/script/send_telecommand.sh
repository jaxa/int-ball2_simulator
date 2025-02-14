#!/bin/bash

COMMAND="{command: {name: \"$1\", format: $2, data: \"$3\"}}"
rosservice call /telecommand_bridge "$COMMAND"
