#!/bin/bash -e

make main RoadsAndJunctionsVis

echo "test"
java RoadsAndJunctionsVis -exec "./main" -seed $1
