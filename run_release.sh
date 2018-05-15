#!/bin/bash -e

make release RoadsAndJunctionsVis

echo "release"
java RoadsAndJunctionsVis -exec "./release" -seed $1
