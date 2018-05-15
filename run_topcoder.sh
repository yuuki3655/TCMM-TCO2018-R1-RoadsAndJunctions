#!/bin/bash -e

make topcoder RoadsAndJunctionsVis

echo "topcoder"
java RoadsAndJunctionsVis -exec "./topcoder" -seed $1
