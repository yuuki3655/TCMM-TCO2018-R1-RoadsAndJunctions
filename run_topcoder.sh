#!/bin/bash -e

make topcoder

javac RoadsAndJunctionsVis.java

echo "topcoder"
java RoadsAndJunctionsVis -exec "./topcoder" -seed $1
