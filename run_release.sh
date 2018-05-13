#!/bin/bash -e

make release

javac RoadsAndJunctionsVis.java

echo "release"
java RoadsAndJunctionsVis -exec "./release" -seed $1
