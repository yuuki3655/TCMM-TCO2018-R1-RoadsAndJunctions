#!/bin/bash -e

make release_parallelized

javac RoadsAndJunctionsVis.java

echo "release_parallelized"
java RoadsAndJunctionsVis -exec "./release_parallelized" -seed $1
