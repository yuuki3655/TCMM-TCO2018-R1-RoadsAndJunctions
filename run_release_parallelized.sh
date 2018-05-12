#!/bin/bash -e

make release
make release_parallelized

javac RoadsAndJunctionsVis.java

echo "non-parallelized"
java RoadsAndJunctionsVis -exec "./release" -seed $1

echo "parallelized"
java RoadsAndJunctionsVis -exec "./release_parallelized" -seed $1
