#!/bin/bash -e

make kruskal

javac RoadsAndJunctionsVis.java

echo "kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1
