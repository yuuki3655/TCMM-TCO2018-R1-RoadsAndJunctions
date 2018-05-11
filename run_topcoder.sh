#!/bin/bash -e

make topcoder

javac RoadsAndJunctionsVis.java

echo "Kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1

echo "main.cpp"
java RoadsAndJunctionsVis -exec "./topcoder" -seed $1
