#!/bin/bash -e

make release

javac RoadsAndJunctionsVis.java

echo "Kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1

echo "main.cpp"
java RoadsAndJunctionsVis -exec "./release" -seed $1
