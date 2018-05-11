#!/bin/bash -e

make main

javac RoadsAndJunctionsVis.java

echo "Kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1

echo "main.cpp"
java RoadsAndJunctionsVis -exec "./main" -seed $1
