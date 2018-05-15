#!/bin/bash -e

make kruskal RoadsAndJunctionsVis

echo "kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1
