#!/bin/bash -e

make kruskal RoadsAndJunctionsVis

echo "kruskal"
java RoadsAndJunctionsVis -exec "./kruskal" -seed $1

mv ${1}.png ${1}_k.png
