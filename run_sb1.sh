#!/bin/bash -e

make sb1 RoadsAndJunctionsVis

echo "submittion_1"
java RoadsAndJunctionsVis -exec "./sb1" -seed $1

mv ${1}.png ${1}_sb1.png
