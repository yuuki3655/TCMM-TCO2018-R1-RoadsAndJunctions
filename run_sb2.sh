#!/bin/bash -e

make sb2 RoadsAndJunctionsVis

echo "submittion_2"
java RoadsAndJunctionsVis -exec "./sb2" -seed $1

mv ${1}.png ${1}_sb2.png
