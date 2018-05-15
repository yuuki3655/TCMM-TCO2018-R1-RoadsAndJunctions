#!/bin/bash -e

make release_parallelized RoadsAndJunctionsVis

echo "release_parallelized"
java RoadsAndJunctionsVis -exec "./release_parallelized" -seed $1

mv ${1}.png ${1}_pa.png
