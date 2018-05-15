#!/bin/bash -e

make sb1 sb2 kruskal release RoadsAndJunctionsVis

./run_kruskal.sh $1
./run_sb1.sh $1
./run_sb2.sh $1
./run_release.sh $1
