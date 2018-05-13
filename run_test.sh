#!/bin/bash -e

make main

javac RoadsAndJunctionsVis.java

echo "test"
java RoadsAndJunctionsVis -exec "./main" -seed $1
