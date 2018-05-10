#!/bin/bash -e

make main

javac RoadsAndJunctionsVis.java
java RoadsAndJunctionsVis -exec "./main" -seed $1
