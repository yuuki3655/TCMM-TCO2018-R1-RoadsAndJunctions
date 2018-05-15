main: main.cpp
		g++-4.9 -std=gnu++11 -W -Wall -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o main -DLOCAL_DEBUG_MODE -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp

release: main.cpp
		g++-4.9 -std=gnu++11 -W -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o release -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp

topcoder: main.cpp
		g++-4.9 -std=gnu++11 -W -Wall -Wno-sign-compare -pipe -mmmx -msse -msse2 -msse3 -o topcoder -DTOPCODER_TEST_MODE -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp

release_parallelized: main.cpp
		g++-4.9 -std=gnu++11 -W -Wall -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o release_parallelized -DLOCAL_ENTRY_POINT_FOR_TESTING -DENABLE_PARALLEL_PROCESSING main.cpp

kruskal: kruskal.cpp
		g++-4.9 -std=gnu++11 -W -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o kruskal -DLOCAL_ENTRY_POINT_FOR_TESTING kruskal.cpp

sb1: sb1.cpp
		g++-4.9 -std=gnu++11 -W -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o sb1 -DLOCAL_ENTRY_POINT_FOR_TESTING sb1.cpp

sb2: sb2.cpp
		g++-4.9 -std=gnu++11 -W -Wno-sign-compare -O2 -pipe -mmmx -msse -msse2 -msse3 -o sb2 -DLOCAL_ENTRY_POINT_FOR_TESTING sb2.cpp

RoadsAndJunctionsVis: RoadsAndJunctionsVis.java
	  javac RoadsAndJunctionsVis.java

RoadsAndJunctionsVisForScoring: RoadsAndJunctionsVisForScoring.java
	  javac RoadsAndJunctionsVisForScoring.java
