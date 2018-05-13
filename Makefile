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
