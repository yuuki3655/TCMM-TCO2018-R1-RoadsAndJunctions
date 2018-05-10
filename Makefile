main: main.cpp
		g++ -std=gnu++11 -W -Wall -Wno-sign-compare -O2 -s -pipe -mmmx -msse -msse2 -msse3 -o main -DLOCAL_DEBUG_MODE -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp

release: main.cpp
		g++ -std=gnu++11 -W -Wall -Wno-sign-compare -O2 -s -pipe -mmmx -msse -msse2 -msse3 -o release -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp

debug: main.cpp
		g++ -std=gnu++11 -W -Wall -Wno-sign-compare -s -pipe -mmmx -msse -msse2 -msse3 -g -rdynamic -o debug -DLOCAL_DEBUG_MODE -DLOCAL_ENTRY_POINT_FOR_TESTING main.cpp
