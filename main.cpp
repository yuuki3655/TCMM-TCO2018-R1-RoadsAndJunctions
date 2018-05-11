// #define LOCAL_DEBUG_MODE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>

using namespace std;

class DisjointSet {
public:
  DisjointSet(int size) : id_(size, -1){};
  bool MergeSet(int x, int y) {
    x = FindSet(x);
    y = FindSet(y);
    if (x != y) {
      if (id_[y] < id_[x])
        swap(x, y);
      id_[x] += id_[y];
      id_[y] = x;
      largest_set_size_ = max(largest_set_size_, -id_[x]);
    }
    return x != y;
  }
  int FindSet(int x) { return id_[x] < 0 ? x : id_[x] = FindSet(id_[x]); }
  int Size(int x) { return -id_[FindSet(x)]; }
  int LargestSetSize() { return largest_set_size_; }

private:
  vector<int> id_;
  int largest_set_size_ = 0;
};

const int MAX_C = 1000;
const int MAX_NC = 100;
const double MAX_JUNCTION_COST = 10.0;
const double MAX_FAILURE_PROB = 0.4;

struct Road {
  // from < to.
  int from;
  int to;
  double distance;
};

inline bool operator<(const Road& a, const Road& b) {
  if (a.distance < b.distance) {
    return true;
  }
  if (a.distance > b.distance) {
    return false;
  }
  if (a.to < b.to) {
    return true;
  }
  if (a.to > b.to) {
    return false;
  }
  return a.from < b.from;
}

struct City {
  int id;
  int x;
  int y;
};

struct Junction {
  int id;
  int x;
  int y;
};

template<typename T>
double distance(const T& a, const T& b) {
  int dx = b.x - a.x;
  int dy = b.y - a.y;
  return sqrt(dx*dx + dy*dy);
}

class RoadsAndJunctions {
 private:
  // Number of cities.
  int NC;
  int MAX_NJ;
  vector<City> cities;
  set<Road> sorted_roads;
  vector<vector<int>> areamap;
  vector<Junction> junctions;

 public:
  vector<int> buildJunctions(int S, vector<int> city_locations,
                             double junctionCost, double failureProbability) {
    NC = city_locations.size() / 2;
    MAX_NJ = 2 * NC;

    cities.reserve(NC);
    for (int i = 0; i < NC; ++i) {
      City city;
      city.id = i;
      city.x = city_locations[2*i];
      city.y = city_locations[2*i + 1];
      cities.emplace_back(move(city));
    }

    areamap.resize(S+1, vector<int>(S+1, 0));
    for (const auto& city : cities) {
      areamap[city.x][city.y] = 1;
    }

    for (int i = 0; i < NC; ++i) {
      for (int j = i + 1; j < NC; ++j) {
        Road road;
        road.from = i;
        road.to = j;
        road.distance = distance(cities[i], cities[j]);
        sorted_roads.emplace(move(road));
      }
    }

    bool updated = true;
    while (updated) {
      updated = false;
      for (int i = 0; i < S; ++i) {
        for (int j = 0; j < S; ++j) {
          if (areamap[i][j]) continue;
        }
      }
    }

    vector<int> retValue;
    retValue.reserve(junctions.size() * 2);
    for (const auto& junction : junctions) {
      retValue.push_back(junction.x);
      retValue.push_back(junction.y);
    }

    return retValue;
  }

  void addJunction(const Junction& junction) {
    for (const auto& city : cities) {
    }
  }

  vector<int> buildRoads(vector<int> junctionStatus) {
    DisjointSet dset(NC + junctionStatus.size());

    const int numValidJunction = count(junctionStatus.begin(), junctionStatus.end(), 1);
    const int numPointsToConnect = NC + numValidJunction;

    auto is_valid_point = [this,&junctionStatus](int id){
      return id < NC || junctionStatus[id - NC];
    };

    // Kruskal's algorithm.
    vector<int> result;
    result.reserve(2 * (numPointsToConnect - 1));
    for (const Road& road : sorted_roads) {
      if (!is_valid_point(road.from) || !is_valid_point(road.to)) continue;
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) continue;
      result.push_back(road.from);
      result.push_back(road.to);
      dset.MergeSet(id1, id2);
      if (dset.LargestSetSize() == numPointsToConnect) break;
    }

    return result;
  }
};

#ifdef LOCAL_ENTRY_POINT_FOR_TESTING
// -------8<------- end of solution submitted to the website -------8<-------

template<class T> void getVector(vector<T>& v) {
  for (int i = 0; i < v.size(); ++i)
    cin >> v[i];
}

int main() {
  RoadsAndJunctions rj;
  int S, C;
  cin >> S >> C;
  vector<int> cities(C);
  getVector(cities);
  double junctionCost, failureProbability;
  cin >> junctionCost >> failureProbability;

  vector<int> ret = rj.buildJunctions(S, cities, junctionCost, failureProbability);
  cout << ret.size() << endl;
  for (int i = 0; i < (int)ret.size(); ++i)
    cout << ret[i] << endl;
  cout.flush();

  int J;
  cin >> J;
  vector<int> junctionStatus(J);
  getVector(junctionStatus);

  ret = rj.buildRoads(junctionStatus);
  cout << ret.size() << endl;
  for (int i = 0; i < (int)ret.size(); ++i)
    cout << ret[i] << endl;
  cout.flush();
}
#endif
