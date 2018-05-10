// #define LOCAL_DEBUG_MODE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

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

class RoadsAndJunctions {
 private:
  // Number of cities.
  int NC;
  int MAX_NJ;
  vector<vector<double>> distances;
  vector<Road> sorted_roads;

 public:
  vector<int> buildJunctions(int S, vector<int> cities, double junctionCost,
                             double failureProbability) {
    NC = cities.size() / 2;
    MAX_NJ = 2 * NC;

    distances.resize(NC, vector<double>(NC, 0));
    for (int i = 0; i < NC; ++i) {
      for (int j = 0; j < NC; ++j) {
        if (i == j) continue;
        int x1 = cities[2*i];
        int y1 = cities[2*i + 1];
        int x2 = cities[2*j];
        int y2 = cities[2*j + 1];
        int dx = x2 - x1;
        int dy = y2 - y1;
        distances[i][j] = distances[j][i] = sqrt(dx*dx + dy*dy);
      }
    }

    sorted_roads.reserve(NC * (NC - 1) / 2);
    for (int i = 0; i < NC; ++i) {
      for (int j = i + 1; j < NC; ++j) {
        Road road;
        road.from = i;
        road.to = j;
        road.distance = distances[i][j];
        sorted_roads.emplace_back(move(road));
      }
    }
    sort(sorted_roads.begin(),
         sorted_roads.end(),
         [](const Road& a, const Road& b){
           return a.distance < b.distance;
         });

    return vector<int>();
  }

  vector<int> buildRoads(vector<int> junctionStatus) {
    DisjointSet dset(NC + junctionStatus.size());

    const int numValidJunction = count(junctionStatus.begin(), junctionStatus.end(), 1);
    const int numPointsToConnect = NC + numValidJunction;

    // Kruskal's algorithm.
    vector<int> result;
    result.reserve(2 * (numPointsToConnect - 1));
    for (const Road& road : sorted_roads) {
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
