// #define LOCAL_DEBUG_MODE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>

using namespace std;

class DisjointSet {
public:
  inline bool MergeSet(int x, int y) {
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
  inline int FindSet(int x) {
    int id = id_.emplace(x, -1).first->second;
    return id < 0 ? x : id_[x] = FindSet(id);
  }
  inline int Size(int x) {
    return -id_[FindSet(x)];
  }
  inline int LargestSetSize() {
    return largest_set_size_;
  }

private:
  unordered_map<int, int> id_;
  int largest_set_size_ = 1;
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

template<typename S, typename T>
inline double distance(const S& a, const T& b) {
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
  unordered_map<int, Junction> junctions;
  unordered_map<int, int> junction_id_to_status_index_;

  int next_available_junction_id_;
  int resetJunctionId() {
    next_available_junction_id_ = NC;
  }
  int generateJunctionId() {
    return next_available_junction_id_++;
  }

  int addJunction(int x, int y) {
    int id = generateJunctionId();
    Junction junction;
    junction.id = id;
    junction.x = x;
    junction.y = y;

    #define ADD_ROAD(x) \
      Road road; \
      road.from = x.id; \
      road.to = junction.id; \
      road.distance = distance(x, junction); \
      sorted_roads.emplace(move(road))

    for (const auto& city : cities) {
      ADD_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      ADD_ROAD(existing_junction.second);
    }
    #undef ADD_ROAD

    junctions.emplace(id, move(junction));
    areamap[x][y] = 2;
    return id;
  }

  void removeJunction(int junction_id) {
    Junction junction = junctions[junction_id];
    junctions.erase(junction_id);
    areamap[junction.x][junction.y] = 0;

    #define REMOVE_ROAD(x) \
      Road road; \
      road.from = x.id; \
      road.to = junction.id; \
      road.distance = distance(x, junction); \
      sorted_roads.erase(road)

    for (const auto& city : cities) {
      REMOVE_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      REMOVE_ROAD(existing_junction.second);
    }
    #undef REMOVE_ROAD
  }

  double calculateScore() {
    double score = 0;
    const int numPointsToConnect = NC + junctions.size();
    DisjointSet dset;
    for (const Road& road : sorted_roads) {
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) continue;
      dset.MergeSet(id1, id2);
      score += road.distance;
      if (dset.LargestSetSize() == numPointsToConnect) return score;
    }
    return score;
  }

 public:
  vector<int> buildJunctions(int S, vector<int> city_locations,
                             double junctionCost, double failureProbability) {
    NC = city_locations.size() / 2;
    MAX_NJ = 2 * NC;

    resetJunctionId();

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
    double best_score = calculateScore();
    int best_x, best_y;
    while (updated) {
      updated = false;
      for (int i = 0; i < S; ++i) {
        for (int j = 0; j < S; ++j) {
          if (areamap[i][j]) continue;
          int jid = addJunction(i, j);
          double score = calculateScore();
          if (score < best_score) {
            best_score = score;
            best_x = i;
            best_y = j;
            updated = true;
          }
          removeJunction(jid);
        }
      }
      if (updated) {
        addJunction(best_x, best_y);
      }
    }

    vector<int> retValue;
    retValue.reserve(junctions.size() * 2);
    for (const auto& junction : junctions) {
      retValue.push_back(junction.second.x);
      retValue.push_back(junction.second.y);
      junction_id_to_status_index_.emplace(
          junction.second.id,
          junction_id_to_status_index_.size());
    }

    return retValue;
  }

  vector<int> buildRoads(vector<int> junctionStatus) {
    const int numValidJunction =
        count(junctionStatus.begin(), junctionStatus.end(), 1);
    const int numPointsToConnect = NC + numValidJunction;

    auto is_valid_point = [this,&junctionStatus](int id){
      return id < NC || junctionStatus[junction_id_to_status_index_[id]];
    };

    // Kruskal's algorithm.
    DisjointSet dset;
    vector<int> result;
    result.reserve(2 * (numPointsToConnect - 1));
    for (const Road& road : sorted_roads) {
      if (!is_valid_point(road.from) || !is_valid_point(road.to)) continue;
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) continue;
      result.push_back(
          road.from < NC ? road.from : NC + junction_id_to_status_index_[road.from]);
      result.push_back(
          road.to < NC ? road.to : NC + junction_id_to_status_index_[road.to]);
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
