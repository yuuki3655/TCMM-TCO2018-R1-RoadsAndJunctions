// #define TOPCODER_TEST_MODE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>
#include <sys/time.h>
#include <future>

using namespace std;

#ifdef LOCAL_DEBUG_MODE
#define debug(x) cerr << x;
#define debug2(x) cerr << x;
#else
#ifdef TOPCODER_TEST_MODE
#define debug(x) cerr << x;
#define debug2(x) ;
#else
#define debug(x) ;
#define debug2(x) ;
#endif
#endif

// I wish I could enable this for marathon match. It's prohibited.
#ifdef ENABLE_PARALLEL_PROCESSING
// Use 8 threads (4 * 2 = 8).
#ifndef PARALLEL_SPLIT_X
#define PARALLEL_SPLIT_X 4
#endif
#ifndef PARALLEL_SPLIT_Y
#define PARALLEL_SPLIT_Y 2
#endif
#endif

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
  inline int FindSet(int x) const {
    int id = id_.emplace(x, -1).first->second;
    return id < 0 ? x : id_[x] = FindSet(id);
  }
  inline int Size(int x) const {
    return -id_[FindSet(x)];
  }
  inline int LargestSetSize() const {
    return largest_set_size_;
  }

private:
  mutable unordered_map<int, int> id_;
  int largest_set_size_ = 1;
};


// These values are just for documentation purpose.
//
// const int MAX_C = 1000;
// const int MAX_NC = 100;
// const double MAX_JUNCTION_COST = 10.0;
// const double MAX_FAILURE_PROB = 0.4;

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

typedef vector<vector<int>> Heatmap;

class RoadsAndJunctions {
 private:
  int S;
  int NC;  // Number of cities.
  int MAX_NJ;
  double J_COST;
  double F_PROB;

  vector<City> cities;
  set<Road> sorted_roads;
  vector<vector<int>> areamap;
  unordered_map<int, Junction> junctions;
  unordered_map<int, int> junction_id_to_status_index_;

  double startTimeSec;

  double getTimeSec() const {
  	timeval tv;
  	gettimeofday(&tv, NULL);
  	return tv.tv_sec + 1e-6 * tv.tv_usec;
  }

  void startTimer() {
    startTimeSec = getTimeSec();
  }

  double normalizedTime() const {
    double diff = getTimeSec() - startTimeSec;
    return diff / 10.0;
  }

  int next_available_junction_id_;
  void resetJunctionId() {
    next_available_junction_id_ = NC;
  }
  int generateJunctionId() {
    return next_available_junction_id_++;
  }

  double tryAddJunction(int x, int y, double longest_road_in_use) const {
    Junction junction;
    junction.id = 100000000;  // temporary id.
    junction.x = x;
    junction.y = y;

    set<Road> sorted_tmp_roads;

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define ADD_ROAD(T) \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
      sorted_tmp_roads.emplace(move(road))

    for (const auto& city : cities) {
      ADD_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      ADD_ROAD(existing_junction.second);
    }
    #undef ADD_ROAD

    // If a distance of the shortest road among newly created roads is longer
    // then the longest load in use in the current minimum spanning tree,
    // those newly created roads never be used. Thus, we can skip further
    // evaluation.
    if (sorted_tmp_roads.begin()->distance + J_COST >= longest_road_in_use) {
      return 1e8;
    }

    double score = 0;
    const int numPointsToConnect = NC + junctions.size() + 1;
    DisjointSet dset;
    auto compute = [&dset, &score, &numPointsToConnect](const Road& road) {
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) return false;
      dset.MergeSet(id1, id2);
      score += road.distance;
      return (dset.LargestSetSize() == numPointsToConnect);
    };

    auto tmp_road_iter = sorted_tmp_roads.begin();
    for (const Road& road : sorted_roads) {
      while (tmp_road_iter != sorted_tmp_roads.end() && *tmp_road_iter < road) {
        if (compute(*tmp_road_iter)) return score;
        ++tmp_road_iter;
      }
      if (compute(road)) return score;
    }
    return score;
  }

  double tryMoveJunction(int junction_id, int x, int y) const {
    Junction junction;
    junction.id = 100000000;  // temporary id.
    junction.x = x;
    junction.y = y;

    set<Road> sorted_tmp_roads;

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define ADD_ROAD(T) \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
      sorted_tmp_roads.emplace(move(road))

    for (const auto& city : cities) {
      ADD_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      ADD_ROAD(existing_junction.second);
    }
    #undef ADD_ROAD

    double score = 0;
    const int numPointsToConnect = NC + junctions.size();
    DisjointSet dset;
    auto compute =
        [&dset, &score, numPointsToConnect, junction_id](const Road& road) {
      if (road.from == junction_id || road.to == junction_id) return false;
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) return false;
      dset.MergeSet(id1, id2);
      score += road.distance;
      return (dset.LargestSetSize() == numPointsToConnect);
    };

    auto tmp_road_iter = sorted_tmp_roads.begin();
    for (const Road& road : sorted_roads) {
      while (tmp_road_iter != sorted_tmp_roads.end() && *tmp_road_iter < road) {
        if (compute(*tmp_road_iter)) return score;
        ++tmp_road_iter;
      }
      if (compute(road)) return score;
    }
    return score;
  }

  int addJunction(int x, int y) {
    int id = generateJunctionId();
    Junction junction;
    junction.id = id;
    junction.x = x;
    junction.y = y;

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define ADD_ROAD(T) \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
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

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define REMOVE_ROAD(T) \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
      sorted_roads.erase(road)

    for (const auto& city : cities) {
      REMOVE_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      REMOVE_ROAD(existing_junction.second);
    }
    #undef REMOVE_ROAD
  }

  double calculateScore() const {
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

  double getLongestRoadInUse() const {
    const int numPointsToConnect = NC + junctions.size();
    DisjointSet dset;
    for (const Road& road : sorted_roads) {
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) continue;
      dset.MergeSet(id1, id2);
      if (dset.LargestSetSize() == numPointsToConnect) return road.distance;
    }
    // This should never happen.
    debug("Oops. Something wrong happens at getLongestRoadInUse.");
    return 100000000;
  }

  tuple<double, Heatmap, int> optimize(const int granularity,
                                       const Heatmap& prev_heatmap,
                                       const bool enable_banning) {
    debug("start optimize: granularity = " << granularity
          << ", banning = " << enable_banning << endl);

    vector<vector<int>> banned(granularity, vector<int>(granularity, 0));

    auto compute = [this, granularity, &prev_heatmap, &banned, enable_banning](
        int i_begin, int i_end, int j_begin, int j_end,
        double best_score, double longest_road_in_use) {
      bool updated = false;
      double prev_score = best_score;
      int best_x, best_y, best_i, best_j;
      for (int i = i_begin; i < i_end; ++i) {
        if (normalizedTime() > 0.8) {
          debug("main loop timed out" << endl;);
          break;
        }
        for (int j = j_begin; j < j_end; ++j) {
          if (!prev_heatmap[i/2][j/2]) continue;
          if (enable_banning && banned[i][j]) continue;

          int x = i * S / granularity + S / (granularity * 2);
          int y = j * S / granularity + S / (granularity * 2);
          x = min(x, S);
          y = min(y, S);
          if (areamap[x][y]) continue;
          double score = tryAddJunction(x, y, longest_road_in_use);
          if (score < best_score) {
            updated = true;
            best_score = score;
            best_x = x;
            best_y = y;
            best_i = i;
            best_j = j;
          } else if (score > prev_score) {
            banned[i][j] = 1;
          }
        }
      }
      return make_tuple(updated, best_score, best_x, best_y, best_i, best_j);
    };

    double best_score = calculateScore();
    double prev_score = best_score;
    double longest_road_in_use = getLongestRoadInUse();
    int best_x, best_y, best_i, best_j;
    Heatmap heatmap(granularity, vector<int>(granularity, 0));
    int heat_count = 0;
    bool updated = true;

    while (updated && junctions.size() + 1 <= MAX_NJ) {
      if (normalizedTime() > 0.8) {
        debug("main loop timed out" << endl;);
        break;
      }
      updated = false;

      #ifdef ENABLE_PARALLEL_PROCESSING
      vector<future<tuple<bool, double, int, int, int, int>>> tasks;
      for (int t = 0; t < PARALLEL_SPLIT_X; ++t) {
        for (int u = 0; u < PARALLEL_SPLIT_Y; ++u) {
          tasks.emplace_back(
            async(launch::async, compute,
                  t * granularity / PARALLEL_SPLIT_X,
                  (t + 1) * granularity / PARALLEL_SPLIT_X,
                  u * granularity / PARALLEL_SPLIT_Y,
                  (u + 1) * granularity / PARALLEL_SPLIT_Y,
                  best_score, longest_road_in_use));
        }
      }
      for (auto& task : tasks) {
        bool local_updated;
        double local_best_score;
        int local_best_x, local_best_y, local_best_i, local_best_j;
        tie(local_updated, local_best_score,
            local_best_x, local_best_y,
            local_best_i, local_best_j) = task.get();
        if (local_updated && local_best_score < best_score) {
          updated = local_updated;
          best_score = local_best_score;
          best_x = local_best_x;
          best_y = local_best_y;
          best_i = local_best_i;
          best_j = local_best_j;
        }
      }
      #else
      tie(updated, best_score, best_x, best_y, best_i, best_j) =
          compute(0, granularity, 0, granularity,
                  best_score, longest_road_in_use);
      #endif

      if (updated) {
        if (granularity != S + 1) {
          const int dsize = 5;
          const int dstep = 5;

          int original_best_x = best_x;
          int original_best_y = best_y;
          for (int i = 0; i < dstep; ++i) {
            for (int j = 0; j < dstep; ++j) {
              int x = original_best_x + (i - dstep / 2) * dsize / dstep;
              int y = original_best_y + (j - dstep / 2) * dsize / dstep;
              x = max(0, min(x, S));
              y = max(0, min(y, S));
              if (x == original_best_x && y == original_best_y) continue;
              double score = tryAddJunction(x, y, longest_road_in_use);
              if (score < best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
              }
            }
          }
        }

        double ev = (prev_score + J_COST) * F_PROB
            + (best_score + J_COST) * (1.0 - F_PROB);
        if (ev < prev_score) {
          int added_junction_id = addJunction(best_x, best_y);
          longest_road_in_use = getLongestRoadInUse();
          heatmap[best_i][best_j] = 1;
          ++heat_count;
          debug2("prev: " << prev_score << ", now: " << best_score
                 << ", longest: " << longest_road_in_use << endl);

          bool redundant_point_creation_done = false;
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              int x = max(0, min(best_x + i - 1, S));
              int y = max(0, min(best_y + j - 1, S));
              if (areamap[x][y]) continue;
              double score = tryAddJunction(x, y, longest_road_in_use);
              double failed_failed =
                  (prev_score + J_COST * 2) * F_PROB * F_PROB;
              double failed_success =
                  (tryMoveJunction(added_junction_id, x, y) + J_COST * 2)
                  * F_PROB * (1.0 - F_PROB);
              double success_failed =
                  (best_score + J_COST * 2) * (1.0 - F_PROB) * F_PROB;
              double success_success =
                  (score + J_COST * 2) * (1.0 - F_PROB) * (1.0 - F_PROB);
              double ev2 =
                  failed_failed + failed_success
                  + success_failed + success_success;
              if (ev2 < ev) {
                debug2("adding extra points: " << ev2 << " < " << ev << endl);
                addJunction(x, y);
                longest_road_in_use = getLongestRoadInUse();
                best_score = score;
              }
              redundant_point_creation_done = true;
              break;
            }
            if (redundant_point_creation_done) break;
          }

          prev_score = best_score;
        } else {
          updated = false;
        }
      }
    }
    debug("score = " << prev_score << ", heat_count = " << heat_count << endl);
    return make_tuple(move(prev_score), move(heatmap), move(heat_count));
  }

 public:
  vector<int> buildJunctions(int s, vector<int> city_locations,
                             double junctionCost, double failureProbability) {
    startTimer();

    S = s;
    NC = city_locations.size() / 2;
    MAX_NJ = 2 * NC;
    J_COST = junctionCost;
    F_PROB = failureProbability;

    debug2("S = " << S << endl);
    debug2("NC = " << NC << endl);
    debug2("JCost = " << J_COST << endl);
    debug2("FProb = " << F_PROB << endl);

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

    double best_score = 1e8;
    unordered_map<int, Junction> best_junctions;
    for (int enable_banning = 1; enable_banning >= 0; --enable_banning) {
      for(int initial_granularity = 16; ; initial_granularity *= 2) {
        initial_granularity = min(initial_granularity, S + 1);
        Heatmap heatmap(initial_granularity, vector<int>(initial_granularity, 1));
        debug("initial_granularity = " << initial_granularity << endl);
        for (int granularity = initial_granularity; ; granularity *= 2) {
          granularity = min(granularity, S + 1);
          double score;
          int heat_count;
          tie(score, heatmap, heat_count) =
              optimize(granularity, heatmap, enable_banning);
          if (score < best_score) {
            best_score = score;
            best_junctions = junctions;
          }
          while (!junctions.empty()) {
            removeJunction(junctions.begin()->first);
          }
          if (heat_count == 0) break;
          if (granularity == S + 1) break;
          if (normalizedTime() > 0.7) break;
        }
        if (initial_granularity == S + 1) break;
        if (normalizedTime() > 0.7) break;
      }
      if (normalizedTime() > 0.7) break;
    }
    for (const auto& junction : best_junctions) {
      addJunction(junction.second.x, junction.second.y);
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

    debug("buildJunctions finished: " << normalizedTime() << endl);
    debug("build " << junctions.size() << " / " << MAX_NJ
          << " junctions" << endl);

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
