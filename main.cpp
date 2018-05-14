// #define TOPCODER_TEST_MODE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>
#include <sys/time.h>
#include <future>
#include <iterator>

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
  if (a.to == b.to && a.from == b.from) return false;
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

struct Point {
  int x;
  int y;
};

inline bool operator<(const Point& a, const Point& b) {
  if (a.x < b.x) return true;
  if (a.x > b.x) return false;
  return a.y < b.y;
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

inline int distance2(int x1, int y1, int x2, int y2) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  return dx * dx + dy * dy;
}

template<typename S, typename T>
inline double distance(const S& a, const T& b) {
  return sqrt(distance2(a.x, a.y, b.x, b.y));
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
    while (tmp_road_iter != sorted_tmp_roads.end()) {
      if (compute(*tmp_road_iter)) return score;
      ++tmp_road_iter;
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
    while (tmp_road_iter != sorted_tmp_roads.end()) {
      if (compute(*tmp_road_iter)) return score;
      ++tmp_road_iter;
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

  set<Road> getRoadsInUse() const {
    set<Road> ret;
    const int numPointsToConnect = NC + junctions.size();
    DisjointSet dset;
    for (const Road& road : sorted_roads) {
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) continue;
      dset.MergeSet(id1, id2);
      ret.insert(road);
      if (dset.LargestSetSize() == numPointsToConnect) return ret;
    }
    // This should never happen.
    debug("Oops. Something wrong happens at getLongestRoadInUse.");
    return ret;
  }

  // Note: this function doesn't support computing diffs by junction removal,
  //     and it assumes all junction IDs in roads are available in
  //     this->junctions.
  set<Point> getAffectedPoints(const set<Road>& prev_roads_in_use,
                               const set<Road>& curr_roads_in_use) const {
    vector<Road> affected_roads;
    affected_roads.reserve(NC + junctions.size());
    set_symmetric_difference(prev_roads_in_use.begin(),
                             prev_roads_in_use.end(),
                             curr_roads_in_use.begin(),
                             curr_roads_in_use.end(),
                             back_inserter(affected_roads));

    auto to_point = [this](int city_or_junction_id) {
      Point p;
      if (city_or_junction_id < NC) {
        p.x = cities[city_or_junction_id].x;
        p.y = cities[city_or_junction_id].y;
      } else {
        p.x = junctions.at(city_or_junction_id).x;
        p.y = junctions.at(city_or_junction_id).y;
      }
      return p;
    };

    set<Point> ret;
    for (const Road& road : affected_roads) {
      ret.emplace(to_point(road.from));
      ret.emplace(to_point(road.to));
    }
    return ret;
  }

  tuple<double, Heatmap, int> optimize(const int granularity,
                                       const Heatmap& prev_heatmap,
                                       const bool enable_banning) {
    debug(endl);
    debug("Start optimize: granularity = " << granularity
          << ", banning = " << enable_banning << endl);

    vector<vector<int>> banned(granularity, vector<int>(granularity, 0));
    vector<vector<double>> score_diff_cache(
        granularity, vector<double>(granularity, 0));
    vector<vector<int>> has_score_diff_cache(
        granularity, vector<int>(granularity, 0));

    auto compute =
        [this, granularity, &prev_heatmap, &banned,
         &score_diff_cache, &has_score_diff_cache, enable_banning] (
            int i_begin, int i_end, int j_begin, int j_end,
            double best_score, double longest_road_in_use,
            const set<Point>& affected_points) {

      auto convert_to_area_coord = [this, granularity](int i) {
        return min(i * S / granularity + S / (granularity * 2), S);
      };

      double longest_road_in_use2 = longest_road_in_use * longest_road_in_use;
      auto in_affected_area =
          [&affected_points, longest_road_in_use2](int x, int y) {
        for (const Point& p : affected_points) {
          if (distance2(x, y, p.x, p.y) <= longest_road_in_use2) {
            return true;
          }
        }
        return false;
      };

      bool updated = false;
      double prev_score = best_score;
      int best_x, best_y, best_i, best_j;

      #ifdef LOCAL_DEBUG_MODE
      bool best_is_from_cache;
      #endif

      for (int i = i_begin; i < i_end; ++i) {
        if (normalizedTime() > 0.8) {
          debug("main loop timed out" << endl;);
          break;
        }
        for (int j = j_begin; j < j_end; ++j) {
          if (!prev_heatmap[i/2][j/2]) continue;
          if (enable_banning && banned[i][j]) continue;

          int x = convert_to_area_coord(i);
          int y = convert_to_area_coord(j);
          if (areamap[x][y]) continue;

          #ifdef LOCAL_DEBUG_MODE
          bool from_cache;
          #endif

          double score;
          if (has_score_diff_cache[i][j] && !in_affected_area(x, y)) {
            score = score_diff_cache[i][j] + prev_score;

            #ifdef LOCAL_DEBUG_MODE
            from_cache = true;
            #endif
          } else {
            score = tryAddJunction(x, y, longest_road_in_use);
            score_diff_cache[i][j] = score - prev_score;
            has_score_diff_cache[i][j] = 1;

            #ifdef LOCAL_DEBUG_MODE
            from_cache = false;
            #endif
          }

          if (score < best_score) {
            updated = true;
            best_score = score;
            best_x = x;
            best_y = y;
            best_i = i;
            best_j = j;

            #ifdef LOCAL_DEBUG_MODE
            best_is_from_cache = from_cache;
            #endif
          } else if (score > prev_score) {
            banned[i][j] = 1;
          }
        }
      }

      #ifdef LOCAL_DEBUG_MODE
      if (best_is_from_cache) {
        double actual_score =
            tryAddJunction(best_x, best_y, longest_road_in_use);
        debug2("Best from cached value. cache = " << best_score
               << " actual = " << actual_score << endl);
      }
      #endif

      return make_tuple(updated, best_score, best_x, best_y, best_i, best_j);
    };

    double best_score = calculateScore();
    double longest_road_in_use = getLongestRoadInUse();
    int best_x = -1, best_y = -1, best_i = -1, best_j = -1;
    Heatmap heatmap(granularity, vector<int>(granularity, 0));
    int heat_count = 0;
    set<Point> affected_points;
    bool updated = true;

    while (updated && junctions.size() + 1 <= MAX_NJ) {
      if (normalizedTime() > 0.8) {
        debug("main loop timed out" << endl;);
        break;
      }
      updated = false;

      const double prev_score = best_score;

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
                  best_score, longest_road_in_use,
                  affected_points));
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
                  best_score, longest_road_in_use,
                  affected_points);
      #endif

      if (updated) {
        // If granularity is not 100%, search near areas of current best
        // junction point.
        const int original_best_x = best_x;
        const int original_best_y = best_y;
        if (granularity != S + 1) {
          const int dsize = 5;
          const int dstep = 5;

          for (int i = 0; i < dstep; ++i) {
            for (int j = 0; j < dstep; ++j) {
              int x = original_best_x + (i - dstep / 2) * dsize / dstep;
              int y = original_best_y + (j - dstep / 2) * dsize / dstep;
              x = max(0, min(x, S));
              y = max(0, min(y, S));
              if (x == original_best_x && y == original_best_y) continue;
              if (areamap[x][y]) continue;
              double score = tryAddJunction(x, y, longest_road_in_use);
              if (score < best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
              }
            }
          }
        }

        static const int DX_TABLE[] = {0, 1, -1, 0,  0, 1,  1, -1, -1};
        static const int DY_TABLE[] = {0, 0,  0, 1, -1, 1, -1,  1, -1};
        auto compute_ev =
            [this, best_x, best_y,
             longest_road_in_use, prev_score](const int num_new_junctions) {
          double ev = 0;
          function<void(int,double,double)> internal =
              [&](int remaining, double cur_prob, double cur_score) {
            if (remaining <= 0) {
              ev += cur_prob * (cur_score + num_new_junctions * J_COST);
              return;
            }

            // Case 1. Failed junction construction.
            internal(remaining - 1, cur_prob * F_PROB, cur_score);

            // Case 2. Successful junction construction.
            for (int i = 0; i < 9; ++i) {
              int x = max(0, min(best_x + DX_TABLE[i], S));
              int y = max(0, min(best_y + DY_TABLE[i], S));
              if (areamap[x][y]) continue;
              if (remaining == 1) {
                double score = tryAddJunction(x, y, longest_road_in_use);
                internal(0, cur_prob * (1.0 - F_PROB), score);
              } else {
                int j_id = addJunction(x, y);
                double score = calculateScore();
                internal(remaining - 1, cur_prob * (1.0 - F_PROB), score);
                removeJunction(j_id);
              }
              return;
            }
            // No available spaces.
            debug("No available space to create redundant points.");
            ev = 1e8;
          };
          internal(num_new_junctions, 1.0, prev_score);
          return ev;
        };

        // Only create new junction if expected value is better than the current
        // score.
        double best_ev = compute_ev(1);
        if (best_ev < prev_score) {
          // Try to improve E.V. by adding redundant junctions.
          // i.e. We create up to 1 + MAX_REDUNDANCY points for one location.
          const int MAX_REDUNDANCY = 4;
          int best_r = 0;
          for (int r = 1; r <= MAX_REDUNDANCY; ++r) {
            double ev = compute_ev(1 + r);
            if (ev < best_ev) {
              debug2("r = " << r << ", " << ev << " < " << best_ev << endl);
              best_ev = ev;
              best_r = r;
            } else {
              // If E.V. didn't improve by adding one redundancy, no need to
              // try adding more.
              break;
            }
          }

          debug2("Create junction at " << best_x << ", " << best_y << endl);
          const set<Road> prev_roads_in_use = getRoadsInUse();

          addJunction(best_x, best_y);
          if (best_r) {
            for (int i = 1, r = best_r; i < 9 && r; ++i) {
              int x = max(0, min(best_x + DX_TABLE[i], S));
              int y = max(0, min(best_y + DY_TABLE[i], S));
              if (areamap[x][y]) continue;
              addJunction(x, y);
              --r;
            }
            best_score = calculateScore();
          }

          const set<Road> roads_in_use = getRoadsInUse();
          longest_road_in_use = roads_in_use.rbegin()->distance;
          heatmap[best_i][best_j] = 1;
          ++heat_count;
          affected_points = getAffectedPoints(prev_roads_in_use, roads_in_use);

          #ifdef LOCAL_DEBUG_MODE
          debug2("Affected points: ");
          for (const Point& p : affected_points) {
            debug2("(" << p.x << ", " << p.y << "), ");
          }
          debug2(endl);
          #endif

          debug2("ev = " << best_ev << ", prev = " << prev_score
                 << ", raw score = " << best_score
                 << ", redundancy = " << best_r
                 << ", longest = " << longest_road_in_use << endl);
        } else {
          debug2("Tried at " << best_x << ", " << best_y << " but ev = "
                << best_ev << " didn't improve " << prev_score << endl);
          debug2("original_best_x = " << original_best_x
                << ", original_best_y = " << original_best_y << endl);
          debug2("raw best_score = " << best_score << endl);
          debug2("No more good junction points found. Exiting from search."
                << endl);

          updated = false;
          break;
        }
      }
    }

    double raw_score = calculateScore();
    double result_score = raw_score + J_COST * junctions.size();
    debug(endl);
    debug("Score = " << result_score << ", raw score = " << raw_score
          << ", junctions = " << junctions.size()
          << ", heat_count = " << heat_count << endl);
    debug(endl);
    return make_tuple(move(result_score), move(heatmap), move(heat_count));
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
        debug("--------------------------------" << endl);
        debug("initial_granularity = " << initial_granularity << endl);
        for (int granularity = initial_granularity; ; granularity *= 2) {
          granularity = min(granularity, S + 1);
          double score;
          int heat_count;
          tie(score, heatmap, heat_count) =
              optimize(granularity, heatmap, enable_banning);
          if (score < best_score) {
            debug("--------------------------------" << endl);
            debug("Found better answer." << endl);
            debug("score: " << best_score << " -> " << score << endl);
            debug("junctions: " << best_junctions.size()
                  << " -> " << junctions.size() << endl);
            debug("--------------------------------" << endl);
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

    debug("--------------------------------" << endl);
    debug("Final results." << endl);
    debug("score: " << best_score
          << " (when all constructions succeeded)" << endl);
    debug("junctions: " << best_junctions.size() << endl);
    debug("--------------------------------" << endl);

    debug("buildJunctions finished: " << normalizedTime() << endl);
    debug("build " << junctions.size() << " / " << MAX_NJ
          << " junctions" << endl);
    debug("S = " << S << endl);
    debug("NC = " << NC << endl);
    debug("JCost = " << J_COST << endl);
    debug("FProb = " << F_PROB << endl);

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
