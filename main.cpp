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

// These values are just for documentation purpose.
//
// const int MAX_C = 1000;
// const int MAX_NC = 100;
// const double MAX_JUNCTION_COST = 10.0;
// const double MAX_FAILURE_PROB = 0.4;

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

const int DX_TABLE[] = {0, 1, -1, 0,  0, 1,  1, -1, -1};
const int DY_TABLE[] = {0, 0,  0, 1, -1, 1, -1,  1, -1};

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

inline Point operator+(const Point& a, const Point& b) {
  Point p;
  p.x = a.x + b.x;
  p.y = a.y + b.y;
  return p;
}

inline Point operator-(const Point& a, const Point& b) {
  Point p;
  p.x = a.x - b.x;
  p.y = a.y - b.y;
  return p;
}

inline Point operator/(const Point& a, double d) {
  Point p;
  p.x = a.x / d;
  p.y = a.y / d;
  return p;
}

inline double cross(const Point& a, const Point& b) {
  return a.x * b.y - a.y * b.x;
}

inline double dot(const Point& a, const Point& b) {
  return a.x * b.x + a.x * b.y;
}

inline double norm(const Point& a) {
  return dot(a, a);
}

// based on http://www.prefield.com/algorithm/geometry/ccw.html
int ccw(const Point &a, const Point &b, const Point &c) {
  Point ab = b - a;
  Point ac = c - a;
  if (cross(ab, ac) > 0)   return +1;       // counter clockwise
  if (cross(ab, ac) < 0)   return -1;       // clockwise
  if (dot(ab, ac) < 0)     return +2;       // c--a--b on line
  if (norm(ab) < norm(ac)) return -2;       // a--b--c on line
  return 0;
}

// based on http://www.prefield.com/algorithm/geometry/convex_hull.html
vector<Point> convexHull(const vector<Point>& ps) {
  int n = ps.size(), k = 0;
  vector<Point> ch(2*n);
  for (int i = 0; i < n; ch[k++] = ps[i++]) // lower-hull
    while (k >= 2 && ccw(ch[k-2], ch[k-1], ps[i]) <= 0) --k;
  for (int i = n-2, t = k+1; i >= 0; ch[k++] = ps[i--]) // upper-hull
    while (k >= t && ccw(ch[k-2], ch[k-1], ps[i]) <= 0) --k;
  ch.resize(k-1);
  return ch;
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
  vector<Point> cities_convex_hull;
  set<Road> sorted_roads;
  vector<vector<int>> areamap;
  unordered_map<int, Junction> junctions;
  unordered_map<int, int> junction_id_to_status_index_;

  mutable vector<vector<double>> initial_score_diff_cache;
  mutable vector<vector<int>> has_initial_score_diff_cache;
  mutable vector<vector<set<Point>>> initial_affected_points_cache;

  double startTimeSec;

  double getTimeSec() const {
    #ifdef ENABLE_PARALLEL_PROCESSING
    timeval tv;
    #else
    static timeval tv;
    #endif
    gettimeofday(&tv, NULL);
    return tv.tv_sec + 1e-6 * tv.tv_usec;
  }

  void startTimer() {
    startTimeSec = getTimeSec();
  }

  mutable int normalizedTimeCallCounter = 0;

  double normalizedTime() const {
    // This is for Topcoder judge server.
    // gettimeofday() is really slow on AWS EC2 which Topcoder's judge server
    // is running. Here we call getTimeSec() only once per several calls of
    // this function to aid and workaround that slowness.
    static const int frequency = 1000;
    static double prevResult = 0;
    if (normalizedTimeCallCounter % frequency == 0) {
      double diff = getTimeSec() - startTimeSec;
      prevResult = diff / 10.0;
    }
    ++normalizedTimeCallCounter;
    return prevResult;
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
    #define ADD_ROAD(T) { \
      Road road; \
      road.from = T.id; \
      road.to = junction.id; \
      road.distance = distance(T, junction); \
      sorted_tmp_roads.emplace(move(road)); \
    }

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

    // This should never happen.
    debug("Oops. Something wrong happens at tryAddJunction." << endl);
    return score;
  }

  pair<double, set<Point>> tryAddJunctionAndComputeAffectedPoints(
      int x, int y, const set<Road>& roads_in_use) const {
    const double longest_road_in_use = roads_in_use.rbegin()->distance;

    Junction junction;
    junction.id = 100000000;  // temporary id.
    junction.x = x;
    junction.y = y;

    set<Road> sorted_tmp_roads;

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define ADD_ROAD(T) { \
      Road road; \
      road.from = T.id; \
      road.to = junction.id; \
      road.distance = distance(T, junction); \
      sorted_tmp_roads.emplace(move(road)); \
    }

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
      set<Point> affected_points;
      Point p;
      p.x = x;
      p.y = y;
      affected_points.emplace(move(p));

      Point p2;
      int city_or_junction_id = sorted_tmp_roads.begin()->from;
      if (city_or_junction_id < NC) {
        p2.x = cities[city_or_junction_id].x;
        p2.y = cities[city_or_junction_id].y;
      } else {
        p2.x = junctions.at(city_or_junction_id).x;
        p2.y = junctions.at(city_or_junction_id).y;
      }
      affected_points.emplace(move(p2));

      return make_pair(1e8, move(affected_points));
    }

    double score = 0;
    const int numPointsToConnect = NC + junctions.size() + 1;
    DisjointSet dset;
    set<Road> new_roads_in_use;
    auto compute = [&dset, &score, &numPointsToConnect, &new_roads_in_use](
        const Road& road) {
      int id1 = dset.FindSet(road.from);
      int id2 = dset.FindSet(road.to);
      if (id1 == id2) return false;
      dset.MergeSet(id1, id2);
      score += road.distance;
      new_roads_in_use.insert(road);
      return (dset.LargestSetSize() == numPointsToConnect);
    };

    auto create_result = [this, &score, &roads_in_use, &new_roads_in_use](){
      return make_pair(
        score, getAffectedPoints(roads_in_use, new_roads_in_use));
    };

    auto tmp_road_iter = sorted_tmp_roads.begin();
    for (const Road& road : sorted_roads) {
      while (tmp_road_iter != sorted_tmp_roads.end() && *tmp_road_iter < road) {
        if (compute(*tmp_road_iter)) return create_result();
        ++tmp_road_iter;
      }
      if (compute(road)) return create_result();
    }
    while (tmp_road_iter != sorted_tmp_roads.end()) {
      if (compute(*tmp_road_iter)) return create_result();
      ++tmp_road_iter;
    }

    // This should never happen.
    debug("Oops. Something wrong happens at "
          "tryAddJunctionAndComputeAffectedPoints." << endl);
    return create_result();
  }

  int addJunction(int x, int y) {
    int id = generateJunctionId();
    Junction junction;
    junction.id = id;
    junction.x = x;
    junction.y = y;

    // Note: use define over lambda because template is not supported in
    // lambda yet. It's coming in C++20!
    #define ADD_ROAD(T) { \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
      sorted_roads.emplace(move(road)); \
    }

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
    #define REMOVE_ROAD(T) { \
      Road road; \
      road.from = min(T.id, junction.id); \
      road.to = max(T.id, junction.id); \
      road.distance = distance(T, junction); \
      sorted_roads.erase(road); \
    }

    for (const auto& city : cities) {
      REMOVE_ROAD(city);
    }
    for (const auto& existing_junction : junctions) {
      REMOVE_ROAD(existing_junction.second);
    }
    #undef REMOVE_ROAD
  }

  inline pair<double, set<Point>> getInitialScore(
    int x, int y, double prev_score, const set<Road>& roads_in_use) const {
    if (has_initial_score_diff_cache[x][y]) {
      return make_pair(initial_score_diff_cache[x][y] + prev_score,
                       initial_affected_points_cache[x][y]);
    }
    double score;
    tie(score, initial_affected_points_cache[x][y]) =
        tryAddJunctionAndComputeAffectedPoints(x, y, roads_in_use);
    initial_score_diff_cache[x][y] = score - prev_score;
    has_initial_score_diff_cache[x][y] = true;
    return make_pair(score, initial_affected_points_cache[x][y]);
  }

  inline double getInitialScoreOnly(
    int x, int y, double prev_score, const set<Road>& roads_in_use) const {
    if (has_initial_score_diff_cache[x][y]) {
      return initial_score_diff_cache[x][y] + prev_score;
    }
    double score;
    tie(score, initial_affected_points_cache[x][y]) =
        tryAddJunctionAndComputeAffectedPoints(x, y, roads_in_use);
    initial_score_diff_cache[x][y] = score - prev_score;
    has_initial_score_diff_cache[x][y] = true;
    return score;
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

    // This should never happen.
    debug("Oops. Something wrong happens at calculateScore." << endl);
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
    debug("Oops. Something wrong happens at getLongestRoadInUse." << endl);
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
    debug("Oops. Something wrong happens at getLongestRoadInUse." << endl);
    return ret;
  }

  // Note: this function doesn't support computing diffs by junction removal,
  //     and it assumes all junction IDs in roads are available in
  //     this->junctions. If unknown junction IDs are found, they are simply
  //     ignored. (the only valid usecase is temporary junction ids called
  //     from tryAddJunctionAndComputeAffectedPoints function).
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
        return make_pair(move(p), true);
      }

      auto j_iter = junctions.find(city_or_junction_id);
      if (j_iter == junctions.end()) {
        return make_pair(move(p), false);
      }
      p.x = j_iter->second.x;
      p.y = j_iter->second.y;
      return make_pair(move(p), true);
    };

    set<Point> ret;
    for (const Road& road : affected_roads) {
      Point p;
      bool valid;
      tie(p, valid) = to_point(road.from);
      if (valid) ret.insert(p);
      tie(p, valid) = to_point(road.to);
      if (valid) ret.insert(p);
    }
    return ret;
  }

  // Based on http://www.prefield.com/algorithm/geometry/convex_contains.html.
  inline bool isInConvexHull(const Point& p) const {
    const int n = cities_convex_hull.size();
    Point g = (cities_convex_hull[0] + cities_convex_hull[n/3]
        + cities_convex_hull[2*n/3]) / 3.0; // inner-point
    int a = 0, b = n;
    while (a + 1 < b) {
      // invariant: c is in fan g-P[a]-P[b]
      int c = (a + b) / 2;
      // angle < 180 deg
      if (cross(cities_convex_hull[a]-g, cities_convex_hull[c]-g) > 0) {
        if (cross(cities_convex_hull[a]-g, p-g) > 0 &&
            cross(cities_convex_hull[c]-g, p-g) < 0) {
          b = c;
        } else {
          a = c;
        }
      } else {
        if (cross(cities_convex_hull[a]-g, p-g) < 0 &&
            cross(cities_convex_hull[c]-g, p-g) > 0) {
          a = c;
        } else {
          b = c;
        }
      }
    }
    b %= n;
    if (cross(cities_convex_hull[a] - p, cities_convex_hull[b] - p) < 0) {
      return false;
    }
    if (cross(cities_convex_hull[a] - p, cities_convex_hull[b] - p) > 0) {
      return true;
    }
    return true;
  }

  inline bool isInConvexHull(int x, int y) const {
    Point p;
    p.x = x;
    p.y = y;
    return isInConvexHull(p);
  };

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
    vector<vector<set<Point>>> affected_points_cache(
        granularity, vector<set<Point>>(granularity));

    auto compute =
        [this, granularity, &prev_heatmap, &banned,
         &score_diff_cache, &has_score_diff_cache, &affected_points_cache,
         enable_banning] (
            int i_begin, int i_end, int j_begin, int j_end,
            double best_score, const set<Road> roads_in_use,
            const set<Point>& affected_points) {

      auto convert_to_area_coord = [this, granularity](int i) {
        return min(
            i * (S + 1) / granularity + (S + 1) / (granularity * 2),
            S);
      };

      double longest_road_in_use = roads_in_use.rbegin()->distance;
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
      auto can_use_cache =
          [this, &in_affected_area, &has_score_diff_cache,
           &affected_points_cache](
              int i, int j, int x, int y) {
        if (!has_score_diff_cache[i][j]) return false;
        if (in_affected_area(x, y)) return false;
        for (const Point& p : affected_points_cache[i][j]) {
          if (in_affected_area(p.x, p.y)) return false;
        }
        return true;
      };

      bool updated = false;
      double prev_score = best_score;
      int best_x = -1;
      int best_y = -1;
      int best_i = -1;
      int best_j = -1;

      #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
      bool best_is_from_cache = false;
      #endif

      for (int i = i_begin; i < i_end; ++i) {
        if (normalizedTime() > 0.9) {
          debug("main loop timed out" << endl;);
          break;
        }
        for (int j = j_begin; j < j_end; ++j) {
          if (normalizedTime() > 0.9) {
            debug("main loop timed out" << endl;);
            break;
          }
          if (!prev_heatmap[i/2][j/2]) continue;
          if (enable_banning && banned[i][j]) continue;

          int x = convert_to_area_coord(i);
          int y = convert_to_area_coord(j);
          if (areamap[x][y]) continue;
          if (!isInConvexHull(x, y)) {
            banned[i][j] = true;
            continue;
          }

          #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
          bool from_cache = false;
          #endif

          double score;
          if (can_use_cache(i, j, x, y)) {
            score = score_diff_cache[i][j] + prev_score;

            #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
            from_cache = true;
            #endif
          } else {
            if (junctions.empty()) {
              tie(score, affected_points_cache[i][j]) =
                  getInitialScore(x, y, prev_score, roads_in_use);
            } else {
              tie(score, affected_points_cache[i][j]) =
                  tryAddJunctionAndComputeAffectedPoints(x, y, roads_in_use);
            }
            score_diff_cache[i][j] = score - prev_score;
            has_score_diff_cache[i][j] = 1;

            #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
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

            #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
            best_is_from_cache = from_cache;
            #endif
          } else if (score > prev_score) {
            banned[i][j] = 1;
          }
        }
      }

      #if defined(LOCAL_DEBUG_MODE) || defined(TOPCODER_TEST_MODE)
      if (best_is_from_cache) {
        double actual_score =
            tryAddJunction(best_x, best_y, longest_road_in_use);
        debug2("Best from cached value. cache = " << best_score
               << ", actual = " << actual_score
               << ", x = " << best_x << ", y = " << best_y << endl);
        if (fabs(best_score - actual_score) > 1e-8) {
          debug("Oops. There is a bug in score cache. cache = "
                << best_score << ", actual = " << actual_score
                << ", error = " << fabs(best_score - actual_score) << endl);
        }
      }
      #endif

      return make_tuple(updated, best_score, best_x, best_y, best_i, best_j);
    };

    double best_score = calculateScore();
    double longest_road_in_use = getLongestRoadInUse();
    set<Road> roads_in_use = getRoadsInUse();
    int best_x = -1, best_y = -1, best_i = -1, best_j = -1;
    Heatmap heatmap(granularity, vector<int>(granularity, 0));
    int heat_count = 0;
    set<Point> affected_points;
    bool updated = true;

    while (updated && junctions.size() + 1 <= MAX_NJ) {
      if (normalizedTime() > 0.85) {
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
                  best_score, roads_in_use, affected_points));
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
                  best_score, roads_in_use, affected_points);
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
              if (!isInConvexHull(x, y)) continue;
              double score;
              if (junctions.empty()) {
                score = getInitialScoreOnly(x, y, prev_score, roads_in_use);
              } else {
                score = tryAddJunction(x, y, longest_road_in_use);
              }
              if (score < best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
              }
            }
          }
        }

        // Only create new junction if expected value is better than the current
        // score.
        double best_ev = (best_score + J_COST) * (1.0 - F_PROB) +
            (prev_score + J_COST) * F_PROB;
        if (best_ev < prev_score) {
          debug2("Create junction at " << best_x << ", " << best_y << endl);
          const set<Road> prev_roads_in_use = roads_in_use;
          addJunction(best_x, best_y);
          best_score = calculateScore();
          roads_in_use = getRoadsInUse();
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

    #ifdef LOCAL_DEBUG_MODE
    debug2(endl);
    debug2("Score = " << (calculateScore() + J_COST * junctions.size())
           << ", raw score = " << calculateScore()
           << ", junctions = " << junctions.size()
           << ", heat_count = " << heat_count << endl << endl);
    debug2("Try making redundant junctions." << endl);
    #endif
    maybeAddRedundantPoints();

    double raw_score = calculateScore();
    double result_score = raw_score + J_COST * junctions.size();
    debug(endl);
    debug("Score = " << result_score << ", raw score = " << raw_score
          << ", junctions = " << junctions.size()
          << ", heat_count = " << heat_count << endl);
    debug(endl);

    return make_tuple(move(result_score), move(heatmap), move(heat_count));
  }

  void maybeAddRedundantPoints() {
    auto compute_ev = [this](const int num_junctions,
                             const int junction_x,
                             const int junction_y,
                             vector<Point>* result_junction_positions) {
      double ev = 0;
      function<void(int,double)> internal =
          [&](int remaining, double cur_prob) {
        if (remaining <= 0) {
          ev += cur_prob * (calculateScore() + num_junctions * J_COST);
          return;
        }

        const int level = num_junctions - remaining;

        // Search best place to build a redundant junction.
        int best_x = -1, best_y = -1;
        if (level < result_junction_positions->size()) {
          best_x = (*result_junction_positions)[level].x;
          best_y = (*result_junction_positions)[level].y;
        } else {
          int best_i = -1;
          double best_score = 1e8;
          for (int i = 0; i < 9; ++i) {
            int x = junction_x + DX_TABLE[i];
            int y = junction_y + DY_TABLE[i];

            if (x < 0 || x > S || y < 0 || y > S) continue;
            if (areamap[x][y]) continue;

            double score = tryAddJunction(x, y, 1e8);
            if (score < best_score) {
              best_score = score;
              best_i = i;
              best_x = x;
              best_y = y;
            }
          }
          if (best_i == -1) {
            // No available spaces.
            debug("No available space to create redundant points.");
            ev = 1e8;
            return;
          }
          result_junction_positions->emplace_back(Point{best_x, best_y});
        }

        // Case 1. Failed junction construction.
        areamap[best_x][best_y] = 2;
        internal(remaining - 1, cur_prob * F_PROB);
        areamap[best_x][best_y] = 0;

        // Case 2. Successful junction construction.
        int j_id = addJunction(best_x, best_y);
        internal(remaining - 1, cur_prob * (1.0 - F_PROB));
        removeJunction(j_id);
      };

      internal(num_junctions, 1.0);
      return ev;
    };

    // Try to improve E.V. by adding redundant junctions.
    // i.e. We create up to 1 + MAX_REDUNDANCY points for one location.
    static const int MAX_REDUNDANCY = 4;

    auto original_junctions = junctions;
    for (const auto& junction : original_junctions) {
      removeJunction(junction.first);
      debug2("maybeAddRedundantPoints for " << junction.second.x
             << ", " << junction.second.y << endl);
      double best_ev = calculateScore();
      int best_r = -1;
      vector<Point> result_positions;
      for (int r = 0; r <= MAX_REDUNDANCY; ++r) {
        double ev = compute_ev(1 + r,
                               junction.second.x,
                               junction.second.y,
                               &result_positions);
        if (ev < best_ev) {
          debug2("r = " << r << ", " << ev << " < " << best_ev << endl);
          best_ev = ev;
          best_r = r;
        } else {
          // If E.V. didn't improve by adding one redundancy, no need to
          // try adding more.
          debug2("r = " << r << ", " << ev << " >= " << best_ev << endl);
          break;
        }
      }
      for (int r = 0; r <= best_r; ++r) {
        addJunction(result_positions[r].x, result_positions[r].y);
      }
    }
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

    initial_score_diff_cache =
        vector<vector<double>>(S+1, vector<double>(S+1, 0));
    has_initial_score_diff_cache =
        vector<vector<int>>(S+1, vector<int>(S+1, 0));
    initial_affected_points_cache =
        vector<vector<set<Point>>>(S+1, vector<set<Point>>(S+1));

    cities.reserve(NC);
    for (int i = 0; i < NC; ++i) {
      City city;
      city.id = i;
      city.x = city_locations[2*i];
      city.y = city_locations[2*i + 1];
      cities.emplace_back(move(city));
    }

    vector<Point> city_points(NC);
    for (int i = 0; i < NC; ++i) {
      city_points[i].x = cities[i].x;
      city_points[i].y = cities[i].y;
    }
    sort(city_points.begin(), city_points.end());
    cities_convex_hull = convexHull(city_points);

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
      if (normalizedTime() > 0.8) break;
      for(int initial_granularity = 64; ; initial_granularity *= 2) {
        if (normalizedTime() > 0.8) break;
        initial_granularity = min(initial_granularity, S + 1);
        Heatmap heatmap(initial_granularity, vector<int>(initial_granularity, 1));
        debug("--------------------------------" << endl);
        debug("initial_granularity = " << initial_granularity << endl);
        for (int granularity = initial_granularity; ; granularity *= 2) {
          if (normalizedTime() > 0.8) break;
          granularity = min(granularity, S + 1);
          double score;
          int heat_count;
          tie(score, heatmap, heat_count) =
              optimize(granularity, heatmap, enable_banning);
          if (score < best_score) {
            debug("// Found better answer." << endl);
            debug("// score: " << best_score << " -> " << score << endl);
            debug("// junctions: " << best_junctions.size()
                  << " -> " << junctions.size() << endl);
            best_score = score;
            best_junctions = junctions;
          }
          while (!junctions.empty()) {
            removeJunction(junctions.begin()->first);
          }
          if (heat_count == 0) break;
          if (granularity == S + 1) break;
        }
        if (initial_granularity == S + 1) break;
      }
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
    debug("normalizedTime() call counter = "
          << normalizedTimeCallCounter << endl);
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
